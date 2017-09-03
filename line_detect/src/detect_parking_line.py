#!/usr/bin/env python
#from line_detector.AntiInstagram import *
from cv_bridge import CvBridge, CvBridgeError
from turtlebot3_auto_msgs.msg import ( BoolStamped, Segment,
    SegmentList, Vector2D)
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils.jpg import image_cv_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from line_detector.line_detector_plot import *
from line_detector.timekeeper import TimeKeeper
import cv2
import numpy as np
import rospy
import threading
import time
import yaml
from math import floor, atan2, pi, cos, sin, sqrt
import rospy

class ParkingLineDetectorNode(object):
    def __init__(self):
        self.node_name = "ParkingLineDetectorNode"

        # Thread lock
        self.thread_lock = threading.Lock()

        # Constructor of line detector
        self.bridge = CvBridge()

        self.stats = Stats()

        # Only be verbose every 10 cycles
        self.intermittent_interval = 1
        self.intermittent_counter = 1

        # color correction
        #self.ai = AntiInstagram()

        # these will be added if it becomes verbose
        self.pub_edge = None
        self.pub_colorSegment = None

        self.detector = None
        self.verbose = None
        self.updateParams(None)

        # Publishers
        self.pub_lines = rospy.Publisher("segment_parking_list", SegmentList, queue_size=1)
	#self.pub_image_origin = rospy.Publisher("image_with_origin", Image, queue_size=1)
	#self.pub_image_cal = rospy.Publisher("image_with_cal", Image, queue_size=1)
        self.pub_image_with_line = rospy.Publisher("image_with_parking_lines", Image, queue_size=1)

        # Subscribers
        self.sub_image = rospy.Subscriber("image_raw/compressed", CompressedImage, self.cbImage, queue_size=1)
        #self.sub_transform = rospy.Subscriber("transform", AntiInstagramTransform, self.cbTransform, queue_size=1)

        rospy.loginfo("[%s] Initialized (verbose = %s)." %(self.node_name, self.verbose))

        rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)

	# calibration data 
	with open('/home/rt/catkin_ws/src/turtlebot3_auto/line_detect/src/callibra_ros.yaml') as fr:
	#with open('/home/rt/catkin_ws/src/turtlebot3_auto/line_detect/src/callibra_self.yaml') as fr:
            self.c = yaml.load(fr)

    def updateParams(self, _event):
        old_verbose = self.verbose
        self.verbose = rospy.get_param('~verbose', True)
        #self.loginfo('verbose = %r' % self.verbose)
        if self.verbose != old_verbose:
            self.loginfo('Verbose is now %r' % self.verbose)

        self.image_size = rospy.get_param('~img_size')
        self.top_cutoff = rospy.get_param('~top_cutoff')
	self.loginfo('verbose = %d' % self.top_cutoff)

        if self.detector is None:
            c = rospy.get_param('~detector')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new detector config: %s' % str(c))

            self.detector = instantiate(c[0], c[1])

        if self.verbose and self.pub_edge is None:
            self.pub_edge = rospy.Publisher("~parkingedge", Image, queue_size=1)
            self.pub_colorSegment = rospy.Publisher("~parkingcolorSegment", Image, queue_size=1)

    def cbImage(self, image_msg):
        self.stats.received()

        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def intermittent_log_now(self):
	return self.intermittent_counter % self.intermittent_interval == 0

    def intermittent_log(self, s):
        if not self.intermittent_log_now():
            return
        self.loginfo('%3d:%s' % (self.intermittent_counter, s))

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            self.stats.skipped()
            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()

    def processImage_(self, image_msg):

        self.stats.processed()

        if self.intermittent_log_now():
            #self.intermittent_log(self.stats.info())
            self.stats.reset()

        tk = TimeKeeper(image_msg)

        self.intermittent_counter += 1

        # Decode from compressed image with OpenCV
        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        tk.completed('decoded')

	#origin image publish
	image_cv_origin_p = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
	image_cv_origin_p.header.stamp = image_msg.header.stamp
        #self.pub_image_origin.publish(image_cv_origin_p)

	#calibration proc && image publish

	#K_undistort = np.array(self.c['camera_matrix'])
	#img_und = cv2.undistort(image_cv, np.array(self.c['camera_matrix']), np.array(self.c['dist_coefs']), newCameraMatrix=K_undistort)
	#img_und_p = self.bridge.cv2_to_imgmsg(img_und, "bgr8")
        #self.pub_image_cal.publish(img_und_p)

        # Resize and crop image and rotation
        hei_original, wid_original = image_cv.shape[0:2]

        if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            image_cv = cv2.resize(image_cv, (self.image_size[1], self.image_size[0]),
                                   interpolation=cv2.INTER_NEAREST)
	
	rotation_center = (self.image_size[1]/2, self.image_size[0]/2)
	rotation_mat = cv2.getRotationMatrix2D(rotation_center, 180, 1.0)
	image_cv = cv2.warpAffine(image_cv, rotation_mat, (self.image_size[1], self.image_size[0]))

        image_cv = image_cv[self.top_cutoff:,:,:]

        tk.completed('resized')

        # apply color correction: AntiInstagram
        # image_cv_corr = image_cv
        #image_cv_corr = self.ai.applyTransform(img_und)
        image_cv_corr = cv2.convertScaleAbs(image_cv)

        tk.completed('corrected')

        # Set the image to be detected
        self.detector.setImage(image_cv_corr)
	hei2_original, wid2_original = image_cv_corr.shape[0:2]
	

        # Detect lines and normals

        white = self.detector.detectLines('white')
        #yellow = self.detector.detectLines('yellow')
        #red = self.detector.detectLines('red')
        #rospy.loginfo('after %.1f %.1f  ',hei2_original , wid2_original)
	#rospy.loginfo('after %d %d  (%.1f %.1f) (%.1f %.1f)',hei2_original , wid2_original, white.lines.x1)

        tk.completed('detected')

        # SegmentList constructor
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp

        # Convert to normalized pixel coordinates, and add segments to segmentList
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.image_size[1], 1./self.image_size[0], 1./self.image_size[1], 1./self.image_size[0]))

	rospy.loginfo('white.lines %d', len(white.lines))
        if len(white.lines) > 0:
            #lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
	    lines_normalized_white = white.lines
            #segmentList.segments.extend(self.toSegmentMsg(lines_normalized_white, white.normals, Segment.WHITE))
	    #rospy.loginfo('white detect ')
        #if len(yellow.lines) > 0:
            #lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
	    #lines_normalized_yellow = yellow.lines	    
            #segmentList.segments.extend(self.toSegmentMsg(lines_normalized_yellow, yellow.normals, Segment.YELLOW))
	    #rospy.loginfo('yellow detect ' )
        #if len(red.lines) > 0:
            #lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
	    #lines_normalized_red = red.lines
            #segmentList.segments.extend(self.toSegmentMsg(lines_normalized_red, red.normals, Segment.RED))

        # Publish segmentList
        self.pub_lines.publish(segmentList)
	#self.loginfo('send segmentList publish finish')


        # VISUALIZATION only below

        if self.verbose:

            # Draw lines and normals
            image_with_lines = np.copy(image_cv_corr)
	    for i in xrange(len(white.lines)):
		#if i == 5 :
		#   break
            	for x1,y1,x2,y2 in white.lines[i]:
                    cv2.line(image_with_lines,(x1,y1),(x2,y2),(0,0,255),3)

	    # arrange 
	    #i = 0
	    px1 = 0
	    py1 = 1
	    px2 = 2 
	    py2 = 3
	    pdeg = 4
	    LM = [[0]*5 for i in range(len(white.lines))]
	    for i in xrange(len(white.lines)):
            	for x1,y1,x2,y2 in white.lines[i]:
		    if x1 <= x2 :
		       LM[i][px1]= x1
		       LM[i][py1]= y1
		       LM[i][px2]= x2
		       LM[i][py2]= y2		      
		    else :
		       LM[i][px1]= x2
		       LM[i][py1]= y2
		       LM[i][px2]= x1
		       LM[i][py2]= y1
	    # add degree and check degree
	    #i = 0
	    pdegM = [[0]*1 for i in range(len(LM))]
	    for i in xrange(len(LM)):
            	LM[i][pdeg] = (( LM[i][py2] - LM[i][py1] )*100) / ( LM[i][px2] - LM[i][px1] )
		pdegM[i] = LM[i][pdeg]				
		rospy.loginfo('(%d, %d) (%d, %d) %d ',LM[i][px1], LM[i][py1], LM[i][px2], LM[i][py2], LM[i][pdeg] )
	    degM=np.mean(pdegM)
	    rospy.loginfo('mean %d ,var  %d', np.mean(pdegM) , np.var(pdegM))

	    # Make Block 
	    i = 0
	    j = 0
	    BM = []
	    BM.append(LM[0])
	    #rospy.loginfo('BLOCK NUMBER1 %d ', len(BM)); 
	    #BM.append(LM[1])
	    #rospy.loginfo('BLOCK NUMBER2 %d ', len(BM)); 
	    #BM.append(LM[2])
	    #rospy.loginfo('BLOCK NUMBER3 %d ', len(BM)); 

	    #for i in xrange(len(LM)) :
	    	#rospy.loginfo('////////////////////////////////////////// %d  ', len(BM))
	    	#for j in xrange(len(BM)):		    
	    	    #if (BM[j][px1] <= LM[i][px1] and LM[i][px1] <= BM[j][px2]) or (BM[j][px1] <= LM[i][px2] and LM[i][px2] <= BM[j][px2]) or  (LM[i][px1] <= BM[j][px1] and BM[j][px1] <= LM[i][px2]) or (LM[i][px1] <= BM[j][px2] and BM[j][px2] <= LM[i][px2]) :
			#inBM = [ BM[j][px1], LM[i][px1]], [BM[j][px2], LM[i][px2] ]
			#BM[j][px1] = np.min(inBM)
			#BM[j][px2] = np.max(inBM)
			#rospy.loginfo('BLOCK NUMBER i%d  j%d  ( %d  %d ) ', i, j ,BM[j][px1], BM[j][px2]);
			#continue   
		    #BM.append(LM[i])
		    #break 
	    
	    px1M = [[LM[i][px1]] for i in range(len(LM))]
	    px2M = [[LM[i][px2]] for i in range(len(LM))]
	    xMix = np.min(px1M)
	    XMax = np.max(px2M)	
            rospy.loginfo('//// min %d max %d ', xMix , XMax)

	    #overlap = False
	    #for j in xrange(xMix,XMax+1):

                #for i in xrange(len(LM)) : 
		    #for k in xrange((LM[i][px1],LM[i][px2]+1) 
			#k == j
			#overlap = True

	    overlap = False
	    for i in xrange(len(LM)) :
	    	rospy.loginfo('////////////////////////////////////////// %d  ', len(BM))
	    	for j in xrange(len(BM)):
		#while True :
		    overlap = False
	    	    for m in xrange(BM[j][px1],BM[j][px2]+1):
			for n in xrange(LM[i][px1],LM[i][px2]+1):
			    if m == n:
				overlap = True

		    if overlap :
			rospy.loginfo('LM %d ,BM %d overlap happen', i , j)
			inBM = [ BM[j][px1], LM[i][px1]], [BM[j][px2], LM[i][px2] ]
			BM[j][px1] = np.min(inBM)
			BM[j][px2] = np.max(inBM)
			break
		    BM.append(LM[i]) 
		    j = j+1
		    break
		       

	    #rospy.loginfo('////////////////////////////////////////// %d  ', len(BM))
	        
            # Publish the frame with lines
            image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
            image_msg_out.header.stamp = image_msg.header.stamp
            self.pub_image_with_line.publish(image_msg_out)

            tk.completed('pub_image')

        if self.verbose:
            #colorSegment = color_segment(white.area, red.area, yellow.area)
	    colorSegment = color_segment(white.area, white.area, white.area)
            edge_msg_out = self.bridge.cv2_to_imgmsg(self.detector.edges, "mono8")
            colorSegment_msg_out = self.bridge.cv2_to_imgmsg(colorSegment, "bgr8")
            self.pub_edge.publish(edge_msg_out)
            self.pub_colorSegment.publish(colorSegment_msg_out)

            tk.completed('pub_edge/pub_segment')


        #self.intermittent_log(tk.getall())


    def onShutdown(self):
        self.loginfo("Shutdown.")

    def toSegmentMsg(self,  lines, normals, color):

        segmentMsgList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2	    
            segment.normal.x = norm_x
            segment.normal.y = norm_y
            segmentMsgList.append(segment)
        return segmentMsgList

class Stats():
    def __init__(self):
        self.nresets = 0
        self.reset()

    def reset(self):
        self.nresets += 1
        self.t0 = time.time()
        self.nreceived = 0
        self.nskipped = 0
        self.nprocessed = 0

    def received(self):
        if self.nreceived == 0 and self.nresets == 1:
            rospy.loginfo('parking_line_detector_node received first image.')
        self.nreceived += 1

    def skipped(self):
        self.nskipped += 1

    def processed(self):
        if self.nprocessed == 0 and self.nresets == 1:
            rospy.loginfo('parking_line_detector_node processing first image.')

        self.nprocessed += 1

    def info(self):
        delta = time.time() - self.t0

        if self.nreceived:
            skipped_perc = (100.0 * self.nskipped / self.nreceived)
        else:
            skipped_perc = 0

        def fps(x):
            return '%.1f fps' % (x / delta)

        m = ('In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' %
             (delta, self.nreceived, fps(self.nreceived),
              self.nprocessed, fps(self.nprocessed),
              self.nskipped, fps(self.nskipped), skipped_perc))
        return m





if __name__ == '__main__':
    rospy.init_node('parking_line_detector',anonymous=False)
    parking_line_detector_node = ParkingLineDetectorNode()
    rospy.on_shutdown(parking_line_detector_node.onShutdown)
    rospy.spin()
