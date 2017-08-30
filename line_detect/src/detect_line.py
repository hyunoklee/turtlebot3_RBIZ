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

class LineDetectorNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"

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
        self.pub_lines = rospy.Publisher("segment_list", SegmentList, queue_size=1)
	self.pub_image_origin = rospy.Publisher("image_with_origin", Image, queue_size=1)
	self.pub_image_cal = rospy.Publisher("image_with_cal", Image, queue_size=1)
        self.pub_image_with_line = rospy.Publisher("image_with_lines", Image, queue_size=1)

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
            self.pub_edge = rospy.Publisher("~edge", Image, queue_size=1)
            self.pub_colorSegment = rospy.Publisher("~colorSegment", Image, queue_size=1)

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
            self.intermittent_log(self.stats.info())
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
        self.pub_image_origin.publish(image_cv_origin_p)

	#calibration proc && image publish

	K_undistort = np.array(self.c['camera_matrix'])
	img_und = cv2.undistort(image_cv, np.array(self.c['camera_matrix']), np.array(self.c['dist_coefs']),
                                newCameraMatrix=K_undistort)
	img_und_p = self.bridge.cv2_to_imgmsg(img_und, "bgr8")
        self.pub_image_cal.publish(img_und_p)

        # Resize and crop image and rotation
        hei_original, wid_original = img_und.shape[0:2]

        if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            img_und = cv2.resize(img_und, (self.image_size[1], self.image_size[0]),
                                   interpolation=cv2.INTER_NEAREST)
	
	rotation_center = (self.image_size[1]/2, self.image_size[0]/2)
	rotation_mat = cv2.getRotationMatrix2D(rotation_center, 180, 1.0)
	img_und = cv2.warpAffine(img_und, rotation_mat, (self.image_size[1], self.image_size[0]))

        img_und = img_und[self.top_cutoff:,:,:]

        tk.completed('resized')

        # apply color correction: AntiInstagram
        # image_cv_corr = image_cv
        #image_cv_corr = self.ai.applyTransform(img_und)
        image_cv_corr = cv2.convertScaleAbs(img_und)

        tk.completed('corrected')

        # Set the image to be detected
        self.detector.setImage(image_cv_corr)
	hei2_original, wid2_original = image_cv_corr.shape[0:2]
	

        # Detect lines and normals

        white = self.detector.detectLines('white')
        yellow = self.detector.detectLines('yellow')
        red = self.detector.detectLines('red')
        #rospy.loginfo('after %.1f %.1f  ',hei2_original , wid2_original)
	#rospy.loginfo('after %d %d  (%.1f %.1f) (%.1f %.1f)',hei2_original , wid2_original, white.lines.x1)

        tk.completed('detected')

        # SegmentList constructor
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp

        # Convert to normalized pixel coordinates, and add segments to segmentList
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.image_size[1], 1./self.image_size[0], 1./self.image_size[1], 1./self.image_size[0]))
        if len(white.lines) > 0:
            #lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
	    lines_normalized_white = white.lines
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_white, white.normals, Segment.WHITE))
        if len(yellow.lines) > 0:
            #lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
	    lines_normalized_yellow = yellow.lines
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_yellow, yellow.normals, Segment.YELLOW))
        if len(red.lines) > 0:
            #lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
	    lines_normalized_red = red.lines
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_red, red.normals, Segment.RED))

        # Publish segmentList
        self.pub_lines.publish(segmentList)
	#self.loginfo('send segmentList publish finish')


        # VISUALIZATION only below

        if self.verbose:

            # Draw lines and normals
            image_with_lines = np.copy(image_cv_corr)
            drawLines(image_with_lines, white.lines, (0, 0, 0))
            drawLines(image_with_lines, yellow.lines, (255, 0, 0))
            drawLines(image_with_lines, red.lines, (0, 255, 0))

            tk.completed('drawn')

            # Publish the frame with lines
            image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
            image_msg_out.header.stamp = image_msg.header.stamp
            self.pub_image_with_line.publish(image_msg_out)

            tk.completed('pub_image')

#         if self.verbose:
            colorSegment = color_segment(white.area, red.area, yellow.area)
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
            rospy.loginfo('line_detector_node received first image.')
        self.nreceived += 1

    def skipped(self):
        self.nskipped += 1

    def processed(self):
        if self.nprocessed == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node processing first image.')

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
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
    rospy.spin()
