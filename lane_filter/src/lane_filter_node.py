#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from turtlebot3_auto_msgs.msg import SegmentList, Segment
from scipy.stats import multivariate_normal, entropy
from scipy.ndimage.filters import gaussian_filter
from math import floor, atan2, pi, cos, sin, sqrt
import time
from geometry_msgs.msg import Twist

class LaneFilterNode(object):
    """

Lane Filter Node

Author: Liam Paull

Inputs: SegmentList from line detector

Outputs: LanePose - the d (lateral displacement) and phi (relative angle)
of the car in the lane

For more info on algorithm and parameters please refer to the google doc:
 https://drive.google.com/open?id=0B49dGT7ubfmSX1k5ZVN1dEU4M2M

    """
    def __init__(self):
        self.node_name = "Lane Filter"
	self.active = True 
        self.white_x = 0
	self.white_y = 0
	self.white_length = 0
	self.white_degree = 0
	self.white_time = 0

	self.yellow_x = 0
	self.yellow_y = 0
	self.yellow_length = 0
	self.yellow_degree = 0
	self.yellow_time = 0

        # Subscribers
        self.sub = rospy.Subscriber("segment_list", SegmentList, self.processSegments, queue_size=1)

        # Publishers
	self.pub_car_cmd = rospy.Publisher('/cmd_vel_line', Twist, queue_size=5)

        #self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)    

    def processSegments(self,segment_list_msg):
        if not self.active:
            return
        t_start = rospy.get_time()

        for segment in segment_list_msg.segments:
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue

	    self.generateVote(segment)

    def generateVote(self,segment):

	if segment.color == segment.YELLOW:
	    if segment.pixels_normalized[0].x <= segment.pixels_normalized[1].x:   
		p1 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])
		p2 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
	    else:
		p1 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
		p2 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])

	elif segment.color == segment.WHITE:
	    img_w = 160
	    img_h = 120 - 30	    
	    segment.pixels_normalized[0].x = 160 - segment.pixels_normalized[0].x
	    segment.pixels_normalized[1].x = 160 - segment.pixels_normalized[1].x	
	    if segment.pixels_normalized[0].x >= segment.pixels_normalized[1].x:   
		p1 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])
		p2 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
	    else:
		p1 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
		p2 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])
	else:
	    return 0,0,0




	delta_p = p2-p1
	if delta_p[0] == 0 :	
	    rospy.loginfo( "delta_p[0] isinfinite happen" )
	    return 
	inclination = abs(delta_p[1] / delta_p[0])	
	extend_y = inclination*p1[0] + p1[1]
	if inclination == 0 :	
	    rospy.loginfo( "inclination isinfinite happen" )
	    return 

	extend_x = extend_y/inclination
	#extend_length = np.linalg.norm([extend_x,0]-[0,extend_y])
	extend_length = extend_x*extend_x + extend_y*extend_y	
	extend_degree =  np.arctan([extend_y,extend_x])
	cur_time = rospy.get_time()

	if extend_x<0 or extend_y<0 :
	    self.onVehicleStop() 
	    rospy.loginfo( "Unexpected situation. So Robot Stop1 " )

	beta = 0.85	
	angular = 0.0
	velocity = 0.08


	if segment.color == segment.WHITE: # right lane is white # IS operating yellow disappear
	    self.white_x = beta*extend_x + (1 - beta)*self.white_x
	    self.white_y = beta*extend_y + (1 - beta)*self.white_y
	    self.white_length = beta*extend_length + (1 - beta)*self.white_length
	    self.white_degree = beta*extend_degree + (1 - beta)*self.white_degree
	    self.white_time = cur_time

	elif segment.color == segment.YELLOW: # right lane is white # IS operating yellow disappear
	    self.yellow_x = beta*extend_x + (1 - beta)*self.yellow_x
	    self.yellow_y = beta*extend_y + (1 - beta)*self.yellow_y
	    self.yellow_length = beta*extend_length + (1 - beta)*self.yellow_length
	    self.yellow_degree = beta*extend_degree + (1 - beta)*self.yellow_degree
	    self.yellow_time = cur_time


    	if (self.white_time - self.yellow_time)> 0.3 :           

	    if 5500 < self.white_length and self.white_length <= 6500 :
    	        angular = 0.19
		#rospy.loginfo( "test1")
	    elif 6500 < self.white_length and self.white_length <= 7500 :
                angular = 0.29
		#rospy.loginfo( "test2")
	    elif 7500 < self.white_length :
    	        angular = 0.39
		#rospy.loginfo( "test3")
	    else :
	        self.onVehicleStop() 
	        rospy.loginfo( "Unexpected situation. So Robot Stop2 " )
                # vehicle stop , and 360 rotation , search line ??

	    rospy.loginfo( 'only white %.0f  , arg %.1f , time_di %.2f',self.white_length , angular, ( self.white_time - self.yellow_time ) )

	else :

	    if self.yellow_length <= 1500 :
	    	angular= 0.39
	    elif 1500 < self.yellow_length and self.yellow_length <= 2500 :
	    	angular =  0.29
	    elif 2500 < self.yellow_length and self.yellow_length <= 3500 :
	    	angular =  0.19
	    elif 3500 < self.yellow_length and self.yellow_length <= 6500 :
		angular =  0
	    elif 6500 < self.yellow_length and self.yellow_length <= 7500 :
	        angular = -0.19
	    elif 7500 < self.yellow_length and self.yellow_length <= 8500 :
	        angular = -0.29
	    elif 8500 < self.yellow_length and self.yellow_length <= 15000 :
	        angular = -0.39
 	    elif 15000 < self.yellow_length :
	        angular = -0.39
		velocity = 0.05

	    rospy.loginfo( ' yellow_length %.0f , arg %.1f ',self.yellow_length , angular )

	twist = Twist() 
	twist.linear.x = velocity ; twist.linear.y = 0.0 ; twist.linear.z = 0.0
	twist.angular.x = 0.0 ; twist.angular.y = 0.0 ; twist.angular.z = angular
	self.pub_car_cmd.publish(twist)
    	#rospy.loginfo('publish velocity %.2f , argural %.1f',velocity, angular )
    	return

    def onVehicleStop(self):
	twist = Twist() 
	twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
	self.pub_car_cmd.publish(twist)

    def onShutdown(self):
	self.onVehicleStop()
        rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
