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
        #self.beliefRV=np.empty(self.d.shape)
        #self.initializeBelief()
        
        self.prev_extend_x = 50
	self.prev_extend_y = 50
	self.prev_extend_d = 50
	self.prev_time = None
	self.prevtwist = Twist()

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
	    if segment.pixels_normalized[0].x >= segment.pixels_normalized[1].x:   
	        p1 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])
                p2 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
	    else:
		p1 = np.array([segment.pixels_normalized[1].x, segment.pixels_normalized[1].y])
                p2 = np.array([segment.pixels_normalized[0].x, segment.pixels_normalized[0].y])	
	    return 0,0,0

	else:
	    return 0,0,0

	delta_p = p2-p1
	inclination = abs(delta_p[1] / delta_p[0])
	extend_y = inclination*p1[0] + p1[1]
	extend_x = extend_y/inclination

        beta = 0.85
        a_extend_x = beta*extend_x + (1 - beta)*self.prev_extend_x
	a_extend_y = beta*extend_y + (1 - beta)*self.prev_extend_y

	extend_d = a_extend_x*a_extend_x + a_extend_y*a_extend_y
	a_extend_d = beta*extend_d + (1 - beta)*self.prev_extend_d
 	#rospy.loginfo('generate %f %f %f %f', segment.pixels_normalized[0].x,segment.pixels_normalized[0].y, segment.pixels_normalized[1].x,segment.pixels_normalized[1].y)
	eculidean = np.linalg.norm(p2-p1)
        
        if segment.color == segment.WHITE: # right lane is white
	    rospy.loginfo('                                   WHITE x %.1f y %.1f x %.1f y %.1f', p1[0],p1[1],p2[0],p2[1] )
	    rospy.loginfo('                                   WHITE d %.1f d %.1f _ %.1f', d1,d2, eculidean )	
	    rospy.loginfo('                                   WHITE d_i %.1f l_i %.1f phi_i %.1f', d_i,l_i,phi_i )	    	
            if(p1[0] > p2[0]): # right edge of white lane
                d_i = d_i - self.linewidth_white
            else: # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth/2

        elif segment.color == segment.YELLOW: # left lane is yellow
	    #rospy.loginfo('YELLOW x %.1f y %.1f x %.1f y %.1f', p1[0],p1[1],p2[0],p2[1] )
	    #rospy.loginfo('YELLOW d %.1f d %.1f _ %.1f', d1,d2, eculidean )
	    #rospy.loginfo('YELLOW d_i %.1f l_i %.1f phi_i %.1f', d_i,l_i,phi_i )
	    rospy.loginfo('YELLOW ex %.1f(%.1f) ey %.1f(%.1f) _ distance %.1f(%.1f) ', extend_x,a_extend_x,extend_y,a_extend_y, extend_d,a_extend_d )

	self.prev_extend_x = a_extend_x
	self.prev_extend_y = a_extend_y
	self.prev_extend_d = a_extend_d	
	
	twist = Twist() 
	twist.linear.x = 0.05; twist.linear.y = 0.0; twist.linear.z = 0.0
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

	if a_extend_d <= 1500 :
	    twist.angular.z = -0.3
	elif 1500 < a_extend_d and a_extend_d <= 2500 :
	    twist.angular.z =  0.2
	elif 2500 < a_extend_d and a_extend_d <= 3500 :
	    twist.angular.z =  0.1
	#############################################################3	
	#elif 5500 < a_extend_d and a_extend_d <= 6500 :
	#    twist.angular.z = -0.1
	elif 6500 < a_extend_d and a_extend_d <= 7500 :
	    twist.angular.z = -0.2
	elif 7500 < a_extend_d and a_extend_d <= 8500 :
	    twist.angular.z = -0.3
	elif 8500 < a_extend_d :#and a_extend_d <= 9500 :
	    twist.angular.z = -0.5
	#elif 9500 < a_extend_d :
	 #   twist.angular.z = -0.5


	self.pub_car_cmd.publish(twist)
        rospy.loginfo('publish twist %.2f ,%.1f ,%.1f _ %.1f ,%.1f ,%.1f',twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z)
	
	self.prevtwist = twist

	#rospy.loginfo('generateVote finish')
        return

    def onShutdown(self):
	twist = Twist() 
	twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
	self.pub_car_cmd.publish(twist)
	#self.pub_car_cmd.publish(twist)
        rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
