#!/usr/bin/env python
import rospy
import numpy as np
import math
from turtlebot3_auto_msgs.msg import  Twist2DStamped, LanePose, ImgSignals
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class motor_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.cmd_vel_line_reading = None
	self.signal_reading = None
	self.odom_reading = None

        self.pub_counter = 0

        # Setup parameters
        #self.setGains()

        # Publicaiton
        # self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_car_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Subscriptions
        # self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
	self.sub_cmd_vel_line_reading = rospy.Subscriber("/cmd_vel_line", Twist, self.cbCmdVelLine, queue_size=1)
	self.sub_signal_reading = rospy.Subscriber("/signals", ImgSignals, self.cbSignal, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        #self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        
        # Stop listening
        self.sub_odom_reading.unregister()
	self.sub_signal_reading.unregister()

        # Send stop command
        #car_control_msg = Twist2DStamped()
        twist = Twist()
        #car_control_msg.v = 0.0		
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        #car_control_msg.omega = 0.0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.publishCmd(twist)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self,twist):        
        self.pub_car_cmd.publish(twist)

    def cbCmdVelLine(self,cmd_vel_line_msg):
        self.cmd_vel_line_reading = cmd_vel_line_msg 
        twist = Twist() 
	twist.linear.x = self.cmd_vel_line_reading.linear.x
        twist.angular.z = self.cmd_vel_line_reading.angular.z    
        self.publishCmd(twist)

    def cbOdometry(self,odom_msg):
        self.odom_reading = odom_msg
	rospy.loginfo('x: %f , y:%f , w:%f ',self.odom_reading.pose.pose.position.x , self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w )

    def cbSignal(self,signal_msg):
	self.signal_reading = signal_msg
	rospy.loginfo('Signal: %s , LED : %s ',self.signal_reading.SIGNAL , self.signal_reading.LED )

if __name__ == "__main__":
    rospy.init_node("motor_controller",anonymous=False)
    motor_control_node = motor_controller()
    rospy.spin()
