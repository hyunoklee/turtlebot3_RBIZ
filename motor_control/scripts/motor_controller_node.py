#!/usr/bin/env python
import rospy
import numpy as np
import math
from turtlebot3_auto_msgs.msg import  Twist2DStamped, LanePose, ImgSignals
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from time import sleep

class motor_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.cmd_vel_line_reading = None
	self.signal_reading = None
	self.odom_reading = None

        self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED
	self.led_state = None
	self.bar_state = None
	self.pub_counter = 0

        # Setup parameters
        #self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
	self.pub_con_state = rospy.Publisher('/con_state', String,queue_size=1)

        # Subscriptions
        # self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
	self.sub_cmd_vel_line_reading = rospy.Subscriber("/cmd_vel_line", Twist, self.cbCmdVelLine, queue_size=1)
	self.sub_signal_reading = rospy.Subscriber("/signals", ImgSignals, self.cbSignal, queue_size=1)
	self.sub_state_change = rospy.Subscriber("/con_state_change", String, self.cbConStateChange, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        #self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))


    def vehicleStop(self):
	twist = Twist() 
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
	self.publishCmd(twist) 
   
    def custom_shutdown(self):        
        # Stop listening
        self.sub_odom_reading.unregister()
	self.sub_signal_reading.unregister()
        # Send stop command
        self.vehicleStop()        
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self,twist):        
        self.pub_car_cmd.publish(twist)


    def cbCmdVelLine(self,cmd_vel_line_msg):
        self.cmd_vel_line_reading = cmd_vel_line_msg 
        twist = Twist() 
        twist.linear.x = self.cmd_vel_line_reading.linear.x
        twist.angular.z = self.cmd_vel_line_reading.angular.z
        if self.con_state == "LINE" :
            self.publishCmd(twist)

    def cbOdometry(self,odom_msg):
        self.odom_reading = odom_msg
        rospy.loginfo('x: %f , y:%f , w:%f ',self.odom_reading.pose.pose.position.x ,  self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w )

    def cbSignal(self,signal_msg):
	self.signal_reading = signal_msg
	rospy.loginfo('Signal: %s , LED : %s ',self.signal_reading.SIGNAL , self.signal_reading.LED )
	if self.signal_reading.SIGNAL == "BAR" :
	    self.con_state = "BAR"
	    self.bar_state = self.signal_reading.BAR #VER, HOR
	elif self.signal_reading.SIGNAL == "PARKIGN" :
	    self.con_state = "PARKIGN"	    
	elif self.signal_reading.SIGNAL == "TUNNEL" :
	    self.con_state = "TUNNEL"
	elif self.signal_reading.SIGNAL == "LED" :
	    self.con_state = "LED"
	    self.led_state = self.signal_reading.LED #RED,YELLOW,GREEN
	self.pub_con_state.publish(self.con_state)
    	#sleep(0.1)

	if self.con_state == "LED":
	    controlByLed(self.led_state)
	elif self.con_state == "BAR":
	    controlByLed(self.bar_state)

    def cbConStateChange(self,con_state_change):
	if con_state_change == "TUNNEL_OUT" :
	    self.con_state = "LINE"
	elif self.signal_reading.SIGNAL == "PARKIGN_OUT" :
	    self.con_state = "LINE"
	self.pub_con_state.publish(self.con_state)

    def controlByLed(self,led_state):
	if led_state == "LED" or led_state == "YELLOW" :
	    vehicleStop()
	elif led_state == "GREEN" :
	    self.con_state = "LINE"
	self.pub_con_state.publish(self.con_state)

    def controlByBar(self,bar_state):
	if bar_state == "HOR" :
	    vehicleStop()
	elif led_state == "VER" :
	    self.con_state = "LINE"
	self.pub_con_state.publish(self.con_state)

if __name__ == "__main__":
    rospy.init_node("motor_controller",anonymous=False)
    motor_control_node = motor_controller()
    rospy.spin()
