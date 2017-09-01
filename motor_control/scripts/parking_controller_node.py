#!/usr/bin/env python
import rospy
import numpy as np
import math
from turtlebot3_auto_msgs.msg import  Twist2DStamped, LanePose, ImgSignals
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from time import sleep

class parking_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
	# parking
	self.ParkingState = "WAIT" # START, START_IN_ROTATION , START_IN_GO, START_OUT_GO, START_OUT_ROTATION , STOP 
	self.ParkingStartP = [0,0,0] # x, y , ang 
	self.ParkingStopP = [0,0,0] # x, y , ang  
	self.ParkingTargetP = [0.3,10] # length , ang  

        # Setup parameters
        #self.setGains()

        # Publishers
        self.pub_cmd_vel_parking = rospy.Publisher("/cmd_vel_parking", Twist, queue_size=1)
	self.pub_con_state_change = rospy.Publisher("/con_state_change", String, queue_size=1)

       # Subscribers
        self.sub_con_state_reading = rospy.Subscriber("/con_state", String, self.cbConState, queue_size=1)
	self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        #self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def vehicleStop(self):
	twist = Twist() 
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
	self.pub_cmd_vel_parking(twist)  
   
    def custom_shutdown(self):        
        # Stop listening
        self.sub_odom_reading.unregister()
	self.sub_con_state_reading.unregister()
        # Send stop command
        self.vehicleStop()        
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def cbConState(self, con_state_msg ):	 
         if self.ParkingState == "WAIT" and con_state_msg == "PARKING" :
	     self.ParkingState = "START"

    def cbOdometry(self,odom_msg):
        self.odom_reading = odom_msg
        OdomP1 = [ self.odom_reading.pose.pose.position.x , self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w ]
        rospy.loginfo('x: %f , y:%f , w:%f ',self.odom_reading.pose.pose.position.x ,  self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w )
	if self.ParkingState == "START" :
	    self.vehicleStop()
	    sleep(1)
     	    self.ParkingStartP = OdomP1
	    self.ParkingState = "START_IN_ROTATION"

	if self.ParkingState == "START_IN_ROTATION" :
	    if OdomP1[2] == self.ParkingTargetP[1] :
		self.vehicleStop()
		sleep(1)
		self.ParkingState = "START_IN_GO"
	    twist = Twist()
	    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.01;
	    self.pub_cmd_vel_parking(twist)
 
	if self.ParkingState == "START_IN_GO" :
	    if np.linalg.norm(self.ParkingStartP-OdomP1) == self.ParkingTargetP[0] :
		self.vehicleStop()
		sleep(1)
		self.ParkingState = "START_OUT_GO"
	    twist = Twist()
	    twist.linear.x = 0.1; twist.linear.y = 0; twist.linear.z = 0;
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
	    self.pub_cmd_vel_parking(twist)

	if self.ParkingState == "START_OUT_GO" :
	    if np.linalg.norm(self.ParkingStopP - OdomP1) == self.ParkingTargetP[0] :
		self.vehicleStop()
		sleep(1)
		self.ParkingState = "START_OUT_ROTATION"
	    twist = Twist()
	    twist.linear.x = -0.1; twist.linear.y = 0; twist.linear.z = 0;
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
	    self.pub_cmd_vel_parking(twist)

	if self.ParkingState == "START_OUT_ROTATION" :
	    if OdomP1[2] == self.ParkingStartP[1] :
		self.vehicleStop()
		sleep(1)
		self.ParkingState = "STOP"
	    twist = Twist()
	    twist.linear.x =  0; twist.linear.y = 0; twist.linear.z = 0;
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -0.01;
	    self.pub_cmd_vel_parking(twist)

	if self.ParkingState == "STOP" :
	    self.pub_cmd_vel_parking("PARKING_OUT")

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


if __name__ == "__main__":
    rospy.init_node("parking_controller",anonymous=False)
    parking_control_node = parking_controller()
    rospy.spin()
