#!/usr/bin/env python

# This work is based on ardrone_tutorial https://github.com/mikehamer/ardrone_tutorials_getting_started

import roslib
import rospy
# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from gazebo_msgs.srv import GetModelState    # for receiving Pose of simulated ardrone

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/drone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand1 = rospy.Publisher('/drone1/cmd_vel',Twist)
                self.pubCommand2 = rospy.Publisher('/drone2/cmd_vel',Twist)
		# Setup regular publishing of control packets
		self.command1 = Twist()
		self.commandTimer1 = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand1)
		self.command2 = Twist()
		self.commandTimer2 = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand2)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
         
        def drone1(self):
                try:
                    subPose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    drone1_Pose=subPose("drone1", "link")
                    #SetCommand(0, Ardrone_Pose.pose.position.x, 0, 0)
                    return drone1_Pose.pose.position.x, drone1_Pose.pose.position.y, drone1_Pose.pose.position.z, drone1_Pose.pose.orientation.x, drone1_Pose.pose.orientation.y, drone1_Pose.pose.orientation.z, drone1_Pose.pose.orientation.w
                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        def drone2(self):
                try:
                    subPose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    drone2_Pose=subPose("drone2", "link")
                    #SetCommand(0, Ardrone_Pose.pose.position.x, 0, 0)
                    return drone2_Pose.pose.position.x, drone2_Pose.pose.position.y, drone2_Pose.pose.position.z, drone2_Pose.pose.orientation.x, drone2_Pose.pose.orientation.y, drone2_Pose.pose.orientation.z, drone2_Pose.pose.orientation.w
                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))
                

	def SetCommand1(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command1.linear.x  = pitch
		self.command1.linear.y  = roll
		self.command1.linear.z  = z_velocity
		self.command1.angular.z = yaw_velocity

	def SetCommand2(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command2.linear.x  = pitch
		self.command2.linear.y  = roll
		self.command2.linear.z  = z_velocity
		self.command2.angular.z = yaw_velocity

	def SendCommand1(self,event):
	        self.pubCommand1.publish(self.command1)

	def SendCommand2(self,event):
	        self.pubCommand2.publish(self.command2)
