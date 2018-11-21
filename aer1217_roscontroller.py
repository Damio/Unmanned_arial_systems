#!/usr/bin/env python2

"""ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import String
from position_controller import PositionController
from aer1217_ardrone_simulator.msg import GazeboState

class ROSControllerNode(object):
    	"""ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
	
	def __init__(self):
		self.time_time = rospy.get_time()
		"""Initialize the ROSControllerNode"""
		# Subscribe to (input) :
		self.sub_des_pos = rospy.Subscriber('desired_pos/',TransformStamped, self.calc_pos)
		self.sub_vicon_data = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre',TransformStamped, self.positioncontroller)

		# Publish to (output) :
		# Change cmd_vel_RHC to cmd_vel
		self.pub_cmd_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 300)

		# Initialize necessary variables
		self.des_pos = TransformStamped()
		self.old_time = 0
		 
		# Initialize messages for publishing :
		self.cmd_vel_msg = Twist()

		# Creating an instance for the postition controller and the desired postition functions 
		self.position_controller = PositionController()
		
	def calc_pos(self, calc_pos):
		self.des_pos = calc_pos

	def positioncontroller(self, current_position):
 
		# Determine time step and adding a wait time such that it allows the drone to stabilize before moving
		current_time = rospy.get_time() - self.time_time
		dt = current_time - self.old_time

		# Extracting desired position variables from the des_pos array
		desired_x = self.des_pos.transform.translation.x
		desired_y = self.des_pos.transform.translation.y
		desired_z = self.des_pos.transform.translation.z
		desired_yaw = self.des_pos.transform.rotation.z
	
		# Saving the current vicon data in variable 
		self.current_position = current_position
                
                # Setting the current location to variables 
		quaternion = np.array([self.current_position.transform.rotation.x,
		                      self.current_position.transform.rotation.y,
		                      self.current_position.transform.rotation.z,
		                      self.current_position.transform.rotation.w])                
		
		# Getting the current position
		actual_x = self.current_position.transform.translation.x
        	actual_y = self.current_position.transform.translation.y
        	actual_z = self.current_position.transform.translation.z
	
		# Getting the euler engles
		euler = euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		z_vel = quaternion[3]		
		
		# Calling the position controller to calculate the command angles 
		command_angles = self.position_controller.positioncontroller(desired_x, desired_y, desired_z, actual_x, actual_y, actual_z, roll, pitch, desired_yaw, yaw, z_vel, dt)
		
		# Extracting command angles: roll, pitch, yaw and climb rates from the variable 
		self.command_roll = command_angles[0]
		self.command_pitch = command_angles[1]
		self.command_yaw_rate = command_angles[2]
		self.command_climb_rate = command_angles[3]
		
		# Writing angeles into command vel 
		self.cmd_vel_msg.linear.x = self.command_roll
		self.cmd_vel_msg.linear.y = self.command_pitch
		self.cmd_vel_msg.angular.z = self.command_yaw_rate 
		self.cmd_vel_msg.linear.z = self.command_climb_rate  
	
		# Publishing values from command vel 
		self.pub_cmd_vel.publish(self.cmd_vel_msg)

		# Update time 
		self.old_time = current_time

if __name__ == '__main__':
    rospy.init_node('ros_controller')
    ROSControllerNode()
    rospy.spin()
