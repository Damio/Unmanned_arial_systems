#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import csv

from numpy import genfromtxt
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, PointStamped
from std_msgs.msg import String

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator


class ROSDesiredPositionGenerator(object):
    	"""ROS interface for publishing desired positions."""
    	# write code here for desired position trajectory generator
	def __init__(self):
		# Publishes the desired position 
		self.pub_desired_pos = rospy.Publisher('desired_pos/',TransformStamped, queue_size = 300)
		self.sub_center_cir = rospy.Subscriber('center', TransformStamped, self.center_location)
		self.sub_key_input = rospy.Subscriber('key',String , self.key_input)
		self.sub_new_height = rospy.Subscriber('des_z', PointStamped, self.hoop_height)

		self.desired_pos = TransformStamped()	
		self.key_in = None
		self.circle_center = TransformStamped()

		# Load desired position
		self.des_x = []
		self.des_y = []
		self.des_yaw = []
		self.desired_z = 1.2
		self.count = 0
		self.height = 1.2
		self.circle_center = TransformStamped()

		# Ensure that the name of the file is changed for the survey path versus the trajectory path
		directory = '/home/isura/Desktop/'
		for d in csv.DictReader(open(directory+'desired_position.csv'), delimiter=','):
			self.des_x.append(float(d['x']))
			self.des_y.append(float(d['y']))
			self.des_yaw.append(float(d['yaw']))
		
		self.max_count = len(self.des_x)
		
		# Reading and storing data from csv file
		# Make sure to change directory for final project
		self.traj_data = genfromtxt('/home/isura/Desktop/path6.csv', delimiter = ',')

	def key_input(self, key_in):
		self.key_in = key_in.data

	def center_location(self, center): 
		self.circle_center = center

	def hoop_height(self, new_height):
		self.desired_z = new_height
		#target = np.array([self.circle_center.transform.translation.x,
		#                      self.circle_center.transform.translation.y,
		#                      self.circle_center.transform.translation.z])
		#center_y = target[0]
		#center_z = target[1]
		#center_radius = target[2]
		
		#H = ((desired_x**2)+(desired_y)**2)**0.5
		#Dia = 2*H*np.tan((92/2)*(np.pi/180))
		#Dia_pix = np.power(np.power(640,2)+np.power(360,2),0.5)

		#desired_z = (180-center_z)*(Dia/Dia_pix)+self.desired_z
		#return self.desired_z

	def update_position(self):
		if self.key_in == "V":
			if self.count < self.max_count:
				self.survey_path(self.count)
				rospy.sleep(0.005)
				self.count=self.count+1

		elif self.key_in == "B":
			if self.count < len(self.traj_data):
				self.trajectory_path(self.count)
				rospy.sleep(0.05)
				self.count = self.count + 1

			elif self.count > len(self.traj_data):
				self.count = self.count 
				self.trajectory_path(self.count)
				rospy.sleep(0.35)

		elif self.key_in == "N":
			# Ensure this is changed for the trajectory or survey whichever one is required
			self.count = self.count
			self.survey_path(self.count)
			rospy.sleep(0.005)

                elif self.key_in == "J":
                	self.count = self.count
                    	self.change_alt(self.count, self.height)
                    	rospy.sleep(0.005)
                    	self.height = self.height + 0.001

                elif self.key_in == "M":
                    	self.count = self.count
                    	self.change_alt(self.count, self.height)
                    	rospy.sleep(0.005)
                    	self.height = self.height - 0.001
		else: 
			self.hover_path()

    	def survey_path(self, count):
		des_x=self.des_x[count]
		des_y=self.des_y[count]
		des_yaw = self.des_yaw[count]
		
		self.desired_pos.transform.translation.x = des_x
                self.desired_pos.transform.translation.y = des_y
                self.desired_pos.transform.translation.z = 1.2
		self.desired_pos.transform.rotation.z = 0
		self.pub_desired_pos.publish(self.desired_pos)

	def trajectory_path(self,count):
		des_x = self.traj_data[count, 0]
		des_y=self.traj_data[count, 1]
		#yaw = self.traj_data[count, 2]
		
		self.desired_pos.transform.translation.x = des_x
                self.desired_pos.transform.translation.y = des_y
                self.desired_pos.transform.translation.z = 1.2
		self.desired_pos.transform.rotation.z = 0
		self.pub_desired_pos.publish(self.desired_pos)
       
	def change_alt(self, count, the_height):
                des_x=self.des_x[count]
                des_y=self.des_y[count]
                #des_yaw = self.des_yaw[count]
                
                self.desired_pos.transform.translation.x = des_x
                self.desired_pos.transform.translation.y = des_y
                self.desired_pos.transform.translation.z = the_height
                self.desired_pos.transform.rotation.z = 0
                self.pub_desired_pos.publish(self.desired_pos)

	def hover_path(self):
		# Remember to change hover position to starting point 
		self.desired_pos.transform.translation.x = -1.9
		self.desired_pos.transform.translation.y = -1.9
		self.desired_pos.transform.translation.z = 1.2
	
		# Publishing information to desired_pos topic 
		self.pub_desired_pos.publish(self.desired_pos)

				
if __name__ == '__main__':
    rospy.init_node('des_position')
    prog = ROSDesiredPositionGenerator()
    while not rospy.is_shutdown():
	prog.update_position()
    rospy.spin()
