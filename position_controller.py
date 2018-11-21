#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist


class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""
    # write code here for position controller
    def __init__(self):
        

        #define normalized thrust parameters
        self.g = 9.81

        #define damping and omega parameters
        #self.rise_time_x = 0.60
        self.x_damping = 0.93	#0.89	#0.84	
        self.x_omega = 0.92	#0.9	#self.rise_time_x / 1.8		

        #self.rise_time_y = 0.45
        self.y_damping = 0.90	#0.90	#0.92
        self.y_omega = 1.0	#self.rise_time_y / 1.8	
 
        self.rise_time_z = 0.80
        self.rise_time_w = 2.2

	# Yaw controller constants 
	self.yaw_damping = 0.85
	self.yaw_omega = 0.3
        

        #define desired parameters, values from the trajectory planner
        #change this one you get the planner
        self.desired_roll = 0
        self.desired_pitch = 0
        self.desired_yaw = 0

        

        # Desired Velocities 
        self.desired_x_vel = 0 
        self.desired_y_vel = 0
        self.desired_z_vel = 0


        #define old values
        #co-ordinates   **********
        self.old_x = 0
        self.old_y = 0
        self.old_z = 0

        #Angles
        self.old_roll = 0
        self.old_pitch= 0
        self.old_yaw = 0    

        self.old_yaw_rate = 0
        self.old_climb_rate = 0


        #velocities
        self.old_x_vel = 0
        self.old_y_vel = 0
        self.old_z_vel = 0
	self.old_desired_x = 0 
	self.old_desired_y = 0    


    def positioncontroller(self, desired_x, desired_y, desired_z, actual_x, actual_y, actual_z, roll, pitch, desired_yaw, yaw, z_vel, dt):

        #calculate the normalized thrust from measurements(z_acc, rho and phi)
        z_acc = (z_vel - self.old_z_vel) / dt
     
        n_thrust = (z_acc + self.g)/(np.cos(pitch) *np.cos(roll))

        #update z_vel
        self.old_z_vel = z_vel
        
        #Determine desired x_acc
        #error in x position
        err_x = desired_x - actual_x
	
        #error in x velocity
        desired_x_vel = (desired_x - self.old_desired_x)/dt
        x_vel = (actual_x - self.old_x)/dt
        err_x_vel = (desired_x_vel - x_vel)   
	
        #update x parameters
        self.old_x = actual_x 
	self.old_desired_x = desired_x 
        
        
	#calculate acceleration
        x_acc = ((2.0 * self.x_damping * self.x_omega) * err_x_vel) + (((self.x_omega)**2)* err_x)

        #Determine desired y_acc
        #error in y position
        err_y = desired_y - actual_y

        #error in y velocity
        desired_y_vel = (desired_y - self.old_desired_y)/dt
        y_vel = (actual_y - self.old_y)/dt
        err_y_vel = (desired_y_vel - y_vel)   
	
        #update y parameters     ******
        self.old_y = actual_y
	self.old_desired_y = desired_y
	
	#print(err_x, err_y)
        #calculate acceleration
        y_acc = ((2.0 * self.y_damping * self.y_omega) * err_y_vel) + (((self.y_omega)**2)* err_y)

        #roll command
        roll_command = np.arcsin(-y_acc / n_thrust)
	if roll_command< -1:
		roll_command = -1
	elif roll_command > 1:
		roll_command = 1

        #pitch command
        pitch_command = np.arcsin(x_acc/(n_thrust * np.cos(roll_command)))
	if pitch_command < -1:
		pitch_command = -1
	elif pitch_command > 1:
		pitch_command = 1

	roll_command = (roll_command * np.cos(yaw)) + (pitch_command * np.sin(yaw))
	
	pitch_command = -(roll_command * np.sin(yaw)) + (pitch_command *np.cos(yaw))

        #yaw rate command
	#if yaw > np.pi:
	#	yaw = yaw + np.pi
	#	print('here')
	#elif yaw < -(np.pi/2):
	#	yaw = yaw + (2*np.pi)
	#	print('here2')
	
	#else: 
	#	yaw = yaw 

        tau_w = self.rise_time_w / 2.2
        yaw_rate_command = (self.desired_yaw - yaw) / tau_w	
	############################################################
	# Yaw Controller 
	#yaw_rate_command = (desired_yaw - yaw)/tau_w
	
	# Update Yaw parameters 
	#self.old_desired_yaw = desired_yaw 
	#self.old_yaw = yaw 
	
	############################################################   

        #climb rate command
        tau_z = self.rise_time_z / 2.2
        climb_rate_command = (desired_z - actual_z) / tau_z
	# Returning the command angles 
	command_angles=[roll_command, pitch_command, yaw_rate_command, climb_rate_command]
        #for exporting the values
        return command_angles


  


  

