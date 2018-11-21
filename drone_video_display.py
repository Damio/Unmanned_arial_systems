#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib
import rospy
import cv2
from geometry_msgs.msg import TransformStamped, Twist, PointStamped
from std_msgs.msg import String

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from ardrone_autonomy.srv import *

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# 2017-03-22 Import libraries from OpenCV
# OpenCV Bridge http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf.transformations import euler_from_quaternion

# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata,queue_size=1) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage,queue_size=1)
		
		#########################################################################################################
		# Added Publishers and subscribers for final project 
		# Subscribe to drone position
		self.subLocation = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', TransformStamped, self.location) 

		# Publish Target Location
		self.pub_image_pos = rospy.Publisher('image_pos', TransformStamped, queue_size = 300)

		# Publish Circle Coordinates (Old will remove)
		#self.pub_center_cir = rospy.Publisher('center', TransformStamped, queue_size = 300) 

		# Publish desired z location based on calculations
		self.pub_des_z = rospy.Publisher('des_z', PointStamped, queue_size = 300)

		# Added initation variables for final project 
		self.actual_location = TransformStamped()
		self.image_pos = TransformStamped()
		#self.center = TransformStamped()
		self.new_z = PointStamped()
		
		# Initiation variables for desired_z and position tracker
		self.position_tracker = [-1.5, -1.5]
		self.num = 001
		self.num2 = 001
		self.desired_z = 1.25
		self.des_z = 1.25
		

		# Timer 
		self.timer = rospy.Time.now()

		# Setting colorbounds in the form of (lower[B,G,R], upper[B,G,R]) 
		self.hoop_colorbounds = [([100, 65, 55], [235, 206, 135])]

		########################################################################################################
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
				
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)
		
		# 2017-03-22 convert ROS images to OpenCV images
		self.bridge = CvBridge()

		# 2017-03-31 Lab 4 processing images variables
		self.processImages = False		
		self.cv_output = None
		self.cv_img = None
		# detect hoop every [x] seconds, don't process too frequently as it may introduce latency
		self.commandTimer = rospy.Timer(rospy.Duration(0.5),self.HoopDetector)

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	###############################################################
	# Added function to extract the location data 
	def location(self, position): 
		self.actual_location = position
		actual_posit = np.array([self.actual_location.transform.translation.x,
		                      self.actual_location.transform.translation.y,
		                      self.actual_location.transform.translation.z])

		self.x_pos = actual_posit[0]
		self.y_pos = actual_posit[1]
		self.z_pos = actual_posit[2]

		quaternion = np.array([self.actual_location.transform.rotation.x,
		                      self.actual_location.transform.rotation.y,
		                      self.actual_location.transform.rotation.z,
		                      self.actual_location.transform.rotation.w])  

		self.roll = quaternion[0]
		self.pitch = quaternion[1]
		self.yaw = quaternion[2]
		self.rise = quaternion[3]

	###############################################################

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:			
					# Convert the ROS image into a QImage which we can display
					if self.processImages == False:
						image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))						
					# display processed image when processing is enabled
					else:
						if self.cv_output is not None:					
							# convert from openCV output cv_output image back to ROS image (Optional for visualization purposes)
							img_msg = self.bridge.cv2_to_imgmsg(self.cv_output, encoding="bgr8")
							# convert to QImage to be displayed
							image = QtGui.QPixmap.fromImage(QtGui.QImage(img_msg.data, img_msg.width, img_msg.height, QtGui.QImage.Format_RGB888))
						else:
							image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))		
					
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)
		
		

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
			# 2017-03-22 we do not recommend saving images in this function as it might cause huge latency
		finally:
			self.imageLock.release()
			
	# 2017-03-22 sample function of saving images.
	def SaveImage(self):
		# Added automatic file naming system for saved images 
		filename = "rawimage_" + str(self.num) + ".jpeg"      		
		# ensure not in the process of acquiring image
		if self.imageLock.acquire():
			try:
				# convert from ROS image to OpenCV image
				cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")

				# Save images to local disk 
				cv2.imwrite("/home/pid/"+filename,cv_image)
		                print("Image saved")

				# Publishing location of saved images
				self.image_pos.transform.translation.x = self.x_pos
				self.image_pos.transform.translation.y = self.y_pos
				self.image_pos.transform.translation.z = self.z_pos
				self.image_pos.transform.rotation.x = self.roll 
				self.image_pos.transform.rotation.y = self.pitch 
				self.image_pos.transform.rotation.z = self.yaw
				self.image_pos.transform.rotation.w = self.rise 

				self.pub_image_pos.publish(self.image_pos)	
 
			except CvBridgeError as e:
				print "Image conversion failed: %s" % e

			self.imageLock.release()
		# Update num for name of file 
		self.num=self.num + 001   

	def EnableImageProcessing(self):  # called from KeyboardController Key_P
		self.processImages = True

	def DisableImageProcessing(self): # called from KeyboardController Key_P
		self.processImages = False

	def HoopDetector(self,event):
		# reference: http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
		if self.image is not None and self.processImages == True:
			if self.imageLock.acquire():
				try:
					# convert from ROS image to OpenCV image
					self.cv_img = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
				except CvBridgeError as e:
					print "Image conversion failed: %s" % e
				self.imageLock.release()
				# carry out color thresholding
				hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)

				for (lower, upper) in self.hoop_colorbounds:
					lower = np.array(lower, dtype = "uint8")
					upper = np.array(upper, dtype = "uint8")
					# find the colors within the specified boundaries and apply mask
					# the mask shows values = 255 for image pixels that fall within the boundaries
					mask = cv2.inRange(self.cv_img, lower, upper)
					self.cv_output = cv2.bitwise_and(self.cv_img, self.cv_img, mask = mask)
					self.ProcessHoopImage(mask)
	
	def ProcessHoopImage(self,mask):
		# Algorithm to detect center of circular hoop based on pixel locations

		#To reduce noise and smootheen image
		gray = cv2.medianBlur(mask,5)
	    
		#Adaptive mean tresholding
		gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,7,5)
		
		#for cleaning
		kernel = np.ones((1,2),np.uint8)   
		gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel, iterations = 2)
		
		#Apply GuassianBlur to further reduce noise and smootheen image
		gray = cv2.GaussianBlur(gray,(5,5),0)
		gray = cv2.medianBlur(gray,5)
		
		# detect circles in the image
		# Returns the x, y and radius detected
	    	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 500, param1=100, param2=75, minRadius=50, maxRadius=0)
	    	filename2 = "proc" + str(self.num2) + ".jpeg"    
                if circles is not None:
		
			#convert the (x,y) coordinates and radius of the circle to integers
			circles = np.round(circles[0, :]).astype("int")
			
			#loop over coordinates and circle radius
			for (x,y,r) in circles:
		    
				circle_x = x
				circle_y = y
				circle_radius = r
       
				# To ensure largest circle is used
                                if circle_radius >= max_r:
                                        max_r = circle_radius
                                        circle_x = x
                                        circle_y = y

                        
			#for viewing the circle and its center
			#cv2.circle(img2, (circle_x,circle_y), max_r, (0, 255, 0), 4)
			#cv2.rectangle(img2, (circle_x - 5, circle_y - 5), (circle_x + 5, circle_y + 5), (0, 128, 255), -1)
                        #cv2.imshow('detected_circle', img2)

			#Save processed image for analysis after lab 
			#cv2.imwrite("/home/pid/" + filename2 ,img2)

			#hoop diameter calculations
                        dia_hoop = 0.86
                        dia_circle = 2 * max_r
			z_pixel = 180 - circle_y

			#new_hoop height
			z_correction = (dia_hoop * z_pixel) / dia_circle
			self.des_z = z_correction + self.z_pos

                        #store for publishing
                        self.new_z.point.z = self.des_z
                        self.pub_des_z.publish(self.new_z)
			
                else:
			print ('NO DETECTION')
			#self.des_z also initialized as 1.25
                       	#if self.hoop_dist < 0.75:
			
			#stay at the same height
                        self.new_z.point.z = self.des_z
                        self.pub_des_z.publish(self.new_z)
		
		self.num2 = self.num2 + 1

	# 2017-03-31 ============================================

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
