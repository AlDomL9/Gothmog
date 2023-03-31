#!/usr/bin/env python
#-*- coding: UTF-8 -*-

"""
Circle identification
    ROS node implementation that reads the Puzzlebot's raw camera image.
    Then, after due processing, identifies green, yellow and/or red 
    circles. This information is then transmitted as an image for 
    further use.
        Also, this node controls the robot so it reacts to the circle 
    detected; as if responding to a semaphore.

Authors:
    Diego Alberto Anaya Márquez   A01379375
    Alejandro Domínguez Lugo      A01378028

Date:
    April 9th, 2022

"""

'''-------------------------------Libraries---------------------------------'''
import cv2
import numpy as np
import rospy
import cv_bridge # Used to convert image between ROS mesage and cv2 data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

'''------------------------ROS node initialization--------------------------'''
rospy.init_node('LineDetector')
rate = rospy.Rate(20)

'''------------------------Callback subscriptions---------------------------'''
def img_callback(data):
	"""
	img_callback
		Recovers raw image data
	
	Parámetros:
		data: Image
			ROS Image
	
	Return:
		None
	
	"""
    
	global img_msg
	img_msg = data

def end_callback():
	print('stopping')
	tw = Twist()
	tw.linear.x = 0
	tw.angular.z = 0
	cmd_vel_pub.publish(tw)

rospy.on_shutdown(end_callback)

'''--------------------Topic subscription and creation----------------------'''
rospy.Subscriber('/video_source/raw', Image, img_callback) # Raw image topic
img_pub = rospy.Publisher('/video_source/lines', Image, queue_size=1) # Processed image publication
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# ROS image data to CV2 image converter
bridge = cv_bridge.CvBridge()

'''-------------------------------Constants---------------------------------'''
# Kernel for image opening
KERNEL = np.ones((5, 5), np.uint8)
# Binary threshold
BIN_THR = 190
# Center threshold
C_TH = 20
# Linear vel
V_MAX = 0.1 #0.15
V_MIN = 0.03 #0.05

'''-------------------------------Main loop---------------------------------'''
img_msg = None
try:
	print('Running')
	while not rospy.is_shutdown():
		if img_msg is not None:
			# Convert image from ROS mesage to CV2 format
			frame = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			
			threshold, otsu = cv2.threshold(gray, BIN_THR, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
			
            # Open image to reduce background noise
			opened = cv2.morphologyEx(otsu, cv2.MORPH_OPEN, KERNEL, iterations=2)
			
			# Cut image to ROI
			opened = opened[int(5 * frame.shape[0] / 8):, :]
			
			result = np.zeros((opened.shape[0], opened.shape[1], frame.shape[2]), np.uint8)
			
			edges = cv2.Canny(opened, 50, 150)
			lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 40, minLineLength=int(np.min(opened.shape) / 4), maxLineGap=30)
			w = 0
			try:
				prev_score = 0
				for line in lines:
					x1, y1, x2, y2 = line[0]
					cv2.line(result, (x1, y1), (x2, y2), (255, 255, 255), 1)
					dy = abs(y1 - y2)
					y_closer = np.max([y1, y2])
					score = dy + 2 * y_closer
					if score > prev_score:
						final_line = (x1, y1, x2, y2)
						prev_score = score
							
					
				x1, y1, x2, y2 = final_line
				cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 1)
				center = int(result.shape[1] / 2)
				kp = 0.4 / result.shape[1]
				if y1 > y2:
					if x1 > center + C_TH:
						x_err = center + C_TH - x1
					elif x1 < center - C_TH:
						x_err = center - C_TH - x1
					else:
						x_err = x1 - x2
				else:
					if x2 > center + C_TH:
						x_err = center + C_TH - x2
					elif x2 < center - C_TH:
						x_err = center - C_TH - x2
					else:
						x_err = x2 - x1
				w = kp * x_err
				m = (V_MAX - V_MIN) / (-result.shape[1] / 2 - C_TH)
				v = m * abs(x_err) + V_MAX
			except:
				pass
			
			t = Twist()
			t.linear.x = v
			t.angular.z = w
			
			cmd_vel_pub.publish(t)
				
			# Convert final image to ROS msg
			image_back = bridge.cv2_to_imgmsg(result, encoding='bgr8')
			
			# Publish image
			img_pub.publish(image_back)
			rate.sleep()
except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
	pass
