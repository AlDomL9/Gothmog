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
from std_msgs.msg import Float32

'''------------------------ROS node initialization--------------------------'''
rospy.init_node('ColorAndCircleDetector')
rate = rospy.Rate(20)

'''------------------------Callback subscriptions---------------------------'''
def end_callback():
    """
	end_callback
		Function to close all open operations when ROS closes
	
	Parámetros:
		None
	
	Return:
		None
	
	"""
    
    # Stop Puzzlebot
    cmd_vel_pub.publish(0.0)

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

'''--------------------Topic subscription and creation----------------------'''
rospy.on_shutdown(end_callback)
rospy.Subscriber('/video_source/raw', Image, img_callback) # Raw image topic
img_pub = rospy.Publisher('/video_source/circles', Image, queue_size=1) # Processed image publication
cmd_vel_pub = rospy.Publisher('/semaphore', Float32, queue_size=1) # Robot movement publication

# ROS image data to CV2 image converter
bridge = cv_bridge.CvBridge()

'''-------------------------------Constants---------------------------------'''
# Color CV2-HSV ranges
# Red needs two range sets due to HSV encoding
LOWER_RED_MIN = np.array([0, 100, 20])
LOWER_RED_MAX = np.array([10, 255, 255])

UPPER_RED_MIN = np.array([160, 100, 20])
UPPER_RED_MAX = np.array([190, 255, 255])

GREEN_MIN = np.array([36, 0, 0])
GREEN_MAX = np.array([86, 255, 255])

YELLOW_MIN = np.array([22, 93, 0])
YELLOW_MAX = np.array([45, 255, 255])

# Kernel for image opening
KERNEL = np.ones((5, 5), np.uint8)

# Color id reference
COLOR_ID = {0: 'green', 1: 'yellow', 2: 'red'}


'''-------------------------------Main loop---------------------------------'''
img_msg = None

try:
	while not rospy.is_shutdown():
		if img_msg is not None:
			# Convert image from ROS mesage to CV2 format
			frame = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			
            # Color mask creation
			lower_red_mask = cv2.inRange(hsv, LOWER_RED_MIN, LOWER_RED_MAX)
			upper_red_mask = cv2.inRange(hsv, UPPER_RED_MIN, UPPER_RED_MAX)
			red_mask = lower_red_mask + upper_red_mask # Red mask creation
			green_mask = cv2.inRange(hsv, GREEN_MIN, GREEN_MAX) # Green mask creation
			yellow_mask = cv2.inRange(hsv, YELLOW_MIN, YELLOW_MAX) # Yellow mask creation
			mask_rg = cv2.bitwise_or(red_mask, green_mask)
			mask = cv2.bitwise_or(yellow_mask, mask_rg)
			
            # Open image to reduce background noise
			opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
			
            # Invert image for blob detection
			inverted = cv2.bitwise_not(opened)
			
            # Define blob detection parametres
			params = cv2.SimpleBlobDetector_Params()
			params.filterByArea = True
			params.minArea = int(np.pi * (np.min(frame.shape[:2]) / 25) ** 2)
			params.maxArea = int(np.pi * (np.min(frame.shape[:2]) / 4) ** 2)
			params.filterByCircularity = True
			params.minCircularity = 0.75
			
            # Blob identification
			detector = cv2.SimpleBlobDetector_create(params)
			keypoints = detector.detect(inverted)
            
            # Prepare an image to draw blob identification and publication
			result = np.zeros(frame.shape, np.uint8)
            
            # If blobs are identified
			if len(keypoints) > 0:
				pixel_colors = []
                
                # Draw blobs identified and obtain characteristic pixel
				for keypoint in keypoints:
                    # Recover blob position
					x = int(keypoint.pt[0])
					y = int(keypoint.pt[1])
					d = keypoint.size
                    
                    # Draw blob
					cv2.circle(result, (x, y), int(d / 2), (255, 255, 255), -1)
                    
                    # Obtain pixel in center of blob
					pixel_colors.append(hsv[y, x])
				
				found_colors = []
                
                # Identify color of characteristic pixel
				for colors in pixel_colors:
					if (colors[0] > LOWER_RED_MIN[0] and colors[0] < LOWER_RED_MAX[0]) or (colors[0] > UPPER_RED_MIN[0] and colors[0] < UPPER_RED_MAX[0]):
						if (colors[1] > LOWER_RED_MIN[1] and colors[1] < LOWER_RED_MAX[1]) or (colors[1] > UPPER_RED_MIN[1] and colors[1] < UPPER_RED_MAX[1]):
							if (colors[2] > LOWER_RED_MIN[2] and colors[2] < LOWER_RED_MAX[2]) or (colors[2] > UPPER_RED_MIN[2] and colors[2] < UPPER_RED_MAX[2]):
								found_colors.append(2)
					if colors[0] > YELLOW_MIN[0] and colors[0] < YELLOW_MAX[0] and colors[1] > YELLOW_MIN[1] and colors[1] < YELLOW_MAX[1] and colors[2] > YELLOW_MIN[2] and colors[2] < YELLOW_MAX[2]:
						found_colors.append(1)
					if colors[0] > GREEN_MIN[0] and colors[0] < GREEN_MAX[0] and colors[1] > GREEN_MIN[1] and colors[1] < GREEN_MAX[1] and colors[2] > GREEN_MIN[2] and colors[2] < GREEN_MAX[2]:
						found_colors.append(0)
				
                # Comand robot to move or stop in response to color of characteristic pixel
				if len(found_colors) > 0:
					color = np.max(found_colors)
					
					print('Color found: ', COLOR_ID[color])
					
                    # If color is red stop, if color yellow move slowly and if green move freely
                    # With current if construction the heirarchy is red then yellow and, finally, green
					if COLOR_ID[color] == 'red':
						cmd_vel_pub.publish(0.0)
					elif COLOR_ID[color] == 'yellow':
						cmd_vel_pub.publish(0.5)
					else:
						cmd_vel_pub.publish(1.0)
			
			# Convert final image to ROS msg
			image_back = bridge.cv2_to_imgmsg(result, encoding='bgr8')
			
			# Publish image
			img_pub.publish(image_back)
			rate.sleep()
except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
	pass
