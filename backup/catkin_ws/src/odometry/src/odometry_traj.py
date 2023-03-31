#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose2D

rospy.init_node('Odometry')

wr = 0.0
wl = 0.0
target = None
prev_target = target

def callback_wl(data):
	global wl
	wl = data.data

def callback_wr(data):
	global wr
	wr = data.data

def callback_tr(data):
	global target
	global th_d
	global prev_target
	target = (data.x, data.y)
	if target != prev_target:
		prev_target = target
		th_d = np.arctan2(target[1] - y, target[0] - x)

rospy.Subscriber('/wl', Float32, callback_wl)
rospy.Subscriber('/wr', Float32, callback_wr)
rospy.Subscriber('/trajectory', Pose2D, callback_tr)

odom_pub = rospy.Publisher('/odometry', Pose2D, queue_size=1)

eth_pub = rospy.Publisher('/error/th', Float32, queue_size=1)
ed_pub = rospy.Publisher('/error/d', Float32, queue_size=1)

rate = rospy.Rate(20)

current_time = rospy.get_time()
previous_time = current_time

r = 0.051
d = 0.175

th = 0
x = 0
y = 0

th_d = th

# target = (2.0, -2.0) # x,y

# err = 0

try:
	print('Odometry running')
	while not rospy.is_shutdown():
		if target is not None:
			current_time = rospy.get_time()
			
			dt = current_time - previous_time
			previous_time = current_time
			x += r * (wr + wl) / 2 * dt * np.cos(th)
			y += r * (wr + wl) / 2 * dt * np.sin(th)
			th += r * (wr - wl) / d * dt
			
			if abs(th) > np.pi:
				s_th = np.sign(th)
				alpha = abs(th) % np.pi
				th = s_th * (alpha - np.pi)
			
			odom = Pose2D()
			odom.x = x
			odom.y = y
			odom.theta = th
			
			print('(x, y): ', x, ', ',y)
			
			odom_pub.publish(odom)
			 
			eth = (th_d - th) % (2 * np.pi)
			if eth > np.pi:
				eth -= 2 * np.pi
			
			ed = np.sqrt((target[0] - x) ** 2 + (target[1] - y) ** 2)
			
			eth_pub.publish(eth)
			ed_pub.publish(ed)
		
		rate.sleep()

except (rospy.ROSInterruptException, rospy.ROSException('topic was closed during publish()')):
	pass
