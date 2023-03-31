#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

rospy.init_node('Controller')

eth = 0
ed = None
v_fact = 0
v = 0
prev_v = v

KW = 0.18 # Must be less than 0.4 / 3.14 = 0.128 to avoid too high speeds
V_STEP = 0.0005
V_MAX = 0.2
ED_THR = 0.07
DEACC_FACT = 5
ALPHA = 1.5

def callback_err_th(data):
	global eth
	eth = data.data
	
def callback_err_d(data):
	global ed
	ed = data.data

def callback_semaphore(data):
	global v_fact
	v_fact = data.data

rospy.Subscriber('/error/th', Float32, callback_err_th)
rospy.Subscriber('/error/d', Float32, callback_err_d)
rospy.Subscriber('/semaphore', Float32, callback_semaphore)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
goal_status_pub = rospy.Publisher('/goal_status', Bool, queue_size=1)
goal_status_pub.publish(False)

def end_callback():
	print('stopping')
	tw = Twist()
	tw.linear.x = 0
	tw.angular.z = 0
	cmd_vel_pub.publish(tw)

rospy.on_shutdown(end_callback)

rate = rospy.Rate(20)

try:
	while not rospy.is_shutdown():
		
		if ed is not None:
			
			if ed < ED_THR:
				goal_status_pub.publish(True)
				t = Twist()
				t.linear.x = 0.0
				t.angular.z = 0.0
			else:
				goal_status_pub.publish(False)
			
			print('ed ', ed)
			print('eth ', eth)
			
			w = eth * KW * v_fact
			v = V_MAX * (1 - np.exp(-ALPHA * (ed ** 2))) * v_fact
			
			if (v - prev_v) > V_STEP:
				v = prev_v + V_STEP
			elif (prev_v - v) > V_STEP:
				v = prev_v - V_STEP * DEACC_FACT
			
			prev_v = v
			
			t = Twist()
			t.linear.x = v
			t.angular.z = w
			
			cmd_vel_pub.publish(t)
			
			rate.sleep()
	
	end_callback()

except (rospy.ROSInterruptException, rospy.ROSException('topic was closed during publish()')):
	pass
