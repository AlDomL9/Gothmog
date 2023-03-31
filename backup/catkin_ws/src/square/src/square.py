#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Dimensiones
R = 0.051 # ya jalo
# L = 0.178 # ya jalo
L = 0.175

# Setup
rospy.init_node('square')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(30)

robot_cmd = Twist()
robot_wl = Float32()
robot_wr = Float32()

def wlCallBack(msg):
	# callback wl
	global robot_wl
	robot_wl = msg

rospy.Subscriber('/wl', Float32, wlCallBack)

def wrCallBack(msg):
	# callback wr
	global robot_wr
	robot_wr = msg
	
rospy.Subscriber('/wr', Float32, wrCallBack)

def end_callback():
	# callback nodo destruido
	robot_cmd.linear.x = 0
	robot_cmd.angular.z = 0
	pub.publish(robot_cmd)

rospy.on_shutdown(end_callback)

def move(fwd_speed=0.0, dist=0.0):
	# Avanza a fwd_speed m/s hasta que alcanza la distancia dist
	t0 = rospy.get_rostime().to_sec()
	robot_cmd.linear.x = fwd_speed
	robot_cmd.angular.z = 0
	estimated_distance = 0.0
	while True:
		# Publica hasta alcanzar el objetivo
		pub.publish(robot_cmd)
		t1 = rospy.get_rostime().to_sec()
		# Calculo de distancia recorrida hasta el momento
		estimated_distance += R * ((robot_wr.data + robot_wl.data) / 2) * (t1 - t0)
		t0 = t1
		if estimated_distance >= dist:
			print(estimated_distance)
			robot_cmd.linear.x = 0
			robot_cmd.angular.z = 0
			pub.publish(robot_cmd)
			# Se asegura de que las llantas frenen completamente
			while abs(robot_wr.data) > 0.01 or abs(robot_wl.data) > 0.01:
				rate.sleep()
			break
		rate.sleep()

def rotate(angular_speed=0.0, angle=0.0):
	# Rota a angular_speed rad/s hasta que alcanza el angulo angle
	t0 = rospy.get_rostime().to_sec()
	robot_cmd.linear.x = 0
	robot_cmd.angular.z = angular_speed
	estimated_angle = 0.0
	while True:
		# Publica hasta alcanzar el objetivo
		pub.publish(robot_cmd)
		t1 = rospy.get_rostime().to_sec()
		# Calculo de rotacion alcanzada hasta el momento
		estimated_angle += R * ((robot_wr.data - robot_wl.data) / L) * (t1 - t0)
		t0 = t1
		if estimated_angle >= angle:
			print(estimated_angle)
			robot_cmd.linear.x = 0
			robot_cmd.angular.z = 0
			pub.publish(robot_cmd)
			# Se asegura de que las llantas frenen completamente
			while abs(robot_wr.data) > 0.01 or abs(robot_wl.data) > 0.01:
				rate.sleep()
			break
		rate.sleep()

try:
	i = 0
	while not rospy.is_shutdown() and i < 4:
		print('side', i+1)
		# Avanza al frente
		# move(0.04, 1) # ya jalo
		move(0.04, 1 - 0.02)
		# Gira 90 deg
		# rotate(0.04, 3.1416 / 2) # ya jalo
		rotate(0.04, math.pi / 2)
		i += 1
		rate.sleep()
except rospy.exceptions.ROSInterruptException:
	pass
