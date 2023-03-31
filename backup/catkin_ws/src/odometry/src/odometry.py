#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

rospy.init_node('Odometry')

# Inicializamos las velocidades de las ruedas en 0
wr = 0.0
wl = 0.0

def callback_wl(data):
	global wl
	wl = data.data

def callback_wr(data):
	global wr
	wr = data.data

# Topicos en los que el robot publica las velocidades de las ruedas
# medidas por los encoders
rospy.Subscriber('/wl', Float32, callback_wl)
rospy.Subscriber('/wr', Float32, callback_wr)

# Topico en el que publicaremos la odometria (x, y, theta)
odom_pub = rospy.Publisher('/odometry', Pose2D, queue_size=1)

# Topicos en los que publicamos los errores
eth_pub = rospy.Publisher('/error/th', Float32, queue_size=1)
ed_pub = rospy.Publisher('/error/d', Float32, queue_size=1)

rate = rospy.Rate(20)

current_time = rospy.get_time()
previous_time = current_time

r = 0.051 # Radio de las llantas
d = 0.175 # Distancia entre las llantas

# Estado inicial
th = 0
x = 0
y = 0

target = (2.0, 0.0) # x,y

try:
	print('Odometry running')
	while not rospy.is_shutdown():
		current_time = rospy.get_time()
		
		dt = current_time - previous_time
		previous_time = current_time
		x += r * (wr + wl) / 2 * dt * np.cos(th)
		y += r * (wr + wl) / 2 * dt * np.sin(th)
		th += r * (wr - wl) / d * dt
		
		# Para que theta vaya de -pi a pi
		if th < -np.pi:
			th += 2 * np.pi

		if th >= np.pi:
			th -= 2 * np.pi
		
		# Publica odometria estimada
		odom = Pose2D()
		odom.x = x
		odom.y = y
		odom.theta = th
		
		odom_pub.publish(odom)
		
		# Calcula el error en theta y distancia
		eth = np.arctan2(target[1] - y, target[0] - x) - th # Restamos x,y del robot para que el error en theta sea relativo al robot y no al sistema global
		ed = np.sqrt((target[0] - x) ** 2 + (target[1] - y) ** 2)
		
		# Publica los errores
		eth_pub.publish(eth)
		ed_pub.publish(ed)
		
		rate.sleep()

except (rospy.ROSInterruptException, rospy.ROSException('topic was closed during publish()')):
	pass
