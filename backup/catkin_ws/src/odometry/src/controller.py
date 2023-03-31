#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

rospy.init_node('Controller')

eth = 0
ed = None
v_fact = 0
# Para regular el aumento de velocidad lineal (saturador)
v = 0 # Velocidad deseada (segun control)
prev_v = v # Velocidad anterior

KW = 0.128 # Debe ser menos de 0.4 / 3.14 = 0.128 para evitar velocidades altas
KV = 0.055
V_STEP = 0.01 # Cambio maximo de velocidad lineal
V_MAX = 0.4 # Velocidad lineal maxima
ED_THR = 0.03 # Umbral de error en distancia para detener el robot
DEACC_FACT = 5
ALPHA = 4

def callback_err_th(data):
	global eth
	eth = data.data
	
def callback_err_d(data):
	global ed
	ed = data.data

def callback_semaphore(data):
	global v_fact
	v_fact = data.data

# Topicos en los que se publican los errores de theta y distancia
rospy.Subscriber('/error/th', Float32, callback_err_th)
rospy.Subscriber('/error/d', Float32, callback_err_d)
rospy.Subscriber('/semaphore', Float32, callback_semaphore)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Si el programa deja de correr (ya sea por interrupcion o porque
# termino), detenemos al robot
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
		
		# Si todavia no se ha publicado el error en distancia, no
		# hacemos nada aun
		if ed is not None:
			
			# Si ya llegamos al objetivo
			if ed < ED_THR:
				# Salimos del ciclo de trabajo
				break
		
			# "Controlador P"
			w = eth * KW * v_fact
			v = V_MAX * (1 - np.exp(-ALPHA * (ed ** 2))) * v_fact
			
			# Si el cambio de velocidad es mayor que V_STEP, forzamos
			# a que aumente en V_STEP
			if (v - prev_v) > V_STEP:
				v = prev_v + V_STEP
			elif (prev_v - v) > V_STEP:
				v = prev_v - V_STEP * DEACC_FACT
			
			# v = np.min([v, V_MAX])
			
			print('v', v)
			
			# Actualizamos prev_v
			prev_v = v
			
			# Publicamos en /cmd_vel
			t = Twist()
			t.linear.x = v
			t.angular.z = w
			
			cmd_vel_pub.publish(t)
			
			rate.sleep()
	
	# Si el ciclo termina quiere decir que ya llegamos al objetivo, por
	# lo tanto, detenemos al robot
	end_callback()

except (rospy.ROSInterruptException, rospy.ROSException('topic was closed during publish()')):
	pass
