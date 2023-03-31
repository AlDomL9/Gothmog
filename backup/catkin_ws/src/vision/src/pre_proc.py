#!/usr/bin/env python
#-*- coding: UTF-8 -*-

"""
Preprocesamiento
	Nodo de preprocesamiento de imágenes
	publicadas como mensaje en ROS.
	
	"Vision is a picture of the future
		that produces passion"
						- Bill Hybels
	
	"Los bestos de bestos, lqm <3 
		besito en el k'chetitoh"
						- Jimena "Riggs" Trachtman
	
Autores:
	Diego Alberto Anaya Márquez - A01379375
	Alejandro Domínguez Lugo - A01378028
	
Fecha:
	26/04/2022
"""

# Importar librerías
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

# Inicializar nodo
rospy.init_node('PreProcessing')

img_msg = None

def img_callback(data):
	"""
	img_callback
		Función para recuperar la información de la imagen
		del tópico de ros /video_source/raw
	
	Parámetros:
		data: Image
			Imagen en ROS
	
	Return:
		None
	
	"""
	global img_msg
	img_msg = data

# Suscripción a tópico de imagen en bruto
rospy.Subscriber('/video_source/raw', Image, img_callback)

# Publicador a tópico de imagen preprocesada
img_pub = rospy.Publisher('/video_source/pre', Image, queue_size=1)

# Crear puente entre ROS y CV2
bridge = cv_bridge.CvBridge()

rate = rospy.Rate(20)

try:
	while not rospy.is_shutdown():
		if img_msg is not None:
			# Convertimos imagen de mensaje ROS a cv2
			result_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
			
			# Preprocesamiento
			result_image = cv2.resize(result_image, (320, 240)) # Reducción de tamaño
			result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY) # Conversión a escala de grises
			result_image = cv2.GaussianBlur(result_image, (3, 3), cv2.BORDER_DEFAULT) # Difuminación Gaussiana (suavizamiento)
			
			# Convertir imagen de cv2 a mensaje ROS
			image_back = bridge.cv2_to_imgmsg(result_image)
			
			# Publicar imagen
			img_pub.publish(image_back)
		
		rate.sleep()
except (rospy.ROSInterruptException, rospy.ROSException("topic was closed during publish()")):
	pass
