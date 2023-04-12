#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from tf.broadcaster import TransformBroadcaster

class KinematicControl:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('kinematic_control')

        # sample time
        self.ts = 0.1

        # Inicializar posicion y orientacion
        self.hx = 0
        self.hy = 0
        self.phi = 0

        # Distancia entre las llantas
        self.d = 0.5

        # Velocidad inicial de las llantas
        self.ur = 0.1
        self.ul = 0.1

        # Suscribirse a la velocidad de las llantas
        rospy.Subscriber('/right_wheel_velocity', Float64, self.right_wheel_callback)
        rospy.Subscriber('/left_wheel_velocity', Float64, self.left_wheel_callback)

        # Publicar la pose del robot
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

        # Tf
        self.tf_br = TransformBroadcaster()

        # control
        self.control_loop()

    def right_wheel_callback(self, data):
        self.ur = data.data

    def left_wheel_callback(self, data):
        self.ul = data.data

    def control_loop(self):
        rate = rospy.Rate(1/self.ts)
        while not rospy.is_shutdown():
            # Modelo cinematico
            hxp = ((self.ur + self.ul)/2) * math.cos(self.phi)
            hyp = ((self.ur + self.ul)/2) * math.sin(self.phi)
            phip = (self.ur - self.ul) / self.d

            # Euler
            self.hx += self.ts * hxp
            self.hy += self.ts * hyp
            self.phi += self.ts * phip

            # Publicar la pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = self.hx
            pose_msg.pose.position.y = self.hy
            pose_msg.pose.orientation.w = math.cos(self.phi/2)
            pose_msg.pose.orientation.z = math.sin(self.phi/2)
            self.pose_pub.publish(pose_msg)

            # Publicar las transformadas
            self.tf_br.sendTransform((self.hx, self.hy, 0), 
                                     (math.cos(self.phi/2), 0, 0, math.sin(self.phi/2)), 
                                     rospy.Time.now(), 
                                     'robot', 'world')

            rate.sleep()

if __name__ == '__main__':
    try:
        KinematicControl()
    except rospy.ROSInterruptException:
        pass
