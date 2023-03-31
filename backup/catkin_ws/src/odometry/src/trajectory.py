#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

rospy.init_node('Trajectory')

arrived = False

TRAJECTORY = ((0, 0), (2, 0), (2, 2), (0, 2), (0, 0))

rate = rospy.Rate(20)

def callback_gs(data):
	global arrived
	arrived = data.data

tr_pub = rospy.Publisher('/trajectory', Pose2D, queue_size=1)
rospy.Subscriber('/goal_status', Bool, callback_gs)

try:
	i = 0
	last_time = rospy.get_rostime().to_sec()
	while not rospy.is_shutdown():
		current_time = rospy.get_rostime().to_sec()
		if arrived and current_time - last_time > 2:
			i += 1
			last_time = rospy.get_rostime().to_sec()
		
		if i == len(TRAJECTORY):
			break
		
		p = Pose2D()
		p.x = TRAJECTORY[i][0]
		p.y = TRAJECTORY[i][1]
		tr_pub.publish(p)
		
		rate.sleep()
		
except (rospy.ROSInterruptException, rospy.ROSException('topic was closed during publish()')):
	pass
