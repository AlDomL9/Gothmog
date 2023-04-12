#!/usr/bin/env python
#------------------------- Imports --------------------------
import rospy
from geometry_msgs.msg import Twist
import sys

#------------------------ Variables -------------------------
rospy.init_node("turn_t")

# Velocity publisher
vel_cmd = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Frequency
rate = rospy.Rate(60)

#------------------------ Functions -------------------------
def end_callback():
    """Callback on shutdown

    """
    # Stop Gothmog
    vel_cmd.linear.x = 0.0
    vel_cmd.angular.z = 0.0
    pub.publish(vel_cmd)

rospy.on_shutdown(end_callback)

def run(verbose = False):
    """Move Gothmog in a straight line for 10 seconds

    Args:
        verbose (bool, optional): To be talkative. Defaults to False.

    """
    # Initiate node
    
    if verbose:
        print("turn_t node initialized, master.")

    # Initiate time
    t0 = rospy.get_rostime().to_sec()

    try:
        while not rospy.is_shutdown():
            tf = rospy.get_rostime().to_sec()

            # If time is over stop
            if tf - t0 >= 10:
                if verbose:
                    print("Task finished, master.")

                v = 0.0
                w = 0.0
                vel_cmd.linear.x = v
                vel_cmd.angular.z = w
                pub.publish(vel_cmd)
                break

            # While time is not over run
            if verbose:
                print(str(tf - t0) + " seconds have passed, master.")

            v = 0.0
            w = 0.1
            vel_cmd.linear.x = v
            vel_cmd.angular.z = w
            pub.publish(vel_cmd)

            rate.sleep()

    except rospy.exceptions.ROSInterruptException:
        pass

#--------------------------- Main ---------------------------
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "--verbose":
            run(verbose = True)
        else:
            print("Wrong usage")
            print("Expecting: turn_t --verbose")
            exit(1)
    run()
