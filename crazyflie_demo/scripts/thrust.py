#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_const_thrust', anonymous=True)
    p = rospy.Publisher('cmd_vel', Twist)
    twist = Twist()
    r = rospy.Rate(50)
    #for i in range(0, 100):
    #    p.publish(twist)
    #    r.sleep()

    twist.linear.z = 32000
    while not rospy.is_shutdown():
        p.publish(twist)
        r.sleep()