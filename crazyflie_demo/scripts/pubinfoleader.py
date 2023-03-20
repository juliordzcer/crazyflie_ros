#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class ImuLinearAccelPublisher(object):
    def __init__(self):
        self.linear_accel_pub = rospy.Publisher('/crazyflie2/AccLeader', Twist, queue_size=10)
        self.imu_sub = rospy.Subscriber('/crazyflie1/imu', Imu, self.imu_callback)

    def imu_callback(self, msg):
        linear_accel = msg.linear_acceleration
        twist_msg = Twist()
        twist_msg.linear.x = linear_accel.x
        twist_msg.linear.y = linear_accel.y
        twist_msg.linear.z = linear_accel.z
        self.linear_accel_pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('PubLeaderInfo')
    imu_linear_accel_publisher = ImuLinearAccelPublisher()
    rospy.spin()