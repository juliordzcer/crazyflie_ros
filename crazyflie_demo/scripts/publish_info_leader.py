#!/usr/bin/env python3

import rospy
import math as m
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Twist
import tf.transformations as t

class ImuLinearAccelPublisher:
    def __init__(self):
        self.linear_accel_pub = rospy.Publisher('/crazyflie2/goalacc', Twist, queue_size=100)
        self.imu_sub = rospy.Subscriber('/crazyflie1/imu', Imu, self.imu_callback)

    def imu_callback(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = msg.linear_acceleration.x
        twist_msg.linear.y = msg.linear_acceleration.y
        twist_msg.linear.z = msg.linear_acceleration.z
        self.linear_accel_pub.publish(twist_msg)

class AttitudePublisher:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.pose_sub = rospy.Subscriber('/crazyflie1/pose', PoseStamped, self.pose_callback)
        self.thrust_sub = rospy.Subscriber('/crazyflie1/cmd_vel', Twist, self.thrust_callback)
        self.yaw_sub = rospy.Subscriber('/crazyflie1/vrpn_client_node/crazyflie1/pose', PoseStamped, self.yaw_callback)
        self.attitude_pub = rospy.Publisher('/crazyflie2/info_leader', Twist, queue_size=100)

    def pose_callback(self, msg):
        # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Extraer pitch y roll de los ángulos rpy
        self.roll = rpy[0]
        self.pitch = rpy[1]

    def thrust_callback(self, msg):
        # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
        self.thrust = msg.linear.z

    def yaw_callback(self, msg):
        # Extraer el ángulo yaw a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = rpy[2]
        # self.yaw = m.degrees(rpy[2])

    def publish_attitude(self):
        attitude = Twist()
        attitude.linear.z = self.thrust
        attitude.angular.x = self.roll
        attitude.angular.y = self.pitch 
        attitude.angular.z = self.yaw
        self.attitude_pub.publish(attitude)

if __name__ == '__main__':
    rospy.init_node('PubLeaderInfo')
    imu_linear_accel_publisher = ImuLinearAccelPublisher()
    attitude_publisher = AttitudePublisher()

    # Mantener el nodo en ejecución
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        attitude_publisher.publish_attitude()
        rate.sleep()



