#!/usr/bin/env python3

import rospy
import math as m
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Twist
import tf.transformations as t
from std_msgs.msg import Float32


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

class GammaPublisher:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.pose_sub   = rospy.Subscriber('/crazyflie1/pose', PoseStamped, self.pose_callback)
        self.yaw_sub    = rospy.Subscriber('/crazyflie1/vrpn_client_node/crazyflie1/pose', PoseStamped, self.yaw_callback)
        self.thrust_sub = rospy.Subscriber('/crazyflie1/leader_u', Float32, self.thrust_callback)
        self.gamma_pub  = rospy.Publisher('/crazyflie2/info_leader', Twist, queue_size=50)

        self.masa = 0.032

    def pose_callback(self, msg):
        # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Extraer pitch y roll de los ángulos rpy
        self.roll = rpy[0]
        self.pitch = rpy[1]

    def yaw_callback(self, msg):
        # Extraer el ángulo yaw a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = rpy[2]

    def thrust_callback(self, msg):
        # Extraer el empuje del seguidor
        self.thrust = msg.data

    def publish_gamma(self):
        gamma = Twist()

        # Calcular las aceleraciones a partir de los ángulos y la velocidad de ascenso
        self.gammax = (self.thrust/self.masa)*(m.cos(self.roll)*m.sin(self.pitch)*m.cos(self.yaw) + m.sin(self.roll)*m.sin(self.yaw))
        self.gammay = (self.thrust/self.masa)*(m.cos(self.roll)*m.sin(self.pitch)*m.sin(self.yaw) - m.sin(self.roll)*m.cos(self.yaw))
        self.gammaz = (self.thrust/self.masa)*(m.cos(self.roll)*m.cos(self.pitch))

        gamma.linear.x = self.gammax
        gamma.linear.y = self.gammay 
        # gamma.linear.z = 0
        gamma.linear.z = self.gammaz
        self.gamma_pub.publish(gamma)

if __name__ == '__main__':
    rospy.init_node('PubLeaderInfo')
    imu_linear_accel_publisher = ImuLinearAccelPublisher()
    gamma_publisher = GammaPublisher()

    # Mantener el nodo en ejecución
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        gamma_publisher.publish_gamma()
        rate.sleep()



