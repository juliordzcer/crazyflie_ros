#!/usr/bin/env python3

import rospy
import math as m
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Twist
import tf.transformations as t


class AttitudePublisher:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.pose_sub = rospy.Subscriber('/crazyflie1/pose', PoseStamped, self.pose_callback)
        self.thrust_sub = rospy.Subscriber('/crazyflie1/cmd_vel', Twist, self.thrust_callback)
        self.yaw_sub = rospy.Subscriber('/crazyflie1/vrpn_client_node/crazyflie1/pose', PoseStamped, self.yaw_callback)
        self.attitude_pub = rospy.Publisher('/crazyflie2/info_leader', Twist, queue_size=50)

        self.masa = 0.032

    def pose_callback(self, msg):
        # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy1 = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Extraer pitch y roll de los ángulos rpy
        self.roll = rpy1[0]
        self.pitch = rpy1[1]
        
    def thrust_callback(self, msg):
        # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
        self.thrust = msg.linear.z

    def yaw_callback(self, msg):
        # Extraer el ángulo yaw a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy2 = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = rpy2[2]
        # self.yaw = m.degrees(rpy[2])

    def publish_attitude(self):
        attitude = Twist()

        # Calcular la fuerza de empuje a partir del valor de thrust
        self.thrust_gramos = 1.0942e-07*self.thrust**2 - 2.1059e-04*self.thrust + 1.5417e-01
        self.thrust_newtons = self.thrust_gramos * 0.00980665

        # Calcular las aceleraciones a partir de los ángulos y la velocidad de ascenso
        self.gammax = (self.thrust_newton/self.masa)*(m.cos(self.roll)*m.sin(self.pitch)*m.cos(self.yaw) + m.sin(self.roll)*m.sin(self.yaw))
        self.gammay = (self.thrust_newton/self.masa)*(m.cos(self.roll)*m.sin(self.pitch)*m.sin(self.yaw) - m.sin(self.roll)*m.cos(self.yaw))
        self.gammaz = (self.thrust_newton/self.masa)*(m.cos(self.roll)*m.cos(self.pitch))

        attitude.linear.x = self.gammax
        attitude.linear.y = self.gammay 
        attitude.linear.z = self.gammaz
        self.attitude_pub.publish(attitude)


if __name__ == '__main__':
    rospy.init_node('PubLeaderInfo')
    attitude_publisher = AttitudePublisher()

    # Mantener el nodo en ejecución
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        attitude_publisher.publish_attitude()
        rate.sleep()