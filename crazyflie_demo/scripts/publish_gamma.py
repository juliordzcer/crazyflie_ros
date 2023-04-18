#!/usr/bin/env python3

import rospy
import math as m
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32

import tf.transformations as t

class AttitudePublisher:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.thrust_sub = rospy.Subscriber('leader_u', Float32, self.thrust_callback)
        self.yaw_sub = rospy.Subscriber('vrpn_client_node/crazyflie/pose', PoseStamped, self.yaw_callback)
        self.attitude_pub = rospy.Publisher('gamma', Twist, queue_size=50)

        self.masa = 0.032

    def pose_callback(self, msg):
        # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Extraer pitch y roll de los ángulos rpy
        self.roll = rpy[0]
        self.pitch = rpy[1]
        
    def thrust_callback(self, msg):
        # Extraer el empuje del seguidor
        self.thrust = msg.data

    def yaw_callback(self, msg):
        # Extraer el ángulo yaw a partir del mensaje "pose"
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = rpy[2]

    def publish_attitude(self):
        attitude = Twist()

        # Calcular las aceleraciones a partir de los ángulos y la velocidad de ascenso
        self.gammax = (self.thrust/self.masa)*(m.cos(self.roll)*m.sin(self.pitch)*m.cos(self.yaw) + m.sin(self.roll)*m.sin(self.yaw))
        self.gammay = (self.thrust/self.masa)*(m.cos(self.roll)*m.sin(self.pitch)*m.sin(self.yaw) - m.sin(self.roll)*m.cos(self.yaw))
        self.gammaz = (self.thrust/self.masa)*(m.cos(self.roll)*m.cos(self.pitch))

        attitude.linear.x = self.gammax
        attitude.linear.y = self.gammay 
        attitude.linear.z = self.gammaz
        self.attitude_pub.publish(attitude)


if __name__ == '__main__':
    rospy.init_node('gamma')
    attitude_publisher = AttitudePublisher()

    # Mantener el nodo en ejecución
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        attitude_publisher.publish_attitude()
        rate.sleep()