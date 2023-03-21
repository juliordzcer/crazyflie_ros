#!/usr/bin/env python3

import rospy
import math as m
from geometry_msgs.msg import PoseStamped, Twist
import tf.transformations as t

def pose_callback(msg):
    # Extraer los ángulos de rotación rpy a partir del mensaje "pose"
    q = msg.pose.orientation
    rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Crear un nuevo mensaje "TwistStamped" con los ángulos en rpy
    attitude = Twist()
    attitude.angular.x = m.degrees(rpy[0]) 
    attitude.angular.y = m.degrees(rpy[1])
    attitude.angular.z = m.degrees(rpy[2])

    # Publicar el mensaje en el topic "attitude_twist"
    attitude_pub.publish(attitude)

if __name__ == '__main__':
    # Inicializar el nodo ROS
    rospy.init_node('attitudeleader')

    # Crear un suscriptor para el topic "pose"
    pose_sub = rospy.Subscriber('pose', PoseStamped, pose_callback)

    # Crear un publicador para el topic "attitude_twist"
    attitude_pub = rospy.Publisher('attitude', Twist, queue_size=10)

    # Mantener el nodo en ejecución
    rospy.spin()