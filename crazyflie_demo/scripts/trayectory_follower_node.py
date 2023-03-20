#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from sensor_msgs.msg import Joy
import tf


class Follower:
    def __init__(self):
        rospy.init_node('formation_follower', anonymous=True)

        self.rate = rospy.get_param('~rate', 100) # Obtener la frecuencia desde ROS parameter server
        self.form_vec = rospy.get_param('~form_vec', [0.1, 0.1, 0.0]) # Obtener el vector de formación desde ROS parameter server

        self.r = 0.0
        self.h = 0.0
        self.t = 0.0

        self.pub_pose = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.pose_leader_sub = rospy.Subscriber('PoseLeader', PoseStamped, self.pose_leader_callback)

    def pose_leader_callback(self, pose_leader_msg):
        pose = PoseStamped()
        pose.header.seq = 0
        pose.header.frame_id = "world"

        # Obtener posición actual del seguidor
        x_f = pose_leader_msg.pose.position.x
        y_f = pose_leader_msg.pose.position.y
        z_f = pose_leader_msg.pose.position.z
        yaw = 0

        # Obtener vector de formación
        form_vec = np.array(self.form_vec)

        # Calcular vector director del líder al seguidor
        pos_leader = np.array([pose_leader_msg.pose.position.x, pose_leader_msg.pose.position.y, pose_leader_msg.pose.position.z])
        pos_follower = np.array([x_f, y_f, z_f])
        v_dir = pos_leader - pos_follower

        # Calcular vector de posición deseada del seguidor
        pos_deseada = pos_follower + v_dir + form_vec

        # Publicar posición deseada
        pose.pose.position.x = pos_deseada[0]
        pose.pose.position.y = pos_deseada[1]
        pose.pose.position.z = pos_deseada[2]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.pub_pose.publish(pose)

    def run(self):
        rate = rospy.Rate(self.rate) # Hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        follower = Follower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
