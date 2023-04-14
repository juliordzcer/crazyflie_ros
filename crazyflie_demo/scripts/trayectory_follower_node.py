#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply


class FollowerNode():
    def __init__(self):
        rospy.init_node('follower_node', anonymous=True)

        # Obtiene los parámetros de formación
        self.formation_x = rospy.get_param('~formation_x', 1)
        self.formation_y = rospy.get_param('~formation_y', 0.0)
        self.formation_z = rospy.get_param('~formation_z', 0.0)
        self.formation_yaw = rospy.get_param('~formation_yaw', 0.0)

        # Crea el publisher para el objetivo de pose
        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=50)

        # Crea el subscriber para la pose del líder
        rospy.Subscriber('vrpn_client_node/crazyflie1/pose', PoseStamped, self.leader_pose_callback)

    def leader_pose_callback(self, leader_pose):
        # Obtiene la pose del líder
        leader_pos = leader_pose.pose.position
        leader_orient = leader_pose.pose.orientation

        # Crea la pose objetivo para el seguidor
        follower_goal = PoseStamped()
        follower_goal.header.stamp = rospy.Time.now()
        follower_goal.header.frame_id = 'world'

        # Agrega la posición objetivo, relativa a la posición del líder
        follower_goal.pose.position.x = leader_pos.x + self.formation_x
        follower_goal.pose.position.y = leader_pos.y + self.formation_y
        follower_goal.pose.position.z = leader_pos.z + self.formation_z

        # Agrega la orientación objetivo, rotada respecto al líder
        leader_quat = (leader_orient.x, leader_orient.y, leader_orient.z, leader_orient.w)
        # follower_quat = quaternion_from_euler(0, 0, 0)
        follower_quat = quaternion_from_euler(0, 0, self.formation_yaw)
        final_quat = quaternion_multiply(leader_quat, follower_quat)
        follower_goal.pose.orientation = Quaternion(*final_quat)

        # Publica la pose objetivo
        self.goal_pub.publish(follower_goal)

if __name__ == '__main__':
    try:
        FollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
