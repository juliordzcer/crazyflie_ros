#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

def cmdVelCallback(msg):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cmd_vel"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = 1.0
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.points = [msg.linear.x, msg.linear.y, msg.linear.z]
    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_plot_node')
    marker_pub = rospy.Publisher('cmd_vel_marker', Marker, queue_size=10)
    cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmdVelCallback, queue_size=10)
    rospy.spin()