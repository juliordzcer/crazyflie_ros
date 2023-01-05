#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

counter = 0
pub = None

def callback_number(msg):
	global counter
	counter = msg.data
	new_msg = Float32()
	new_msg.data = counter
	pub.publish(new_msg)

if __name__ == '__main__':
	rospy.init_node('Angles')

	sub = rospy.Subscriber("/Angles", [], callback_number)

	pub = rospy.Publisher("/number_count", Float32, queue_size=10)

	rospy.spin()
