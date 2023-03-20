#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")
    yaw = rospy.get_param("~yaw")


    rate = rospy.Rate(r)
    
    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()



# roll = 0
# pitch = 0
# pub = None

# def Callback(data):
#     global roll
#     global pitch
#     roll = data.linear.x
#     pitch = data.linear.y

#     worldFrame = rospy.get_param("~worldFrame", "world")
    
#     r = rospy.get_param("~rate")

#     x = rospy.get_param("~x")
#     y = rospy.get_param("~y")
#     z = rospy.get_param("~z")
    
#     yaw = rospy.get_param("~yaw")

#     # print(roll)
#     # print("-------------------------")
#     # print(pitch)
#     # print("-------------------------")
#     # print(yaw)
#     # print("-------------------------")
    
    
#     rate = rospy.Rate(r)
#     new_msg = PoseStamped()
#     new_msg.header.seq = 0
#     new_msg.header.seq += 1
#     new_msg.header.stamp = rospy.Time.now()
#     new_msg.header.stamp = rospy.Time.now()
#     new_msg.header.frame_id = worldFrame
#     new_msg.pose.position.x = x
#     new_msg.pose.position.y = y
#     new_msg.pose.position.z = z
#     quaternion = tf.transformations.quaternion_from_euler(roll, pitch , yaw)
#     new_msg.pose.orientation.x = quaternion[0]
#     new_msg.pose.orientation.y = quaternion[1]
#     new_msg.pose.orientation.z = quaternion[2]
#     new_msg.pose.orientation.w = quaternion[3]
#     pub.publish(new_msg)


# if __name__ == '__main__':

#     rospy.init_node('publish_pose', anonymous=True)

#     tf_prefix = rospy.get_param("~prefix")
#     name = rospy.get_param("~name")

#     sub = rospy.Subscriber("/"+ tf_prefix + "/cmd_vel", Twist, Callback)
#     pub = rospy.Publisher(name, PoseStamped, queue_size=1)
#     rospy.spin()

