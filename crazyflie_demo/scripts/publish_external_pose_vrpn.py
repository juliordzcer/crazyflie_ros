#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from crazyflie_driver.srv import UpdateParams

def onNewTransform(pose):
    global msg
    global pub
    global firstTransform

    if firstTransform:
        # initialize kalman filter
        rospy.set_param("kalman/initialX", pose.pose.position.x)
        rospy.set_param("kalman/initialY", pose.pose.position.y)
        rospy.set_param("kalman/initialZ", pose.pose.position.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        rospy.set_param("locSrv/extPosStdDev", 1e-3)
        rospy.set_param("locSrv/extQuatStdDev", 0.5e-1)

        update_params(["kalman/resetEstimation", "locSrv/extPosStdDev", "locSrv/extQuatStdDev"]) 
        firstTransform = False

    else:
        msg.header.frame_id = pose.header.frame_id
        msg.header.stamp = pose.header.stamp
        msg.header.seq += 1
        msg.pose.position = pose.pose.position
        msg.pose.orientation = pose.pose.orientation

        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vrpn', anonymous=True)
    topic = rospy.get_param("~topic", "vrpn_client_node/crazyflie1/pose")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_pose", PoseStamped, queue_size=1)
    rospy.Subscriber(topic, PoseStamped, onNewTransform)

    rospy.spin()





