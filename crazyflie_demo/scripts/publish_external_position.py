#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
from crazyflie_driver.srv import UpdateParams

def onNewTransform(self, pose):
    global firstTransform

    if firstTransform:
        # initialize kalman filter
        rospy.set_param("kalman/initialX", pose.pose.position.x)
        rospy.set_param("kalman/initialY", pose.pose.position.y)
        rospy.set_param("kalman/initialZ", pose.pose.position.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        update_params(["kalman/resetEstimation"]) 
        firstTransform = False


if __name__ == '__main__':
    rospy.init_node('publish_external_position', anonymous=True)
    topic = rospy.get_param("~topic", "vrpn_client_node/crazyflie/pose")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    rospy.Subscriber(topic, PoseStamped, onNewTransform)

    rospy.spin()