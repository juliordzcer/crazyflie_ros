#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped  # PoseStamped added to support vrpn_client
from crazyflie_driver.srv import UpdateParams


class PublishExternalPosition:
    def __init__(self):
        rospy.init_node('publish_external_position_vrpn', anonymous=True)
        self.topic = rospy.get_param("~topic", "/crazyflie1/vrpn_client_node/crazyflie1/pose")

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', UpdateParams)

        self.firstTransform = True

        self.msg = PointStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()

        self.pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
        rospy.Subscriber(self.topic, PoseStamped, self.onNewTransform)

    def onNewTransform(self, pose):
        if self.firstTransform:
            # initialize kalman filter
            rospy.set_param("kalman/initialX", pose.pose.position.x)
            rospy.set_param("kalman/initialY", pose.pose.position.y)
            rospy.set_param("kalman/initialZ", pose.pose.position.z)
            self.update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

            rospy.set_param("kalman/resetEstimation", 1)
            self.update_params(["kalman/resetEstimation"])

            
            self.firstTransform = False

        else:
            self.msg.header.frame_id = pose.header.frame_id
            self.msg.header.stamp = pose.header.stamp
            self.msg.header.seq += 1
            self.msg.point.x = pose.pose.position.x
            self.msg.point.y = pose.pose.position.y
            self.msg.point.z = pose.pose.position.z
            self.pub.publish(self.msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    external_position = PublishExternalPosition()
    external_position.run()
