#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie.srv import UpdateParams

class PublishExternalPosition:
    def __init__(self):
        rospy.init_node('publish_external_position_vrpn', anonymous=True)
        self.topic = rospy.get_param("~topic", "/vrpn_client_node/crazyflie1/pose")

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', UpdateParams)

        self.firstTransform = True

        self.pub = rospy.Publisher("external_pose", PoseStamped, queue_size=1)
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
            self.msg = PoseStamped()
            self.msg.header.seq = 0
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.seq += 1
            self.msg.pose.position.x = pose.pose.position.x 
            self.msg.pose.position.y = pose.pose.position.y
            self.msg.pose.position.z = pose.pose.position.z
            self.msg.pose.orientation.x = pose.pose.orientation.x
            self.msg.pose.orientation.y = pose.pose.orientation.y
            self.msg.pose.orientation.z = pose.pose.orientation.z
            self.msg.pose.orientation.w = pose.pose.orientation.w

            self.pub.publish(self.msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.sleep(7)
    external_position = PublishExternalPosition()
    external_position.run()
