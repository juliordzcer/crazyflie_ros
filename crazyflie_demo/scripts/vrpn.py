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
        update_params(["kalman/resetEstimation"]) 
        firstTransform = False

    else:
        msg.header.frame_id = pose.header.frame_id
        msg.header.stamp = pose.header.stamp
        msg.header.seq += 1
        msg.point.x = pose.pose.position.x
        msg.point.y = pose.pose.position.y
        msg.point.z = pose.pose.position.z
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vrpn', anonymous=True)
    topic = rospy.get_param("~topic", "/crazyflie1/vrpn_client_node/crazyflie1/pose")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.Subscriber(topic, PoseStamped, onNewTransform)

    rospy.spin()

# #!/usr/bin/env python3

# import rospy
# import tf
# from geometry_msgs.msg import PoseStamped, TransformStamped
# from crazyflie_driver.srv import UpdateParams, Takeoff, Land

# class PublishExternalPosition:
#     def __init__(self):
#         rospy.init_node('publish_external_position_vrpn', anonymous=True)
#         self.topic = rospy.get_param("~topic", "/vrpn_client_node/crazyflie1/pose")

#         rospy.wait_for_service('update_params')
#         rospy.loginfo("found update_params service")
#         self.update_params = rospy.ServiceProxy('update_params', UpdateParams)

#         self.firstTransform = True

#         self.pub = rospy.Publisher("external_pose", PoseStamped, queue_size=1)
#         rospy.Subscriber(self.topic, PoseStamped, self.onNewTransform)

#     def onNewTransform(self, pose):
#         if self.firstTransform:
#             # initialize kalman filter
#             rospy.set_param("kalman/initialX", pose.pose.position.x)
#             rospy.set_param("kalman/initialY", pose.pose.position.y)
#             rospy.set_param("kalman/initialZ", pose.pose.position.z)
#             self.update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

#             rospy.set_param("kalman/resetEstimation", 1)
#             self.update_params(["kalman/resetEstimation"])

#             self.firstTransform = False
#         else:
#             self.msg = PoseStamped()
#             self.msg.header.seq = 0
#             self.msg.header.stamp = rospy.Time.now()
#             self.msg.header.seq += 1
#             self.msg.pose.position.x = pose.pose.position.x
#             self.msg.pose.position.y = pose.pose.position.y
#             self.msg.pose.position.z = pose.pose.position.z
#             self.msg.pose.orientation.x = pose.pose.orientation.x
#             self.msg.pose.orientation.y = pose.pose.orientation.y
#             self.msg.pose.orientation.z = pose.pose.orientation.z
#             self.msg.pose.orientation.w = pose.pose.orientation.w

#             self.pub.publish(self.msg)

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     external_position = PublishExternalPosition()
#     external_position.run()
