#!/usr/bin/env python3

import rospy
import math as m
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.srv import UpdateParams
import tf.transformations as t

class ExternalPosition:
    def __init__(self):
        topic = rospy.get_param("~topic", "vrpn_client_node/crazyflie/pose")
        self.yaw_sub = rospy.Subscriber(topic, PoseStamped, self.Newtransform)
        self.extpos_pub = rospy.Publisher('external_position', PoseStamped, queue_size=50)
        self.num_transforms_received = 0
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', UpdateParams)
        self.x = 0
        self.y = 0
        self.z = 0

    def Newtransform(self, msg):
        self.num_transforms_received += 1

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        if self.num_transforms_received == 1:
            # initialize kalman filter
            rospy.set_param("kalman/initialX", self.x)
            rospy.set_param("kalman/initialY", self.y)
            rospy.set_param("kalman/initialZ", self.z)
            self.update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])
            rospy.set_param("kalman/resetEstimation", 1)
            rospy.sleep(2) 
            self.update_params(["kalman/resetEstimation"]) 
            rospy.set_param("locSrv/extPosStdDev", 1e-3)
            # rospy.set_param("locSrv/extQuatStdDev", 0.5e-1)
            self.update_params(["locSrv/extPosStdDev"])



            


    def PubPosition(self):
        Position = PoseStamped()
        Position.pose.position.x = self.x
        Position.pose.position.y = self.y
        Position.pose.position.z = self.z
        self.extpos_pub.publish(Position)

if __name__ == '__main__':
    rospy.init_node('extpos')
    rospy.sleep(12)  # espera 20 segundos antes de iniciar
    ext_pos = ExternalPosition()

    # Mantener el nodo en ejecución
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        ext_pos.PubPosition()
        rate.sleep()








# #!/usr/bin/env python3

# import rospy
# import tf
# from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped #PoseStamped added to support vrpn_client
# from crazyflie_driver.srv import UpdateParams

# def onNewTransform(self, pose):
#     global firstTransform

#     if firstTransform:
#         # initialize kalman filter
#         rospy.set_param("kalman/initialX", pose.pose.position.x)
#         rospy.set_param("kalman/initialY", pose.pose.position.y)
#         rospy.set_param("kalman/initialZ", pose.pose.position.z)
#         update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

#         rospy.set_param("kalman/resetEstimation", 1)
#         update_params(["kalman/resetEstimation"]) 
#         firstTransform = False


# if __name__ == '__main__':
#     rospy.init_node('publish_external_position', anonymous=True)
#     topic = rospy.get_param("~topic", "vrpn_client_node/crazyflie/pose")

#     rospy.wait_for_service('update_params')
#     rospy.loginfo("found update_params service")
#     update_params = rospy.ServiceProxy('update_params', UpdateParams)

#     firstTransform = True

#     rospy.Subscriber(topic, PoseStamped, onNewTransform)

#     rospy.spin()