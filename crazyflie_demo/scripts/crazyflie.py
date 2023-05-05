#!/usr/bin/env python3

import rospy
import numpy as np
from crazyflie.srv import *
from crazyflie.msg import *

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])

class Crazyflie:
    def __init__(self, prefix, tf):
        self.prefix = prefix
        self.tf = tf

        rospy.wait_for_service(prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.landService = rospy.ServiceProxy(prefix + "/land", Land)
        rospy.wait_for_service(prefix + "/stop")
        self.stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        rospy.wait_for_service(prefix + "/go_to")
        self.goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)
        rospy.wait_for_service(prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)

    def takeoff(self, targetHeight, duration):
        self.takeoffService(targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration):
        self.landService(targetHeight, rospy.Duration.from_sec(duration))

    def stop(self):
        self.stopService()

    def goTo(self, goal, yaw, duration, relative = False):
        gp = arrayToGeometryPoint(goal)
        self.goToService(gp, yaw, rospy.Duration.from_sec(duration), relative)

    def position(self):
        self.tf.waitForTransform("world", "/crazyflie" + str(self.id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/crazyflie" + str(self.id), rospy.Time(0))
        return np.array(position)

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())
