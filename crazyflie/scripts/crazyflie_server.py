#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from crazyflie.srv import *
from crazyflie.msg import *

import numpy as np

# Importando las librerias de python de bitcraze.
import cflib
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from threading import Thread


class CrazyflieROS:
    Disconnected = 0
    Connecting = 1
    Connected = 2

    def __init__(self, link_uri, tf_prefix, roll_trim, pitch_trim ,enable_logging_pose):
        self.link_uri = link_uri
        self.tf_prefix = tf_prefix
        self.roll_trim = roll_trim
        self.pitch_trim = pitch_trim
        self.enable_logging_pose = enable_logging_pose
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.link_quality_updated.add_callback(self._link_quality_updated)

        # Publishers.
        self._subCmdVel = rospy.Subscriber(tf_prefix + "/cmd_vel", Twist, self._cmdVelChanged)
        self._subExtPose = rospy.Subscriber(tf_prefix + "/ExtPose", PoseStamped, self._ExtPoseChanged)
        self._subCmdHover = rospy.Subscriber(tf_prefix + "/cmd_hover", Hover, self._cmd_hover_Changed)
        self._subCmdPosition = rospy.Subscriber(tf_prefix + "/cmd_position", Position, self._cmd_position_Changed)
        self._subCmdFollower = rospy.Subscriber(tf_prefix + "/cmd_follower", Follower, self._cmd_follower_Changed)
        self._pubPose = rospy.Publisher(tf_prefix + "/pose", Twist, queue_size=10)

        self._state = CrazyflieROS.Disconnected

        # Services
        rospy.Service(tf_prefix + "/update_params", UpdateParams, self._update_params)
        rospy.Service(tf_prefix + "/emergency", Empty, self._emergency)
        rospy.Service(tf_prefix + "/takeoff", Takeoff, self._takeoff)
        rospy.Service(tf_prefix + "/land", Land, self._land)
        rospy.Service(tf_prefix + "/stop", Stop, self._stop)
        rospy.Service(tf_prefix + "/go_to", GoTo, self._goTo)

        self._isEmergency = False

        Thread(target=self._update).start()

    def _try_to_connect(self):
        rospy.loginfo("Connecting to %s" % self.link_uri)
        self._state = CrazyflieROS.Connecting
        self._cf.open_link(self.link_uri)

    def _connected(self, link_uri):

        rospy.loginfo("Connected to %s" % link_uri)
        self._state = CrazyflieROS.Connected


        if self.enable_logging_pose:
            self._lg_pose = LogConfig(name="Pose", period_in_ms=10)

            # Posicion 
            self._lg_pose.add_variable("stateEstimate.x", "float")
            self._lg_pose.add_variable("stateEstimate.y", "float")
            self._lg_pose.add_variable("stateEstimate.z", "float")
            
            # Orientacion en quaterniones
            self._lg_pose.add_variable('stabilizer.roll', 'float')
            self._lg_pose.add_variable('stabilizer.pitch', 'float')
            self._lg_pose.add_variable('stabilizer.yaw', 'float')
            
            try:
                self._cf.log.add_config(self._lg_pose)
                # This callback will receive the data
                self._lg_pose.data_received_cb.add_callback(self._log_data_Pose)
                # This callback will be called on errors
                self._lg_pose.error_cb.add_callback(self._log_error)
                # Start the logging
                self._lg_pose.start()
            except KeyError as e:
                rospy.logwarn('Could not start log configuration,'
                    '{} not found in TOC'.format(str(e)))

                print()
            except AttributeError:
                rospy.logfatal("Could not add logconfig since some variables are not in TOC")


        p_toc = self._cf.param.toc.toc
        for group in p_toc.keys():
            self._cf.param.add_update_callback(group=group, name=None, cb=self._param_callback)
            for name in p_toc[group].keys():
                ros_param = "/{}/{}/{}".format(self.tf_prefix, group, name)
                cf_param = "{}.{}".format(group, name)
                if rospy.has_param(ros_param):
                    self._cf.param.set_value(cf_param, rospy.get_param(ros_param))
                else:
                    self._cf.param.request_param_update(cf_param)


    def _connection_failed(self, link_uri, msg):
        rospy.logfatal("Connection to %s failed: %s" % (link_uri, msg))
        self._cf.close_link()
        self._state = CrazyflieROS.Disconnected

    def _connection_lost(self, link_uri, msg):
        rospy.logfatal("Connection to %s lost: %s" % (link_uri, msg))
        self._state = CrazyflieROS.Disconnected

    def _disconnected(self, link_uri):
        rospy.logfatal("Disconnected from %s" % link_uri)
        self._state = CrazyflieROS.Disconnected

    def _link_quality_updated(self, percentage):
        if percentage < 80:
            rospy.logwarn("Connection quality is: %f" % (percentage))

    def _log_error(self, logconf, msg):
        rospy.logfatal("Error when logging %s: %s" % (logconf.name, msg))


    def _log_data_Pose(self, timestamp, data, logconf):
        msg = Twist()
        msg.linear.x = data["stateEstimate.x"]
        msg.linear.y = data["stateEstimate.y"]
        msg.linear.z = data["stateEstimate.z"]
        
        # measured in quaternions
        msg.angular.x = (data['stabilizer.roll'])
        msg.angular.y = (data["stabilizer.pitch"])
        msg.angular.z = (data["stabilizer.yaw"])

        self._pubPose.publish(msg)

    def _param_callback(self, name, value):
        ros_param = "{}/{}".format(self.tf_prefix, name.replace(".", "/"))
        rospy.set_param(ros_param, value)

    def _update_params(self, req):
        rospy.loginfo("Update parameters %s" % (str(req.params)))
        for param in req.params:
            ros_param = "/{}/{}".format(self.tf_prefix, param)
            cf_param = param.replace("/", ".")
            print(cf_param)
            #if rospy.has_param(ros_param):
            self._cf.param.set_value(cf_param, str(rospy.get_param(ros_param)))
        return UpdateParamsResponse()
    
    def _takeoff(self, req):
        rospy.loginfo("Takeoff requested")
        height = req.height
        duration = req.duration.to_sec()
        self._cf.high_level_commander.takeoff(height, duration)
        return TakeoffResponse()  
    
    def _land(self, req):
        rospy.loginfo(self.tf_prefix + "Land requested")
        height = req.height
        duration = req.duration.to_sec()
        self._cf.high_level_commander.land(height, duration)
        return LandResponse()
    
    def _stop(self):
        rospy.loginfo(self.tf_prefix + "Stop requested")
        self._cf.high_level_commander.stop()
        return StopResponse()

    def _goTo(self, req):
        rospy.loginfo(self._tf_prefix + " GoTo requested")
        goal = req.goal
        yaw = req.yaw
        duration = req.duration.to_sec()
        relative = req.relative
        self._cf.high_level_commander.go_to(goal.x, goal.y, goal.z, yaw, duration, relative)
        return GoToResponse() 

    def _emergency(self, req):
        rospy.logfatal("Emergency requested!")
        self._isEmergency = True
        self._cf.loc.send_emergency_stop()
        return EmptyResponse()

    def _cmdVelChanged(self, msg):
        roll    = msg.linear.y + self.roll_trim
        pitch   = msg.linear.x + self.pitch_trim
        yawrate = msg.angular.z
        thrust  = min(max(0, int(msg.linear.z)), 60000)
        self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def _ExtPoseChanged(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        quat = msg.pose.orientation
        if isnan(quat.x):
            self._cf.extpos.send_extpos(x, y, z)
        else:
            self._cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)

    def _cmd_hover_Changed(self, msg):
        vx = msg.vx
        vy = msg.vy
        z = msg.z_distance
        yawrate = msg.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yawrate, z)

    def _cmd_position_Changed(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        yaw = msg.yaw
        self._cf.commander.send_position_setpoint(x, y, z, yaw)

    def _cmd_follower_Changed(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        yaw = msg.yaw
        u_l = msg.u_l
        roll_l = msg.roll_l
        pitch_l = msg.pitch_l
        yaw_l = msg.yaw_l
        self._cf.commander.send_follower_setpoint(x, y, z, yaw, u_l, roll_l, pitch_l, yaw_l)


    def _update(self):
        while not rospy.is_shutdown():
            if self._isEmergency:
                break
            if self._state == CrazyflieROS.Disconnected:
                self._try_to_connect()
            elif self._state == CrazyflieROS.Connected:
                rospy.sleep(0.2)
            else:
                rospy.sleep(0.5)
        rospy.sleep(0.1)
        self._cf.close_link()



def add_crazyflie(req):
    rospy.loginfo("Adding %s as %s with trim(%f, %f). Logging_pose: %s" % (req.uri, req.tf_prefix, req.roll_trim, req.pitch_trim, str(req.enable_logging_pose)))
    CrazyflieROS(req.uri, req.tf_prefix, req.roll_trim, req.pitch_trim, req.enable_logging_pose)
    return AddCrazyflieResponse()

if __name__ == '__main__':
    rospy.init_node('crazyflie_server')

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers()

    rospy.Service("add_crazyflie", AddCrazyflie, add_crazyflie)
    rospy.spin()

