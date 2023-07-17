#!/usr/bin/env python3
import rospy
import os
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from scipy import io

import tf.transformations as t


class DataRecorder:
    def __init__(self):
        # Variables para almacenar los datos
        self.Quad1 = []
        self.Quad2 = []
        self.is_saving_data = False

        # Inicializar el nodo ROS
        rospy.init_node('guardar_datos')

        # Obtener el valor del rosparam
        self.ros_param1 = rospy.get_param('~n1', 0)
        self.ros_param2 = rospy.get_param('~n2', 0)
        
        self.suffix1 = ''
        if self.ros_param1 == 5:
            self.suffix1 = 'SMC'
        elif self.ros_param1 == 6:
            self.suffix1 = 'BC'
        elif self.ros_param1 == 7:
            self.suffix1 = 'TC'
        elif self.ros_param1 == 8:
            self.suffix1 = 'STA'
        elif self.ros_param1 == 9:
            self.suffix1 = 'NTSMC'
        elif self.ros_param1 == 10:
            self.suffix1 = 'STSMC'
        elif self.ros_param1 == 11:
            self.suffix1 = 'PID'

        self.suffix2 = ''
        if self.ros_param2 == 5:
            self.suffix2 = 'SMC'
        elif self.ros_param2 == 6:
            self.suffix2 = 'BC'
        elif self.ros_param2 == 7:
            self.suffix2 = 'TC'
        elif self.ros_param2 == 8:
            self.suffix2 = 'STA'
        elif self.ros_param2 == 9:
            self.suffix2 = 'NTSMC'
        elif self.ros_param2 == 10:
            self.suffix2 = 'STSMC'
        elif self.ros_param2 == 11:
            self.suffix2 = 'PID'


        self.base_name_1 = "Agente1_" + self.suffix1
        self.base_name_2 = "Agente2_" + self.suffix2

        self.contador = 0

        rospy.Subscriber('/crazyflie1/cmd_vel', Twist, self.CMD_VEL1)
        rospy.Subscriber('/crazyflie1/vrpn_client_node/crazyflie1/pose', PoseStamped, self.VRPNPOSE1)
        rospy.Subscriber('/crazyflie1/goal', PoseStamped, self.GOAL1)
        rospy.Subscriber('/crazyflie1/pose', PoseStamped, self.POSE1)
        rospy.Subscriber('/crazyflie1/goalacc', Twist, self.GOALACC1)

        rospy.Subscriber('/crazyflie2/cmd_vel', Twist, self.CMD_VEL2)
        rospy.Subscriber('/crazyflie2/vrpn_client_node/crazyflie2/pose', PoseStamped, self.VRPNPOSE2)
        rospy.Subscriber('/crazyflie2/goal', PoseStamped, self.GOAL2)
        rospy.Subscriber('/crazyflie2/pose', PoseStamped, self.POSE2)
        rospy.Subscriber('/crazyflie1/goalacc', Twist, self.GOALACC2)

    def CMD_VEL1(self, msg):
        self.u1 = msg.linear.z
        self.phid1 = msg.linear.y
        self.thetad1 = msg.linear.x

    def VRPNPOSE1(self, msg):
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psi1 = rpy[2]
        self.x1 = msg.pose.position.x
        self.y1 = msg.pose.position.y
        self.z1 = msg.pose.position.z

    def GOAL1(self, msg):
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psid1 = rpy[2]
        self.xd1 = msg.pose.position.x
        self.yd1 = msg.pose.position.y
        self.zd1 = msg.pose.position.z

    def POSE1(self, msg):
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.phi1 = rpy[0]
        self.theta1 = rpy[1]
        self.tau_phi1 = msg.pose.position.x
        self.tau_theta1 = msg.pose.position.y
        self.tau_psi1 = msg.pose.position.z

    def CMD_VEL2(self, msg):
        self.u2 = msg.linear.z
        self.phid2 = msg.linear.y
        self.thetad2 = msg.linear.x

    def VRPNPOSE2(self, msg):
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psi2 = rpy[2]
        self.x2 = msg.pose.position.x
        self.y2 = msg.pose.position.y
        self.z2 = msg.pose.position.z

    def GOAL2(self, msg):
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psid2 = rpy[2]
        self.xd2 = msg.pose.position.x
        self.yd2 = msg.pose.position.y
        self.zd2 = msg.pose.position.z

    def POSE2(self, msg):
        q = msg.pose.orientation
        rpy = t.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.phi2 = rpy[0]
        self.theta2 = rpy[1]
        self.tau_phi2 = msg.pose.position.x
        self.tau_theta2 = msg.pose.position.y
        self.tau_psi2 = msg.pose.position.z

    def GOALACC1(self, msg):
        angular_x = msg.angular.x
        if angular_x == 10 and not self.is_saving_data:
            self.is_saving_data = True
            rospy.loginfo("Comenzando a guardar los datos...")
        elif angular_x == 0 and self.is_saving_data:
            self.is_saving_data = False
            rospy.loginfo("Dejando de guardar los datos...")

        if self.is_saving_data:
            self.Quad1.append([self.x1, self.y1, self.z1, self.phi1, self.theta1, self.psi1, self.xd1, self.yd1, self.zd1, self.phid1, self.thetad1, self.psid1, self.u1, self.tau_phi1, self.tau_theta1, self.tau_psi1])

    def GOALACC2(self, msg):
        angular_x = msg.angular.x
        if angular_x == 10 and not self.is_saving_data:
            self.is_saving_data = True
        elif angular_x == 0 and self.is_saving_data:
            self.is_saving_data = False

        if self.is_saving_data:
            self.Quad2.append([self.x2, self.y2, self.z2, self.phi2, self.theta2, self.psi2, self.xd2, self.yd2, self.zd2, self.phid2, self.thetad2, self.psid2, self.u2, self.tau_phi2, self.tau_theta2, self.tau_psi2])

    def save_data(self):
        # Construir la ruta completa al directorio "home"
        home_dir = os.path.expanduser("~/Experimentos")

        # Crear la carpeta "Experimentos" si no existe
        if not os.path.exists(home_dir):
            os.makedirs(home_dir)

        # # Construir la ruta completa a la carpeta según self.suffix1 y self.suffix2
        # folder_path_1 = os.path.join(home_dir, self.suffix1)
        # folder_path_2 = os.path.join(home_dir, self.suffix2)

        # # Crear las carpetas si no existen
        # if not os.path.exists(folder_path_1):
        #     os.makedirs(folder_path_1)
        # if not os.path.exists(folder_path_2):
        #     os.makedirs(folder_path_2)

        # Construir la ruta completa al archivo .mat para Quad1
        mat_path_quad1 = os.path.join(home_dir, self.base_name_1 + ".mat")

        # # Verificar si el archivo .mat ya existe para Quad1
        # while os.path.exists(mat_path_quad1):
        #     self.contador += 1
        #     mat_path_quad1 = os.path.join(home_dir, self.base_name_1 + str(self.contador) + ".mat")

        # Guardar los datos de Quad1 en un archivo .mat
        data_dict_quad1 = {'Quad1_data': self.Quad1}
        io.savemat(mat_path_quad1, data_dict_quad1)

        # Construir la ruta completa al archivo .mat para Quad2
        mat_path_quad2 = os.path.join(home_dir, self.base_name_2 + ".mat")

        # # Verificar si el archivo .mat ya existe para Quad2
        # while os.path.exists(mat_path_quad2):
        #     self.contador += 1
        #     mat_path_quad2 = os.path.join(home_dir, self.base_name_2 + str(self.contador) + ".mat")

        # Guardar los datos de Quad2 en un archivo .mat
        data_dict_quad2 = {'Quad2_data': self.Quad2}
        io.savemat(mat_path_quad2, data_dict_quad2)


if __name__ == '__main__':
    recorder = DataRecorder()
    rospy.on_shutdown(recorder.save_data)
    rospy.spin()
