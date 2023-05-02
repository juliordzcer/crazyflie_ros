#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from math import cos, sin, pi

class TurtleSimController:
    def __init__(self):
        # Inicializar el nodo ROS y el objeto Twist
        rospy.init_node('turtle_sim_controller')
        self.twist = Twist()

        # Suscribirse al topic "pose" de TurtleSim para conocer la posición y orientación actual
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        # Inicializar el servicio "teleport_absolute" de TurtleSim para poder mover el objeto Turtle
        rospy.wait_for_service('/turtle1/teleport_absolute')
        self.teleport_absolute = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

        # Crear un publisher para el topic "/turtle1/cmd_vel"
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Definir los parámetros de la trayectoria
        self.radius = 1
        self.angle = 0
        self.angular_velocity = 0.1
        self.linear_velocity = 0.5

    def update_pose(self, pose):
        # Guardar la posición y orientación actual del objeto Turtle
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta

    def move_turtle(self):
        # Calcular la siguiente posición y orientación del objeto Turtle
        self.x += self.radius * cos(self.angle)
        self.y += self.radius * sin(self.angle)
        self.theta += self.angular_velocity

        # Ajustar la orientación para que esté en el rango de [0, 2*pi)
        self.theta %= 2 * pi

        # Crear un Vector3 con las coordenadas x, y, z de la nueva posición
        position = Vector3(self.x, self.y, 0)

        # Crear un Vector3 con los ángulos de euler (yaw, pitch, roll) de la nueva orientación
        orientation = Vector3(0, 0, self.theta)

        # Llamar al servicio "teleport_absolute" para mover el objeto Turtle a la nueva posición y orientación
        self.teleport_absolute(position, orientation)

        # Configurar el objeto Twist con la velocidad lineal y angular deseada
        self.twist.linear.x = self.linear_velocity
        self.twist.angular.z = self.angular_velocity

        # Publicar el objeto Twist en el topic "/turtle1/cmd_vel"
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        # Iniciar un loop para mover el objeto Turtle continuamente
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.move_turtle()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtleSimController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
