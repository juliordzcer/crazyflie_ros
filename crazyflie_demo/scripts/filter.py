#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist


class LowPassFilter:
    def __init__(self, frequency_cutoff, damping_ratio):
        self.frequency_cutoff = frequency_cutoff
        self.damping_ratio = damping_ratio
        self.initialized = False
        self.input_prev = 0
        self.input_prev2 = 0
        self.output_prev = 0
        self.output_prev2 = 0

        omega = 2 * math.pi * frequency_cutoff
        alpha = math.sin(omega) / (2 * damping_ratio)
        beta = math.cos(omega)

        self.b0 = (1 - beta) / 2
        self.b1 = 1 - beta
        self.b2 = (1 - beta) / 2
        self.a1 = -2 * alpha * beta
        self.a2 = alpha * alpha

    def filter(self, input):
        if not self.initialized:
            self.input_prev = input
            self.initialized = True

        output = self.b0 * input + self.b1 * self.input_prev + self.b2 * self.input_prev2 - self.a1 * self.output_prev - self.a2 * self.output_prev2

        self.input_prev2 = self.input_prev
        self.input_prev = input
        self.output_prev2 = self.output_prev
        self.output_prev = output

        return output


class VelFilterNode:
    def __init__(self):
        rospy.init_node('filter', anonymous=True)

        # Obtener el coeficiente de filtro de los parámetros del nodo
        self.damping_ratio = rospy.get_param('~damping_ratio', 0.7)
        self.frequency_cutoff = rospy.get_param('~frequency_cutoff', 10)

        # Crear los filtros de velocidad lineal y angular para x, y, z
        self.linear_vel_filter_x = LowPassFilter(self.frequency_cutoff, self.damping_ratio)
        self.linear_vel_filter_y = LowPassFilter(self.frequency_cutoff, self.damping_ratio)
        self.linear_vel_filter_z = LowPassFilter(self.frequency_cutoff, self.damping_ratio)
        self.angular_vel_filter_z = LowPassFilter(self.frequency_cutoff, self.damping_ratio)

        # Suscribirse al topic de \cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Publicar en el topic de \filtered_cmd_vel
        self.filtered_cmd_vel_pub = rospy.Publisher('filtered_cmd_vel', Twist, queue_size=1)

    def cmd_vel_callback(self, msg):
        # Filtrar la velocidad lineal y angular de la senal de entrada
        filtered_linear_vel_x = self.linear_vel_filter_x.filter(msg.linear.x)
        filtered_linear_vel_y = self.linear_vel_filter_y.filter(msg.linear.y)
        filtered_linear_vel_z = self.linear_vel_filter_z.filter(msg.linear.z)
        filtered_angular_vel_z = self.angular_vel_filter_z.filter(msg.angular.z)

        # Crear un nuevo mensaje de Twist con las velocidades filtradas
        filtered_cmd_vel = Twist()
        filtered_cmd_vel.linear.x = filtered_linear_vel_x
        filtered_cmd_vel.linear.y = filtered_linear_vel_y
        filtered_cmd_vel.linear.z = filtered_linear_vel_z
        filtered_cmd_vel.angular.z = filtered_angular_vel_z

        # Publicar la senal de \cmd_vel filtrada
        self.filtered_cmd_vel_pub.publish(filtered_cmd_vel)


if __name__ == '__main__':
    try:
        node = VelFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist

# class LowPassFilter:
#     def __init__(self, alpha):
#         self.alpha = alpha
#         self.initialized = False
#         self.prev_output = 0

#     def filter(self, input):
#         if not self.initialized:
#             self.prev_output = input
#             self.initialized = True

#         output = self.alpha * input + (1 - self.alpha) * self.prev_output
#         self.prev_output = output

#         return output

# class VelFilterNode:
#     def __init__(self):
#         rospy.init_node('filter', anonymous=True)

#         # Obtener el coeficiente de filtro de los parámetros del nodo
#         self.filter_coeff = rospy.get_param('~filter_coeff', 0.7)

#         # Crear los filtros de velocidad lineal y angular para x, y, z
#         self.linear_vel_filter_x = LowPassFilter(self.filter_coeff)
#         self.linear_vel_filter_y = LowPassFilter(self.filter_coeff)
#         self.linear_vel_filter_z = LowPassFilter(self.filter_coeff)
#         self.angular_vel_filter_z = LowPassFilter(self.filter_coeff)

#         # Suscribirse al topic de \cmd_vel
#         self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

#         # Publicar en el topic de \filtered_cmd_vel
#         self.filtered_cmd_vel_pub = rospy.Publisher('filtered_cmd_vel', Twist, queue_size=1)

#     def cmd_vel_callback(self, msg):
#         # Filtrar la velocidad lineal y angular de la senal de entrada
#         filtered_linear_vel_x = self.linear_vel_filter_x.filter(msg.linear.x)
#         filtered_linear_vel_y = self.linear_vel_filter_y.filter(msg.linear.y)
#         filtered_linear_vel_z = self.linear_vel_filter_z.filter(msg.linear.z)
#         filtered_angular_vel_z = self.angular_vel_filter_z.filter(msg.angular.z)

#         # Crear un nuevo mensaje de Twist con las velocidades filtradas
#         filtered_cmd_vel = Twist()
#         filtered_cmd_vel.linear.x = filtered_linear_vel_x
#         filtered_cmd_vel.linear.y = filtered_linear_vel_y
#         filtered_cmd_vel.linear.z = filtered_linear_vel_z
#         filtered_cmd_vel.angular.z = filtered_angular_vel_z

#         # Publicar la senal de \cmd_vel filtrada
#         self.filtered_cmd_vel_pub.publish(filtered_cmd_vel)

# if __name__ == '__main__':
#     try:
#         node = VelFilterNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
