#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def crazyflie_publisher():
    # Inicializar el nodo ROS
    rospy.init_node('crazyflie_publisher', anonymous=True)

    # Crear un objeto para publicar en el topic /crazyflie1/cmd_vel
    pub = rospy.Publisher('/crazyflie1/cmd_vel', Twist, queue_size=10)

    # Crear un mensaje de tipo Twist para publicar la velocidad lineal
    twist_msg = Twist()
    twist_msg.linear.z = 25000.0

    # Establecer la frecuencia del bucle del publisher (0.001 segundos = 1000 Hz)
    rate = rospy.Rate(1000)

    # Duración del tiempo de publicación (10 segundos)
    duration = 10.0

    # Tiempo inicial
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        # Publicar el mensaje
        pub.publish(twist_msg)

        # Comprobar si ha pasado el tiempo de publicación
        if rospy.get_time() - start_time >= duration:
            break

        # Dormir el tiempo restante para mantener la frecuencia especificada
        rate.sleep()

if __name__ == '__main__':
    try:
        crazyflie_publisher()
    except rospy.ROSInterruptException:
        pass