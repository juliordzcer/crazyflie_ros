#!/usr/bin/env python3

import rospy
from crazyflie_driver.msg import GenericLogData
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

def log_callback(data):
    # Este callback se llama cada vez que recibimos un mensaje del Crazyflie
    rospy.loginfo("Received message from log block {}: {}".format(data.log_id, data.values))

def main():
    # Obtenemos la URI del Crazyflie desde los argumentos del nodo
    uri = rospy.get_param('~uri', uri_helper.uri_from_env())

    # Creamos una instancia de la clase Crazyflie
    cf = Crazyflie()

    # Nos conectamos al Crazyflie
    cf.connected.add_callback(lambda uri: rospy.loginfo('Crazyflie connected on {}'.format(uri)))
    cf.disconnected.add_callback(lambda uri: rospy.loginfo('Crazyflie disconnected from {}'.format(uri)))
    cf.connection_failed.add_callback(lambda uri, msg: rospy.loginfo('Connection to {} failed: {}'.format(uri, msg)))
    cf.connection_lost.add_callback(lambda uri, msg: rospy.loginfo('Connection to {} lost: {}'.format(uri, msg)))
    cf.open_link(uri)

    # Nos suscribimos al mensaje que envía el Crazyflie
    rospy.Subscriber('crazyflie_generic_log', GenericLogData, log_callback)

    # Configuramos el rate del bucle principal
    rate = rospy.Rate(0.5)

    # Bucle principal
    while not rospy.is_shutdown() and cf.is_connected():
        # Enviamos un mensaje al Crazyflie
        cf.param.set_value('test_param', 'Hello World')

        # Esperamos dos segundos
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('crazyflie_node')
    main()
