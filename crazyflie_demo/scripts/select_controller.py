#!/usr/bin/env python3

import rospy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class BooleanSubscriber:
    def __init__(self):
        rospy.init_node("controllerselect")
        rospy.Subscriber("boolean_topic", Bool, self.callback)
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
        self.info_enviada = False  # Bandera para controlar el envío de información
        self.servicio_utilizado = False  # Bandera para controlar si el servicio se ha utilizado

    def callback(self, data):
        if data.data and not self.info_enviada:  # Si el valor recibido es True y la información no ha sido enviada
            n = rospy.get_param("~n", 1)
            rospy.set_param("stabilizer/controller", n)
            # rospy.set_param("stabilizer/controller", 1)
            self._update_params(["stabilizer/controller"])
            self.info_enviada = True  # Actualizar la bandera indicando que la información ha sido enviada
        elif not data.data and not self.servicio_utilizado:  # Si el valor recibido es False y el servicio no se ha utilizado
            n = rospy.get_param("~n", 1)
            rospy.set_param("stabilizer/controller", n)
            self._update_params(["stabilizer/controller"])
            self.servicio_utilizado = True  # Actualizar la bandera indicando que el servicio se ha utilizado

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    boolean_subscriber = BooleanSubscriber()
    boolean_subscriber.run()
