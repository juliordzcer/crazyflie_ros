#!/usr/bin/env python3

import rospy

def main():
    rospy.init_node('update_params_node', anonymous=True)

    # Obtener los valores de los parámetros desde ROS
    cf_ns = rospy.get_param('~frame', ' ')
    controller = rospy.get_param('~n', 2)

    # Esperar un poco para asegurarse de que el nodo de ROS se haya iniciado correctamente
    rospy.sleep(1)

    # Actualizar los parámetros en el espacio de nombres especificado
    rospy.set_param(cf_ns + "/commander/enHighLevel", 1)
    rospy.set_param(cf_ns + "/stabilizer/estimator", 2)
    rospy.set_param(cf_ns + "/stabilizer/controller", controller)
    rospy.set_param(cf_ns + "/locSrv/extPosStdDev", 1e-3)
    rospy.set_param(cf_ns + "/locSrv/extQuatStdDev", 0.5e-1)
    rospy.set_param(cf_ns + "/kalman/resetEstimation", 1)

    rospy.loginfo('Parámetros actualizados para el espacio de nombres: %s', cf_ns)

if __name__ == '__main__':
    main()