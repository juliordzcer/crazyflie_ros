#!/usr/bin/env python3
import rospy
import csv
import os
from geometry_msgs.msg import Twist
from scipy import io

# Variables para almacenar los datos
data_list = []

def callback(data):
    # Extraer los datos Twist del mensaje recibido
    x = data.angular.x
    y = data.angular.y
    z = data.angular.z

    # Agregar los datos a la lista
    data_list.append([x, y, z])

# Inicializar el nodo ROS
rospy.init_node('guardar_datos_pose')

# Obtener la ruta completa al directorio "home"
home_dir = os.path.expanduser("~")

# Construir la ruta completa al archivo CSV en el directorio "home"
csv_path = os.path.join(home_dir, "datos_pose.csv")

# Construir la ruta completa al archivo .mat en el directorio "home"
mat_path = os.path.join(home_dir, "datos_pose.mat")

# Abrir el archivo CSV para escribir los datos
csv_file = open(csv_path, 'w')
csv_writer = csv.writer(csv_file)

# Escribir los encabezados en el archivo CSV
csv_writer.writerow(['Angular X', 'Angular Y', 'Angular Z'])

# Suscribirse al tópico '/crazyflie/pose'
rospy.Subscriber('/crazyflie/pose', Twist, callback)

# Ejecutar el bucle principal de ROS
rospy.spin()

# Cerrar el archivo CSV al finalizar
csv_file.close()

# Guardar los datos en un archivo .mat
data_dict = {'datos': data_list}
io.savemat(mat_path, data_dict)
