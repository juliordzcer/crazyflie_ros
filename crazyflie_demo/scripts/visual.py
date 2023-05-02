#!/usr/bin/env python3

import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from geometry_msgs.msg import Twist


class RosGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Crear la ventana principal
        self.setWindowTitle('Gráfica 3D de Twist')
        self.setGeometry(100, 100, 800, 600)
        self.canvas = Canvas(self, width=5, height=4)
        self.setCentralWidget(self.canvas)


class Canvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111, projection='3d')
        super().__init__(fig)

        # Inicializar las variables para los datos de la gráfica
        self.xdata = []
        self.ydata = []
        self.zdata = []

        # Suscribirse al topic de ROS "cmd_vel"
        rospy.Subscriber('/turtle1/cmd_vel', Twist, self.update_plot)

    def update_plot(self, twist):
        # Obtener los datos de velocidad lineal y angular del Twist
        x = twist.linear.x
        y = twist.linear.y
        z = twist.linear.z
        rx = twist.angular.x
        ry = twist.angular.y
        rz = twist.angular.z

        # Agregar los datos a las listas de datos
        self.xdata.append(x)
        self.ydata.append(y)
        self.zdata.append(z)

        # Limitar el número de datos a mostrar en la gráfica para evitar ralentizar la aplicación
        max_data_points = 1000
        if len(self.xdata) > max_data_points:
            self.xdata = self.xdata[-max_data_points:]
            self.ydata = self.ydata[-max_data_points:]
            self.zdata = self.zdata[-max_data_points:]

        # Actualizar la gráfica
        self.axes.cla()
        self.axes.set_xlabel('X')
        self.axes.set_ylabel('Y')
        self.axes.set_zlabel('Z')
        self.axes.scatter(self.xdata, self.ydata, self.zdata, c=np.arange(len(self.xdata)), cmap='plasma')
        self.draw()


if __name__ == '__main__':
    # Iniciar un nodo de ROS llamado "ros_gui"
    rospy.init_node('ros_gui')

    # Iniciar la aplicación PyQt5
    app = QApplication([])
    gui = RosGUI()
    gui.show()

    # Ejecutar la aplicación
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    app.exec_()



# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float32, Bool
# from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QSlider, QCheckBox, QVBoxLayout,QPushButton
# from PyQt5.QtCore import Qt


# class ROSParamGUI(QWidget):
#     def __init__(self, param_name, toggle_name):
#         super().__init__()

#         # Crea un nuevo publisher que publique en el parámetro especificado
#         self.pub = rospy.Publisher(param_name, Float32, queue_size=10)

#         # Crea un nuevo publisher que publique en el toggle especificado
#         self.toggle_pub = rospy.Publisher(toggle_name, Bool, queue_size=10)

#         # Crea una etiqueta que muestre el nombre del parámetro
#         self.label = QLabel(param_name, self)

#         # Crea un slider para cambiar el valor del parámetro
#         self.slider = QSlider(Qt.Horizontal, self)
#         self.slider.setMinimum(0)
#         self.slider.setMaximum(100)
#         self.slider.setSingleStep(1)
#         self.slider.setValue(50)

#         # Conecta la señal de cambio del slider con el método de actualización del parámetro
#         self.slider.valueChanged.connect(self.update_param)

#         # Crea un checkbox para cambiar el valor del toggle
#         self.checkbox = QCheckBox("Toggle", self)

#         # Conecta la señal de cambio del checkbox con el método de actualización del toggle
#         self.checkbox.stateChanged.connect(self.update_toggle)

#         # Crea un diseño vertical para la ventana y agrega los widgets
#         layout = QVBoxLayout()
#         layout.addWidget(self.label)
#         layout.addWidget(self.slider)
#         layout.addWidget(self.checkbox)
#         self.setLayout(layout)

#         # Crear un botón
#         self.button = QPushButton('Iniciar', self)
#         self.button.clicked.connect(self.on_button_clicked)

#     def update_param(self, value):
#         # Actualiza el valor del parámetro y publícalo
#         self.pub.publish(Float32(float(value) / 100))

#     def update_toggle(self, state):
#         # Actualiza el valor del toggle y publícalo
#         self.toggle_pub.publish(Bool(bool(state)))

#     def on_button_clicked(self):
#         # Manejar el evento de clic del botón
#         rospy.set_param('iniciar', True)


# if __name__ == '__main__':
#     # Inicializa el nodo de ROS
#     rospy.init_node('ros_param_gui')

#     # Crea una nueva aplicación PyQt
#     app = QApplication([])

#     # Crea una nueva ventana con el nombre del parámetro y el toggle que deseas cambiar
#     gui = ROSParamGUI('/my_param', '/my_toggle', '/Iniciar')

#     # Muestra la ventana y ejecuta la aplicación
#     gui.show()
#     app.exec_()
