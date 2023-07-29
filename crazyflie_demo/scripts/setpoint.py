#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from crazyflie_driver.msg import Full
from scipy import io
import os

class TrajectoryCircle:
    def __init__(self):
        self.r = rospy.get_param("~r", 0.0)  # radio
        self.h = rospy.get_param("~h", 0.0)  # altura
        self.t = 0.0
        self.rt = 100.0
        self.button_pressed = False
        self.start_time = rospy.Time()
        self.data_to_save = []
        self.xi = rospy.get_param("~xi", 0.0)
        self.yi = rospy.get_param("~yi", 0.0)
        self.zi = rospy.get_param("~zi", 0.0)
        self.pub = rospy.Publisher('cmd_full', Full, queue_size=1)
        self.start_pub = rospy.Publisher('/start', Twist, queue_size=1)  
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[2] == 1 and not self.button_pressed:
            rospy.loginfo('Trayectoria iniciada')
            self.button_pressed = True
            self.t = 0.0
            self.start_time = rospy.Time.now()
            self.data_to_save = []
        elif joy_msg.buttons[10] == 1 and self.button_pressed:
            rospy.loginfo('Trayectoria reiniciada')
            self.r = 0.0
            self.h = 0.0
            self.t = 0.0
            self.button_pressed = False
            self.start_time = rospy.Time()
            self.publish_start_msg(0) 

    def publish_start_msg(self, angular_x):
        start_msg = Twist()
        start_msg.angular.x = angular_x
        self.start_pub.publish(start_msg)

    def trajectory_circle(self):
        rospy.sleep(1)  
        rate = rospy.Rate(self.rt)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if self.button_pressed:
                if not self.start_time.is_zero():
                    elapsed_time = (current_time - self.start_time).to_sec()
                    rospy.loginfo('Tiempo: ' + str(elapsed_time))
                    if elapsed_time >= 60:
                        self.publish_start_msg(0)
                        break

                full_msg = Full()
                full_msg.header = Header()
                full_msg.header.stamp = rospy.Time.now()

                t = elapsed_time
                w = np.pi / 6
                p = 15

                x = self.r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) + self.xi
                y =  self.r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t) + self.yi
                z = (self.h/2) * (1 + np.tanh(t-2.5)) + self.zi
                yaw = 0

                dx = -self.r * (np.arctan(p) + np.arctan(t - p)) * w * np.sin(w * t)
                dy = self.r * (np.arctan(p) + np.arctan(t - p)) * w * np.cos(w * t)
                dz = (self.h/2) * np.tanh(t-2.5) * (1 - np.tanh(t-2.5))
                dyaw = 0

                ddx = -self.r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.cos(w * t)
                ddy = -self.r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.sin(w * t)
                ddz = (self.h/2) * np.tanh(t-2.5) * (1 - np.tanh(t-2.5)) * (1 - 2*np.tanh(t-2.5)) 
                ddyaw = 0

                full_msg.twist1 = Twist()
                full_msg.twist1.linear.x = x
                full_msg.twist1.linear.y = y
                full_msg.twist1.linear.z = z
                full_msg.twist1.angular.x = dx
                full_msg.twist1.angular.y = dy
                full_msg.twist1.angular.z = dz

                full_msg.twist2 = Twist()
                full_msg.twist2.linear.x = ddx
                full_msg.twist2.linear.y = ddy
                full_msg.twist2.linear.z = ddz
                full_msg.twist2.angular.x = yaw
                full_msg.twist2.angular.y = dyaw
                full_msg.twist2.angular.z = ddyaw

                self.publish_start_msg(10)
                self.data_to_save.append([x - self.xi, y - self.yi, z - self.zi, yaw])
                self.pub.publish(full_msg)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('trajectory_circle', anonymous=True)
    trajectory = TrajectoryCircle()
    trajectory.trajectory_circle()

    experiments_dir = os.path.expanduser('~/Experimentos')
    if not os.path.exists(experiments_dir):
        os.makedirs(experiments_dir)

    file_path = os.path.join(experiments_dir, 'trajectory_data.mat')
    io.savemat(file_path, {'data': np.array(trajectory.data_to_save)})

    rospy.spin()


# #!/usr/bin/env python3
# import numpy as np
# import rospy
# import tf
# from std_msgs.msg import Header, Bool
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
# from crazyflie_driver.msg import Full
# from scipy import io
# import os

# r = 0.0
# h = 0.0
# t = 0.0
# rt = 10.0
# gr = 0

# button_pressed = False
# start_time = rospy.Time()

# # List to store data for saving in .mat file
# data_to_save = []

# def joy_callback(joy_msg):
#     global r, h, t, button_pressed, gr, start_time

#     if joy_msg.buttons[2] == 1 and not button_pressed:
#         rospy.loginfo('Trayectoria iniciada')
#         button_pressed = True
#         r = radio  # 0.16
#         h = altura  # 0.4
#         t = 0.0
#         gr = 10
#         start_time = rospy.Time.now()  # Store the starting time

#         # Clear the data list when starting a new trajectory
#         global data_to_save
#         data_to_save = []

#     elif joy_msg.buttons[10] == 1 and button_pressed:
#         rospy.loginfo('Trayectoria reiniciada')
#         r = 0.0
#         h = 0.0
#         t = 0.0
#         gr = 0
#         button_pressed = False
#         start_time = rospy.Time()  # Reset the starting time
#         start_msg = Twist()
#         start_msg.angular.x= 0
#         start_pub.publish(start_msg) 

# def trajectory_circle():
#     global pub, r, h, t, rt, xi, yi, zi, button_pressed, data_to_save

#     rospy.sleep(1)  # Wait 1 second before starting the trajectory
    
#     rate = rospy.Rate(rt)  # rt Hz

#     while not rospy.is_shutdown():
#         current_time = rospy.Time.now()

#         if button_pressed:  # Verificar si la trayectoria ha sido iniciada
#             if not start_time.is_zero():
#                 elapsed_time = (current_time - start_time).to_sec()
#                 rospy.loginfo('Tiempo: ' + str(elapsed_time))
#                 if elapsed_time >= 60:
#                     start_msg = Twist()
#                     start_msg.angular.x = 0
#                     start_pub.publish(start_msg) 
#                     break

#             full_msg = Full()
#             full_msg.header = Header()
#             full_msg.header.stamp = rospy.Time.now()

#             t = elapsed_time
#             w = np.pi / 6
#             p = 15

#             x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) + xi
#             y =  r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t) + yi
#             z = (h/2) * (1 + np.tanh(t-2.5)) + zi
#             yaw = 0

#             dx = -r * (np.arctan(p) + np.arctan(t - p)) * w * np.sin(w * t)
#             dy = r * (np.arctan(p) + np.arctan(t - p)) * w * np.cos(w * t)
#             dz = (h/2) * np.tanh(t-2.5) * (1 - np.tanh(t-2.5))
#             dyaw = 0

#             ddx = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.cos(w * t)
#             ddy = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.sin(w * t)
#             ddz = (h/2) * np.tanh(t-2.5) * (1 - np.tanh(t-2.5)) * (1 - 2*np.tanh(t-2.5)) 
#             ddyaw = 0

#             full_msg.twist1 = Twist()
#             full_msg.twist1.linear.x = x
#             full_msg.twist1.linear.y = y
#             full_msg.twist1.linear.z = z
#             full_msg.twist1.angular.x = dx
#             full_msg.twist1.angular.y = dy
#             full_msg.twist1.angular.z = dz

#             full_msg.twist2 = Twist()
#             full_msg.twist2.linear.x = ddx
#             full_msg.twist2.linear.y = ddy
#             full_msg.twist2.linear.z = ddz
#             full_msg.twist2.angular.x = yaw
#             full_msg.twist2.angular.y = dyaw
#             full_msg.twist2.angular.z = ddyaw

#             start_msg = Twist()
#             start_msg.angular.x = 10
#             start_pub.publish(start_msg)
            
#             data_to_save.append([x - xi, y - yi, z - zi, yaw])

#             pub.publish(full_msg)

#         rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node('trajectory_circle', anonymous=True)
#     xi = rospy.get_param("~xi", 0.0)
#     yi = rospy.get_param("~yi", 0.0)
#     zi = rospy.get_param("~zi", 0.0)

#     altura = rospy.get_param("~h", 0.0)
#     radio = rospy.get_param("~r", 0.0)

#     pub = rospy.Publisher('cmd_full', Full, queue_size=1)
#     start_pub = rospy.Publisher('/start', Twist, queue_size=1)  # Add the boolean publisher

#     joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

#     trajectory_circle()

#     # Save the data to the Experimentos directory
#     experiments_dir = os.path.expanduser('~/Experimentos')
#     if not os.path.exists(experiments_dir):
#         os.makedirs(experiments_dir)

#     file_path = os.path.join(experiments_dir, 'trajectory_data.mat')
#     io.savemat(file_path, {'data': np.array(data_to_save)})

#     rospy.spin()