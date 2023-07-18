#!/usr/bin/env python3
from std_msgs.msg import Bool

import rospy
import numpy as np
from crazyflie_driver.msg import Position
from sensor_msgs.msg import Joy
import tf

r = 0.0
h = 0.0
t = 0.0
rt = 100
gr = 0

button_pressed = False
start_time = rospy.Time()

def joy_callback(joy_msg):
    global r, h, t, button_pressed, gr, start_time

    if joy_msg.buttons[2] == 1 and not button_pressed:
        rospy.loginfo('Trayectoria iniciada')
        button_pressed = True
        r = radio  # 0.16
        h = altura  # 0.4
        t = 0.0
        gr = 10
        start_time = rospy.Time.now()  # Store the starting time
        boolean_pub.publish(True)  # Publish True to boolean_topic
    elif joy_msg.buttons[10] == 1 and button_pressed:
        rospy.loginfo('Trayectoria reiniciada')
        r = 0.0
        h = 0.0
        t = 0.0
        gr = 0
        button_pressed = False
        start_time = rospy.Time()  # Reset the starting time
        boolean_pub.publish(False)  # Publish False to boolean_topic

def trajectory_circle():
    global pub, r, h, t, rt, xi, yi, zi, button_pressed

    rospy.sleep(1)  # Wait 1 second before starting the trajectory
    
    rate = rospy.Rate(rt)  # rt Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        if button_pressed:  # Verificar si la trayectoria ha sido iniciada
            if not start_time.is_zero():
                elapsed_time = (current_time - start_time).to_sec()
                rospy.loginfo('Tiempo: ' + str(elapsed_time))
                if elapsed_time >= 60:
                    # pose.x = xi
                    # pose.y = yi   
                    pose.z = max(zi * np.exp(-0.1 * (elapsed_time - 55)), 0)
                    pose.yaw = yaw
                    pub.publish(pose)
                    break

            pose = Position()
            pose.header.seq = 0
            pose.header.frame_id = "world"

            w = np.pi / 6.
            p = 15

            x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) + xi
            y = r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t) + yi
            z = (h / 2) * (1 + np.tanh(t - 2.5)) + zi
            yaw = 0

            pose.x = x
            pose.y = y
            pose.z = z
            pose.yaw = yaw

            pub.publish(pose)

        t += 0.01
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('trajectory_circle', anonymous=True)
    xi = rospy.get_param("~xi", 0.0)
    yi = rospy.get_param("~yi", 0.0)
    zi = rospy.get_param("~zi", 0.0)

    altura = rospy.get_param("~h", 0.0)
    radio = rospy.get_param("~r", 0.0)

    pub = rospy.Publisher('cmd_position', Position, queue_size=1)
    boolean_pub = rospy.Publisher('boolean_topic', Bool, queue_size=1)  # Add the boolean publisher

    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

    trajectory_circle()
    rospy.spin()

    


# from std_msgs.msg import Bool

# import rospy
# import numpy as np
# from crazyflie_driver.msg import Full
# from std_msgs.msg import Header
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
# import tf

# r = 0.0
# h = 0.0
# t = 0.0
# rt = 100
# gr = 0

# button_pressed = False
# start_time = rospy.Time()

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
#         boolean_pub.publish(True)  # Publish True to boolean_topic
#     elif joy_msg.buttons[10] == 1 and button_pressed:
#         rospy.loginfo('Trayectoria reiniciada')
#         r = 0.0
#         h = 0.0
#         t = 0.0
#         gr = 0
#         button_pressed = False
#         start_time = rospy.Time()  # Reset the starting time
#         boolean_pub.publish(False)  # Publish False to boolean_topic

# def trajectory_circle():
#     global pub, r, h, t, rt, xi, yi, zi, button_pressed

#     rospy.sleep(1)  # Wait 1 second before starting the trajectory
    
#     rate = rospy.Rate(rt)  # rt Hz

#     while not rospy.is_shutdown():
#         current_time = rospy.Time.now()

#         if button_pressed:  # Verificar si la trayectoria ha sido iniciada
#             if not start_time.is_zero():
#                 elapsed_time = (current_time - start_time).to_sec()
#                 # rospy.loginfo('Tiempo: ' + str(elapsed_time))
#                 if elapsed_time >= 60:
#                     break

#             full_msg = Full()
#             full_msg.header = Header()
#             full_msg.header.stamp = rospy.Time.now()

#             w = np.pi / 6.
#             p = 15

#             x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t)
#             y =  r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t)
#             z = (h/2) * (1 + np.tanh(t-2.5))
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

#             pub.publish(full_msg)
#         rate.sleep()

#         t += 0.01
#         rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node('trajectory_circle', anonymous=True)
#     xi = rospy.get_param("~xi", 0.0)
#     yi = rospy.get_param("~yi", 0.0)
#     zi = rospy.get_param("~zi", 0.0)

#     altura = rospy.get_param("~h", 0.0)
#     radio = rospy.get_param("~r", 0.0)

#     pub = rospy.Publisher('cmd_position', Full, queue_size=1)
#     boolean_pub = rospy.Publisher('boolean_topic', Bool, queue_size=1)  # Add the boolean publisher

#     joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

#     trajectory_circle()
#     rospy.spin()


