#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Joy
import tf

r = 0.0
h = 0.0
t = 0.0

rt = 100

button_pressed = bool


def joy_callback(joy_msg):
    
    global r,h,t, button_pressed
    if joy_msg.buttons[5] == 1 and not button_pressed:
        rospy.loginfo('Trayectoria iniciada')
        button_pressed = True
        r = 0.16
        h = 0.4
        t = 0.0
    elif joy_msg.buttons[5] == 1 and button_pressed:
        rospy.loginfo('Trayectoria reiniciada')
        r = 0.0
        h = 0.0
        t = 0.0
        button_pressed = False
    elif not joy_msg.buttons[5] == 1:
        button_pressed = False



def trajectory_circle():
    
    
    while not rospy.is_shutdown():
        pose = PoseStamped()
        # vel = Twist()
        acc = Twist()
        pose.header.seq = 0
        pose.header.frame_id = "world"
        global r,h,t,rt
        rate = rospy.Rate(rt) # rt Hz
        w = np.pi/9.
        p = 15

        x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) - 0.25
        y = r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t)
        z = (h/2) * (1 + np.tanh(t-7.5)) 
        yaw = 0

        # Publicar posicion actual 
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        pub_pose.publish(pose)

        # Publicar aceleracion
        ax = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.cos(w * t)
        ay = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.sin(w * t)
        az = (h/2) * np.tanh(t-7.5) * (1 - np.tanh(t-7.5)) * (1 - 2*np.tanh(t-7.5))
        a_yaw = 0

            # Publicar aceleracion
        acc.linear.x = ax
        acc.linear.y = ay
        acc.linear.z = az
        acc.angular.z = a_yaw
        pub_acc.publish(acc)

            
            # Actualizar el tiempo
        t += 0.01
        rate.sleep()

if __name__ == '__main__':

    rospy.init_node('trajectory_circle', anonymous=True)
    
    pub_pose = rospy.Publisher('goal', PoseStamped, queue_size=1)
    pub_acc = rospy.Publisher('goalacc', Twist, queue_size=1)   

    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    
    trajectory_circle() 
    
    
    rospy.spin()

