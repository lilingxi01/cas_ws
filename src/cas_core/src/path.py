#!/usr/bin/env python3
import random
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState

def set_model_state(model_name,relative_entity_name, x, y, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')
    state_msg = ModelState()
    roll=0
    pitch=0

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    state_msg.model_name = model_name
    state_msg.reference_frame = relative_entity_name
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = qx
    state_msg.pose.orientation.y = qy
    state_msg.pose.orientation.z = qz
    state_msg.pose.orientation.w = qw

    try:
        sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        sms(state_msg)

        return x,y
    except rospy.ServiceException:
        print("Service call failed")

def path():    

    rospy.init_node('path', anonymous=True)
    pub = rospy.Publisher('/triton_robot/vel_cmd', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(1)


    set_model_state('triton_robot','ground_plane',0,-3,0)
    vel_msg=Twist()
    collision = False
    #runs for 10 seconds
    for i in range(100):
        if(not collision):
            vel_msg.linear.y = 0.4
            vel_msg.angular.z = 0.0
            pub.publish(vel_msg)
            rate.sleep()
        else:
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            pub.publish(vel_msg)
            rate.sleep()

    vel_msg.linear.y = 0.0
    pub.publish(vel_msg)



if __name__ == '__main__':
    try:
        path()
    except rospy.ROSInterruptException:
        pass