#!/usr/bin/env python3
import random
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
from sklearn.linear_model import LinearRegression

''' Trying to use linear regression to predict the path of the ball
def decision(pos):
    if(np.sqrt(pos[-1][0]**2+pos[-1][1]**2)<1):
        path = LinearRegression().fit([pos[:,0]], pos[:,1].reshape(1,1))
    print(f"coef: {path.coef_} intercept: {path.intercept_}")

    '''
def decision(pos):
    if(np.sqrt(pos[-1][0]**2+pos[-1][1]**2)<1):
        slope = (pos[-1][1]-pos[0][1])/(pos[-1][0]-pos[0][0])
        y_intercept = pos[0][1]-slope*pos[0][0]
        x_intercept = -y_intercept/slope
        if(x_intercept>0):
            return 1 #left
        else:
            return 2 #right
    return 0 #straight

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
        if(decision()==0):
            vel_msg.linear.y = 0.4
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            pub.publish(vel_msg)
            rate.sleep()
        elif(decision()==1):
            vel_msg.linear.y = 0.4
            vel_msg.linear.x = 1.0
            vel_msg.angular.z = 0.0
            pub.publish(vel_msg)
            rate.sleep()
        else:
            vel_msg.linear.y = 0.4
            vel_msg.linear.x = -1.0
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