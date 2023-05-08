#!/usr/bin/python3

#future imports
from __future__ import print_function

#ros imports
import rospy
from geometry_msgs.msg import Twist

#std imports
from threading import Thread
import time
import sys
from pynput import keyboard


#establish ros node and publisher to velocity
rospy.init_node("teleop_robot")
vel_pub = rospy.Publisher("/triton_lidar/vel_cmd", Twist, queue_size=2)

LIN_SPEED = 0.2
ANG_SPEED = 1.0

vel_msg = Twist()
key_state = {}
def key_update(key, state):

    #key is pressed for the first time
    if key not in key_state:
        key_state[key] = state
        return True

    # key changed state
    if state != key_state[key]:
        key_state[key] = state
        return True

    #no change
    return False



stop_display = False
def key_press(key):
    if key == keyboard.Key.esc:
        global stop_display
        stop_display = True
        print('\nPress Ctrl+C to exit')
        return False
    try:
        #character input
        k = key.char
    except:
        #arrow key/other input
        k = key.name


    #check if press changes state
    change = key_update(key, True)
    if change:
        global vel_msg, LIN_SPEED, ANG_SPEED
        if   k in ['w', 'up']:
            vel_msg.linear.y += LIN_SPEED
        elif k in ['s', 'down']:
            vel_msg.linear.y -= LIN_SPEED
        elif k in ['d', 'right']:
            vel_msg.linear.x += LIN_SPEED
        elif k in ['a', 'left']:
            vel_msg.linear.x -= LIN_SPEED
        elif k in ['e']:
            vel_msg.angular.z -= ANG_SPEED
        elif k in ['q']:
            vel_msg.angular.z += ANG_SPEED
        elif k in ['x']:
            LIN_SPEED += 0.1
        elif k in ['z']:
            LIN_SPEED -= 0.1
    return True
    

def key_release(key):
    try:
        #character input
        k = key.char
    except:
        #arrow key/other input
        k = key.name

    change = key_update(key, False)
    if change:
        global vel_msg
        if   k in ['w', 'up']:
            vel_msg.linear.y = 0
        elif k in ['s', 'down']:
            vel_msg.linear.y = 0
        elif k in ['d', 'right']:
            vel_msg.linear.x = 0
        elif k in ['a', 'left']:
            vel_msg.linear.x = 0
        elif k in ['e']:
            vel_msg.angular.z = 0
        elif k in ['q']:
            vel_msg.angular.z = 0
        elif k in ['x']:
            pass
        elif k in ['z']:
            pass
    return True
    

rate = rospy.Rate(10)
def user_display():
    print('Use WSAD or the ARROW KEYS to control Triton.\nUse Q & E to rotate Triton.\nUse x/z to increase/decrease speed')
    while True:
        try:
            print('\r' + ' '*80,end='')
            sys.stdout.flush()
            log_str = "\r\t\tX: {}\tY: {}\tTHETA: {}\t".format(vel_msg.linear.x,
                                                          vel_msg.linear.y,
                                                          vel_msg.angular.z)
            print(log_str, end=' ')
            sys.stdout.flush()

            global stop_display
            if stop_display:
                exit(0) 

            if not rospy.is_shutdown():
                rate.sleep()
                vel_pub.publish(vel_msg)
            else:
                exit(0)
        except KeyboardInterrupt:
            exit(0)


#start key listener thread
key_listener = keyboard.Listener(on_press=key_press, on_release=key_release) 
key_listener.start()

#start user display thread
display_thread = Thread(target=user_display)
display_thread.start()

#update ros topics on main thread
rospy.spin()

