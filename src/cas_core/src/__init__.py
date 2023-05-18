#!/usr/bin/env python3

import os
from datetime import datetime
import sys
import signal
import math
import random

import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from sim_helper import start_ball_spawning


# ========== Termination Handler ==========

def signal_handler(signal, frame):
    print('Program stopped.\n')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# ========== Main Functions ==========

# TODO: Write main code (functions) here.


# ========== Main Actions ==========


HORIZONTAL_MOVE_RATE = 0.05
HORIZONTAL_MAX_SPEED = 0.2

global curr_action
curr_action = None
curr_horizontal_velocity = 0.0


def action_lifecycle():
    global curr_action, curr_horizontal_velocity

    if curr_action == "left":
        curr_horizontal_velocity = min(curr_horizontal_velocity - HORIZONTAL_MOVE_RATE, \
                -HORIZONTAL_MAX_SPEED)
    elif curr_action == "right":
        curr_horizontal_velocity = max(curr_horizontal_velocity + HORIZONTAL_MOVE_RATE, \
                HORIZONTAL_MAX_SPEED)
    else:
        if curr_horizontal_velocity > 0:
            curr_horizontal_velocity = max(curr_horizontal_velocity - HORIZONTAL_MOVE_RATE, 0)
        elif curr_horizontal_velocity < 0:
            curr_horizontal_velocity = min(curr_horizontal_velocity + HORIZONTAL_MOVE_RATE, 0)

    velocity_message = Twist()
    topic_name = "/triton_robot/vel_cmd"
    velocity_publisher = rospy.Publisher(topic_name, Twist, queue_size=10)

    velocity_message.linear.x = curr_horizontal_velocity
    velocity_publisher.publish(velocity_message)


def move_left():
    global curr_action
    curr_action = "left"


def move_right():
    global curr_action
    curr_action = "right"


# ========== Camera Data Callback ==========

global camera_data
camera_data = None


def camera_data_callback():
    global camera_data


# ========== Main Lifecycle ==========

def main_lifecycle():
    # TODO: Implement this function.

    lifecycle_loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        action_lifecycle()

        lifecycle_loop.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("cas-core", anonymous=True)
        rospy.loginfo("[CAS] Core started.")

        # It has a rospy sleep inside.
        # So the main lifecycle will only start after the ball starting spawning.
        start_ball_spawning()

        main_lifecycle()

    except rospy.ROSInterruptException:
        rospy.loginfo("General error happened.")

