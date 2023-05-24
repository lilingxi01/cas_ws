#!/usr/bin/env python3

import os
import math
from datetime import datetime
import sys
import signal

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import DeleteModel
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from torchvision import transforms

from sim_helper import start_ball_spawning
from kalman import kalman_filter
from frame import organize_frame


# ========== Termination Handler ==========

def signal_handler(signal, frame):
    print('Program stopped.\n')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# ========== Collision Callback ==========

global removed_balls
removed_balls = []


def collision_callback(data):
    if len(data.states) == 0:
        return
    for state in data.states:
        collision1_name = state.collision1_name.split('::')[0]
        collision2_name = state.collision2_name.split('::')[0]
        if collision1_name.startswith('mock_ball_') and collision2_name == 'triton_robot':
            delete_model(collision1_name)


def delete_model(model_name):
    global removed_balls
    if model_name in removed_balls:
        return
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_service(model_name)
        removed_balls.append(model_name)
    except:
        pass


# ========== Main Functions ==========

# TODO: Write main code (functions) here.


# ========== Main Actions ==========


HORIZONTAL_MOVE_RATE = 0.05
HORIZONTAL_MAX_SPEED = 0.1
FORWARD_MOVE_SPEED = 0.1

global curr_action, cumulative_horizontal_displacement, curr_horizontal_velocity
curr_action = None
cumulative_horizontal_displacement = 0.0
curr_horizontal_velocity = 0.0


def action_lifecycle():
    global curr_action, cumulative_horizontal_displacement, curr_horizontal_velocity

    cumulative_horizontal_displacement += curr_horizontal_velocity
    dynamic_center_point = -cumulative_horizontal_displacement / 10

    if curr_action == "left":
        curr_horizontal_velocity = min(curr_horizontal_velocity - HORIZONTAL_MOVE_RATE, \
                -HORIZONTAL_MAX_SPEED)
    elif curr_action == "right":
        curr_horizontal_velocity = max(curr_horizontal_velocity + HORIZONTAL_MOVE_RATE, \
                HORIZONTAL_MAX_SPEED)
    else:
        if curr_horizontal_velocity > dynamic_center_point:
            curr_horizontal_velocity = max(curr_horizontal_velocity - HORIZONTAL_MOVE_RATE, \
                    dynamic_center_point)
        elif curr_horizontal_velocity < dynamic_center_point:
            curr_horizontal_velocity = min(curr_horizontal_velocity + HORIZONTAL_MOVE_RATE, \
                    dynamic_center_point)

    velocity_message = Twist()
    topic_name = "/triton_robot/vel_cmd"
    velocity_publisher = rospy.Publisher(topic_name, Twist, queue_size=10)

    velocity_message.linear.x = curr_horizontal_velocity
    velocity_message.linear.y = FORWARD_MOVE_SPEED
    velocity_publisher.publish(velocity_message)


def move_left():
    global curr_action
    curr_action = "left"


def move_right():
    global curr_action
    curr_action = "right"


# ========== Camera Data Callback ==========

global depth_map, prev_frames, kalman_outputs
depth_map = None
prev_frames = []
kalman_outputs = []

bridge = CvBridge()


def color_image_callback(msg):
    global depth_map, prev_frames, kalman_outputs
    if depth_map is None:
        rospy.loginfo("Depth map is not ready yet.")
        return
    try:
        # Convert ROS depth image message to OpenCV image
        model = YOLO("yolov8n.pt")
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = model(cv_image)
        curr_frames = []
        print('balls:', len(results[0].boxes))
        for box in results[0].boxes:
            x = int(box.xywh[0][0])
            y = int(box.xywh[0][1])
            depth_val = depth_map[y][x] + 0.18
            obj_angle = ((x/640)*87)-43.5
            object_coord = calculate_coordintes(depth_val, obj_angle)
            if object_coord[0] == np.nan or object_coord[1] == np.nan:
                continue
            curr_frames.append(object_coord)
        curr_frames = np.array(curr_frames)

        print('curr_frames', curr_frames)

        organized_frames = organize_frame(prev_frames, kalman_outputs, curr_frames)
        print(organized_frames)

        kalman_outputs = kalman_filter(organized_frames)
        prev_frames = organized_frames


    except Exception as e:
        rospy.logerr(e)


def depth_image_callback(msg):
    global depth_map
    l = np.frombuffer(msg.data, dtype=np.uint8)
    depth_image = l.reshape(-1, 640, 4)
    depth_image_bytes = depth_image.view(dtype=np.uint8).reshape(depth_image.shape + (-1,))
    depth_map = np.frombuffer(depth_image_bytes, dtype=np.float32).reshape(depth_image.shape[:2])
    

def calculate_coordintes(depth_val, obj_angle):
    a = depth_val
    A = math.radians(90)
    B = math.radians(obj_angle)
    C = math.radians(90 - obj_angle)
    b = (a * math.sin(B)) / math.sin(A)
    c = (a * math.sin(C)) / math.sin(A)
    return (b, c)


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

        rospy.Subscriber("/bumper_states", ContactsState, collision_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, depth_image_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback)

        main_lifecycle()

    except rospy.ROSInterruptException:
        rospy.loginfo("General error happened.")

