#!/usr/bin/env python3

import sys
import math
import signal
import random

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Vector3, Wrench
from std_msgs.msg import Bool


def signal_handler(signal, frame):
    print('Program stopped.\n')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# Define the size of the ball.
ball_diameter = 0.15
spawn_interval = 20
throw_force = 5
ground_offset = 0


def gaussian_distribution(mean, sigma, bounds):
    random_number = np.random.normal(mean, sigma)
    if random_number < bounds[0]:
        return bounds[0]
    elif random_number > bounds[1]:
        return bounds[1]
    return random_number


global should_spawn_balls, ball_index
should_spawn_balls = False
ball_index = 0


def apply_force_to_ball(model, model_pose, force_theta):
    theta_radians = math.radians(force_theta)
    force_x = throw_force * math.cos(theta_radians)
    force_y = throw_force * math.sin(theta_radians)

    model_state = ModelState()
    model_state.model_name = model
    model_state.pose = model_pose
    model_state.twist.linear.x = force_x
    model_state.twist.linear.y = force_y
    model_state.twist.linear.z = ground_offset

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.sleep(0.5)

    try:
        pub.publish(model_state)
        return True
    except rospy.ROSSerializationException as e:
        rospy.logerr(f"Failed to publish ModelState message: {e}")
        return False


def spawn_red_ball(pose):
    global ball_index

    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Create the model SDF directly (without using a file).
    red_ball_sdf = f'''
    <sdf version="1.6">
      <model name="red_ball">
        <pose>{pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.x} {pose.orientation.y} {pose.orientation.z}</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <sphere>
                <radius>{ball_diameter / 2}</radius>
              </sphere>
            </geometry>
            <surface>
              <bounce>
                <restitution_coefficient>1.0</restitution_coefficient>
                <threshold>1e-3</threshold>
              </bounce>
              <friction>
                <ode>
                  <mu>0.1</mu>
                  <mu2>0.1</mu2>
                </ode>
              </friction>
            </surface>
          </collision>
          <visual name="visual">
            <geometry>
              <sphere>
                <radius>{ball_diameter / 2}</radius>
              </sphere>
            </geometry>
            <material>
              <ambient>1 0 0 1</ambient>
              <diffuse>1 0 0 1</diffuse>
            </material>
          </visual>
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.4</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.4</iyy>
              <iyz>0</iyz>
              <izz>0.4</izz>
            </inertia>
          </inertial>
        </link>
      </model>
    </sdf>
    '''


    # Spawn the ball.
    try:
        response = spawn_model(f"mock_ball_{ball_index}", red_ball_sdf, "", pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

    # Apply a force to the ball so it moves.
    if response.success:
        # We know that the distance between robot and ball is 5.5.
        # And we know the shifting of the robot perpendicularly is pose.x.
        # Then we want to know its angle pointing to the ball.
        mean_angle = math.degrees(math.atan(pose.position.x / 5)) + 270
        force_angle = gaussian_distribution(mean_angle, 10, (240, 300))
        response = apply_force_to_ball(f"mock_ball_{ball_index}", pose, force_angle)

    ball_index += 1
    return response


def subscribe_to_spawn_balls():
    def callback(msg):
        global should_spawn_balls
        should_spawn_balls = msg.data
        rospy.loginfo(f'[CAS_SIM] Should ball spawn: {should_spawn_balls}')

    rospy.Subscriber("/cas_sim/spawn_balls", Bool, callback)




global spawned_balls
spawned_balls = 0


def main_loop():
    global should_spawn_balls, spawned_balls

    # Start listening to the spawn_balls topic.
    subscribe_to_spawn_balls()

    curr_tick = 0

    rate = rospy.Rate(10)
    rospy.wait_for_service("/gazebo/spawn_sdf_model")

    while not rospy.is_shutdown():
        curr_tick += 1
        rate.sleep()
        if curr_tick % spawn_interval != 0:
            continue
        if should_spawn_balls:
            # When generating too many balls, stop for this session.
            if spawned_balls >= 100:
                spawned_balls = 0
                should_spawn_balls = False
                continue
            spawned_balls += 1
            ball_pose = Pose()
            # Randomize position x using normal distribution.
            ball_pose.position.x = gaussian_distribution(0, 0.5, [-2, 2])
            ball_pose.position.y = 2
            ball_pose.position.z = ball_diameter / 2 + ground_offset

            if spawn_red_ball(ball_pose):
                rospy.loginfo("[CAS_SIM] Mock ball spawned successfully.")
            else:
                rospy.logerr("[CAS_SIM] Failed to spawn mock ball.")


if __name__ == "__main__":
    rospy.init_node("mock_ball_system")

    main_loop()

