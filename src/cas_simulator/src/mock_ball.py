#!/usr/bin/env python3

import sys
import math
import signal
import random

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
ball_diameter = 0.2
spawn_interval = 30
throw_force = 10
ground_offset = 0.02


global should_spawn_balls, ball_index
should_spawn_balls = False
ball_index = 0


# TODO: Random angle has not been done yet.


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
        force_angle = random.randint(240, 300)
        response = apply_force_to_ball(f"mock_ball_{ball_index}", pose, force_angle)

    ball_index += 1
    return response


def subscribe_to_spawn_balls():
    def callback(msg):
        global should_spawn_balls
        should_spawn_balls = msg.data
        rospy.loginfo(f'[CAS_SIM] Should ball spawn: {should_spawn_balls}')

    rospy.Subscriber("/cas_sim/spawn_balls", Bool, callback)


def main_loop():
    global should_spawn_balls

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
            ball_pose = Pose()
            # Randomize position x between -1.5 and 1.5.
            ball_pose.position.x = random.uniform(-2.5, 2.5)
            ball_pose.position.y = 2.5
            ball_pose.position.z = ball_diameter / 2 + ground_offset

            if spawn_red_ball(ball_pose):
                rospy.loginfo("Mock ball spawned successfully.")
            else:
                rospy.logerr("Failed to spawn mock ball.")


if __name__ == "__main__":
    rospy.init_node("mock_ball_system")

    main_loop()

