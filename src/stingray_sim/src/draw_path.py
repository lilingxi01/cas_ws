#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

rospy.init_node('path_drawing')

path_pub = rospy.Publisher("/triton/path", Path, queue_size=10)
path_msg = Path()

def handle_pose(data):
    global path_msg
    path_msg.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path_msg.poses.append(pose)
    path_pub.publish(path_msg)

odom_sub = rospy.Subscriber('/triton/odom', Odometry, handle_pose)

rospy.spin()

