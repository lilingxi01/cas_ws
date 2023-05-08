#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

rospy.init_node('pose_pub')

odom_pub=rospy.Publisher ('/triton/odom', Odometry,queue_size=1)


rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='triton'

tf_broadcast = tf.TransformBroadcaster()

r = rospy.Rate(10)


while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish (odom)

    pos_msg = odom.pose.pose.position
    orient_msg = odom.pose.pose.orientation

    tf_broadcast.sendTransform((pos_msg.x, pos_msg.y, pos_msg.z),
                               (orient_msg.x, orient_msg.y, orient_msg.z, orient_msg.w),
                               rospy.Time().now(),
                               "triton_link",
                               "odom")
    r.sleep()
