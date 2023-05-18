import rospy
from std_msgs.msg import Bool


def start_ball_spawning():
    pub = rospy.Publisher('/cas_sim/spawn_balls', Bool, queue_size=10)

    rospy.loginfo('[CAS] Ball will be spawned in 6 seconds.')
    rospy.sleep(6)
    
    # Create the Bool message
    msg = Bool()
    msg.data = True
    
    # Publish the message
    pub.publish(msg)
    
    rospy.loginfo("[CAS] Published message to start spawning balls.")
    
