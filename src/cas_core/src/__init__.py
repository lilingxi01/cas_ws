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

from sim_helper import start_ball_spawning


# ========== Termination Handler ==========

def signal_handler(signal, frame):
    print('Program stopped.\n')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# ========== Main Loop ==========

# TODO: Write main code (functions) here.


if __name__ == "__main__":
    try:
        rospy.init_node("cas-core", anonymous=True)

        rospy.loginfo("[CAS] Core started.")

        start_ball_spawning()

        rospy.wait_for_message("/cas_sim/spawn_balls", Bool)

    except rospy.ROSInterruptException:
        rospy.loginfo("General error happened.")

