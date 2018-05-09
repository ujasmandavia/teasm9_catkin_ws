#!/usr/bin/env python

import rospy
import os

def kill_frontier():
    os.system("rosnode kill explore_client \n")
    os.system("rosnode kill explore_server \n")
    rospy.loginfo('Frontier Exploration has been TERMINATED \n\n\n')
