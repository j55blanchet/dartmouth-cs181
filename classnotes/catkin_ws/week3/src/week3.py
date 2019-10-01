#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math

import rospy
from geometry_msgs.msg import Twist

# Starts the node and processes the parameter
def init():
    rospy.init_node("task1")

def main():
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(1)
    rate.sleep()

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    init()
    main()