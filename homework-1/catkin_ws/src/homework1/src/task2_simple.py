#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math

import rospy
from geometry_msgs.msg import Twist

# Starts the node and processes the parameter
def init():
    rospy.init_node("task2_simple")

def rotate(publisher, rate, ang):
    print("Rotating %.0f degrees" % math.degrees(ang))
    move(publisher, rate, 0, ang)
    
def translate(publisher, rate, dist):
    print("Moving %.2fm forward" % dist)
    move(publisher, rate, dist, 0)

def move(publisher, rate, x_vel, z_rotate):
    cmd_vel = Twist()
    cmd_vel.linear.x = x_vel
    cmd_vel.angular.z = z_rotate
    publisher.publish(cmd_vel)
    rate.sleep()

def main():
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(1)
    rate.sleep()

    semicircle_radius = rospy.get_param('~radius', default=2)
    d_arclength = math.pi * semicircle_radius
    
    print("Semicircle Radius Parameter: %.2fm" % semicircle_radius)

    rotate(publisher, rate, -math.pi / 2)           # Rotate downwards
    translate(publisher, rate, semicircle_radius)    # Go to bottom left point
    rotate(publisher, rate, math.pi / 2)            # Head rightwards
    move(publisher, rate, d_arclength, math.pi)     # Curve in an arc
    rotate(publisher, rate, math.pi / 2)            # Head down again
    translate(publisher, rate, semicircle_radius)    # Get back to starting position
    
    print("Done!")

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    init()
    main()