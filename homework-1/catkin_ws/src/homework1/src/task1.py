#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math

import rospy
from geometry_msgs.msg import Twist

TRAPEZOID_ACUTE_ANGLE_RAD = 1.178       # 67.5 deg
TRAPEZOID_OBTUSE_ANGLE_RAD = 1.9635     # 112.5 deg
TRAPEZOID_ANGLED_SIDES_FACTOR = 0.7654  # = sqrt(2 - sqrt(2))

# Starts the node and processes the parameter
def init():
    rospy.init_node("task1")

def rotate(publisher, rate, ang):
    print("Rotating %.1f degrees" % math.degrees(ang))
    cmd_vel = Twist()
    cmd_vel.angular.z = ang
    publisher.publish(cmd_vel)
    rate.sleep()
    
def translate(publisher, rate, dist):
    print("Moving %.2fm forward" % dist)
    cmd_vel = Twist()
    cmd_vel.linear.x = dist
    publisher.publish(cmd_vel)
    rate.sleep()

def main():
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    trapezoid_radius = rospy.get_param('~radius', default=2)

    # This rate must be <= 1, or else the robot won't have time 
    # to complete a turn before the next command is sent.
    rate = rospy.Rate(0.9)
    rate.sleep()


    print("Trapazoid Radius Parameter: %.2fm" % trapezoid_radius)
    print("Trapazoid Acute Angle: %.1f deg" % math.degrees(TRAPEZOID_ACUTE_ANGLE_RAD))
    print("Trapazoid Obtuse Angle: %.1f deg" % math.degrees(TRAPEZOID_OBTUSE_ANGLE_RAD))
    print("Trapazoid Angled Sides Factor: %.3f" % TRAPEZOID_ANGLED_SIDES_FACTOR)

    rotate(publisher, rate, -math.pi/2)                                             # Rotate downwards
    translate(publisher, rate, trapezoid_radius)                                    # Go forward to bottom-left point
    rotate(publisher, rate, math.pi - TRAPEZOID_ACUTE_ANGLE_RAD)                    # Rotate to head right
    translate(publisher, rate, trapezoid_radius * TRAPEZOID_ANGLED_SIDES_FACTOR)    # Go forward to bottom-right point
    rotate(publisher, rate, math.pi - TRAPEZOID_OBTUSE_ANGLE_RAD)                   # Rotate to head upwards
    translate(publisher, rate, math.sqrt(2) * trapezoid_radius)                     # Go forward to top-right point
    rotate(publisher, rate, math.pi - TRAPEZOID_OBTUSE_ANGLE_RAD)                   # Rotate to head left
    translate(publisher, rate, trapezoid_radius * TRAPEZOID_ANGLED_SIDES_FACTOR)    # Go forward to top-left point
    rotate(publisher, rate, math.pi - TRAPEZOID_ACUTE_ANGLE_RAD)                    # Rotate to head down
    translate(publisher, rate, trapezoid_radius)                                    # Go forward back to starting position

    print("Done!")

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    init()
    main()