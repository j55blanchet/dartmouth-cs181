#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math

import rospy
from geometry_msgs.msg import Twist

TRAPAZOID_ACUTE_ANGLE_RAD = abs(math.atan(math.sqrt(2) / (1 - math.sqrt(2))))
TRAPAZOID_OBTUSE_ANGLE_RAD = math.pi  -TRAPAZOID_ACUTE_ANGLE_RAD
TRAPAZOID_ANGLED_SIDES_FACTOR = math.sqrt(2 - math.sqrt(2))

# Starts the node and processes the parameter
def init():
    rospy.init_node("task1")

def rotate(publisher, rate, ang):
    print("Rotating %.0f degrees" % math.degrees(ang))
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
    rate.sleep()

def main():
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    trapazoid_radius = rospy.get_param('~radius', default=2)

    rate = rospy.Rate(1)
    rate.sleep()


    print("Trapazoid Radius Parameter: %.2fm" % trapazoid_radius)
    print("Trapazoid Acute Angle: %.0f deg" % math.degrees(TRAPAZOID_ACUTE_ANGLE_RAD))
    print("Trapazoid Obtuse Angle: %.0f deg" % math.degrees(TRAPAZOID_OBTUSE_ANGLE_RAD))
    print("Trapazoid Angled Sides Factor: %.3f" % TRAPAZOID_ANGLED_SIDES_FACTOR)

    rotate(publisher, rate, -math.pi/2)                                             # Rotate downwards
    translate(publisher, rate, trapazoid_radius)                                    # Go forward to bottom-left point
    rotate(publisher, rate, math.pi - TRAPAZOID_ACUTE_ANGLE_RAD)                    # Rotate to head right
    translate(publisher, rate, trapazoid_radius * TRAPAZOID_ANGLED_SIDES_FACTOR)    # Go forward to bottom-right point
    rotate(publisher, rate, math.pi - TRAPAZOID_OBTUSE_ANGLE_RAD)                   # Rotate to head upwards
    translate(publisher, rate, math.sqrt(2) * trapazoid_radius)                     # Go forward to top-right point
    rotate(publisher, rate, math.pi - TRAPAZOID_OBTUSE_ANGLE_RAD)                   # Rotate to head left
    translate(publisher, rate, trapazoid_radius * TRAPAZOID_ANGLED_SIDES_FACTOR)    # Go forward to top-left point
    rotate(publisher, rate, math.pi - TRAPAZOID_ACUTE_ANGLE_RAD)                    # Rotate to head down
    translate(publisher, rate, trapazoid_radius)                                    # Go forward back to starting position

    print("Done!")

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    init()
    main()