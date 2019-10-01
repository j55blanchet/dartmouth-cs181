#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

# Threshold for robot to consider itself as facing the next target point
ANGLE_THRESHOLD = math.pi / 720

# Threshold for robot to consider itself as standing on a point
DISTANCE_THRESHOLD = 0.02

cur_pose = None
target_points = []     # The target points of the trapazoid, in the reference frame of the original position

# Starts the node and processes the parameter
def init():
    global target_points
    rospy.init_node("homework1_driver")
    
    # Get trapazoid radius based off of launch parameter
    trapazoid_radius = float(rospy.get_param('~radius'))
    print("Trapazoid radius: %s" % trapazoid_radius)
    
    # Compute points of trapazoid, relative to (0, 0)
    #    - The points will be adjusted to take into account
    #      the initial position of the robot when the first
    #      pose message is received
    target_points = [
        Point(0, -trapazoid_radius, 0), # bottom-left
        Point(trapazoid_radius / math.sqrt(2), 
                -trapazoid_radius / math.sqrt(2), 
                0), # bottom-right
        Point(trapazoid_radius / math.sqrt(2), 
                trapazoid_radius / math.sqrt(2), 
                0),# top-right
        Point(0, trapazoid_radius, 0),# top-left
        Point(0, 0, 0) # original position
    ]
    print("Target points (relative to 0, 0): ", target_points)

# Called when we receive an updated location from turtle simulator
def pose_callback(pose):
    global cur_pose

    # if this is the first pose update we've receieved, update the 
    # locations of all target points
    if cur_pose is None:    
        for i in range(len(target_points)):
            old_target_point = (target_points[i].x, target_points[i].y)
            target_points[i].x = target_points[i].x + pose.x
            target_points[i].y = target_points[i].y + pose.y
            print("Target %d adjusted from %.2f, %.2f to %.2f, %.2f" % (i, old_target_point[0], old_target_point[1], target_points[i].x, target_points[i].y))

    cur_pose = pose

# Compute the difference in radians between 
def get_needed_rotation(pose, target_point):
    angle_to_target = math.atan2(target_point.y - pose.y, target_point.x - pose.x)
    return angle_to_target - pose.theta

# Calculates what movement is necessary using a
# rotate-then-translate method
def calculate_movement(cur_pose, target_point):
    cmd_vel = Twist()

    needed_rotation = get_needed_rotation(cur_pose, target_point)
    distance_to_target = math.hypot(target_point.x - cur_pose.x, target_point.y - cur_pose.y)
    hasArrived = False
    print("Current pose: (%.2f, %.2f)@%.0d deg, needed rotation: %.0f deg, %.2f rad, distance: %.2f" % (cur_pose.x,  cur_pose.y, math.degrees(cur_pose.theta), math.degrees(needed_rotation), needed_rotation, distance_to_target))

    # Rotate first if we're not heading in the right direction
    if abs(needed_rotation) > ANGLE_THRESHOLD and distance_to_target > DISTANCE_THRESHOLD:
        cmd_vel.angular.z = needed_rotation
        print("Rotating by %.2f to direct towards %.2f, %.2f" % (cmd_vel.angular.z, target_point.x, target_point.y))
    
    # If we're heading in the right direction, then move forward
    elif distance_to_target > DISTANCE_THRESHOLD:
        cmd_vel.linear.x = distance_to_target
        print("Heading forward to %.2f, %.2f" % (target_point.x, target_point.y))

    # If we've arrived at a point, let the caller know so next target
    # point can be queued up!
    else:
        hasArrived = True
        print("ARRIVED AT %.2f, %.2f" % (target_point.x, target_point.y))

    return (hasArrived, cmd_vel)


def main():
    target_point_i = 0
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():

        cmd_vel = Twist()
        hasArrived = False

        # Only perform a movement if we've located our initial position
        # (and we're not done going through our target points)
        if cur_pose is not None and target_point_i < len(target_points):
            print("  target i: %d" % target_point_i)
            target_point = target_points[target_point_i]
            (hasArrived, cmd_vel) = calculate_movement(cur_pose, target_point)        

        if hasArrived:
            print("ARRIVED AT POINT %d, NOW HEADING TO POINT %d" % (target_point_i, target_point_i+1))
            target_point_i += 1


        publisher.publish(cmd_vel)
        rate.sleep()

if __name__ == "__main__":
    init()
    main()