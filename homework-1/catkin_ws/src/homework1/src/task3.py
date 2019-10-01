#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math
import json
import os

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

# Threshold for robot to consider itself as facing the next target point
ANGLE_THRESHOLD = math.pi / 180

# Threshold for robot to consider itself as standing on a point
DISTANCE_THRESHOLD = 0.4




# Reads in the poly-line and generates a list of target points
def process_parameters():

    input_filename = rospy.get_param("~polyline_file", default="task3_input.json")
    target_json = []

    print("Opening input polyline file...")
    print("   Note: CWD is %s" % os.getcwd())

    # I consulted https://www.geeksforgeeks.org/with-statement-in-python/ for info on with statements
    with open(input_filename, 'r') as file:
        target_json = json.load(file)

    print("Processing input polyline...")

    target_points = []
    point_i = 0

    for point_json in target_json:
        point = Point(point_json["x"], point_json["y"], 0)
        target_points.append(point)
        print("   Got point %02d: (%0.2f, %0.2f)" %(point_i, point.x, point.y))
        point_i += 1

    return target_points

# Called when we receive an updated location from turtle simulator
def pose_callback(pose, cur_pose):
    # copy pose into the cur_pose object
    cur_pose.x = pose.x
    cur_pose.y = pose.y
    cur_pose.theta = pose.theta

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
    print("    Needed rotation: %.0f deg, %.2f rad" % (math.degrees(needed_rotation), needed_rotation))
    print("    Needed distance: %0.3f" % distance_to_target)

    # Rotate first if we're not heading in the right direction
    if abs(needed_rotation) > ANGLE_THRESHOLD and distance_to_target > DISTANCE_THRESHOLD:
        cmd_vel.angular.z = needed_rotation
        print("    ==> Rotating by %.2f to direct towards %.2f, %.2f" % (cmd_vel.angular.z, target_point.x, target_point.y))
    
    # If we're heading in the right direction, then move forward
    elif distance_to_target > DISTANCE_THRESHOLD:
        cmd_vel.linear.x = distance_to_target
        print("    ==> Heading forward to %.2f, %.2f" % (target_point.x, target_point.y))

    # If we've arrived at a point, let the caller know so next target
    # point can be queued up!
    else:
        hasArrived = True
        print("    ==> ARRIVED AT %.2f, %.2f" % (target_point.x, target_point.y))

    return (hasArrived, cmd_vel)


def main(initial_pose, target_points):

    target_point_i = 0
    cur_pose = initial_pose
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback, callback_args=cur_pose)
    
    rate = rospy.Rate(1)

    finished = False
    
    while not rospy.is_shutdown():

        if not finished:
            print("Current Pose: (%.2f, %.2f)@%.2f rad" % (cur_pose.x, cur_pose.y, cur_pose.theta))

        cmd_vel = Twist()
        hasArrived = False

        # Only perform a movement if we've located our initial position
        # (and we're not done going through our target points)
        if target_point_i < len(target_points):
            print("    Calculating movement for point %d" % target_point_i)
            target_point = target_points[target_point_i]
            (hasArrived, cmd_vel) = calculate_movement(cur_pose, target_point)        

        if hasArrived:
            if target_point_i >= len(target_points) - 1:
                print("    Arrived at point %d, the final point", target_point_i)
            else:
                print("    Arrived at point %d, will now head to point %d" % (target_point_i, target_point_i+1))
            target_point_i += 1
            finished = True

        publisher.publish(cmd_vel)
        rate.sleep()
        

if __name__ == "__main__":
    print("\n\nStarting task3 driver...\n")
    rospy.init_node("task3")
    print("Waiting for intitial pose...")
    initial_pose = rospy.wait_for_message("/turtle1/pose", Pose, 5)
    target_points = process_parameters()
    main(initial_pose, target_points)



    