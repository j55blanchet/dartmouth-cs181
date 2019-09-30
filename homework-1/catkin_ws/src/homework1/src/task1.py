#!/usr/bin/python

from __future__ import print_function
from __future__ import division

import sys
import math

import rospy
import roslib
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose

####  Task: (text copied from prompt)
#
# Write a ROS node that will drive the robot to draw a simplified "D" represented as an 
# isosceles trapezoid. The robot starts at the center of the longer base, with the robot 
# local frame x-axis perpendicular to it and the vertices of the shorter base vertices 
# are determined by the vectors of radius r and angle +/45 degrees. For example, in the 
# figure below, the robot is at (5.5,5.5) m, with theta=0, and the radius is 2 m. 
#
ANGLE_THRESHOLD = math.pi / 360

cur_pose = None
first_pose = None
target_points = []     # The target points of the trapazoid, in the reference frame of the original position

# utility function taken from https://stackoverflow.com/questions/57739846/is-there-a-processing-map-equivalent-for-python
def map_range(value, start1, stop1, start2, stop2):
   return (value - start1) / (stop1 - start1) * (stop2 - start2) + start2

def constrain(value, min, max):
    return  min if value < min else (max if value > max else value) 

def init():
    global target_points

    # Parse arguments (search for radius)
    print('Number of arguments:', len(sys.argv), 'arguments.')
    print('Argument List:', str(sys.argv))
    
    if len(sys.argv) != 2:
        sys.stderr.write("Error: expecting one argument for the radius (in meters) \n")
        exit(-1)
    
    trapazoid_radius = float(sys.argv[1])
    target_points = [
        Vector3(0, -trapazoid_radius, 0), # bottom-left
        Vector3(trapazoid_radius / math.sqrt(2), 
                -trapazoid_radius / math.sqrt(2), 
                0), # bottom-right
        Vector3(trapazoid_radius / math.sqrt(2), 
                trapazoid_radius / math.sqrt(2), 
                0),# top-right
        Vector3(0, trapazoid_radius, 0)# top-left
    ]
    print("Target points: ", target_points)

    rospy.init_node("homework1_driver")
    
def pose_callback(pose):
    br = tf.TransformBroadcaster()
    
    global cur_pose, first_pose
    cur_pose = pose
    if first_pose is None:
        first_pose = pose


def get_needed_rotation(pose, target_point):
    angle_to_target = math.atan2(target_point.y - pose.y, target_point.x - pose.x)
    return angle_to_target - pose.theta

def calculate_movement(cur_pose, target_point):

    cmd_vel = Twist()
    needed_rotation = get_needed_rotation(cur_pose, target_point)
    hasArrived = False

    if abs(needed_rotation) > math.pi * (180 * 3):
        cmd_vel.angular.z = constrain(needed_rotation, -1, 1)
        print("Rotating by %02f to direct towards %02f, %02f" % (cmd_vel.angular.z, target_point.x, target_point.y))
        
    elif math.hypot(target_point.x - cur_pose.x, target_point.y - cur_pose.y) > 0:
        cmd_vel.linear.x = 1
        print("Heading forward to %02f, %02f" % (target_point.x, target_point.y))

    else:
        hasArrived = True
        print("Arrived at %02f, %02f, heading to next target point" % (target_point.x, target_point.y))

    return (hasArrived, cmd_vel)


def main():
    target_point_i = 0
    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():

        cmd_vel = Twist()
        hasArrived = False

        if cur_pose is not None and target_point_i < len(target_points):
            target_point = target_points[target_point_i]
            (hasArrived, cmd_vel) = calculate_movement(cur_pose, target_point)        

        if hasArrived:
            target_point_i += 1

        publisher.publish(cmd_vel)
        rate.sleep()

if __name__ == "__main__":
    init()
    main()