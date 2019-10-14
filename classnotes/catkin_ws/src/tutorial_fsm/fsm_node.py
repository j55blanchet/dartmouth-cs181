#!/usr/bin/env python


# rosrun stage_ros stageros -d /opt/ros/kinetic/share/turtlebot_stage/maps/stage/maze.world

# rosrun stage_ros stageros -d src/turtlebot_simulator/turtlebot_stage/maps/stage/robopark_plan.world

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from functools import reduce
    # Format: 
    # std_msgs/Header header
    # uint32 seq
    # time stamp
    # string frame_id
    # float32 angle_min
    # float32 angle_max
    # float32 angle_increment
    # float32 time_increment
    # float32 scan_time
    # float32 range_min
    # float32 range_max
    # float32[] ranges
    # float32[] intensities


RATE = 10
SPEED = 0.7
TURN_SPEED = 0.3
DISTANCE_THRESHOLD = 1.0


class RobotState:
    class State:
        stopped = 0
        go_forward = 1
        turn_left = 2

    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_sub = rospy.Subscriber("base_scan", LaserScan, self.scan_callback, queue_size=1)
        self.rate = rospy.Rate(RATE)
        self.state = RobotState.State.stopped

    def scan_callback(self, msg):
        min_dist = reduce(min, msg.ranges)
        print("Min dist: %0.2f" % min_dist)
        if min_dist < DISTANCE_THRESHOLD:
            self.state = RobotState.State.turn_left
            print("    Too close: STOPPING!")
        else:
            self.state = RobotState.State.go_forward

    def spin(self):
        while not rospy.is_shutdown():
            cmd_vel = Twist()

            # if we see something closeby, stop
            if self.state == RobotState.State.stopped:
                pass

            # if we don't see anything, go straight
            elif self.state == RobotState.State.go_forward:
                cmd_vel.linear.x = SPEED
            elif self.state == RobotState.State.turn_left:
                cmd_vel.angular.z = TURN_SPEED
            else:
                print("ERROR: Invalid State", self.state)
            
            self.cmd_pub.publish(cmd_vel)
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("fsm_node")

    r = RobotState()
    r.spin()