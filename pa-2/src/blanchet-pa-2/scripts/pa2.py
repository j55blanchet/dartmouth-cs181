#!/usr/bin/env python

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from functools import reduce

class Constants:
    RATE = 2
    LINEAR_VEL = 0.5
    PROPORTIONAL_GAIN = 2
    DERIVITIVE_GAIN = 0.0
    TARGET_DIST = 0.5
    OBSTACLE_STOP_DISTANCE = 0.1
    
is_verbose = True

# TODO
# * Address issue where obstacle is no longer in the way
# 


# Print if verbosity is enabled.
#  * Referenced: https://stackoverflow.com/questions/26286203/custom-print-function-that-wraps-print
def printVerbose(*args, **kwargs):
    if is_verbose:
        print("".join(map(str,args)), **kwargs)


class WallFollowerNode:

    class State:
        INITIALIZING =      0   # Robot is starting up; don't do anything yet
        BLOCKED_OBSTACLE =  1   # Waiting due to blocked obstacle
        FOLLOWING_WALL =    2   # Following a wall. Wall affinity should be set at this point
        @staticmethod 
        def get_string(affinity):
            return {
                WallFollowerNode.State.INITIALIZING: "INITIALIZING",
                WallFollowerNode.State.BLOCKED_OBSTACLE: "BLOCKED_OBSTACLE",
                WallFollowerNode.State.FOLLOWING_WALL: "FOLLOWING_WALL"
            }.get(affinity, "INVALID")

    class WallAffinity:
        UNDECIDED = 0
        LEFT =      1
        RIGHT =     2
        @staticmethod 
        def get_string(affinity):
            return {
                WallFollowerNode.WallAffinity.UNDECIDED: "UNDECIDED",
                WallFollowerNode.WallAffinity.LEFT: "LEFT",
                WallFollowerNode.WallAffinity.RIGHT: "RIGHT"
            }.get(affinity, "INVALID")

    def __init__(self):
        # Setup rospy plumbing
        rospy.init_node(self.__class__.__name__)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_sub = rospy.Subscriber("base_scan", LaserScan, self.on_scan, queue_size=1)
        self.rate = rospy.Rate(Constants.RATE)

        # Instance variables
        self.state = WallFollowerNode.State.INITIALIZING
        self.wall_affinity = WallFollowerNode.WallAffinity.UNDECIDED
        self.last_error = 0
        self.ang_vel = 0
        self.lin_vel = 0
        self.last_scan = None

    def on_scan(self, msg):
        self.last_scan = msg
    
    def decide_action(self):

        if self.last_scan is None:
            return
        scan = self.last_scan

        # Calculate essential measurements
        range_mid_index = len(scan.ranges) / 2
        min_dist = reduce(min, scan.ranges)
        min_dist_right = reduce(min, scan.ranges[:range_mid_index])
        min_dist_left = reduce(min, scan.ranges[range_mid_index:])
        self.decideWallAffinity(min_dist_left, min_dist_right, scan.range_max)
        error = Constants.TARGET_DIST - min_dist
        error_change = error - self.last_error
        self.last_error = error
        
        # PD Controller Calculation
        prop_ctl = Constants.PROPORTIONAL_GAIN * error
        deriv_ctl = Constants.DERIVITIVE_GAIN * error_change
        control = prop_ctl + deriv_ctl
        if self.wall_affinity is WallFollowerNode.WallAffinity.RIGHT:
            self.ang_vel = -control
        elif self.wall_affinity is WallFollowerNode.WallAffinity.LEFT:
            self.ang_vel = control
        else:
            self.ang_vel = 0

        printVerbose("CONTROLLING")
        printVerbose("\tDist(min)=%0.2f   Dist(left)=%0.2f   Dist(right)=%0.2f" % (min_dist, min_dist_left, min_dist_right))
        printVerbose("\tTarget=%0.2f      Error=%0.2f   Error(change)=%0.2f" % (Constants.TARGET_DIST, error, error_change))
        printVerbose("\tNetCtl=%0.2f      PropCtl=%0.2f  DerivCtl=%0.2f" % (control,prop_ctl, deriv_ctl))
        printVerbose("\tWallAffty=%s   AngVel=%0.2f" % (WallFollowerNode.WallAffinity.get_string(self.wall_affinity), self.ang_vel))

        if min_dist < Constants.OBSTACLE_STOP_DISTANCE:
            printVerbose("\tAction=Stopping for obstacle")
            self.state = WallFollowerNode.State.BLOCKED_OBSTACLE
            self.lin_vel = 0
        else:
            printVerbose("\tAction=Following wall")
            self.state = WallFollowerNode.State.FOLLOWING_WALL
            self.lin_vel = Constants.LINEAR_VEL

    def decideWallAffinity(self, min_dist_left, min_dist_right, range_max):
        if self.wall_affinity is not WallFollowerNode.WallAffinity.UNDECIDED:
            # We've already decided on a wall to follow
            return
        elif min_dist_right < min_dist_left:
            printVerbose("Choosing to follow right wall")
            self.wall_affinity = WallFollowerNode.WallAffinity.RIGHT
        elif min_dist_left < min_dist_right or not min_dist_left == range_max:
            printVerbose("Choosing to follow left wall")
            self.wall_affinity = WallFollowerNode.WallAffinity.LEFT
        
    def spin(self):
        printVerbose()
        while not rospy.is_shutdown():
            self.decide_action()
            cmd_vel = Twist()
            cmd_vel.linear.x = self.lin_vel
            cmd_vel.angular.z = self.ang_vel
            self.cmd_pub.publish(cmd_vel)
            self.rate.sleep()

if __name__ == "__main__":
    is_verbose = rospy.get_param("verbose", True)
    followerNode = WallFollowerNode()
    followerNode.spin()
