#!/usr/bin/env python

from __future__ import print_function
import rospy
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from functools import reduce

class Constants:
    STARTUP_DELAY_SEC = 3

    RATE = 10
    LINEAR_VEL = 0.5
    PROPORTIONAL_GAIN = 3
    DERIVITIVE_GAIN = 20.0
    TARGET_DIST = 0.5
    
    WALL_FOLLOW_CONE_ANGLE = math.pi * 2 / 3 # 120 deg

    # Parameters controlling when to stop for obstacles
    #   * If an obstacle is detecting in the front 60 deg of
    #     the robot withing 0.5m, the robot will stop
    OBSTACLE_DETECTION_FRONT_CONE_ANGLE = math.pi / 3 # 60 degrees
    OBSTACLE_DETECTION_STOP_DISTANCE = 0.5       # 0.5m

is_verbose = True

# TODO
# * Address issue where obstacle is no longer in the way
# 


# Print if verbosity is enabled.
#  * Referenced: https://stackoverflow.com/questions/26286203/custom-print-function-that-wraps-print
def printVerbose(*args, **kwargs):
    if is_verbose:
        print("".join(map(str,args)), **kwargs)

class Utils:
    """Utility class containing various useful static methods"""

    @staticmethod
    def get_scan_within_angle(scan_msg, min_angle, max_angle):
        """Returns the subset of the scan's readings that fall within the angles specified.
        
        Arguments:
            scan_msg {LaserScan} -- The scan whose .ranges readings we'll filter
            min_angle {float} -- The lower bound of the angle range to return readings of
            max_angle {float} -- The upper bound of the angle range to return readings of
        
        Returns:
            List[float] -- Readings that fall within the angles specified
        """
        if min_angle < scan_msg.angle_min:
            print("WARNING: Specified min_angle (%0.2f) is beyond sensor angle_min (%0.2f)" % (min_angle, scan_msg.angle_min))
            min_angle = scan_msg.angle_min
        
        if max_angle > scan_msg.angle_max:
            print("WARNING: Specified max angle (%0.2f) is beyond sensor angle_max (%0.2f)" % (max_angle, scan_msg.angle_max))
            max_angle = scan_msg.angle_max

        index_start = Utils.valmap(min_angle, scan_msg.angle_min, scan_msg.angle_max, 0, len(scan_msg.ranges) - 1)
        index_end = Utils.valmap(max_angle, scan_msg.angle_min, scan_msg.angle_max, 0, len(scan_msg.ranges) - 1)
        return scan_msg.ranges[int(round(index_start)): int(round(index_end))]

    @staticmethod
    # Copied from https://www.raspberrypi.org/forums/viewtopic.php?t=149371
    def valmap(value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


class WallFollowerNode:

    class State:
        INITIALIZING =      0   # Robot is starting up; don't do anything yet
        BLOCKED_OBSTACLE =  1   # Waiting due to blocked obstacle
        FOLLOWING_WALL =    2   # Following a wall. Wall affinity should be set at this point
        @staticmethod 
        def get_string(state):
            return {
                WallFollowerNode.State.INITIALIZING: "INITIALIZING",
                WallFollowerNode.State.BLOCKED_OBSTACLE: "BLOCKED_OBSTACLE",
                WallFollowerNode.State.FOLLOWING_WALL: "FOLLOWING_WALL"
            }.get(state, "INVALID")

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

        # We're assuming that the scan is symmetric such that reading at the midpoint
        # of scan.ranges corresponds to the reading directly in front of the robot
        assert(scan.angle_min == -scan.angle_max)

        # Calculate essential measurements
        range_mid_index = len(scan.ranges) / 2
        front_ranges = Utils.get_scan_within_angle(scan, -Constants.OBSTACLE_DETECTION_FRONT_CONE_ANGLE / 2, Constants.OBSTACLE_DETECTION_FRONT_CONE_ANGLE / 2)
        left_ranges = Utils.get_scan_within_angle(scan, math.pi / 2 - Constants.WALL_FOLLOW_CONE_ANGLE / 2, math.pi / 2 + Constants.WALL_FOLLOW_CONE_ANGLE / 2)
        right_ranges = Utils.get_scan_within_angle(scan, -math.pi / 2 - Constants.WALL_FOLLOW_CONE_ANGLE / 2, -math.pi / 2 + Constants.WALL_FOLLOW_CONE_ANGLE / 2)
        min_dist = reduce(min, scan.ranges)
        min_dist_obstacle = reduce(min, front_ranges)
        min_dist_right = reduce(min, right_ranges)
        min_dist_left = reduce(min, left_ranges)
        self.decide_wall_affinity(min_dist_left, min_dist_right, scan.range_max)

        ctl_min_dist = min_dist
        if self.wall_affinity == WallFollowerNode.WallAffinity.RIGHT:
            ctl_min_dist = min_dist_right
        elif self.wall_affinity == WallFollowerNode.WallAffinity.LEFT:
            ctl_min_dist = min_dist_left

        error = Constants.TARGET_DIST - ctl_min_dist
        error_change = error - self.last_error
        self.last_error = error
        
        # PD Controller Calculation
        prop_ctl = Constants.PROPORTIONAL_GAIN * error
        deriv_ctl = Constants.DERIVITIVE_GAIN * error_change
        control = prop_ctl + deriv_ctl
        if self.wall_affinity is WallFollowerNode.WallAffinity.RIGHT:
            self.ang_vel = control
        elif self.wall_affinity is WallFollowerNode.WallAffinity.LEFT:
            self.ang_vel = -control
        else:
            self.ang_vel = 0

        action = "NONE"
        if min_dist_obstacle < Constants.OBSTACLE_DETECTION_STOP_DISTANCE:
            action = "Stopping for obstacle"
            self.state = WallFollowerNode.State.BLOCKED_OBSTACLE
            self.lin_vel = 0
        else:
            action = "Following wall"
            self.state = WallFollowerNode.State.FOLLOWING_WALL
            self.lin_vel = Constants.LINEAR_VEL

        printVerbose("CONTROLLING")
        printVerbose("\tDist(obstacle)=%0.2f (left)=%0.2f   (right)=%0.2f (min)=%0.2f" % (min_dist_obstacle, min_dist_left, min_dist_right, min_dist))
        printVerbose("\tTarget=%0.2f         Error=%0.2f   Error(change)=%0.2f" % (Constants.TARGET_DIST, error, error_change))
        printVerbose("\tNetCtl=%0.2f         PropCtl=%0.2f  DerivCtl=%0.2f" % (control,prop_ctl, deriv_ctl))
        printVerbose("\tWallAffty=%s   AngVel=%0.2f   LinVel=%0.2f" % (WallFollowerNode.WallAffinity.get_string(self.wall_affinity), self.ang_vel, self.lin_vel))
        printVerbose("\tAction=%s" % action)

    def decide_wall_affinity(self, min_dist_left, min_dist_right, range_max):
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
    
    # Wait before starting everything to give the stage time to load the world
    time.sleep(Constants.STARTUP_DELAY_SEC)

    is_verbose = rospy.get_param("verbose", True)
    followerNode = WallFollowerNode()
    followerNode.spin()
