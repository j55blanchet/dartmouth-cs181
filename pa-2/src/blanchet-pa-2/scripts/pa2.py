#!/usr/bin/env python

from __future__ import print_function, division
import rospy
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from functools import reduce

class Constants:
    STARTUP_DELAY_SEC = 3

    RATE = 20
    LINEAR_VEL = 2

    PROPORTIONAL_GAIN = 6
    DERIVITIVE_GAIN = 11.0
    DERIVITIVE_HISTORY_COUNT = 3

    TARGET_DIST = 0.6
    DANGER_STOP_DIST = TARGET_DIST * 0.9
    WARNING_SLOW_DIST =  DANGER_STOP_DIST + 0.3
    
    WALL_FOLLOW_CONE_ANGLE = math.pi / 60 # 3 deg

    WALL_FIND_DISTANCE = TARGET_DIST * 1.4
    WALL_LOST_DISTANCE = 2 * WALL_FIND_DISTANCE

    # Parameters controlling when to stop for obstacles
    #   * If an obstacle is detecting in the front 60 deg of
    #     the robot withing 0.5m, the robot will stop
    OBSTACLE_DETECTION_FRONT_CONE_ANGLE = math.pi / 3 # 60 degrees

    BEHAVIOR_TURNIN_REQUIRED_RATIO = 1.4
    BEHAVIOR_TURNIN__ANG_VEL = 2
    BEHAVIOR_TURNOUT_REQUIRED_RATIO = 1.4
    BEHAVIOR_TURNOUT_ANG_VEL = 2
    BEHAVIOR_WALLFIND_INITIAL_ANG_VEL = 1.5

is_verbose = True


# Print if verbosity is enabled.
#  * Referenced: https://stackoverflow.com/questions/26286203/custom-print-function-that-wraps-print
def printVerbose(*args, **kwargs):
    if is_verbose:
        print("".join(map(str,args)), **kwargs)

class Utils:
    """Utility class containing various useful static methods"""

    @staticmethod
    # Copied from https://www.raspberrypi.org/forums/viewtopic.php?t=149371
    def valmap(value, istart, istop, ostart, ostop):
        """Remaps a value from one range to another range
        
        Arguments:
            value {Number} -- Input value
            istart {float} -- Input range start
            istop {float} -- Input range end
            ostart {float} -- Remapped (output) range start
            ostop {float} -- Remapped (output) range end
        
        Returns:
            float -- Remapped value
        """
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

class WallFollowerNode:

    class State:
        INITIALIZING =      0   # Robot is starting up; don't do anything yet
        FIND_WALL =         1   # Robot is too far away from any walls and will go in spirals until it finds a wall to follow
        FOLLOWING_WALL =    2   # Standard Mode: Following a wall. Wall affinity should be set at this point
        FOLLOW_WALL_SLOW =  3   # Slow mode: Robot is following a wall, but an obstacle is approaching. Turn normally, but slow down forward momentum
        TURN_IN =           4   # Robot is going around a corner and needs to turn into the wall
        STOPPED_TURNOUT =   5   # Robot is too close to something and needs to turn away from the wall it normally follow
        
        @staticmethod 
        def get_string(state):
            return {
                WallFollowerNode.State.INITIALIZING: "INITIALIZING",
                WallFollowerNode.State.FIND_WALL: "FIND_WALL",
                WallFollowerNode.State.FOLLOWING_WALL: "FOLLOWING_WALL",
                WallFollowerNode.State.FOLLOW_WALL_SLOW: "FOLLOW_WALL_SLOW",
                WallFollowerNode.State.TURN_IN: "TURN_IN",
                WallFollowerNode.State.STOPPED_TURNOUT: "STOPPED_TURNOUT",
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
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_sub = rospy.Subscriber("base_scan", LaserScan, self.on_scan, queue_size=1)
        self.rate = rospy.Rate(Constants.RATE)

        # Instance variables
        self.state = WallFollowerNode.State.INITIALIZING
        self.wall_affinity = WallFollowerNode.WallAffinity.UNDECIDED
        self.error_history = [0] # start with an error in the history to prevent divide by zero error
        self.last_scan = None
        self.last_processed_scan = None
        self.wallfind_ang_vel = Constants.BEHAVIOR_WALLFIND_INITIAL_ANG_VEL

    def on_scan(self, msg):
        # printVerbose("Got msg #%d" % msg.header.seq)
        self.last_scan = msg

    def _get_dist_at_heading(self, heading):
        """Returns the scan reading nearest the specified heading        
        Arguments:
            heading {float} -- The direction to find the distance to (+ is in the direction of wall affinity)
        Returns:
            float -- Distance reading at given heading (0 if not found)
        """
        if self.last_scan is None:
            return 0        
        scan = self.last_scan

        if self.wall_affinity is WallFollowerNode.WallAffinity.LEFT:
            heading = -heading

        index = Utils.valmap(heading, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1)
        return scan.ranges[int(index)]

    def _get_min_dist_within_sector(self, min_angle, max_angle):
        """Returns the distance to the nearest point in the given sector (+ is in the direction of wall affinity)
        
        Arguments:
            min_angle {float} -- Start of sector
            max_angle {float} -- End of sector
        
        Returns:
            float -- Distance to nearest point in sector
        """
        ranges = self._get_scan_within_sector(min_angle, max_angle)
        return reduce(min, ranges)

    def _get_scan_within_sector(self, min_angle, max_angle, debugging=False):
        """Returns the subset of the scan's readings that fall within the angles specified, accounting for wall affinity
        
        Arguments:
            scan_msg {LaserScan} -- The scan whose .ranges readings we'll filter
            min_angle {float} -- The lower bound of the angle range to return readings of
            max_angle {float} -- The upper bound of the angle range to return readings of
        
        Returns:
            List[float] -- Readings that fall within the angles specified
        """
        if self.last_scan is None:
            return []        
        scan = self.last_scan

        # for left wall-affinity, we need to negate the angle readings to have positive
        # be towards the wall-side
        if self.wall_affinity is WallFollowerNode.WallAffinity.RIGHT:
            min_angle = -min_angle
            max_angle = -max_angle
            
        if min_angle > max_angle:
            swap = min_angle
            min_angle = max_angle
            max_angle = swap

        if min_angle < scan.angle_min:
            min_angle = scan.angle_min
        if max_angle > scan.angle_max:
            max_angle = scan.angle_max
            
        index_start = int(Utils.valmap(min_angle, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1))
        index_end = int(Utils.valmap(max_angle, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1))
        return scan.ranges[int(index_start): int(index_end)]

    def compute_next_state(self):
        """Compute the next state of the state machine, based off current inputs"""
        
        scan = self.last_scan
        min_dist = self._get_min_dist_within_sector(-math.pi, math.pi)

        # First, check if we're even near a well. If far away, enter recovery mode (an expanding spiral)
        if min_dist > Constants.WALL_LOST_DISTANCE:
            # If we just lost the wall, print a warning message and commence the search spiral
            if self.state is not WallFollowerNode.State.FIND_WALL:
                printVerbose("Robot lost the wall that it was following. Entering recovery mode: find wall")
                self.wall_affinity = WallFollowerNode.WallAffinity.UNDECIDED
                self.state = WallFollowerNode.State.FIND_WALL
                self.wallfind_ang_vel = Constants.BEHAVIOR_WALLFIND_INITIAL_ANG_VEL
            return
        
        # Ensure we're fairly close to a wall to chose our wall affinity.
        if min_dist > Constants.WALL_FIND_DISTANCE and self.wall_affinity is WallFollowerNode.WallAffinity.UNDECIDED:
            self.state = WallFollowerNode.State.FIND_WALL
            return            

        self.decide_wall_affinity()

        # We're assuming that the scan is symmetric such that reading at the midpoint
        # of scan.ranges corresponds to the reading directly in front of the robot
        assert(scan.angle_min == -scan.angle_max)

        # We'll use a few key readings to determine state. "In" refers to the direction of wall-affinity
        fw_dist = self._get_min_dist_within_sector(-Constants.OBSTACLE_DETECTION_FRONT_CONE_ANGLE / 2, Constants.OBSTACLE_DETECTION_FRONT_CONE_ANGLE / 2)
        fw_in_dist = self._get_min_dist_within_sector(math.pi / 4, math.pi / 2)
        bw_in_dist = self._get_min_dist_within_sector(math.pi / 2, math.pi * 3/4)        
        
        if fw_dist < Constants.DANGER_STOP_DIST:
            # If we're too close to the wall, we must stop. This prevents
            # us from hitting obstacles
            self.state = WallFollowerNode.State.STOPPED_TURNOUT
        elif fw_in_dist > bw_in_dist * Constants.BEHAVIOR_TURNIN_REQUIRED_RATIO:
            # Bigger gap in front then in back: turn in
            self.state = WallFollowerNode.State.TURN_IN
        elif fw_dist < Constants.WARNING_SLOW_DIST:
            # Slow down if we're approaching a wall (this gives the robot more time to turn and avoid it)
            self.state = WallFollowerNode.State.FOLLOW_WALL_SLOW
        else:
            # The normal (default) state
            self.state = WallFollowerNode.State.FOLLOWING_WALL 
        
        return

    def compute_control(self):
        """Determine control for robot (based off the state machine)
        
        Returns:
            (float, float) -- Linear velocity and angular velocity for control
        """
        if self.state is WallFollowerNode.State.INITIALIZING:
            printVerbose("Initializing.")
            return (0, 0)
        

        inwards_min_dist = self._get_min_dist_within_sector(math.pi / 2 - Constants.WALL_FOLLOW_CONE_ANGLE / 2, 
                                                            math.pi / 2 + Constants.WALL_FOLLOW_CONE_ANGLE / 2)
        error = inwards_min_dist - Constants.TARGET_DIST

        # Note: for calculating the derivitive term, we're using an error history. This is because
        #       scans come back frequently and the derivitive term is unreliable if we only look
        #       at the most recent term
        recent_error_avg = sum(self.error_history) / len(self.error_history)
        error_change = error - recent_error_avg
        self.error_history.append(error)
        if len(self.error_history) > Constants.DERIVITIVE_HISTORY_COUNT:
            self.error_history.pop(0)
        
        # PD Controller Calculation
        prop_ctl = Constants.PROPORTIONAL_GAIN * error
        deriv_ctl = Constants.DERIVITIVE_GAIN * error_change
        inward_control = prop_ctl + deriv_ctl

        # If we're following the right wall, we want a positive control to turn the robot
        # clockwise, so we need to negate the angular velocity (& later adjustments)
        turn_in_sign = 1
        if self.wall_affinity is WallFollowerNode.WallAffinity.RIGHT:
            turn_in_sign = -1
        
        actual_control = inward_control * turn_in_sign
        ang_vel = actual_control
        lin_vel = Constants.LINEAR_VEL

        if self.state is WallFollowerNode.State.FIND_WALL:
            # Slowly decrease the angular velocity to make the robot go in bigger and bigger spirals
            ang_vel = self.wallfind_ang_vel * 0.995
            self.wallfind_ang_vel = ang_vel
        
        elif self.state is WallFollowerNode.State.TURN_IN:
            # When going around corners, give robot a boost to get around the corner
            ang_vel += Constants.BEHAVIOR_TURNIN__ANG_VEL * turn_in_sign

        elif self.state is WallFollowerNode.State.STOPPED_TURNOUT:
            # Stop the robot and make it turn away from wall
            ang_vel = -Constants.BEHAVIOR_TURNOUT_ANG_VEL * turn_in_sign
            lin_vel = 0
        
        elif self.state is WallFollowerNode.State.FOLLOW_WALL_SLOW:
            # Slow down when approaching a wall to give more time for robot to make adjustments
            lin_vel /= 2

        self.last_processed_scan = self.last_scan

        printVerbose("CONTROLLING")
        printVerbose("\tMeasure=%0.2f           Target=%0.2f" % (inwards_min_dist, Constants.TARGET_DIST))
        printVerbose("\tError=%0.2f             Error(change)=%0.2f" % (error, error_change))
        printVerbose("\tControl(in)=%0.2f           Cp=(%0.2f)   + Cd=(%0.2f)" % (inward_control, prop_ctl, deriv_ctl))
        printVerbose("\tLinVel=%0.2f            AngVel=%0.2f" % (lin_vel, ang_vel))
        printVerbose("\tState=%s   WallAffty=%s" % (WallFollowerNode.State.get_string(self.state).ljust(15), WallFollowerNode.WallAffinity.get_string(self.wall_affinity)))
        
        return (lin_vel, ang_vel)

    def decide_wall_affinity(self):
        """Set wall affinity if not already set"""
        if self.wall_affinity is not WallFollowerNode.WallAffinity.UNDECIDED:
            # We've already decided on a wall to follow
            return
        
        ldist = self._get_min_dist_within_sector(math.pi, 0)
        rdist = self._get_min_dist_within_sector(-math.pi, 0)

        self.wall_affinity = WallFollowerNode.WallAffinity.RIGHT if rdist < ldist else WallFollowerNode.WallAffinity.LEFT
        printVerbose("Decided wall affinity. ldist: %0.2f. rdist: %0.2f" % (ldist, rdist))
        printVerbose("   ==> Chosen Affinity: %s" % WallFollowerNode.WallAffinity.get_string(self.wall_affinity))
        
    def spin(self):
        """Start control loop of this controller node"""
        while not rospy.is_shutdown():
            if self.last_scan is not None and self.last_scan is not self.last_processed_scan:
                self.compute_next_state()
                (lin_vel, ang_vel) = self.compute_control()
                cmd_vel = Twist()
                cmd_vel.linear.x = lin_vel
                cmd_vel.angular.z = ang_vel
                self.cmd_pub.publish(cmd_vel)
            self.rate.sleep()

if __name__ == "__main__":
    
    rospy.init_node(WallFollowerNode.__name__)
    delay_before_launch = rospy.get_param("~startDelay", 0)
    printVerbose("Delaying for %d seconds before starting" % delay_before_launch)

    # Wait before starting everything to give the stage time to load the world
    time.sleep(delay_before_launch)

    is_verbose = rospy.get_param("~verbose", True)
    followerNode = WallFollowerNode()
    followerNode.spin()
