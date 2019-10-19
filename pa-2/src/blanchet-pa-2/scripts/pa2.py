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

    PROPORTIONAL_GAIN = 2
    DERIVITIVE_GAIN = 80.0

    TARGET_DIST = 0.5
    DANGER_STOP_DIST = 0.5 + 0.25
    WARNING_SLOW_DIST =  DANGER_STOP_DIST + 0.25
    
    WALL_FOLLOW_CONE_ANGLE = math.pi / 60 # 3 deg

    WALL_FIND_DISTANCE = 1 
    WALL_LOST_DISTANCE = 2 * WALL_FIND_DISTANCE

    # Parameters controlling when to stop for obstacles
    #   * If an obstacle is detecting in the front 60 deg of
    #     the robot withing 0.5m, the robot will stop
    OBSTACLE_DETECTION_FRONT_CONE_ANGLE = math.pi / 3 # 60 degrees

    BEHAVIOR_TURNIN_REQUIRED_RATIO = 1.4
    BEHAVIOR_TURNOUT_REQUIRED_RATIO = 1.2
    BEHAVIOR_TURN_BOOST = 1.5
    BEHAVIOR_WALLFIND_INITIAL_ANG_VEL = 1.5

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
        TURN_OUT =          5   # Robot is approaching a corner and needs to turn out from the wall
        STOPPED_TURNIN =    6   # Robot is too close to something and needs to turn towards the wall it was following
        STOPPED_TURNOUT =   7   # Robot is too close to something and needs to turn away from the wall it normally follows
        

        @staticmethod 
        def get_string(state):
            return {
                WallFollowerNode.State.INITIALIZING: "INITIALIZING",
                WallFollowerNode.State.FOLLOWING_WALL: "FOLLOWING_WALL",
                WallFollowerNode.State.FOLLOW_WALL_SLOW: "FOLLOW_WALL_SLOW",
                WallFollowerNode.State.TURN_IN: "TURN_IN",
                WallFollowerNode.State.TURN_OUT: "TURN_OUT",
                WallFollowerNode.State.STOPPED_TURNIN: "STOPPED_TURNIN",
                WallFollowerNode.State.STOPPED_TURNOUT: "STOPPED_TURNOUT"
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
        self.last_scan = None

        self.wallfind_ang_vel = Constants.BEHAVIOR_WALLFIND_INITIAL_ANG_VEL


    def on_scan(self, msg):
        self.last_scan = msg

    def _get_dist_at_heading(self, heading):
        if self.last_scan is None:
            return []        
        scan = self.last_scan

        if self.wall_affinity is WallFollowerNode.WallAffinity.LEFT:
            heading = -heading

        index = Utils.valmap(heading, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1)
        return scan.ranges[int(index)]


    def _get_min_dist_within_sector(self, min_angle, max_angle, debugging=False):
        ranges = self._get_scan_within_sector(min_angle, max_angle, debugging=debugging)
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

        if debugging:
            printVerbose("_get_scan_within_sector --> params: %0.2f, %0.2f" % (min_angle, max_angle))

        if self.last_scan is None:
            return []        
        scan = self.last_scan

        # for left wall-affinity, we need to negate the angle readings to have positive
        # be towards the wall-side
        if self.wall_affinity is WallFollowerNode.WallAffinity.RIGHT:
            min_angle = -min_angle
            max_angle = -max_angle
            if debugging:
                printVerbose("_get_scan_within_sector --> negating angles due to LEFT wall affinity")

        if min_angle > max_angle:
            swap = min_angle
            min_angle = max_angle
            max_angle = swap
            if debugging:
                printVerbose("_get_scan_within_sector --> swapping min and max angle because min is less then max")

        

        if min_angle < scan.angle_min:
            if debugging:
                printVerbose("_get_scan_within_sector --> adjusting min_angle from %0.2f to the scan.angle_min of %0.2f" % (min_angle, scan.angle_min))
            min_angle = scan.angle_min
        if max_angle > scan.angle_max:
            max_angle = scan.angle_max
            if debugging:
                printVerbose("_get_scan_within_sector --> adjusting max from %0.2f to the scan.angle_max of %0.2f" % (max_angle, scan.angle_max))

        index_start = int(Utils.valmap(min_angle, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1))
        index_end = int(Utils.valmap(max_angle, scan.angle_min, scan.angle_max, 0, len(scan.ranges) - 1))
        if debugging:
            printVerbose("_get_scan_within_sector --> returning ranges [%d, %d] out of [%d, %d]" % (index_start, index_end, 0, len(scan.ranges)))
        return scan.ranges[int(index_start): int(index_end)]

    def compute_next_state(self):

        if self.last_scan is None:
            return
        scan = self.last_scan

        min_dist = self._get_min_dist_within_sector(-math.pi, math.pi)
        if min_dist > Constants.WALL_LOST_DISTANCE:
            printVerbose("Robot lost the wall that it was following. Entering recovery mode: find wall")
            self.wall_affinity = WallFollowerNode.WallAffinity.UNDECIDED
            self.state = WallFollowerNode.State.FIND_WALL
            self.wallfind_ang_vel = Constants.BEHAVIOR_WALLFIND_INITIAL_ANG_VEL
            return
        
        if min_dist > Constants.WALL_FIND_DISTANCE and self.wall_affinity is WallFollowerNode.WallAffinity.UNDECIDED:
            self.state = WallFollowerNode.State.FIND_WALL
            return            

        self.decide_wall_affinity()

        # We're assuming that the scan is symmetric such that reading at the midpoint
        # of scan.ranges corresponds to the reading directly in front of the robot
        assert(scan.angle_min == -scan.angle_max)

        fw_dist = self._get_min_dist_within_sector(-Constants.OBSTACLE_DETECTION_FRONT_CONE_ANGLE / 2, Constants.OBSTACLE_DETECTION_FRONT_CONE_ANGLE / 2)
        fw_in_dist = self._get_min_dist_within_sector(math.pi / 4, math.pi / 2)
        bw_in_dist = self._get_min_dist_within_sector(math.pi / 2, math.pi * 3/4)
        # in_dist = min(fw_in_dist, bw_in_dist)
        # fw_in_45_dist = self._get_dist_at_heading(math.pi / 4)
        # fw_out_45_dist = self._get_dist_at_heading(-math.pi / 4)

        
        if fw_dist < Constants.DANGER_STOP_DIST:
            self.state = WallFollowerNode.State.STOPPED_TURNOUT
        elif fw_in_dist > bw_in_dist * Constants.BEHAVIOR_TURNIN_REQUIRED_RATIO:
            self.state = WallFollowerNode.State.TURN_IN
        elif fw_dist < Constants.BEHAVIOR_TURNOUT_REQUIRED_RATIO * Constants.TARGET_DIST:
            self.state = WallFollowerNode.State.TURN_OUT
        elif fw_dist < Constants.WARNING_SLOW_DIST:
            self.state = WallFollowerNode.State.FOLLOW_WALL_SLOW
        else:
            self.state = WallFollowerNode.State.FOLLOWING_WALL 
        
        return

    def compute_control(self):
        
        if self.state is WallFollowerNode.State.INITIALIZING:
            printVerbose("Initializing.")
            return (0, 0)

        inwards_min_dist = self._get_min_dist_within_sector(math.pi / 4, math.pi * 3/4)
        error = inwards_min_dist - Constants.TARGET_DIST
        error_change = error - self.last_error
        self.last_error = error
        
        # PD Controller Calculation
        prop_ctl = Constants.PROPORTIONAL_GAIN * error
        deriv_ctl = Constants.DERIVITIVE_GAIN * error_change
        control = prop_ctl + deriv_ctl


        turn_in_sign = 1
        if self.wall_affinity is WallFollowerNode.WallAffinity.RIGHT:
            turn_in_sign = -1
        
        control *= turn_in_sign
        ang_vel = control
        lin_vel = Constants.LINEAR_VEL

        if self.state is WallFollowerNode.State.FIND_WALL:
            # Slowly decrease the angular velocity to make the robot go in bigger and bigger spirals
            ang_vel = self.wallfind_ang_vel * 0.99
            self.wallfind_ang_vel = control
        # elif self.state is WallFollowerNode.State.FOLLOWING_WALL:
        #     # All good. Use control directly
        #     pass
        elif self.state is WallFollowerNode.State.TURN_IN:
            if ang_vel < 0:
                printVerbose("WARRRRRNINGGGG: control is trying to turn OUT during TURN_IN state")
            ang_vel += Constants.BEHAVIOR_TURN_BOOST * turn_in_sign

        elif self.state is WallFollowerNode.State.TURN_OUT:
            if ang_vel > 0:
                printVerbose("WARRRRRNINGGGG: control is trying to turn IN during TURN_OUT state")
            ang_vel -= Constants.BEHAVIOR_TURN_BOOST * turn_in_sign

        elif self.state is WallFollowerNode.State.STOPPED_TURNIN:
            if ang_vel < 0:
                printVerbose("WARRRRRNINGGGG: control is trying to turn OUT during TURN_IN state")
            ang_vel = Constants.BEHAVIOR_TURN_BOOST * turn_in_sign
            lin_vel = 0
        elif self.state is WallFollowerNode.State.STOPPED_TURNOUT:
            ang_vel = -Constants.BEHAVIOR_TURN_BOOST * turn_in_sign
            lin_vel = 0
        
        elif self.state is WallFollowerNode.State.FOLLOW_WALL_SLOW:
            lin_vel /= 2

        printVerbose("CONTROLLING")
        # printVerbose("\tDist(obstacle)=%0.2f (left)=%0.2f   (right)=%0.2f (min)=%0.2f (ctl)=%0.2f" % (min_dist_obstacle, min_dist_left, min_dist_right, min_dist, ctl_min_dist))
        printVerbose("\tMeasure=%0.2f           Target=%0.2f" % (inwards_min_dist, Constants.TARGET_DIST))
        printVerbose("\tError=%0.2f             Error(change)=%0.2f" % (error, error_change))
        printVerbose("\tControl=%0.2f           Cp%0.2f   + Cd=%0.2f" % (control, prop_ctl, deriv_ctl))
        printVerbose("\tLinVel=%0.2f            AngVel=%0.2f" % (lin_vel, ang_vel))
        printVerbose("\tState=%s   WallAffty=%s" % (WallFollowerNode.State.get_string(self.state).ljust(15), WallFollowerNode.WallAffinity.get_string(self.wall_affinity)))
        
        return (lin_vel, ang_vel)

    def decide_wall_affinity(self):
        if self.wall_affinity is not WallFollowerNode.WallAffinity.UNDECIDED:
            # We've already decided on a wall to follow
            return
        
        ldist = self._get_min_dist_within_sector(math.pi, 0)
        rdist = self._get_min_dist_within_sector(-math.pi, 0)

        self.wall_affinity = WallFollowerNode.WallAffinity.RIGHT if rdist < ldist else WallFollowerNode.WallAffinity.LEFT
        printVerbose("Decided wall affinity. ldist: %0.2f. rdist: %0.2f" % (ldist, rdist))
        printVerbose("   ==> Chosen Affinity: %s" % WallFollowerNode.WallAffinity.get_string(self.wall_affinity))
        
    def spin(self):
        while not rospy.is_shutdown():
            self.compute_next_state()
            (lin_vel, ang_vel) = self.compute_control()
            cmd_vel = Twist()
            cmd_vel.linear.x = lin_vel
            cmd_vel.angular.z = ang_vel
            self.cmd_pub.publish(cmd_vel)
            self.rate.sleep()

if __name__ == "__main__":
    
    delay_before_launch = rospy.get_param("~startDelay", 0)
    printVerbose("Delaying for %d seconds before starting" % delay_before_launch)

    # Wait before starting everything to give the stage time to load the world
    time.sleep(delay_before_launch)

    is_verbose = rospy.get_param("~verbose", True)
    followerNode = WallFollowerNode()
    followerNode.spin()
