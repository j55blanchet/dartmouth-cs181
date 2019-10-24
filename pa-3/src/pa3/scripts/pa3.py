#!/usr/bin/env python

from __future__ import print_function, division

import math
import random
import time
from Queue import PriorityQueue

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker



# Online Documentation Links
#   
#   All Visualization Messages: http://wiki.ros.org/visualization_msgs
#   Marker: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
#   Point: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html
#   OccupancyGrid: http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html


# Utilities & Constants
GRID_FIND_TIMEOUT = 5
VISUALIZATION_PERSIST_TIME = 25 # in seconds
POSE_VISUALIZATION_ADVANCE_RATE = 16 # Hz
SQRT2 = math.sqrt(2)

def format_point(point):
    return "(%0.3f, %0.3f)" % (point.x, point.y)


class PathFinder:

    def __init__(self, grid):
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=20)
        self.poseseq_pub = rospy.Publisher("/pose_sequence", PoseStamped, queue_size=1)
        self.grid = grid

        # Give roscore enough time to connect publishers and subscribers
        time.sleep(0.2) 

    def find_path(self, start, goal):
        """Finds a path from the starting node to the goal node, based off an occupancy grid

        Arguments:
            grid {OccupancyGrid} -- The grid with obstacles to find a path around
            start {Point} -- The starting pose (must be in occupancy grid reference frame)
            goal {Point} -- The goal pose (must be in the occupancy grid reference frame)
        
        Keyword Arguments:
            grid_topic {str} -- The topic where an OccupancyGrid is published (default: {"/map"})
        
        Returns:        
            [PoseStamped] -- A path (sequence of poses) that the robot should take to get to the destination (empty if this is impossible)
        """
        (start_col, start_row) = self.get_cell_for_point(start)
        (goal_col, goal_row) = self.get_cell_for_point(goal)

        if start_col < 0 or start_col >= self.grid.info.width or start_row < 0 or start_row >= self.grid.info.height or \
        goal_col  < 0 or goal_col  >= self.grid.info.width or goal_row  < 0 or goal_row  >= self.grid.info.height:       
            rospy.logwarn("No path could be found. Starting point %s or ending point %s were out of bounds" % (format_point(start), format_point(goal)))
            return []

        self.visualize_cell(start_col, start_row, ColorRGBA(0, 1.0, 1.0, 1.0))
        self.visualize_cell(goal_col, goal_row, ColorRGBA(0, 0.7, 0, 1.0))

        if self.is_occupied(start_col, start_row):
            rospy.logwarn("No path could be found. Starting point %s is in an occupied cell" % format_point(start))
            return []
        
        if self.is_occupied(goal_col, goal_row):
            rospy.logwarn("No path could be found. Goal point %s is in an occupied cell" % format_point(goal))
            return []

        if start_col is goal_col and start_row is goal_row:
            rospy.loginfo("Start point %s and end point %s are in the same occupancy cell (%d, %d). Returning empty path" %(format_point(start), format_point(goal), start_col, start_row))
            return []
            
        def calc_priority(col, row, cost):
            """Calculates and returns the priority of the cell based of the existing path cost and remaining path heuristic"""
            return cost + math.hypot(goal_col - col, goal_row - row)

        def get_empty_neighbors(col, row):
            """Returns the neighbor cells in the form of (col, row, delta_cost) to the current cell, using 8-way connectivity"""
            neighbors = [
                (col - 1, row - 1, SQRT2), (col, row - 1, 1), (col + 1, row - 1, SQRT2),
                (col - 1, row, 1),                            (col + 1, row, 1),
                (col - 1, row + 1, SQRT2), (col, row + 1, 1), (col + 1, row + 1, SQRT2)
            ]
            onmap_neighbors = [n for n in neighbors if n[0] >= 0 and n[0] < self.grid.info.width and n[1] >= 0 and n[1] < self.grid.info.height]
            return [n for n in onmap_neighbors if not self.is_occupied(col, row)]

        # This function uses an A* algorithm. 
        #   > The priority is calculated as cost(node) + heuristic(node).
        #   > cost(node) = path length from start to node
        #   > heuristic(node) = euclidian distance from node to goal

        #
        # Entries in the frontier are formatted as (priority, col, row, path_cost)
        #   > using a priority queue ensures that we'll always be processing the node
        #     with the lowest estimated total path cost (which is the priority)
            
        min_path_costs = {
            (start_col, start_row): 0
        }
        back_pointers = {}

        frontier = PriorityQueue()
        frontier.put((calc_priority(start_col, start_row, 0), start_col, start_row, 0))

        
        while not frontier.empty():
            (_, col, row, path_cost) = frontier.get()

            # If there was a quicker way found to this node, ignore it
            if path_cost is not min_path_costs[(col, row)]:
                continue

            neighbors = get_empty_neighbors(col, row)

            for (ncol, nrow, dcost) in neighbors:
                npath_cost = path_cost + dcost

                # If this node has already been explored with a faster min_path_cost, ignore it
                if (ncol, nrow) in min_path_costs and min_path_costs[(ncol, nrow)] <= npath_cost:
                    continue

                min_path_costs[(ncol, nrow)] = npath_cost
                npriority = calc_priority(ncol, nrow, npath_cost)
                frontier.put((npriority, ncol, nrow, npath_cost))
                back_pointers[(ncol, nrow)] = (col, row)

                if ncol is goal_col and nrow is goal_row:
                    break

            if (goal_col, goal_row) in back_pointers:
                break

        # Reconstruct the path (while loop won't execute if no path was found)
        poses = []
        final_path_length = 0
        cell = (goal_col, goal_row)
        seq = 0
        while cell in back_pointers:
            point = self.get_point_for_cell(cell[0], cell[1])
            pose = PoseStamped(
                header=Header(
                    seq=seq,
                    frame_id='map'),
                pose=Pose(position=point)
            )
            # TODO: Determine orientation
            poses.append(pose)
            cell = back_pointers[cell]
            seq += 1

            npoint = self.get_point_for_cell(cell[0], cell[1])
            final_path_length += math.hypot(point.x - npoint.x, point.y - npoint.y)

        if len(poses) is 0:
            rospy.loginfo("No path was found from start %s to goal %s. Returning empty path" % (format_point(start), format_point(goal)))
        else:
            rospy.loginfo("Found path of length %0.2fm found from start %s to goal %s (%d cells)" % (final_path_length, format_point(start), format_point(goal), len(poses)))
        return poses
    
    def publish_path(self, pose_seq):

        # Add visualization to rviz
        marker = Marker(
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                id=int(random.random() * 1000),
                lifetime=rospy.Duration(secs=VISUALIZATION_PERSIST_TIME), # forever
                scale=Vector3(0.1, 0.1, 0),
                header=Header(frame_id='map'),
                points=[p.pose.position for p in pose_seq],
                color=ColorRGBA(0.0, 1.0, 0.0, 1.0)
        )
        self.marker_pub.publish(marker)

        # Publish poses gradually (helps visualize progression)
        rate = rospy.Rate(POSE_VISUALIZATION_ADVANCE_RATE)
        for stampedpose in reversed(pose_seq):
            self.poseseq_pub.publish(stampedpose)
            rate.sleep()
    
    def get_cell_for_point(self, p):
        """Gets the column and row that a point falls on
        
        Arguments:
            grid {OccupancyGrid} -- The grid to detect the cell of
            p {Point} -- The location within the grid to find the cell of
        
        Returns:
            (Int, Int) -- The (column, row) of the 
        """
        col = int(p.x / self.grid.info.resolution)
        row = int(p.y / self.grid.info.resolution)
        return (col, row)

    def is_occupied(self, col, row):
        return self.grid.data[col + self.grid.info.width * row] > 50

    def get_point_for_cell(self, col, row):
        x = self.grid.info.origin.position.x + self.grid.info.resolution * col
        y = self.grid.info.origin.position.y + self.grid.info.resolution * row
        return Point(x=x, y=y)

    def visualize_cell(self, col, row, color):
        marker = Marker(
                type=Marker.SPHERE,
                action=Marker.ADD,
                id= (1 + col * self.grid.info.width + row),
                lifetime=rospy.Duration(secs=VISUALIZATION_PERSIST_TIME),
                scale=Vector3(0.5, 0.5, 0.0),
                header=Header(frame_id='map'),
                pose=Pose(position=self.get_point_for_cell(col, row)),
                points=[self.get_point_for_cell(col, row)],
                color=color
        )
        self.marker_pub.publish(marker)

if __name__ == "__main__":  
    rospy.init_node("path_finder")

    grid = rospy.wait_for_message("/map", OccupancyGrid, GRID_FIND_TIMEOUT)
    pathfinder = PathFinder(grid)

    w = grid.info.resolution * grid.info.width
    h = grid.info.resolution * grid.info.height

    start = Point(random.random() * w, random.random() * h, 0)
    end = Point(random.random() * w, random.random() * h, 0)

    pose_seq = pathfinder.find_path(start, end)
    pathfinder.publish_path(pose_seq)