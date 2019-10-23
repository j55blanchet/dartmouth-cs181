#!/usr/bin/env python

from __future__ import print_function, division

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

TOPIC_TIMEOUT = 20

# Online Documentation Links
#   
#   All Visualization Messages: http://wiki.ros.org/visualization_msgs
#   Marker: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
#   Point: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Point.html
#   OccupancyGrid: http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html

def find_path(start, goal):
    """Finds a path from the starting node to the goal node, based off an occupancy grid 
    
    Arguments:
        start {Point} -- The starting pose (must be in occupancy grid reference frame)
        goal {Point} -- The goal pose (must be in the occupancy grid reference frame)
    
    Keyword Arguments:
        grid_topic {str} -- The topic where an OccupancyGrid is published (default: {"/map"})
    
    Returns:        
        [Pose] -- A path (sequence of poses) that the robot should take to get to the destination (empty if this is impossible)
    """

    poses = []
    # TODO: Implement A* Algorithm
    return poses

def publish_path(pose_seq):
    pass

def get_cell_for_point(grid, p):
    """Gets the column and row that a point falls on
    
    Arguments:
        grid {OccupancyGrid} -- The grid to detect the cell of
        p {Point} -- The location within the grid to find the cell of
    
    Returns:
        (Int, Int) -- The (column, row) of the 
    """
    col = int(grid.info.width * grid.info.resolution / p.x)
    row = int(grid.info.width * grid.info.resolution / p.y)
    return (col, row)

def get_cell_data(grid, col, row):
    return grid.data[col * grid.info.height + row]

def get_point_for_cell(grid, col, row):
    x = grid.info.origin.position.x + grid.info.resolution * col
    y = grid.info.origin.position.y + grid.info.resolution * row
    return Point(x=x, y=y)



def visualize_grid_occupancy(grid):
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=20)

    row = 0
    rate = rospy.Rate(4)

    while not rospy.is_shutdown():

        points = []
        colors = []

        for col in range(grid.info.width):
            data = get_cell_data(grid, col, row)
            point = get_point_for_cell(grid, col, row)
            points.append(point)
            if data > 50:
                colors.append(ColorRGBA(1.0, 0.0, 0.0, 0.5))
            else:
                colors.append(ColorRGBA(0.0, 1.0, 0.0, 0.5))

        marker = Marker(
            type=Marker.POINTS,
            action=Marker.ADD,
            id=0,
            lifetime=rospy.Duration(1.5),
            scale=Vector3(0.5, 0.5, 0.5),
            header=Header(frame_id='map'),
            points=points,
            colors=colors
        )
        marker_pub.publish(marker)
        row = (row + 1) % grid.info.height
        rate.sleep()

if __name__ == "__main__":  
    rospy.init_node("pa3-path-finder")
    grid = rospy.wait_for_message("/map", OccupancyGrid, TOPIC_TIMEOUT)
    visualize_grid_occupancy(grid)

    # pose_seq = find_path(None, None)
    # publish_path(pose_seq)