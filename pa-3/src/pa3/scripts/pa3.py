
from __future__ import print_function, division

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


def find_path(start, goal, grid_topic="/map"):
    """Finds a path from the starting node to the goal node, based off an occupancy grid 
    
    Arguments:
        start {Pose} -- The starting pose (must be in occupancy grid reference frame)
        goal {Pose} -- The goal pose (must be in the occupancy grid reference frame)
    
    Keyword Arguments:
        grid_topic {str} -- The topic where an OccupancyGrid is published (default: {"/map"})
    
    Returns:        
        [type] -- [description]
    """

    # First, load the map
    rospy.loginfo("Loading map")
    grid = rospy.wait_for_message(grid_topic, OccupancyGrid, 5)

        
    print("Grid: " + grid)
    poses = []
    
    return poses


def publish_path(pose_seq):
    pass


if __name__ == "__main__":  
    pose_seq = find_path(None, None)
    publish_path(pose_seq)