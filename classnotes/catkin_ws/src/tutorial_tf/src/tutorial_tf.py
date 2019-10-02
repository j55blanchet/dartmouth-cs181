#!/usr/bin/env python

from __future__ import print_function, division

import rospy
from sensor_msgs.msg import LaserScan
import tf
from numpy import matrix

def laser_callback(msg):
    print(msg.header.stamp.to_sec())
    print(msg.header.frame_id)
    print(msg.angle_min)
    print(msg.angle_increment)
    print(len(msg.ranges))

if __name__ == "__main__":
    rospy.init_node("tutorial_tf")
    rospy.Subscriber("base_scan", LaserScan, laser_callback)

    listener = tf.TransformListener()
    rate = rospy.Rate(0.5) # runs every two seconds

    while not rospy.is_shutdown():
        try:
            # Get the most recent transformation matrix
            #    time(0) indicates most recent transformation
            #    trans is a vector indicating the translation
            #    rot is rotation  represented as a quaternion
            (trans, rot) = listener.lookupTransform('base_link', 'base_laser_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        print(trans, rot)

        # turns translation vector into homogeneous matrix representation
        r = tf.transformations.translation_matrix(trans)

        # turns quaternion representation into a homogeneous matrix
        R = tf.transformations.quaternion_matrix(rot)    

        print(r.dot(R)) # Gets the result matrix (total matrix)
        rate.sleep()