#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import copy
import re
import importlib

import numpy as np
import roslib
import rospy
import rospkg
import sensor_msgs.msg
import apritag3_ros.msg
import apriltag_map_service.msg
from visim import label_utils

roslib.load_manifest('apriltag_map_service')


class Map_Locate(object):

    def __init__(self):

        #JSON providing for dictionary lookup of Apriltag IDs to pose in map, assumption 
        #is standard coordinates for map points but any can be used with map_coord_system param
        self.apriltag_map = rospy.get_param("~apriltag_map_json")
        #camera is in FRD from apriltag3_ros, where Z is forward, X is right, Y is down
        #below rotations and offsets can be performed in quaternion pose to locate camera from device
        self.cam_to_dev = rospy.get_param("~camera_to_device_pose")
        #map is assumed to be in standard coord, but using this any coord system can be implemented **TODO**
        self.map_coord = rospy.get_param("~map_coord_system")

        # publishers
        self.AprilMapPose_pub = rospy.Publisher(
            "AprilMapPose", apriltag_map_service.msg.AprilMapPose, queue_size=0)
        # subscriptions
        self.apriltag3_ros_sub = rospy.Subscriber(
            "AprilTagDetectionArray", apritag3_ros.msg.AprilTagDetectionArray.detections, self.callback)

    def callback(self, data):
        
        apriltag_detect = self.data
        AprilMapPose_msg = apriltag_map_service.msg.AprilMapPose()

        for tgs in len(apriltag_detect.detections):
            apriltag_found = apriltag_detect.detections[tgs]
            AprilMapPose_msg.map_lookup_pose = apriltag_ID_to_map(apriltag_found.pose, apriltag_found[0])

            AprilMapPose_msg.header.stamp = apriltag_detect.header.stamp
            AprilMapPose_msg.apriltag_id = apriltag_detect.id[0]
            self.AprilMapPose_pub.publish(AprilMapPose_msg)

    def apriltag_ID_to_map(self, apriltag_read_pose, apriltag_id):



def main(args):
    "main function"
    rospy.init_node('pose_lookup_for_map')
    ld = Map_Locate()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

# vim: set et fenc=utf-8 ft=python ff=unix sts=0 sw=4 ts=4 :
