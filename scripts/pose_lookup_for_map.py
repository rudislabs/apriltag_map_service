#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import copy
import re
import importlib
import json
import numpy as np
import roslib
import rospy
import rospkg
import sensor_msgs.msg
import apriltag3_ros.msg
import apriltag_map_service.msg
import numpy as np
import math as m

roslib.load_manifest('apriltag_map_service')


def toQt_xyzw(RPY): #Roll (X), Pitch (Y), Yaw (Z)
    if len(RPY) == 6:
        R, P, Y = [RPY[3],RPY[4],RPY[5]]
    elif len(RPY) == 3:
        R, P, Y = [RPY[0],RPY[1],RPY[2]]

    #Abbreviations for the various angular functions
    cy = m.cos(Y*0.5)
    sy = m.sin(Y*0.5)
    cp = m.cos(P*0.5)
    sp = m.sin(P*0.5)
    cr = m.cos(R*0.5)
    sr = m.sin(R*0.5)

    qw= cy*cp*cr + sy*sp*sr
    qx= cy*cp*sr - sy*sp*cr
    qy= sy*cp*sr + cy*sp*cr
    qz= sy*cp*cr - cy*sp*sr
    return ([qx, qy, qz, qw])


def toEA_RPY(qt):
    if len(qt) == 7:
        qx,qy,qz,qw = [qt[3],qt[4],qt[5],qt[6]]
    elif len(qt) == 4:
        qx,qy,qz,qw = [qt[0],qt[1],qt[2],qt[3]]
    #roll (x-axis rotation)
    sinr_cosp = 2.0*(qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0*(qx*qx + qy*qy)
    R = m.atan2(sinr_cosp, cosr_cosp)

    #pitch (y-axis rotation)
    sinp = 2.0*(qw*qy - qz*qx)
    if (m.fabs(sinp) >= 1):
        P = m.copysign(m.pi/2, sinp) # use 90 degrees if out of range
    else:
        P = m.asin(sinp)

    #yaw (z-axis rotation)
    siny_cosp = 2.0*(qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
    Y = m.atan2(siny_cosp, cosy_cosp)
    return([R, P, Y])


def easyDEG_RPY(RPY):
    if len(RPY) == 6:
        Rdeg, Pdeg, Ydeg = [(180/m.pi)*RPY[3],(180/m.pi)*RPY[4],(180/m.pi)*RPY[5]]
    elif len(RPY) == 3:
        Rdeg, Pdeg, Ydeg = [(180/m.pi)*RPY[0],(180/m.pi)*RPY[1],(180/m.pi)*RPY[2]]
    """print([Rdeg, Pdeg, Ydeg])
                if Rdeg > 180:
                    Rdeg = Rdeg-360
                if Rdeg <= -180:
                    Rdeg = Rdeg+360
                    
                if Pdeg > 180:
                    Pdeg = Pdeg-360
                if Pdeg <= -180:
                    Pdeg = Pdeg+360
                    
                if Ydeg > 180:
                    Ydeg = Ydeg-360
                if Ydeg <= -180:
                    Ydeg = Ydeg+360"""
    return([Rdeg, Pdeg, Ydeg])
    
def qmult(q1, q0):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


class Map_Locate(object):

    def __init__(self):

        #JSON providing for dictionary lookup of Apriltag IDs to pose in map, assumption 
        #is standard coordinates for map points but any can be used with map_coord_system param
        apriltag_map_file = rospy.get_param("~apriltag_map_json", 'testaprilpose.json')
        #camera is in FRD from apriltag3_ros, where Z is forward, X is right, Y is down
        #below rotations and offsets can be performed in quaternion pose to locate camera from device
        #self.cam_to_dev = np.fromstring(rospy.get_param("~camera_to_device_pose",'[0.0,0.0,0.0,0.0,0.0,0.0]'))
        #map is assumed to be in standard coord, but using this any coord system can be implemented **TODO**
        self.map_coord = rospy.get_param("~map_coord_system",'standard')
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltag_map_service')
        apriltag_map_json_path = os.path.join(self.pkg_path, 'ipython', apriltag_map_file)
        with open(apriltag_map_json_path, 'r') as atmpf:
            self.AprilMapPoseDict = json.load(atmpf)

        # publishers
        self.AprilMapPose_pub = rospy.Publisher(
            "AprilMapPose", apriltag_map_service.msg.AprilMapPose, queue_size=0)
        # subscriptions
        self.apriltag3_ros_sub = rospy.Subscriber(
            "AprilTagDetectionArray", apriltag3_ros.msg.AprilTagDetectionArray, self.callback)

    def callback(self, data):
        
        apriltag_detect = data
        AprilMapPose_msg = apriltag_map_service.msg.AprilMapPose()
        #print(apriltag_detect.detections[0].pose.pose.pose.position)
        for tgs in range(0,len(apriltag_detect.detections)):
            apriltag_found = apriltag_detect.detections[tgs]
            AprilMapPose_msg.map_lookup_pose = self.apriltag_ID_to_map(apriltag_found.pose.pose.pose, apriltag_found.id[0])

            #AprilMapPose_msg.header.stamp = apriltag_found.header.stamp
            AprilMapPose_msg.apriltag_id = apriltag_found.id[0]
            self.AprilMapPose_pub.publish(AprilMapPose_msg)

    def apriltag_ID_to_map(self, atrd, apriltag_id):
        apriltag_key = "TAG_{}".format(apriltag_id)
        aptg_rd_pos = [atrd.position.x, atrd.position.y, atrd.position.z, atrd.orientation.x ,atrd.orientation.y, atrd.orientation.z, atrd.orientation.w]
        print("Found Apriltag Label Map Location: {}".format(self.AprilMapPoseDict[apriltag_key]))
        if self.map_coord == 'standard':
            ApriltagReadMapTransM = [aptg_rd_pos[0],-aptg_rd_pos[2],aptg_rd_pos[1]]
            ApriltagReadRotRad = toEA_RPY(aptg_rd_pos[3:])
            ApriltagReadRotDeg = easyDEG_RPY(ApriltagReadRotRad)
            ApriltagReadMapRotRad = [ApriltagReadRotRad[0], -ApriltagReadRotRad[2], ApriltagReadRotRad[1]]
            ApriltagReadMapRotDeg = easyDEG_RPY(ApriltagReadMapRotRad)
            ApriltagReadMapRotQt = toQt_xyzw(ApriltagReadMapRotRad)

        print("Apriltag Read Pose no Map Coord: {},{}".format(aptg_rd_pos[0:3], ApriltagReadRotDeg))
        print("Apriltag Read Pose Map Coord: {},{}".format(ApriltagReadMapTransM, ApriltagReadMapRotDeg))
        aptg_map_msg = geometry_msgs.msg.Pose()
        aptg_map_msg.position.x = ApriltagReadMapTransM[0]
        aptg_map_msg.position.y = ApriltagReadMapTransM[1]
        aptg_map_msg.position.z = ApriltagReadMapTransM[2]
        aptg_map_msg.orientation.x = ApriltagReadMapRotQt[0]
        aptg_map_msg.orientation.y = ApriltagReadMapRotQt[1]
        aptg_map_msg.orientation.z = ApriltagReadMapRotQt[2]
        aptg_map_msg.orientation.w = ApriltagReadMapRotQt[3]
        return aptg_map_msg



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
