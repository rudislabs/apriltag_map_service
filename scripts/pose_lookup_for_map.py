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
import geometry_msgs.msg

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


def easyDEG_RPY(RPY, fix_roll):
    if len(RPY) == 6:
        Rdeg, Pdeg, Ydeg = [(180.0/m.pi)*RPY[3],(180.0/m.pi)*RPY[4],(180.0/m.pi)*RPY[5]]
    elif len(RPY) == 3:
        Rdeg, Pdeg, Ydeg = [(180.0/m.pi)*RPY[0],(180.0/m.pi)*RPY[1],(180.0/m.pi)*RPY[2]]
    Rdeg_mod = 0.0
    if fix_roll:
        if Rdeg > 0:
            Rdeg_mod = Rdeg-180
        elif Rdeg < 0:
            Rdeg_mod = Rdeg+180
    Rdeg = Rdeg_mod
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
        #Camera is in FRD from apriltag3_ros, where Z is forward, X is right, Y is down
        #But we pull in device pose update in map pose where Y is forward, X is right, Z is up (standard coordinates)
        self.cam_to_dev = np.fromstring(rospy.get_param("~camera_to_device_poser","0.0,0.0,0.0"), sep=',').astype(float)
        #map is assumed to be in standard coord, but using this any coord system can be implemented **TODO**
        self.map_coord = rospy.get_param("~map_coord_system",'standard')
        
        self.debug_PoseDeg = bool(rospy.get_param("~debug_ApriltagMapPoseDeg",'false'))

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltag_map_service')

        apriltag_map_json_path = os.path.join(self.pkg_path, 'ipython', apriltag_map_file)
        with open(apriltag_map_json_path, 'r') as atmpf:
            self.AprilTagMapPoseDict = json.load(atmpf)

        # publishers
        self.AprilTagMapPose_pub = rospy.Publisher(
            "AprilTagMapPose", apriltag_map_service.msg.AprilTagMapPose, queue_size=0)
        # subscriptions
        self.apriltag3_ros_sub = rospy.Subscriber(
            "AprilTagDetectionArray", apriltag3_ros.msg.AprilTagDetectionArray, self.callback)

    def callback(self, data):
        
        apriltag_detect = data
        AprilTagMapPose_msg = apriltag_map_service.msg.AprilTagMapPose()
        #print(apriltag_detect.detections[0].pose.pose.pose.position)
        for tgs in range(0,len(apriltag_detect.detections)):
            apriltag_found = apriltag_detect.detections[tgs]
            AprilTagMapPose_msg.map_lookup_pose = self.apriltag_ID_to_map(apriltag_found.pose.pose.pose, apriltag_found.id[0])

            #AprilTagMapPose_msg.header.stamp = apriltag_found.header.stamp
            AprilTagMapPose_msg.apriltag_id = apriltag_found.id[0]
            self.AprilTagMapPose_pub.publish(AprilTagMapPose_msg)

    def apriltag_ID_to_map(self, atrd, apriltag_id):
        apriltag_key = "TAG_{}".format(apriltag_id)
        aptg_rd_pos = [atrd.position.x, atrd.position.y, atrd.position.z, atrd.orientation.x ,atrd.orientation.y, atrd.orientation.z, atrd.orientation.w]
        AprilMapLabel = self.AprilTagMapPoseDict[apriltag_key]
        if self.map_coord == 'standard':
            ApriltagReadMapTransM = [-aptg_rd_pos[0]+self.cam_to_dev[0],-aptg_rd_pos[2]+self.cam_to_dev[1],aptg_rd_pos[1]+self.cam_to_dev[2]]
            ApriltagReadRotRad = toEA_RPY(aptg_rd_pos[3:])
            ApriltagReadRotDeg = easyDEG_RPY(ApriltagReadRotRad, True)
            ApriltagReadMapRotRad = [-ApriltagReadRotRad[0], -ApriltagReadRotRad[2], ApriltagReadRotRad[1]]
            ApriltagReadMapRotDeg = easyDEG_RPY(ApriltagReadMapRotRad, False)
            ApriltagReadMapRotQt = toQt_xyzw(ApriltagReadMapRotRad)
            AprilTagMapPoseRad = [AprilMapLabel[0]-(m.sin(AprilMapLabel[5])*ApriltagReadMapTransM[0]), 
                                AprilMapLabel[1]-(m.sin(AprilMapLabel[5])*ApriltagReadMapTransM[1]), 
                                AprilMapLabel[2]+ApriltagReadMapTransM[2], 
                                ApriltagReadMapRotRad[0], ApriltagReadMapRotRad[1], ApriltagReadMapRotRad[2]+(m.pi/2.0+AprilMapLabel[5])]
            ApriltagMapRotDeg = easyDEG_RPY(AprilTagMapPoseRad, False)
            ApriltagMapRotQt = toQt_xyzw(AprilTagMapPoseRad)
            AprilTagMapPoseQt = [AprilTagMapPoseRad[0],AprilTagMapPoseRad[1],AprilTagMapPoseRad[2],ApriltagMapRotQt[0],ApriltagMapRotQt[1],ApriltagMapRotQt[2],ApriltagMapRotQt[3]]
        
        if self.debug_PoseDeg:
        	print("Apriltag Map Pose Deg: [X:{:.2f} m, Y:{:.2f} m, Z:{:.2f} m, R:{:.2f} Deg, P:{:.3f} Deg, Y:{:.3f} Deg]".format(AprilTagMapPoseRad[0],AprilTagMapPoseRad[1],AprilTagMapPoseRad[2],ApriltagMapRotDeg[0],ApriltagMapRotDeg[1],ApriltagMapRotDeg[2]))
        
        aptg_map_msg = geometry_msgs.msg.Pose()
        aptg_map_msg.position.x = AprilTagMapPoseQt[0]
        aptg_map_msg.position.y = AprilTagMapPoseQt[1]
        aptg_map_msg.position.z = AprilTagMapPoseQt[2]
        aptg_map_msg.orientation.x = AprilTagMapPoseQt[3]
        aptg_map_msg.orientation.y = AprilTagMapPoseQt[4]
        aptg_map_msg.orientation.z = AprilTagMapPoseQt[5]
        aptg_map_msg.orientation.w = AprilTagMapPoseQt[6]
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
