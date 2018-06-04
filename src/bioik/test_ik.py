#!/usr/bin/env python
# Copyright (c) 2013 Worcester Polytechnic Institute
#   Author: Jim Mainrpice <jmainprice@wpi.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -*- coding: utf-8 -*-

# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py ../ormodels/human_wpi_bio.xml --info=joints
# on that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
import os
import sys
import time
from copy import deepcopy
from numpy import *
from numpy import linalg as la
from TransformMatrix import *
from rodrigues import *
import marker_utils
from BioHumanIk import *

mapping = [-1, 6, 7, 8, 16, 17, 18, 20, 21, 22, 24, 25, 26]

TCP_IP = '127.0.0.1'
TCP_PORT = 5005

# Joint and link names
# -------------------------------------------------------------------------
# TorsoX        6           6         PelvisBody        TorsoDummyX
# TorsoY        7           7         TorsoDummyX       TorsoDummyY
# TorsoZ        8           8         TorsoDummyY       TorsoDummyZ
# xTorsoTrans   9           9         TorsoDummyZ       TorsoDummyTransX
# yTorsoTrans   10          10        TorsoDummyTransX  TorsoDummyTransY
# zTorsoTrans   11          11        TorsoDummyTransY  TorsoDummyTransZ
# TorsoTrans    12          12        TorsoDummyZ       torso
# HeadZ         13          13        torso             HeadDummyZ
# HeadY         14          14        HeadDummyZ        HeadDummyY
# HeadX         15          15        HeadDummyY        head
# rShoulderX    16          16        TorsoDummyTransZ  rShoulderDummyX
# rShoulderZ    17          17        rShoulderDummyX   rShoulderDummyZ
# rShoulderY    18          18        rShoulderDummyZ   rHumerus
# rArmTrans     19          19        rHumerus          rElbowDummy1
# rElbowX       20          20        rElbowDummy1      rElbowDummy2
# rElbowY       21          21        rElbowDummy2      rElbowDummy3
# rElbowZ       22          22        rElbowDummy3      rRadius
# rForearmTrans 23          23        rRadius           rWristDummy
# rWristX       24          24        rWristDummy       rWristDummyX
# rWristY       25          25        rWristDummyX      rWristDummyY
# rWristZ       26          26        rWristDummyY      rHand


# Reminder of marker order
# -------------------------------------------------------------------------
# 0 xyphoid process
# 1 T8
# 2 sternal notch
# 3 C7
# 4 Acromion process
# 5 Glenohumeral cntr of rot. (post)
# 6 Medial epicondyle
# 7 lateral epicondyle
# 8 ulnar styloid
# 9 radial styloid
# 10 2nd metacarpal head


class TestBioHumanIk(BioHumanIk):

    def __init__(self):

        BioHumanIk.__init__(self)

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../../ormodels/human_wpi_bio.xml")

        self.human = self.env.GetRobots()[0]

        # Get pelvis torso offset and set human to position
        self.offset_pelvis_torso_init = self.human.GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()[0:3, 3]

        self.handles = []

        # Should store the marker set
        self.markers = None
        self.q = None

        self.offset_pelvis_torso = None
        self.offset_torso = array([0., 0., 0.])
        self.offset_torso_shoulder = None
        self.offset_shoulder_elbow = None
        self.offset_elbow_wrist = None
        self.t_torso = None
        self.t_pelvis = None
        self.t_trans = matrix(eye(4))

    # Marker array should be the size of the number of markers
    # the markers should be ordered as described in the reminder
    def set_markers(self, markers):

        self.markers = markers

    def get_markers_in_frame(self, t_0):

        inv_torso = la.inv(t_0)

        points_3d = len(self.markers)*[array([0, 0, 0])]

        for i, p in enumerate(self.markers):
            points_3d[i] = array(array(inv_torso).dot(array(append(p, 1.0))))[0:3]

        return points_3d

    def save_markers_to_file(self):

        with open("./matlab/markers_tmp.csv", 'w') as m_file:

            line_str = ""

            # Construct frame centered at torso with orientation
            # set by the pelvis frame, add rotation offset for matlab code
            t_trans = deepcopy(self.t_pelvis)
            t_trans[0:3, 3] = deepcopy(self.t_torso[0:3, 3])
            t_trans = t_trans * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))

            # inv_pelvis = la.inv(self.t_pelvis)
            # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * t_trans, 2))

            t_trans = la.inv(t_trans)

            for marker in self.markers:

                marker = array(array(t_trans).dot(append(marker, 1)))[0:3]

                line_str += str(marker[0]*1000) + ','
                line_str += str(marker[1]*1000) + ','
                line_str += str(marker[2]*1000) + ','

            line_str = line_str.rstrip(',')
            line_str += '\n'
            m_file.write(line_str)

    def set_pelvis_frame(self, t_pelvis):

        # Get pelvis frame in world

        # print "t_pelvis : ", t_pelvis
        # print "t_pelvis[3:7] ", t_pelvis[3:7]
        # print "t_pelvis[0:3] ", t_pelvis[0:3]

        mat = MakeTransform(rotationMatrixFromQuat(array(t_pelvis[3:7])), matrix(t_pelvis[0:3]))

        new_x = -array(transpose(mat[:, 2]).tolist()[0][:3])  # get z vector from matrix
        new_x[2] = 0
        new_x /= la.norm(new_x)

        new_z = array([0.0, 0.0, 1.0])

        new_y = cross(new_x, new_z)
        new_y /= la.norm(new_y)
        new_y = -new_y

        mat[:, 0][:3] = transpose(array([new_x]))
        mat[:, 1][:3] = transpose(array([new_y]))
        mat[:, 2][:3] = transpose(array([new_z]))

        self.t_pelvis = matrix(mat)

        # Compute the offset between the trunk and the pelvis
        # the offset_pelvis_torso_init is hard coded and the human model

        trunk_center = (self.markers[0] + self.markers[1])/2

        inv_pelvis = la.inv(self.t_pelvis)
        trunk_center = array(array(inv_pelvis).dot(append(trunk_center, 1)))[0:3]

        self.offset_pelvis_torso = trunk_center
        self.offset_pelvis_torso -= self.offset_pelvis_torso_init

        # print "offset_pelvis_torso : ", self.offset_pelvis_torso

    def set_human_model_sizes(self):

        # Place the human according to the torso and pelvis frame
        # future notes: when placing the human according to the pelvis frame
        # the torso offset should be applied
        t_offset = MakeTransform(eye(3), matrix(self.offset_pelvis_torso))
        self.human.SetTransform(array(self.t_pelvis * t_offset))

        # Set elbow size
        # self.human.SetDOFValues([-self.offset_torso[0]], [9])
        # self.human.SetDOFValues([self.offset_torso[1]], [10])
        # self.human.SetDOFValues([-self.offset_torso[2]], [11])
        self.human.SetDOFValues([self.offset_torso[0]], [9])
        self.human.SetDOFValues([self.offset_torso[1]], [10])
        self.human.SetDOFValues([self.offset_torso[2]], [11])
        self.human.SetDOFValues([self.offset_shoulder_elbow], [19])
        self.human.SetDOFValues([self.offset_elbow_wrist], [23])

    def compute_dist_to_points(self, frame_id=0):

        # Get joint centers
        p_torso_origin = (self.markers[0] + self.markers[1])/2
        p_shoulder_center = array([self.markers[4][0], self.markers[4][1], self.markers[5][2]])
        p_elbow_center = (self.markers[6] + self.markers[7])/2
        p_wrist_center = (self.markers[9] - self.markers[8])/2 + self.markers[8]

        # Get the points in the global frame
        # inv_pelvis = la.inv(self.t_pelvis)
        inv_pelvis = eye(4)

        p0 = array(array(inv_pelvis).dot(append(p_torso_origin, 1)))[0:3]
        p1 = array(array(inv_pelvis).dot(append(p_shoulder_center, 1)))[0:3]
        p2 = array(array(inv_pelvis).dot(append(p_elbow_center, 1)))[0:3]
        p3 = array(array(inv_pelvis).dot(append(p_wrist_center, 1)))[0:3]

        dist = 0.0

        for j in self.human.GetJoints():

            p_link = j.GetHierarchyChildLink().GetTransform()[0:3, 3]

            if j.GetName() == "TorsoZ":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p0)
                print "p_link : ", p_link
                print "dist torso : ", dist
                # l = self.human.GetLink("TorsoDummyY")
                # self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 1.0))
            if j.GetName() == "rShoulderX":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.05, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p1)
                print "dist shoulder : ", dist
            if j.GetName() == "rElbowZ":
                self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p2)
                print "dist elbow : ", dist
            if j.GetName() == "rWristX":
                self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p3)
                print "dist wrist : ", dist
            #if j.GetName() == "rShoulderZ":
            #    self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 0.3))

        # for j in self.human.GetJoints():
        #     if j.GetName() == "TorsoX":  # j.GetName() == "rShoulderX" or
        #         t_link = j.GetHierarchyChildLink().GetTransform()
        #         self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

        self.handles.append(self.env.plot3(p1, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        self.handles.append(self.env.plot3(p2, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        self.handles.append(self.env.plot3(p3, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))

        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("TorsoZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rShoulderY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rElbowZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rWristY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rArmTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * self.t_torso, 1))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))
        # self.handles.append(misc.DrawAxes(self.env, eye(4), 2))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))

        # print "joint : ", self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform()[0:3, 3]

        self.handles.append(misc.DrawAxes(self.env, self.trunkE, .2))
        self.handles.append(misc.DrawAxes(self.env, self.UAE, .2))
        self.handles.append(misc.DrawAxes(self.env, self.LAE, .2))
        self.handles.append(misc.DrawAxes(self.env, self.handE, .2))

        return dist

    def draw_markers(self, config, d_torso, d_shoulder_elbow, d_elbow_wrist):

        del self.handles[:]

        # Get the points in the global frame
        # inv_pelvis = la.inv(self.t_pelvis)
        t_0 = eye(4)

        points = self.get_markers_in_frame(t_0)

        colors = []
        nb_points = len(points)
        for n in linspace(0.0, 1.0, num=nb_points):
            colors.append((float(n)*1, (1-float(n))*1, 0))

        points_3d = squeeze(points)

        self.handles.append(self.env.plot3(points=points_3d, pointsize=0.02, colors=array(colors), drawstyle=1))

        q_cur = self.get_human_configuration(config)
        self.human.SetDOFValues(q_cur[0:self.human.GetDOF()])

        # self.offset_torso_shoulder = d_torso_shoulder
        self.offset_shoulder_elbow = d_shoulder_elbow
        self.offset_elbow_wrist = d_elbow_wrist
        self.offset_torso = d_torso
        self.set_human_model_sizes()

        # print "(plane of elevation, ", q_cur[16]*180/pi, " , elevation, ", q_cur[17]*180/pi, \
        #     " , axial rotation, ", q_cur[18]*180/pi, ")"
        self.compute_dist_to_points()

    # Map the joint angles and set to radians
    def get_human_configuration(self, config=None):

        if config is None:
            motion = genfromtxt('./matlab/outputik.csv', delimiter=',')
            motion = delete(motion, 0, axis=0)  # Remove first row...
        else:
            motion = [config]

        print "---------------------------"
        # print "motion : ", motion

        for configuration in motion:  # motion should be one row. otherwise take the last element

            self.q = self.human.GetDOFValues()

            for i, dof in enumerate(configuration):

                if mapping[i] >= 0:
                    self.q[mapping[i]] = dof * pi / 180

                # if mapping[i] == 16:
                #     self.q[mapping[i]] = -self.q[mapping[i]]
                # if mapping[i] == 17:
                #     self.q[mapping[i]] = -self.q[mapping[i]]
                # if mapping[i] == 18:
                #     self.q[mapping[i]] = -self.q[mapping[i]]

                # rElbowX       20          20        rElbowDummy1      rElbowDummy2
                # rElbowY       21          21        rElbowDummy2      rElbowDummy3
                # rElbowZ       22          22        rElbowDummy3      rRadius
                # rForearmTrans 23          23        rRadius           rWristDummy
                # rWristX       24          24        rWristDummy       rWristDummyX
                # rWristY       25          25        rWristDummyX      rWristDummyY
                # rWristZ       26          26        rWristDummyY      rHand

        return self.q

if __name__ == "__main__":

    # Load markers from file
    # raw_markers = genfromtxt('data/points.csv', delimiter=',')
    # raw_markers = delete(raw_markers, 0, axis=0)  # Remove first row
    # raw_markers = delete(raw_markers, s_[0:2], 1)  # Remove two columns
    # raw_markers /= 1000  # Set markers in meters
    # (m,) = raw_markers[0].shape  # number of values in the marker set

    # use human_mocap_second.zip
    marker_file = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/second/markers_fixed_cut.csv'
    object_file = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/second/objects_fixed_cut.csv'
    [frames_m, frames_o] = marker_utils.load_file(marker_file, object_file)

    h = TestBioHumanIk()

    for i in range(len(frames_m)):

        h.set_markers(frames_m[i][0:11])
        h.set_pelvis_frame(frames_o[i][0][0:7])
        markers = h.get_markers_in_pelvis_frame(h.markers, h.t_pelvis)
        [q, d_torso, d_shoulder_elbow, d_elbow_wrist] = h.compute_ik(markers)
        h.draw_markers(q, d_torso, d_shoulder_elbow, d_elbow_wrist)

        time.sleep(0.01)

        if i == 1:
            print "press enter to continue"
            sys.stdin.readline()

        # print "Press return to next."
        # sys.stdin.readline()

    print "press enter to exit"
    sys.stdin.readline()