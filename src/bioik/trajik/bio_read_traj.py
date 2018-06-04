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

from openravepy import *
import os
import sys
import time
from copy import deepcopy
from numpy import *
from numpy import linalg as la
from TransformMatrix import *
from rodrigues import *
from itertools import permutations

mapping = [-1, 8, 7, 6, 18, 17, 16, 20, 24, 23, 22]

# TorsoX        6           6         PelvisBody        TorsoDummyX
# TorsoY        7           7         TorsoDummyX       TorsoDummyY
# TorsoZ        8           8         TorsoDummyY       TorsoDummyTransX
# xTorsoTrans   9           9         TorsoDummyTransX  TorsoDummyZ
# yTorsoTrans   10          10        TorsoDummyTransX  TorsoDummyTransY
# zTorsoTrans   11          11        TorsoDummyTransY  TorsoDummyTransZ
# rShoulderX    16          16        TorsoDummyTransZ  rShoulderDummyX
# rShoulderZ    17          17        rShoulderDummyX   rShoulderDummyZ
# rShoulderY    18          18        rShoulderDummyZ   rHumerus
# rArmTrans     19          19        rHumerus          rElbowDummy1
# rElbowZ       20          20        rElbowDummy1      rRadius
# rForearmTrans 21          21        rRadius           rWristDummy
# rWristX       22          22        rWristDummy       rWristDummyX
# rWristY       23          23        rWristDummyX      rWristDummyY
# rWristZ       24          24        rWristDummyY      rHand


class Human():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.Reset()
        self.env.Load("../../ormodels/human_wpi_bio.xml")
        # self.env.Load("../ormodels/human_wpi_new.xml")
        # self.orEnv.Load("robots/pr2-beta-static.zae")

        self.markers = genfromtxt('points.csv', delimiter=',')

        self.motion = genfromtxt('outputik.csv', delimiter=',')
        self.motion = delete(self.motion, 0, axis=0)  # Remove first row ...

        # Print the array
        # print motion.shape
        # for row in self.motion:
        #     print ' '.join(map(str, row))

        self.human = self.env.GetRobots()[0]
        self.handles = []

        # Set torso at origin of the scene
        self.offset_pelvis_torso = self.human.GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()[0:3, 3]
        print self.offset_pelvis_torso

        self.offset_torso_shoulder = None
        self.offset_shoulder_elbow = None
        self.offset_elbow_wrist = None
        self.torso_origin = None

        # Set model size from file
        self.set_model_size()
        self.traj = None

        # self.t_torso = eye(4)
        # m = matrix([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])

        # self.handles.append(misc.DrawAxes(self.env, eye(4), 3.3))

        print self.t_torso
        self.human.SetTransform(array(MakeTransform(eye(3), matrix(-self.offset_pelvis_torso))))

    def get_markers_in_torso_frame(self):

        # Remove two first columns
        markers = deepcopy(self.markers)
        markers = numpy.delete(markers, s_[0:2], 1)
        markers /= 1000

        points_all = []

        for i, points in enumerate(markers):

            inv_torso = la.inv(self.t_torso)

            (m,) = points.shape  # number of values in the marker set
            points_3d = [points[n:n+3] for n in range(0, m, 3)]

            for j, p in enumerate(points_3d):
                points_3d[j] = array(array(inv_torso).dot(array(append(p, 1.0))))[0:3]

            points_all.append(points_3d)

        return points_all

    def play_trajectory(self):

        traj = self.get_trajectory(self.motion)
        self.human.GetController().SetPath(traj)
        self.human.WaitForController(0)

    def set_model_size(self, frame_id=0):

        # Remove two first columns and set markers in meters
        markers = deepcopy(self.markers)
        markers = numpy.delete(markers, s_[0:2], 1)
        markers /= 1000

        (m,) = markers[frame_id].shape
        p = [markers[frame_id][n:n+3] for n in range(0, m, 3)]

        self.torso_origin = (p[0] + p[1])/2
        self.t_torso = MakeTransform(rodrigues([0, 0, pi]), matrix(self.torso_origin))

        #points_all = self.get_markers_in_torso_frame()

        self.torso_origin = (p[0] + (p[1]))/2
        p_shoulder_center = array([p[4][0], p[4][1], p[5][2]])
        p_elbow_center = (p[6] + p[7])/2
        p_wrist_center = (p[9] - p[8])/2 + p[8]

        self.offset_torso_shoulder = p_shoulder_center - self.torso_origin
        self.offset_shoulder_elbow = la.norm(p_shoulder_center - p_elbow_center)
        self.offset_elbow_wrist = la.norm(p_wrist_center - p_elbow_center)

        # Get shoulder center in the global frame
        t_torso = self.t_torso
        inv_torso = la.inv(t_torso)
        offset_torso = array(array(inv_torso).dot(append(p_shoulder_center, 1)))[0:3]

        # offset_torso = [0]*3
        # offset_torso[0] = p[0]
        # offset_torso[1] = p[2]
        # offset_torso[2] = -p[1]

        # Get shoulder center in the torso frame after rotation
        t_torso = self.human.GetJoint("TorsoZ").GetHierarchyChildLink().GetTransform()
        inv_torso = la.inv(t_torso)
        offset_torso = array(array(inv_torso).dot(append(offset_torso, 1)))[0:3]

        # print "p ", p
        # print "offset_torso ", offset_torso

        # TODO FIX FRAME HERE
        self.human.SetDOFValues([offset_torso[0]], [9])
        self.human.SetDOFValues([offset_torso[1]], [10])
        self.human.SetDOFValues([offset_torso[2]], [11])
        self.human.SetDOFValues([self.offset_shoulder_elbow], [19])
        self.human.SetDOFValues([self.offset_elbow_wrist], [21])


        # TEST
        # xyphoid_t8 = p[0] - p[1]
        # trunk_center = p[0] - 0.5*xyphoid_t8
        # c7_sternal = p[2] - p[3]
        # c7s_midpt = p[3] + 0.5*c7_sternal
        #
        # mat = MakeTransform(eye(3), matrix(trunk_center))
        #
        # new_y = array(transpose(mat[:, 0]).tolist()[0][:3])
        # new_x = array(transpose(mat[:, 1]).tolist()[0][:3])
        # new_z = array(transpose(mat[:, 2]).tolist()[0][:3])
        #
        # new_y = c7s_midpt - trunk_center
        # new_z = cross(new_y, xyphoid_t8)
        # new_x = cross(new_y, new_z)
        #
        # new_x /= la.norm(new_x)
        # new_y /= la.norm(new_y)
        # new_z /= la.norm(new_z)
        #
        # mat[0][0, 0] = new_x[0]
        # mat[0][0, 1] = new_y[0]
        # mat[0][0, 2] = new_z[0]
        # mat[1][0, 0] = new_x[1]
        # mat[1][0, 1] = new_y[1]
        # mat[1][0, 2] = new_z[1]
        # mat[2][0, 0] = new_x[2]
        # mat[2][0, 1] = new_y[2]
        # mat[2][0, 2] = new_z[2]
        #
        # self.handles.append(misc.DrawAxes(self.env, la.inv(self.t_torso) * mat, 1))

    def play_markers(self):

        t = 0.0  # Time for playing
        alpha = 4  # Time scaling

        points_all = self.get_markers_in_torso_frame()

        for i, points in enumerate(points_all):

            colors = []
            nb_points = len(points)
            for n in linspace(0.0, 1.0, num=nb_points):
                colors.append((float(n)*1, (1-float(n))*1, 0))

            points_3d = squeeze(points)

            self.handles.append(self.env.plot3(points=points_3d, pointsize=0.02, colors=array(colors), drawstyle=1))

            dt = self.markers.item((i, 1)) - t  # self.markers(1, i) is time
            t = self.markers.item((i, 1))

            if self.traj is not None:  # Set robot to trajectory configuration if it exists
                q_cur = self.traj.Sample(t)
                self.human.SetDOFValues(q_cur[0:self.human.GetDOF()])
                print "(plane of elevation, ", q_cur[16]*180/pi, " , elevation, ", q_cur[17]*180/pi, \
                    " , axial rotation, ", q_cur[18]*180/pi, ")"

            # draws center of joints points
            self.set_model_size(i)
            self.compute_dist_to_points(i)

            sys.stdin.readline()

            time.sleep(alpha*dt)

            del self.handles[:]

    def compute_dist_to_points(self, frame_id=0):

        # Remove two first columns and set markers in meters
        markers = deepcopy(self.markers)
        markers = numpy.delete(markers, s_[0:2], 1)
        markers /= 1000

        # Get markers as 3D arrays
        (m,) = markers[frame_id].shape
        p = [markers[frame_id][n:n+3] for n in range(0, m, 3)]

        # Get joint centers
        p_shoulder_center = array([p[4][0], p[4][1], p[5][2]])
        p_elbow_center = (p[6] + p[7])/2
        p_wrist_center = (p[9] - p[8])/2 + p[8]

        # Get the points in the global frame
        inv_torso = la.inv(self.t_torso)
        p1 = array(array(inv_torso).dot(append(p_shoulder_center, 1)))[0:3]
        p2 = array(array(inv_torso).dot(append(p_elbow_center, 1)))[0:3]
        p3 = array(array(inv_torso).dot(append(p_wrist_center, 1)))[0:3]

        dist = 0.0

        for j in self.human.GetJoints():

            p_link = j.GetHierarchyChildLink().GetTransform()[0:3, 3]

            if j.GetName() == "TorsoZ":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - array([0, 0, 0]))
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

        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("TorsoY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform(), 1))
        self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rShoulderY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rElbowZ").GetHierarchyChildLink().GetTransform(), 1))
        self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rWristY").GetHierarchyChildLink().GetTransform(), 1))
        self.handles.append(misc.DrawAxes(self.env, self.t_torso, 1))
        #self.handles.append(misc.DrawAxes(self.env, eye(4), 2))

        # print "joint : ", self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform()[0:3, 3]

        return dist

    # Map the joint angles and set to radians
    def get_configuration(self, q):

        wp = self.human.GetDOFValues()

        for i, dof in enumerate(q):
            if mapping[i] >= 0:
                wp[mapping[i]] = dof * pi / 180
            if mapping[i] == 16:
                wp[mapping[i]] = -wp[mapping[i]]
            if mapping[i] == 17:
                wp[mapping[i]] = -wp[mapping[i]]
            if mapping[i] == 18:
                wp[mapping[i]] = -wp[mapping[i]]

        return wp

    # Construct an OpenRave trajectory
    def get_trajectory(self, motion):

        config_spec = self.human.GetActiveConfigurationSpecification()
        g = config_spec.GetGroupFromName('joint_values')
        g.interpolation = 'linear'
        config_spec = ConfigurationSpecification()
        config_spec.AddGroup(g)
        config_spec.AddDeltaTimeGroup()

        self.traj = RaveCreateTrajectory(self.human.GetEnv(), '')
        self.traj.Init(config_spec)
        t = 0.0
        # alpha = 4  # Time scaling

        for q in motion:
            dt = q[0] - t  # q[0] is time
            t = q[0]
            wp = append(self.get_configuration(q), dt)
            self.traj.Insert(self.traj.GetNumWaypoints(), wp)

        return self.traj


if __name__ == "__main__":

    h = Human()
    # print "Press return to play trajectory."
    sys.stdin.readline()

    # h.human.SetDOFValues([-90 * pi / 180], [16])  # Plane of Elevation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # h.human.SetDOFValues([90 * pi / 180], [17])  # Elevation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # h.human.SetDOFValues([90 * pi / 180], [16])  # Plane of Elevation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # h.human.SetDOFValues([-30 * pi / 180], [18])  # Axial Rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()

    # h.human.SetDOFValues([30 * pi / 180], [18])  # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.human.SetDOFValues([-30 * pi / 180], [18])  # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.human.SetDOFValues([30 * pi / 180], [18])  # In rotation
    # print "Press return to play trajectory."
    # sys.stdin.readline()
    #
    # h.human.SetDOFValues([90 * pi / 180], [24])  # Wrist
    # print "Press return to play trajectory."
    # sys.stdin.readline()


    h.get_trajectory(h.motion)

    while True:
        h.play_markers()
        print "Press return to exit."
        sys.stdin.readline()

    # while True:
    #     h.play_trajectory()
    #     print "Press return to exit."
    #     sys.stdin.readline()

    # for idx in list(permutations([16, 17, 18])):
    #
    #     mapping = [-1, 8, 6, 7, idx[0], idx[1], idx[2], 19, 21, 22]
    #
    #     for angles in list(permutations([1, -1, 0])):
    #         # h.play_trajectory()
    #         offset = angles
    #         q = h.get_configuration(h.motion[0])
    #         h.human.SetDOFValues(q)
    #         print mapping
    #         print offset
    #         print h.compute_dist_to_points()