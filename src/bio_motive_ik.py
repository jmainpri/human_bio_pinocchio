#!/usr/bin/env python

# Copyright (c) 2018 University of Stuttgart
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                           Jim Mainprice on Monday May 5 2018

from mocap_interface import *
from bioik import *
import numpy.linalg as la
from list_joint_names import *

class BioMotiveIk():

    def __init__(self, semantics):
        self.semantics_ = semantics
        self._q = joint_names_from_urdf(utils.human_urdf_path())
        self._q = dict.fromkeys( self._q.iterkeys(), 0. ) 

    def joint_frame(self, t_p_inv, frame, label):
        """ Transform the frame in another frame """
        T = self.semantics_.transform(frame, label)
        return Affine3d(t_p_inv * T.matrix())

    def joint_state(self, frame):
        t_pelvis    = self.semantics_.pelvis(frame)
        t_pelvis    = Affine3d(
            t_pelvis.matrix() *
            MakeTransform(rodrigues([0, 0, -pi/2]), matrix([0, 0, 0])))
        t_p_inv     = la.inv(t_pelvis.matrix())

        # t_torso = self.joint_frame(t_p_inv, frame, 'Torso')
        r_shoulder  = self.joint_frame(t_p_inv, frame, 'rShoulder')
        r_elbow     = self.joint_frame(t_p_inv, frame, 'rElbow')
        r_wrist     = self.joint_frame(t_p_inv, frame, 'rWrist')
        l_shoulder  = self.joint_frame(t_p_inv, frame, 'lShoulder')
        l_elbow     = self.joint_frame(t_p_inv, frame, 'lElbow')
        l_wrist     = self.joint_frame(t_p_inv, frame, 'lWrist')

        t_torso = Affine3d(eye(4))
        t_torso.translation += array([0.037432, 0, 0.139749])

        pelvis_euler    = euler_from_matrix(t_pelvis.linear(), 'rxyz')
        torso_euler     = array([0.]*3)
        shoulder_euler  = euler_from_matrix(
            t_offset * r_shoulder.linear(), 'ryxy')
        shoulder_euler  = array([0.]*3) 
        elbow_euler     = array([0.]*3) # euler_from_matrix(r_elbow.linear(), 'rxyz')
        wrist_euler     = array([0.]*3) # euler_from_matrix(r_wrist.linear(), 'rxyz')

        # matrix([[-1.0, 0.0, 0.0], [0.0, 0.0, 1], [0.0, 1.0, 0.0]])

        # Expresses the shoulder center in
        # in the torso frame.
        t_torso_inv = Affine3d(la.inv(t_torso.matrix()))
        shoulder_center = t_torso_inv * r_shoulder.translation
        
        d_r_shoulder_to_elbow = la.norm(
            r_shoulder.translation - 
            r_elbow.translation) 
        
        d_r_elbow_to_wrist = la.norm(
            r_elbow.translation - 
            r_wrist.translation)

        q = self._q.copy()
        q["PelvisTransX"]       = t_pelvis.translation[0]
        q["PelvisTransY"]       = t_pelvis.translation[1]
        q["PelvisTransZ"]       = t_pelvis.translation[2]
        q["PelvisRotX"]         = pelvis_euler[0]
        q["PelvisRotY"]         = pelvis_euler[1]
        q["PelvisRotZ"]         = pelvis_euler[2]
        q["TorsoX"]             = torso_euler[0]
        q["TorsoZ"]             = torso_euler[1]
        q["TorsoY"]             = torso_euler[2]
        q["rShoulderTransX"]    = shoulder_center[0]    # X
        q["rShoulderTransY"]    = shoulder_center[2]    # Z
        q["rShoulderTransZ"]    = -shoulder_center[1]   # -Y
        q["rArmTrans"]          = d_r_shoulder_to_elbow
        q["rForeArmTrans"]      = d_r_elbow_to_wrist
        q["rShoulderY1"]        = shoulder_euler[0]
        q["rShoulderX"]         = shoulder_euler[0]
        q["rShoulderY2"]        = shoulder_euler[2]
        q["rElbowZ"]            = elbow_euler[0]
        q["rElbowX"]            = elbow_euler[1]
        q["rElbowY"]            = elbow_euler[2]
        q["rWristZ"]            = wrist_euler[0]
        q["rWristX"]            = wrist_euler[1]
        q["rWristY"]            = wrist_euler[2]
        return q


