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
        self._q = dict.fromkeys(self._q.iterkeys(), 0.)
        self._frames_to_draw = {}

    def joint_frame(self, t_p_inv, frame, label):
        """ Transform the frame in another frame """
        T = self.semantics_.transform(frame, label)
        return Affine3d(t_p_inv * T.matrix())

    def frame_to_draw(self):
        return self._frames_to_draw

    def joint_state(self, frame):
        t_pelvis = self.semantics_.pelvis(frame)
        t_pelvis = Affine3d(
            t_pelvis.matrix() *
            MakeTransform(rodrigues([0, 0, -pi / 2]), matrix([0, 0, 0])))
        t_p_inv = la.inv(t_pelvis.matrix())

        # t_torso = self.joint_frame(t_p_inv, frame, 'Torso')
        t = np.eye(4)
        r_shoulder = self.joint_frame(t, frame, 'rShoulder')
        r_elbow = self.joint_frame(t, frame, 'rElbow')
        r_wrist = self.joint_frame(t, frame, 'rWrist')
        l_shoulder = self.joint_frame(t, frame, 'lShoulder')
        l_elbow = self.joint_frame(t, frame, 'lElbow')
        l_wrist = self.joint_frame(t, frame, 'lWrist')

        t_torso = Affine3d(eye(4))
        t_torso.translation += array([0.037432, 0, 0.139749])

        # Expresses the shoulder center in
        # in the torso frame.
        t_torso_inv = la.inv(t_torso.matrix())
        shoulder_in_pelvis = Affine3d(t_p_inv) * r_shoulder.translation
        shoulder_center = Affine3d(t_torso_inv) * shoulder_in_pelvis

        d_r_shoulder_to_elbow = la.norm(
            r_shoulder.translation -
            r_elbow.translation)

        d_r_elbow_to_wrist = la.norm(
            r_elbow.translation -
            r_wrist.translation)

        # SHOULDER
        r_shoulder_l = (
            t_pelvis.linear() *
            rodrigues([pi / 2, 0, 0]))

        shoulder_tmp = r_shoulder.linear()
        shoulder_tmp[:, 0] = r_shoulder.linear()[:, 2]
        shoulder_tmp[:, 2] = r_shoulder.linear()[:, 1]
        shoulder_tmp[:, 1] = r_shoulder.linear()[:, 0]

        UAE = r_shoulder_l.transpose() * shoulder_tmp

        shoulder = Affine3d(
            r_shoulder.translation,
            quaternion_from_matrix(r_shoulder_l))
        self._frames_to_draw["bio_motive_shoulder"] = shoulder

        torso = t_pelvis.matrix() * t_torso.matrix()
        self._frames_to_draw["bio_motive_torso"] = Affine3d(torso)

        # ELBOW
        elbow_tmp = r_elbow.linear()
        elbow_tmp[:, 0] = r_elbow.linear()[:, 1]
        elbow_tmp[:, 1] = r_elbow.linear()[:, 0]
        elbow_tmp[:, 2] = r_elbow.linear()[:, 2]

        LAE = r_shoulder_l.transpose() * elbow_tmp

        # HAND
        wrist_tmp = r_wrist.linear()
        wrist_tmp[:, 0] = r_wrist.linear()[:, 1]
        wrist_tmp[:, 1] = r_wrist.linear()[:, 0]
        wrist_tmp[:, 2] = r_wrist.linear()[:, 2]

        handE = r_shoulder_l.transpose() * wrist_tmp

        # ----------------------------------------------------------------
        # Get eulers angles from matrices

        pelvis_euler = euler_from_matrix(t_pelvis.linear(), 'rxyz')
        torso_euler = array([0.] * 3)

        shoulder_euler = euler_from_matrix(UAE, 'ryxy')

        LA_about_UA = UAE.transpose() * LAE
        elbow_euler = euler_from_matrix(LA_about_UA, 'rzxy')

        hand_about_LA = LAE.transpose() * handE
        wrist_euler = euler_from_matrix(hand_about_LA, 'rzxy')

        # matrix([[-1.0, 0.0, 0.0], [0.0, 0.0, 1], [0.0, 1.0, 0.0]])
        q = self._q.copy()
        q["PelvisTransX"] = t_pelvis.translation[0]
        q["PelvisTransY"] = t_pelvis.translation[1]
        q["PelvisTransZ"] = t_pelvis.translation[2]
        q["PelvisRotX"] = pelvis_euler[0]
        q["PelvisRotY"] = pelvis_euler[1]
        q["PelvisRotZ"] = pelvis_euler[2]
        q["TorsoX"] = torso_euler[0]
        q["TorsoZ"] = torso_euler[1]
        q["TorsoY"] = torso_euler[2]
        q["rShoulderTransX"] = shoulder_center[0]    # X
        q["rShoulderTransY"] = shoulder_center[2]    # Z
        q["rShoulderTransZ"] = -shoulder_center[1]   # -Y
        q["rArmTrans"] = d_r_shoulder_to_elbow
        q["rForeArmTrans"] = d_r_elbow_to_wrist
        q["rShoulderY1"] = shoulder_euler[0]
        q["rShoulderX"] = shoulder_euler[1]
        q["rShoulderY2"] = shoulder_euler[2]
        q["rElbowZ"] = elbow_euler[0]
        q["rElbowX"] = elbow_euler[1]
        q["rElbowY"] = elbow_euler[2]
        q["rWristZ"] = wrist_euler[0]
        q["rWristX"] = wrist_euler[1]
        q["rWristY"] = wrist_euler[2]
        return q
