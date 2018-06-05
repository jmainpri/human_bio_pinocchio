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

class BioMotiveIk():

    def __init__(self, semantics):
        self.semantics_ = semantics

    def joint_frame(self, t_p_inv, labeled_frame, label):
        """ Transform the frame in another frame """"
        M = self.semantics_.transform(labeled_frame, label).matrix()
        return Affine3d(t_p_inv * M)

    def joint_state(self, labeled_frame, semantics):
        t_pelvis    = self.semantics_.pelvis(labeled_frame).matrix()
        t_p_inv     = la.inv(t_pelvis.matrix())
        t_torso     = self.joint_frame(t_p_inv, labeled_frame, 'Torso'))
        r_shoulder  = self.joint_frame(t_p_inv, labeled_frame, 'rShoulder'))
        r_elbow     = self.joint_frame(t_p_inv, labeled_frame, 'rElbow'))
        r_wrist     = self.joint_frame(t_p_inv, labeled_frame, 'rWrist'))
        l_shoulder  = self.joint_frame(t_p_inv, labeled_frame, 'lShoulder'))
        l_elbow     = self.joint_frame(t_p_inv, labeled_frame, 'lElbow'))
        l_wrist     = self.joint_frame(t_p_inv, labeled_frame, 'lWrist'))

        q = {}
        t_torso_a = Affine3d(t_torso)
        shoulder_center = t_torso_a * r_shoulder.translation
        q["rShoulderTransX"] = shoulder_center[0]
        q["rShoulderTransY"] = shoulder_center[1]
        q["rShoulderTransZ"] = shoulder_center[2]
