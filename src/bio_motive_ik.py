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

class BioMotiveIk():

    def __init__(self, semantics):
        self.semantics_ = semantics

    def joint_state(self, labeled_frame, semantics):
        t_pelvis        = semantics_.pelvis(labeled_frame)
        t_torso         = semantics_.torso(labeled_frame)
        r_shoulder      = semantics_.r_shoulder(labeled_frame)
        r_elbow         = semantics_.r_elbow(labeled_frame)
        r_wrist         = semantics_.r_wrist(labeled_frame)
        l_shoulder      = semantics_.l_shoulder(labeled_frame)
        l_elbow         = semantics_.l_elbow(labeled_frame)
        l_wrist         = semantics_.l_wrist(labeled_frame)
