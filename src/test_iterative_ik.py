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
import numpy as np
import iterative_ik
import utils
import pinocchio as se3
from pinocchio.utils import *

def initialize():
    # Load config files
    human=se3.RobotWrapper(utils.human_urdf_path())
    data=utils.human_config_data()
    
    # Setup default translations
    q_default=np.zeros(human.q0.shape)
    for name, value in data['right_arm_default'].iteritems():
        q_default[human.index(name)] = value
    human.q0 = q_default

    # Setup the active dofs indices
    active_dofs=[human.index(name) for name in data["right_arm_dofs"]]
    return [human, active_dofs]


def sample_configs():
    [human, active_dofs] = initialize()
    print "shape of q : ", human.q0.shape
    print "human : ", human
    for i in range(20):
        q_active=rand((len(active_dofs)))
        q = human.q0
        q[active_dofs] = q_active
        human.forwardKinematics(q)
        p = human.data.oMi[human.index("rWristY")].translation
        print "p_{} : {}".format(i, p.transpose())

if __name__== "__main__":
    sample_configs()
