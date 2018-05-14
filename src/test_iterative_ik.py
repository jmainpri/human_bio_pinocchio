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
from iterative_ik import *

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
    return human, active_dofs

def forward_kinematics(q_all, human, wrist_index):
    human.forwardKinematics(q_all)
    return human.data.oMi[wrist_index].translation

def sample_configs(human, active_dofs, wrist_name):
    print "shape of q : ", human.q0.shape
    print "human : ", human
    wrist_index = human.index(wrist_name)
    nb_configs = 20
    data = [None] * nb_configs
    for i in range(nb_configs):
        q_active=rand((len(active_dofs)))
        q_all=np.copy(human.q0)
        q_all[active_dofs] = q_active
        p = forward_kinematics(q_all, human, wrist_index)
        data[i] = (q_all, p)
        print "p_{} : {}".format(i, p.transpose())
    return data

def test_iterative_ik(human, active_dofs, wrist_name, data):
    wrist_index = human.index(wrist_name)
    print "active dofs : ", active_dofs
    print "q_0.shape : ", human.q0.shape
    print "wrist_index : ", wrist_index

    iterative_ik=IterativeIK(
        lambda q: human.jacobian(q, wrist_index),
        lambda q: forward_kinematics(q, human, wrist_index),
        active_dofs)
    iterative_ik.q_full = np.copy(human.q0)
    for config in data:
        succeeded = iterative_ik.solve(
            config[0][active_dofs], 
            config[1])
        if not succeeded:
            return False
    return True


if __name__== "__main__":
    wrist_name = "rWristY"
    human, active_dofs = initialize()
    data = sample_configs(human, active_dofs, wrist_name)
    success = test_iterative_ik(human, active_dofs, wrist_name, data)
    print  "success : ", success

