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
import utils
import pinocchio as se3
from pinocchio.utils import *
from pinocchio_ik import *
from iterative_ik import *
import os
from bioik import *

def get_joint_state(robot, q):
    q_tmp = robot.q0.copy()
    for name, value in q.iteritems():
        idx = robot.model.getJointId(name) - 1
        q_tmp[idx] = value
    return q_tmp

def main():
    robot = se3.RobotWrapper(utils.human_urdf_path())

    q={}
    q["rShoulderY1"]  = 0.
    q["rShoulderX"]   = 0.
    q["rShoulderY2"]  = 0.

    robot.forwardKinematics(get_joint_state(robot, q))

    joint_name = "rShoulderY1"
    data = robot.data.oMi[robot.model.getJointId(joint_name)].copy()
    t_origin=data.rotation

    joint_name = "rShoulderTransZ"
    data = robot.data.oMi[robot.model.getJointId(joint_name)].copy()
    # print se3.SE3(data.rotation, zero(3))

    shoulder_euler_1 = np.array([0] * 3)
    shoulder_euler_1[0] = 0.
    shoulder_euler_1[1] = 0.3
    shoulder_euler_1[2] = 1.
    print shoulder_euler_1

    q={}
    q["rShoulderY1"]  = shoulder_euler_1[0]
    q["rShoulderX"]   = shoulder_euler_1[1]
    q["rShoulderY2"]  = shoulder_euler_1[2]

    robot.forwardKinematics(get_joint_state(robot, q))

    joint_name = "rShoulderY1"
    data = robot.data.oMi[robot.model.getJointId(joint_name)].copy()
    # print se3.SE3(data.rotation, zero(3))

    joint_name = "rShoulderX"
    data = robot.data.oMi[robot.model.getJointId(joint_name)].copy()
    # print se3.SE3(data.rotation, zero(3))

    joint_name = "rShoulderY2"
    data = robot.data.oMi[robot.model.getJointId(joint_name)].copy()
    print se3.SE3(data.rotation, zero(3))

    shoulder_euler_2 = euler_from_matrix(
        t_origin.transpose() * data.rotation, 'ryxy')

    q["rShoulderY1"]  = shoulder_euler_2[0]
    q["rShoulderX"]   = shoulder_euler_2[1]
    q["rShoulderY2"]  = shoulder_euler_2[2]

    robot.forwardKinematics(get_joint_state(robot, q))

    joint_name = "rShoulderY2"
    data = robot.data.oMi[robot.model.getJointId(joint_name)].copy()
    print se3.SE3(data.rotation, zero(3))

    print shoulder_euler_1
    print shoulder_euler_2

if __name__== "__main__":
    main()

