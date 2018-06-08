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

from pinocchio.utils import *
from pinocchio.explog import exp, log
from numpy.linalg import pinv, norm
import pinocchio as se3
import utils
import operator

# filename = "../urdf/r2_robot.urdf"
# filename = utils.human_urdf_path()


def joint_names_from_urdf(filename):
    robot = se3.RobotWrapper(filename)
    q = robot.q0
    joint_names = {}
    print "shape of q : ", q.shape
    print "human : ", robot
    for name in robot.model.names:
        joint_names[name] = robot.index(name) - 1
    return joint_names


if __name__ == '__main__':
    # filename = "../urdf/r2_robot.urdf"
    names = joint_names_from_urdf(utils.human_urdf_path())
    sorted_names = sorted(names.items(), key=operator.itemgetter(1))
    for name, idx in sorted_names:
        print('{0:16} ==> {1:3d}'.format(name, idx))
