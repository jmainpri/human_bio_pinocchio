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
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import utils

print se3
human = se3.RobotWrapper(utils.human_urdf_path())
q = human.q0
wrist_name = "rWristY"
wrist_index = human.index(wrist_name)
print "shape of q : ", q.shape
print "wrist_name : ", wrist_name
print "wrist_index : ", wrist_index

for i in range(20):
    q_rand=rand(q.shape)
    human.forwardKinematics(q_rand)
    J = human.jacobian(q_rand, wrist_index)
    print "Jacobian shape : ", J.shape
    np.set_printoptions(
        suppress=True, 
        formatter={'float_kind':'{:5.2f}'.format}, 
        linewidth=100000)
    print J[:, 23:37]

    # print "J : ", J.transpose()
    # p = human.data.oMi[].translation
    # print "p_{} : {}".format(i, p.transpose())
