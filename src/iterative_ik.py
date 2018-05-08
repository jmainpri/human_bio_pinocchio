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
from numpy.linalg import pinv,norm

def IterativeIK:
    def __init__(self):
        self.magnitude_ = 0.1
        self.jacobian_fct_ = None
        self.active_indices_ = None
        self.lower_limits_ = None
        self.upper_limits_ = None
        self.imits_ = None
        self.check_joint_limits_ = None
        self.sigmoid_joint_limits_ = None
        self.nb_steps_ = None
        self.max_distance_to_target_ = None
        self.iterations_ = None
        self.configurations_ = np.array([])
        self.conservative_joint_limit_threshold_ = 1e-5
        self.verbose_ = True

    def interpolated_configurations(self):
        return self.configurations_

    # This function will return true if the configuration violates
    # joint limits. the configuration passed as input should be only
    # for active dofs and match in size the lower and upper argument of 
    # of the class. A bad joint indicies list is mantained
    def check_violate_joint_limits(self, q):
        assert q.shape[0] == self.active_indices_
        assert q.shape[0] == self.lower_limits_.shape[0] 
        assert q.shape[0] == self.upper_limits_.shape[0]
        violate_limit = False
        theshold = self.conservative_joint_limit_threshold_
        for idx in range(self.active_indices_):
        if (q[i] < (self.lower_limits_[idx] + theshold) or
            q[i] > (self.upper_limits_[idx] - theshold)):
            # note this will never add the same joint
            # twice, even if bClearBadJoints = false
            bad_joint_indices.append(i)
            violate_limit = True

            if self.verbose_: 
                # cout << "does not respect joint limits : "
                #     << active_joints_[i]->getName();
                print "ith joint : ", i
                print " , lowerLimit : ", lowerLimit
                print " , upperLimit : ", upperLimit
                print " , q_j : ", q[idx] 

        return violate_limit;

    def solve(self, x_des, q):



