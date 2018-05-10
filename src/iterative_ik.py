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


def normalize(v):
    v_norm=norm(v)
    if v_norm==0:
        print "norm is 0"
        v_norm=np.finfo(v.dtype).eps
    return v/v_norm


def IterativeIK:
     """ 
        This class will compute the inverse kinematics
        for a general manipulator, it relies on the foward kinematics
        function as well as the Jacobian.
    """
    def __init__(self):
        self.__verbose = True
        self.__jacobian_fct_ = None
        self.__forward_kinematics_fct_ = None
        self.__magnitude = 0.1
        self.__active_indices = None
        self.__lower_limits_ = None
        self.__upper_limits_ = None
        self.__imits = None
        self.__check_joint_limits_ = None
        self.__sigmoid_joint_limits = None
        self.__max_nb_iterations = None
        self.__max_distance_to_target = None
        self.__iterations_ = None
        self.__configurations = []
        self.__conservative_joint_limit_threshold = 1e-5
        self.__q_full = None
        

    def interpolated_configurations(self):
        return self.configurations_

    def full_dof_config(self, q)
        """ 
        Returns a configuration of the full dimension
        """
        assert len(self.__active_indices) <= q.shape[0]
        if len(self.__active_indices) == q.shape[0]:
            return q
        q_full = self.__q_full
        q_full[self.__active_indices] = q
        return q_full

    def jacobian(self, q):
        """ 
        Returns the jacobian of the active dofs
        the class assumes that the underlying library 
        uses a full dof configuration model
        """"
        J = self.__jacobian_fct_(self.full_dof_config(q))
        return J[:, self.__active_indices]

    def foward_kinematics(self, q):
        """ 
        Returns the foward kinematics
        the class assumes that the underlying library 
        uses a full dof configuration model
        """"
        x = self.__forward_kinematics_fct_(self.full_dof_config(q))
        return x

    def check_violate_joint_limits(self, q):
        """ 
        This function will return true if the configuration violates
        oint limits. the configuration passed as input should be only
        for active dofs and match in size the lower and upper argument of 
        of the class. A bad joint indicies list is mantained
        """
        assert q.shape[0] == self.__active_indices
        assert q.shape[0] == self.__lower_limits_.shape[0] 
        assert q.shape[0] == self.__upper_limits_.shape[0]
        violate_limit = False
        theshold = self.__conservative_joint_limit_threshold_
        for idx in range(self.__active_indices_):
        if (q[i] < (self.__lower_limits_[idx] + theshold) or
            q[i] > (self.__upper_limits_[idx] - theshold)):
            # note this will never add the same joint
            # twice, even if bClearBadJoints = false
            bad_joint_indices.append(i)
            violate_limit = True

            if self.__verbose_: 
                # cout << "does not respect joint limits : "
                #     << active_joints_[i]->getName();
                print "ith joint : ", i
                print " , lower limit : ", self.__lower_limits_[idx]
                print " , upper limit : ", self.__upper_limits_[idx]
                print " , q_j : ", q[idx] 

        return violate_limit


    def single_step(self, q, x_des, x_pose):
        """
        Solve for the velocities by computing
        the pseudo inverse of the Jacobian matrix.
        
        We compute the jacobian and apply it directly.
        The jacobian J = dx/dq is the first order derivative of 
        FK : C -> X, where C is the configuration space, and X is the 
        task space. So if (m : nb of rows, n : nb of collumns)
            m = dim(X) and n = dim(C).

        It assumes that the jacobian is computed for all dofs 
        in active_indices_
        """
        J = self.jacobian(q)
        assert q.shape[0] == self.__active_indices
        assert x_des.shape[0] == x_pose.shape[0]  # size of task space
        assert x_des.shape[0] == J.shape[0]       # size of task space
        assert q.shape[0]     == J.shape[1]       # size of config space 
        J_plus = np.matrix(pinv(J))

        # Warning the way we compute the difference in angle
        # should be worked out to handle task space distances
        # using quaternions.
        dq_active_dofs = J_plus * self.__magnitude * normalize(x_des - x_pose)
        dq = np.zeros(q.shape)
        dq[self.__active_indices] = dq_active_dofs
        return dq


    def solve(self, q, x_des):
        """
        Solve for a full inverse kinematics pose
        the function saves the intermediate configuration
        to generate a trajectory.
        """
        violates_limits = self.check_violate_joint_limits(q)
        if violates_limits:
            print "Warning configuration violates joint limits"

        has_succeeded = False
        self.__configurations = []
        q_tmp = q
        for it in range(self.__max_nb_iterations):
            q_tmp += self.single_step(q_tmp, x_des, x_pose)
            self.__configurations.append(q_tmp)
            x_pose = self.foward_kinematics(q_tmp)
            dist = norm(x_des - x_pose)
            if dist < self.__max_distance_to_target:
                has_succeeded = True
                break
        if 





