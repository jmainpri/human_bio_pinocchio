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
        # print "norm is 0"
        v_norm=np.finfo(v.dtype).eps
    return v/v_norm

class IterativeIK:
    """ 
        This class will compute the inverse kinematics
        for a general manipulator, it relies on the foward kinematics
        function as well as the Jacobian.
    """
    def __init__(self,
        dist_fct, 
        jacobian_fct,
        forward_kinematics_fct, 
        active_dofs=None,
        lower_limits=[],
        upper_limits=[]):

        self.verbose = True
        self.debug = True
        self.q_full = None
        self.configurations = []
        
        self.__dist_fct = dist_fct
        self.__jacobian_fct = jacobian_fct
        self.__forward_kinematics_fct = forward_kinematics_fct
        self.__active_dofs = active_dofs
        self.__lower_limits = lower_limits
        self.__upper_limits = upper_limits
        
        self.__check_joint_limits = (
            len(self.__lower_limits)>0 and 
            len(self.__upper_limits)>0 )

        self.__sigmoid_joint_limits = None
        self.__magnitude = 0.01
        self.__max_nb_iterations = 3000
        self.__max_distance_to_target = 0.01
        self.__conservative_joint_limit_threshold = 1e-5
        

    def interpolated_configurations(self):
        return self.configurations_

    def full_dof_config(self, q_active):
        """
        Returns a configuration of the full dimension
        """
        assert len(self.__active_dofs) == q_active.shape[0]
        if q_active.shape == self.q_full.shape:
            return q_active
        q_tmp = np.copy(self.q_full)
        q_tmp[self.__active_dofs] = q_active
        return q_tmp

    def jacobian(self, q):
        """
        Returns the jacobian of the active dofs
        the class assumes that the underlying library 
        uses a full dof configuration model
        """
        q_full = self.full_dof_config(q)
        J = self.__jacobian_fct(q_full)
        if self.debug:
            print "J.shape : ", J.shape
            print "J:"
            print J
            print J[0:3, self.__active_dofs]
        return J[:, self.__active_dofs]

    def forward_kinematics(self, q):
        """ 
        Returns the foward kinematics
        the class assumes that the underlying library 
        uses a full dof configuration model
        """
        q_full = self.full_dof_config(q)
        x = self.__forward_kinematics_fct(q_full)
        return x

    def check_violate_joint_limits(self, q):
        """ 
        This function will return true if the configuration violates
        oint limits. the configuration passed as input should be only
        for active dofs and match in size the lower and upper argument of 
        of the class. A bad joint indicies list is mantained
        """
        assert q.shape[0] == len(self.__active_dofs)
        assert q.shape[0] == len(self.__lower_limits)
        assert q.shape[0] == len(self.__upper_limits)
        violate_limit = False
        theshold = self.__conservative_joint_limit_threshold
        bad_joint_indices = []
        for idx in range(len(self.__active_dofs)):
            if( q[idx] < (self.__lower_limits[idx] + theshold) or 
                q[idx] > (self.__upper_limits[idx] - theshold)):
                # note this will never add the same joint
                # twice, even if bClearBadJoints = false
                bad_joint_indices.append(idx)
                violate_limit = True

                if self.verbose: 
                    # cout << "does not respect joint limits : "
                    #     << active_joints_[i]->getName();
                    print "ith joint : ", idx
                    print " , lower limit : ", self.__lower_limits[idx]
                    print " , upper limit : ", self.__upper_limits[idx]
                    print " , q_j : ", q[idx] 

        return violate_limit


    def single_step(self, q, dist):
        """
        Solve for the velocities by computing
        the pseudo inverse of the Jacobian matrix.
        
        We compute the jacobian and apply it directly.
        The jacobian J = dx/dq is the first order derivative of 
        FK : C -> X, where C is the configuration space, and X is the 
        task space. So if (m : nb of rows, n : nb of collumns)
            m = dim(X) and n = dim(C).

        It assumes that the jacobian is computed for all dofs 
        in active_indices
        """
        J = self.jacobian(q)
        # print "q :"
        # print q.transpose()
        # print "J : "
        # print J
        assert q.shape[0] == len(self.__active_dofs)
        #assert x_des.shape[0] == x_pose.shape[0]  # size of task space
        assert 6 == J.shape[0]       # size of task space
        assert q.shape[0]     == J.shape[1]       # size of config space 

        # J = np.eye(J.shape[0], J.shape[1])
        J_plus = np.matrix(pinv(J[0:3,:], rcond=1e-8))
        # J_plus = np.matrix(J).transpose()
        # print "J"
        # print J
        # print "J_plus"
        # print J_plus

        # Warning the way we compute the difference in angle
        # should be worked out to handle task space distances
        # using quaternions.
        dist[0:3] = normalize(dist[0:3])
        dist[3:6] = normalize(dist[3:6])
        dq = J_plus * self.__magnitude * dist[0:3]
        if self.debug:
            print "dq : ", dq.transpose()
            print "x_des - x_pose : ", dist.transpose()
            print "J_plus : "
            print J_plus
        return dq

    def solve(self, q, x_des):
        """
        Solve for a full inverse kinematics pose
        the function saves the intermediate configuration
        to generate a trajectory.
        Assumes q is given for active dofs, all configurations
        in this function are only assumed for active dofs.
        """
        print "q_init = ", q.transpose()
        print "x_des = ", x_des
        if self.__check_joint_limits:
            violates_limits = self.check_violate_joint_limits(q)
            if violates_limits:
                print "Warning configuration violates joint limits"
        has_succeeded = False
        self.configurations = []
        q_tmp = np.copy(q)
        x_pose = self.forward_kinematics(q_tmp)
        dist = self.__dist_fct(x_des, x_pose)
        print " - Init  dist : ", dist.transpose()
        for it in range(self.__max_nb_iterations):
            q_tmp += self.single_step(q_tmp, dist)
            x_pose = self.forward_kinematics(q_tmp)
            dist = self.__dist_fct(x_des, x_pose)
            if norm(dist) < self.__max_distance_to_target:
                has_succeeded = True
            if self.verbose and it % 100 == 0:
                print "dist : ", dist.transpose()
            self.configurations.append(self.full_dof_config(q_tmp))
            if has_succeeded or (it == 10 and self.debug):
                break
        print " - final dist : ", dist.transpose()
        if not has_succeeded:
            print "Ik could not converge in given number of iterations"
        else:
            print "succeeded (iter : {})".format(it)
            if self.__check_joint_limits:
                violates_limits = self.check_violate_joint_limits(q_tmp)
                if violates_limits:
                    # has_succeeded = False
                    print "Ik succeeded but violates joint limits"
        return has_succeeded
