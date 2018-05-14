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
import os

class TestIterativeIk:

    def __init__(self, urdf_path):
        # Load config files
        #  utils.human_urdf_path()
        self.human=se3.RobotWrapper(urdf_path)
        data=utils.human_config_data()
        
        # Setup default translations
        q_default=np.zeros(self.human.q0.shape)
        for name, value in data['right_arm_default'].iteritems():
            q_default[self.human.index(name)] = value
        self.human.q0 = q_default

        # active dofs indices
        self.active_dofs=[
            self.human.index(name) for name in data["right_arm_dofs"]]

        # dof limits
        model = self.human.model
        self.lower_limits = model.lowerPositionLimit[self.active_dofs]
        self.upper_limits = model.upperPositionLimit[self.active_dofs]
        assert len(self.lower_limits) == len(self.upper_limits)

        # wrist index
        wrist_name = "rWristY"
        self.wrist_index = self.human.index(wrist_name)

        # store trajectories
        self.trajectories = []

    def forward_kinematics(self, q_all):
        self.human.forwardKinematics(q_all)
        return np.array(self.human.data.oMi[self.wrist_index].translation)

    def sample_q(self):
        nb_active_dofs = len(self.lower_limits)
        q_rand = np.random.uniform(self.lower_limits, self.upper_limits)
        assert nb_active_dofs == q_rand.shape[0]
        for i in range(q_rand.shape[0]):
            assert q_rand[i] > self.lower_limits[i]
            assert q_rand[i] < self.upper_limits[i]
        return q_rand

    def sample_q_normal(self, q_mean):
        nb_active_dofs = len(self.lower_limits)
        q_rand = np.zeros(q_mean.shape)
        for i in range(q_mean.shape[0]):
            q_rand[i] = np.random.normal(q_mean[i], 
                (self.upper_limits[i] - self.lower_limits[i]) / 10. )
        q_rand = np.clip(q_rand, self.lower_limits, self.upper_limits)
        assert q_mean.shape == q_rand.shape
        assert nb_active_dofs == q_rand.shape[0]
        for i in range(q_rand.shape[0]):
            assert q_rand[i] >= self.lower_limits[i]
            assert q_rand[i] <= self.upper_limits[i]
        return q_rand

    def sample_configs(self):
        print "shape of q : ", self.human.q0.shape
        print "human : ", self.human
        nb_configs = 10
        data = [None] * nb_configs
        for i in range(nb_configs):
            # q_active=rand((len(active_dofs)))
            q_active=self.sample_q()
            q_all=np.copy(self.human.q0)
            q_all[self.active_dofs] = q_active
            p = self.forward_kinematics(q_all)
            data[i] = np.array([q_all, p])
            print "p_{} : {}".format(i, p.transpose())
        return np.array(data)

    def run(self, data):
        print "active dofs : ", self.active_dofs
        print "q_0.shape : ", self.human.q0.shape
        print "wrist_index : ", self.wrist_index

        iterative_ik=IterativeIK(
            lambda q: self.human.jacobian(q, self.wrist_index, True, False),
            lambda q: self.forward_kinematics(q),
            self.active_dofs,
            self.lower_limits,
            self.upper_limits)
        iterative_ik.verbose = False
        iterative_ik.q_full = np.copy(self.human.q0)
        nb_succeed=0
        for i, config in enumerate(data):
            print '******* --- IK nb. {} --- *******'.format(i)
            iterative_ik.configurations = []
            succeeded = iterative_ik.solve(
                self.sample_q_normal(config[0][self.active_dofs]), config[1])
            self.trajectories.append(np.stack(iterative_ik.configurations))
            if succeeded:
                nb_succeed += 1
        print "Total nb if success : {} over {}".format(
            nb_succeed, len(data))
        return True

    def save_data_file(self, data_out):
        # print "shape of dataset : ", data_out.shape
        directory=os.path.abspath(os.path.dirname(__file__)) + "/data"
        if not os.path.exists(directory):
            os.makedirs(directory)
        import h5py
        f = h5py.File(directory + '/trajectories.hdf5', 'w')
        # print data_out[:,1].shape
        f.create_dataset("configurations", data=np.stack(data_out[:,0]))
        f.create_dataset("points", data=np.stack(data_out[:,1]))
        for i, t in enumerate(self.trajectories):
            f.create_dataset("trajectories_{:04d}".format(i), 
                data=np.stack(t))
        f.close()


if __name__== "__main__":
    test_iterative_ik = TestIterativeIk(utils.human_urdf_path())
    data = test_iterative_ik.sample_configs()
    success = test_iterative_ik.run(data)
    test_iterative_ik.save_data_file(data)
    print  "success : ", success

