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
from pinocchio_ik import *
from iterative_ik import *
import os

class TestPlanarIterativeIk(PinocchioIterativeIk):

    def __init__(self):
        # active dofs indices
        self.active_dofs=[0, 1]

        PinocchioIterativeIk.__init__(
            self,
            "../urdf/r2_robot.urdf",
            "link2_to_end",
            self.active_dofs)

    def index_joint(self, name):
        return self.robot.model.getJointId(name)-1


if __name__== "__main__":
    test_iterative_ik = TestPlanarIterativeIk()
    data = test_iterative_ik.sample_configs()
    success = test_iterative_ik.run(data)
    test_iterative_ik.save_data_file(data)
    print  "success : ", success

