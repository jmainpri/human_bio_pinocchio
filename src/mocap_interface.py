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
# import pinocchio as se3
# from pinocchio.utils import *
# from pinocchio.explog import *
from iterative_ik import *
import os
import h5py

class HumanMocapFrame:

    def __init__(self):
        self.__load_config()

    def __load_config(self):
        self.__config = utils.mocap_config_data()
        print self.__config['labels']


class HumanMocapData:

    def __init__(self, filename):
        self.__load_file(filename)

    def nb_key_frames(self):
        return len(self._data)

    def __load_file(self, filename):
        file = h5py.File(filename, 'r')
        skeletonname = file['skeletons'].keys()[0]
        self._data = file['skeletons'][skeletonname][:]  # open as np array


if __name__ == '__main__':
    mocap_frame = HumanMocapFrame()

