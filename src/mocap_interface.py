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
import os
import h5py
import argparse


class Affine3d:
    """
        3D dimensional affine/homogenous transform
        The rotation is encoded using quaternions
    """
    translation = None
    rotation = None
    def __init__(self, t, r):
        self.translation = t
        self.rotation = r

    def __str__(self):
        ss = "Transform :\n"
        ss += " - translation (x = {:.4f}, y = {:.4f}, z = {:.4f})\n".format(
            self.translation[0], self.translation[1], self.translation[2])
        ss += " - rotation \
   (x = {:.4f}, y = {:.4f}, z = {:.4f}, w = {:.4f})\n".format(
            self.rotation[0], self.rotation[1], 
            self.rotation[2], self.rotation[3])
        return ss


class HumanMocapSemantics:
    """
        Configurable interface for mocap data frames.
        Gives a semantic interpratation of the marker data in a frame.
    """
    def __init__(self):
        self.__load_config()

    def __load_config(self):
        
        self.__config = utils.mocap_config_data()

        labels = self.__config['labels']
        self.r_shoulder     = lambda frame : frame[labels['rShoulder']]
        self.r_elbow        = lambda frame : frame[labels['rElbow']]
        self.r_wrist        = lambda frame : frame[labels['rWrist']]
        self.l_shoulder     = lambda frame : frame[labels['lShoulder']]
        self.l_elbow        = lambda frame : frame[labels['lElbow']]
        self.l_wrist        = lambda frame : frame[labels['lWrist']]
        self.pelvis         = lambda frame : frame[labels['Pelvis']]

        self.right_arm_joints = (
            lambda f : [f[_] for _ in self.__config['right_arm_joints']])
        self.left_arm_joints = (
            lambda f : [f[_] for _ in self.__config['left_arm_joints']])


class HumanMocapData:
    """
       Loads mocap data and labels it.
    """
    def __init__(self, filename):
        self.__load_data(filename)
        self.semantics = HumanMocapSemantics()

    def __load_data(self, filename):
        """
            Loads data from hdf5 file
        """
        file = h5py.File(filename, 'r')
        skeletonname = file['skeletons'].keys()[0]
        self._data = file['skeletons'][skeletonname][:]  # open as np array

    def nb_frames(self):
        return len(self._data)

    def frame(self, i):
        """ 
            All joint data is concatenated in one frame
        """
        frame = self._data[i]
        labeled_frame = {}
        for i in range(0, len(frame), 7):
            translation = frame[i: i + 3]     # 3 fields position data
            rotation = frame[i + 3: i + 7]    # 4 fields orientation data
            labeled_frame["joint" + str(i)] = Affine3d(translation, rotation)
        return labeled_frame


def main():
    parser = argparse.ArgumentParser(description='Test mocap data so far.')
    parser.add_argument(dest='filepath', type=str)
    args = parser.parse_args()
    mocap_data = HumanMocapData(args.filepath)
    hz=120
    print "Mocap Data"
    print " - filepath : ", args.filepath
    print " - nb_frames : ", mocap_data.nb_frames()
    print " - Hz : ", hz
    print " - dt : ", 1./hz
    print " - duration : {} min".format(mocap_data.nb_frames() * 1./(60.*hz))

    for i in range(2000, 2100, 30):
        frame = mocap_data.frame(i)
        print "Frame : ", i
        print mocap_data.semantics.r_wrist(frame)

if __name__ == '__main__':
    main()
