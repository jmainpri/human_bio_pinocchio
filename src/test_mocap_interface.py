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

from mocap_interface import *
import argparse


def test_interface():
    parser = argparse.ArgumentParser(description='Test mocap data so far.')
    parser.add_argument('--file', dest='filepath', type=str,
                        default="../../../mocap-mlr/interpolated/mocap_data_1517838370.hdf5")
    args = parser.parse_args()
    mocap_data = HumanMocapData(args.filepath)
    hz = 120
    print "Mocap Data"
    print " - filepath : ", args.filepath
    print " - nb_frames : ", mocap_data.nb_frames()
    print " - Hz : ", hz
    print " - dt : ", 1. / hz
    print " - duration : {} min".format(mocap_data.nb_frames() * 1. / (60. * hz))

    for i in range(2000, 2100, 30):
        frame = mocap_data.frame(i)
        print "Frame : ", i
        print mocap_data.semantics.r_wrist(frame)
        print mocap_data.semantics.r_wrist(frame).matrix()

if __name__ == '__main__':
    test_interface()
