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
import os
import yaml


def human_urdf_path():
    """
    Uses rospack to file the human urdf file, if ros is not installed
    simply assumes that the packages are installed in the same
    directory
    """
    directory = "../../human_bio_urdf"
    try:
        import rospkg
        rospack = rospkg.RosPack()
        directory = rospack.get_path('human_bio_urdf')
    except Exception as e:
        print "rospack failed with error code: {}".format(e)
    filepath = directory + "/urdf/human_bio.urdf"
    if os.path.exists(filepath):
        print "found URDF at : {}".format(filepath)
        return filepath
    else:
        print "human_bio.urdf not found !!!"
        return str("")

def human_config_data():
    """
    Loads data from config file
    """
    with open(os.path.dirname(os.path.realpath(__file__)) +  os.sep + 
        "../config/human_urdf.yaml", 'r') as stream:
        try:
            data = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    return data

def mocap_config_data():
    """
    Loads motion capture config data
    """
    with open(os.path.dirname(os.path.realpath(__file__)) +  os.sep + 
        "../config/mocap.yaml", 'r') as stream:
        try:
            data = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    return data
