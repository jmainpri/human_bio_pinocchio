#!/usr/bin/python

# openrave-robot.py ../../ormodels/human_wpi_bio.xml --info=joints

from test_bio_human_ik import *

import sys
import time
from copy import deepcopy


class TestTwoArmIk(TestBioHumanIk):

    def __init__(self, name):

        TestBioHumanIk.__init__(self, name)

        self.nb_humans = 1
        self.rarm_only = False
        self.use_elbow_pads = False
        self.compute_left_arm = True
        self.environment_file = "../../ormodels/humans_env_two_arms.xml"

if __name__ == "__main__":

    data_folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/'

    folder = data_folder + 'two_arm_test_data/'
    name = "[0800-3511]"
    m_file = folder + name + 'markers.csv'
    o_file = folder + name + 'objects.csv'

    print "try to load file : ", m_file
    print "try to load file : ", o_file

    test = TestTwoArmIk()
    test.initialize(m_file, o_file)

    print len(test.mapping)

    while True:
        test.play_skeleton()
        test.save_file(name)
        print "press enter to exit"
        sys.stdin.readline()
