#!/usr/bin/python

# openrave-robot.py ../../ormodels/human_wpi_bio.xml --info=joints

from test_bio_human_ik import *

import sys
import time
from copy import deepcopy
import os
import sys


class GenerateIkLib(TestBioHumanIk):

    def __init__(self):

        TestBioHumanIk.__init__(self)

        self.nb_humans      = 2
        self.rarm_only      = True
        self.use_elbow_pads = True
        self.compute_left_arm = False
        self.environment_file = "../../ormodels/humans_bio_env.xml"


def generate_ik_library(source_dir, target_dir):

    split_id = 0

    test = GenerateIkLib()

    blocks = sorted([name for name in os.listdir(source_dir) if os.path.isdir(os.path.join(source_dir, name))])

    for block in blocks:

        path = os.path.join(source_dir, block)
        runs = sorted([name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name))])

        for run in runs:

            file_path = os.path.join(path, run)
            out_path = os.path.join(target_dir, block, run)
            print
            print 'Performing IK in block : ', block, ' , run : ', run
            print 'in folder : ', file_path
            print'================================================'
            print 'Outpath : ', out_path

            if not os.path.exists(out_path):
                os.makedirs(out_path)

            files = sorted([name for name in os.listdir(file_path)])

            splits = []
            for f in files:
                split, marker_type = f.split("]")
                split += "]"
                if split not in splits:
                    splits.append(split)

            for split in splits:

                m_file = file_path + "/" + split + "markers.csv"
                o_file = file_path + "/" + split + "objects.csv"

                test.initialize(m_file, o_file, test.drawer)
                print m_file
                print o_file

                test.play_skeleton()
                test.save_file(split, out_path)

                split_id += 1

                print "DONE with split : ", split_id

if __name__ == "__main__":

    if len(sys.argv) >= 4:
        for index in range(1, len(sys.argv)):
            if sys.argv[index] == "-s" and index+1 < len(sys.argv):
                source_dir = str(sys.argv[index+1])
            if sys.argv[index] == "-t" and index+1 < len(sys.argv):
                target_dir = str(sys.argv[index+1])

        generate_ik_library(source_dir, target_dir)
    else:
        print "************************"
        print "******  WARNING  *******"
        print "************************"
        print "USAGE -> GenerateIkLibrary.py -s PATH_TO_SOURCE_LIBRARY -t PATH_TO_TARGET_LIBRARY"
    
    print "try to load file : ", m_file
    print "try to load file : ", o_file

