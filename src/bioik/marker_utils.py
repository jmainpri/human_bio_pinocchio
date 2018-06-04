import time
import csv
import numpy as np
from numpy import linalg as la

# 0 xyphoid process
# 1 T8
# 2 sternal notch
# 3 C7
# 4 Acromion process
# 5 Glenohumeral cntr of rot. (post)
# 6 Medial epicondyle
# 7 lateral epicondyle
# 8 ulnar styloid
# 9 radial styloid
# 10 2nd metacarpal head

# ChestFront_raw      = frame.marker_list[i+0].array     -> 0
# ChestBack_raw       = frame.marker_list[i+1].array     -> 1
# SternumFront_raw    = frame.marker_list[i+2].array     -> 2
# SternumBack_raw     = frame.marker_list[i+3].array     -> 3
# rShoulderFront_raw  = frame.marker_list[i+4].array     -> 4
# rShoulderBack_raw   = frame.marker_list[i+5].array
# rElbowOuter_raw     = frame.marker_list[i+6].array
# rElbowInner_raw     = frame.marker_list[i+7].array
# rWristOuter_raw     = frame.marker_list[i+8].array
# rWristInner_raw     = frame.marker_list[i+9].array
# rPalm_raw           = frame.marker_list[i+10].array

# lShoulderFront_raw  = frame.marker_list[i+11].array
# lShoulderBack_raw   = frame.marker_list[i+12].array
# lElbowOuter_raw     = frame.marker_list[i+13].array
# lElbowInner_raw     = frame.marker_list[i+14].array
# lWristOuter_raw     = frame.marker_list[i+15].array
# lWristInner_raw     = frame.marker_list[i+16].array
# lPalm_raw           = frame.marker_list[i+17].array

#--------------------------------------------------------------
# ELBOWPAD SETUP

# ** Markers
# ChestFront        0
# ChestBack         1
# SternumFront      2
# SternumBack       3
# rShoulderFront    4
# rShoulderBack     5
# rWristOuter       6
# rWristInner       7
# rPalm             8

# ** Objects
# Pelvis            0
# Head              1
# rElbow            2

def remap_to_matlab(in_markers):

    out_markers = in_markers

    tmp = out_markers[6]
    out_markers[6] = in_markers[7]
    out_markers[7] = tmp

    tmp = out_markers[9]
    out_markers[9] = in_markers[8]
    out_markers[8] = tmp

    return out_markers


def load_file(m_filepath, o_filepath):

    print "Trying to open file"

    with open(m_filepath, 'r') as m_file:
        with open(o_filepath, 'r') as o_file:

            marker_file = [row for row in csv.reader(m_file, delimiter=',')]
            object_file = [row for row in csv.reader(o_file, delimiter=',')]

    nb_lines = min(len(marker_file), len(object_file))
    last_frame = nb_lines

    frames_m = []
    frames_o = []

    for row in range(nb_lines):

        markers = []
        objects = []

        m_cells = marker_file[row]
        o_cells = object_file[row]

        # Load Objects
        count = int(o_cells[2])

        for i in range(3, count*9, 9):
            name = str(o_cells[i])
            occluded = int(o_cells[i+1])
            x = float(o_cells[i+2])
            y = float(o_cells[i+3])
            z = float(o_cells[i+4])
            qx = float(o_cells[i+5])
            qy = float(o_cells[i+6])
            qz = float(o_cells[i+7])
            qw = float(o_cells[i+8])

            objects.append(np.array([x, y, z, qx, qy, qz, qw]))

        # Load Markers
        sec = float(m_cells[0])
        nsec = float(m_cells[1])
        count = int(m_cells[2])

        nb_seen = 0
        for i in range(3, count*4, 4):
            id = nb_seen
            name = str(m_cells[i])
            x = float(m_cells[i+1])
            y = float(m_cells[i+2])
            z = float(m_cells[i+3])
            nb_seen += 1

            markers.append(np.array([x, y, z]))

        frames_m.append(markers)
        frames_o.append(objects)

    for frames in frames_m:
        frames = remap_to_matlab(frames)

    print "# configs loaded : " + str(len(frames_m))
    return [frames_m, frames_o]

