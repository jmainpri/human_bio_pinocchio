#!/usr/bin/python

from MocapDrawer import *
from MocapCommon import *
from BioHumanIk import *

import sys
import time
from copy import deepcopy


class TestBioHumanIk(BioHumanIk):

    def __init__(self, name=""):

        BioHumanIk.__init__(self)

        self.nb_humans      = 2
        self.rarm_only      = True
        self.use_elbow_pads = True

        self.handles = []

        self.traj_human1 = []
        self.traj_human2 = []

        self.environment_file = "../../ormodels/humans_bio_env.xml"

        #self.mapping = [-1, 6, 7, 8, 16, 17, 18, 20, 21, 22, 24, 25, 26]
        self.mapping = []

        self.drawer = None
        self.env = None

    def initialize(self, m_file="", o_file="", drawer=None):

        NB_MARKERS = get_nb_markers(self.use_elbow_pads, self.rarm_only)

        if drawer is None:

            self.drawer = Drawer(NB_MARKERS, self.nb_humans, self.use_elbow_pads, self.rarm_only)
            self.env = self.drawer.env

            self.env.Load(self.environment_file)
            print "loading ", self.environment_file

            self.humans = self.env.GetRobots()
            if self.nb_humans == 1:
                self.humans = [self.env.GetRobots()[0]]
            self.change_color_human()

            # Get torso offset
            self.offset_pelvis_torso_init = self.humans[0].GetJoint("TorsoX").GetHierarchyChildLink().GetTransform()[0:3, 3]

            t_cam = array([[ -0.655253290114, -0.106306078558, 0.747891799297, -0.302201271057] , \
                        [ -0.725788890663, 0.363116971923, -0.584274379801, 2.68592453003] , \
                        [ -0.209460287369, -0.925659269042, -0.315089361376, 2.25037527084] , \
                        [ 0.0, 0.0, 0.0, 1.0]])
            self.env.GetViewer().SetCamera(t_cam)

        else:
            self.drawer = drawer
            self.env = self.drawer.env

        self.drawer.load_file(m_file, o_file)

        self.mapping = []
        self.mapping.append(-1)
        self.mapping.append(self.humans[0].GetJoint("TorsoX").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("TorsoZ").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("TorsoY").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rShoulderY1").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rShoulderX").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rShoulderY2").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rElbowZ").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rElbowX").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rElbowY").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rWristZ").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rWristX").GetDOFIndex())
        self.mapping.append(self.humans[0].GetJoint("rWristY").GetDOFIndex())

        if not self.rarm_only:
            self.mapping.append(self.humans[0].GetJoint("lShoulderY1").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lShoulderX").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lShoulderY2").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lElbowZ").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lElbowX").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lElbowY").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lWristZ").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lWristX").GetDOFIndex())
            self.mapping.append(self.humans[0].GetJoint("lWristY").GetDOFIndex())
            self.compute_left_arm = True

    def change_color_human(self):

        if len(self.humans) <= 1:
            return

        links = []
        for jIdx, j in enumerate(self.humans[1].GetJoints()):
            # print "%s, \t%.3f, \t%.3f" % (j.GetName(), j.GetLimits()[0], j.GetLimits()[1])
            l = j.GetFirstAttached()
            if l is not None : links.append(l)
            l = j.GetSecondAttached()
            if l is not None : links.append(l)

        for l in links:
            for g in l.GetGeometries():
                # print g.GetDiffuseColor()
                if set(g.GetDiffuseColor()) & set([0.80000001, 0., 0.01]):
                    g.SetDiffuseColor([0., 0., 0.8])

    def PrintView(self):

        while True:

            print "GetViewer().GetCameraTransform() : "
            t_cam = self.env.GetViewer().GetCameraTransform()

            line_str = ""
            line_str += 'array(['
            for i in range(4):
                line_str += '['
                for j in range(4):
                    line_str += ' ' + str(t_cam[i, j]) + ','
                line_str = line_str.rstrip(',')
                line_str += ']'
                if i < 3:
                    line_str += ' , \\ \n'
            line_str += '])'

            print line_str
            print "Press return to get view matrix."
            sys.stdin.readline()

    def remap(markers, in_markers):

        out_markers = deepcopy(in_markers)

        # Remap wrist
        tmp = out_markers[6]
        out_markers[6] = in_markers[7]
        out_markers[7] = tmp

        return out_markers

    def set_human_model_sizes(self, human, t_pelvis, offset_pelvis_torso,
                              r_offset_torso, r_offset_shoulder_elbow, r_offset_elbow_wrist,
                              l_offset_torso, l_offset_shoulder_elbow, l_offset_elbow_wrist):
        # Place the human according to the torso and pelvis frame
        # future notes: when placing the human according to the pelvis frame
        # the torso offset should be applied
        t_offset = MakeTransform(eye(3), matrix(offset_pelvis_torso))
        human.SetTransform(array(eye(4)))

        t_waist = t_pelvis * t_offset

        waist_center = array(transpose(t_waist[:, 3]).tolist()[0][:3])
        waist_rot = euler_from_matrix(t_waist, 'rxyz')

        human.SetDOFValues(waist_rot, [3, 4, 5])
        human.SetDOFValues(waist_center, [0, 1, 2])

        # Set elbow size
        # self.human.SetDOFValues([-self.offset_torso[0]], [9])
        # self.human.SetDOFValues([self.offset_torso[1]], [10])
        # self.human.SetDOFValues([-self.offset_torso[2]], [11])
        human.SetDOFValues([r_offset_torso[0]], [human.GetJoint("rShoulderTransX").GetDOFIndex()])
        human.SetDOFValues([r_offset_torso[1]], [human.GetJoint("rShoulderTransY").GetDOFIndex()])
        human.SetDOFValues([r_offset_torso[2]], [human.GetJoint("rShoulderTransZ").GetDOFIndex()])
        human.SetDOFValues([r_offset_shoulder_elbow], [human.GetJoint("rArmTrans").GetDOFIndex()])
        human.SetDOFValues([r_offset_elbow_wrist], [human.GetJoint("rForeArmTrans").GetDOFIndex()])

        # print r_offset_torso[1]

        if self.compute_left_arm:
            human.SetDOFValues([l_offset_torso[0]], [human.GetJoint("lShoulderTransX").GetDOFIndex()])
            human.SetDOFValues([l_offset_torso[1]], [human.GetJoint("lShoulderTransY").GetDOFIndex()])
            human.SetDOFValues([l_offset_torso[2]], [human.GetJoint("lShoulderTransZ").GetDOFIndex()])
            human.SetDOFValues([l_offset_shoulder_elbow], [human.GetJoint("lArmTrans").GetDOFIndex()])
            human.SetDOFValues([l_offset_elbow_wrist], [human.GetJoint("lForeArmTrans").GetDOFIndex()])

        # Map the joint angles and set to radians
    def get_human_configuration(self, human, config):

        motion = [config]
        for configuration in motion:  # motion should be one row. otherwise take the last element

            q = human.GetDOFValues()
            for i, dof in enumerate(configuration):
                if self.mapping[i] >= 0:
                    q[self.mapping[i]] = dof * pi / 180
        return q

    def compute_dist_to_points(self, human, markers, t_elbow):

        # Get joint centers
        p_torso_origin = (markers[0] + markers[1])/2

        p_r_shoulder_center = array([markers[4][0], markers[4][1], markers[5][2]])
        if self.use_elbow_pads:
            p_r_elbow_center = array(transpose(t_elbow[:, 3]).tolist()[0][:3])
        else:
            p_r_elbow_center = array([0, 0, 0])
        p_r_wrist_center = (markers[6] + markers[7])/2

        # Get the points in the global frame
        # inv_pelvis = la.inv(t_pelvis)
        inv_pelvis = eye(4)

        p0 = array(array(inv_pelvis).dot(append(p_torso_origin, 1)))[0:3]

        p1r = array(array(inv_pelvis).dot(append(p_r_shoulder_center, 1)))[0:3]
        p2r = array(array(inv_pelvis).dot(append(p_r_elbow_center, 1)))[0:3]
        p3r = array(array(inv_pelvis).dot(append(p_r_wrist_center, 1)))[0:3]

        if self.compute_left_arm:

            p_l_shoulder_center = array([markers[11][0], markers[11][1], markers[12][2]])
            if self.use_elbow_pads:
                p_l_elbow_center = array(transpose(t_elbow[:, 3]).tolist()[0][:3])
            else:
                p_l_elbow_center = array([0, 0, 0])
            p_l_wrist_center = (markers[13] + markers[14])/2

            p1l = array(array(inv_pelvis).dot(append(p_l_shoulder_center, 1)))[0:3]
            p2l = array(array(inv_pelvis).dot(append(p_l_elbow_center, 1)))[0:3]
            p3l = array(array(inv_pelvis).dot(append(p_l_wrist_center, 1)))[0:3]

        dist = 0.0

        print_dist = False

        for j in human.GetJoints():

            p_link = j.GetHierarchyChildLink().GetTransform()[0:3, 3]

            if j.GetName() == "TorsoZ":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.02, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p0)
                if print_dist:
                    print "p_link : ", p_link
                    print "dist torso : ", dist
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                # l = self.human.GetLink("TorsoDummyY")
                # self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 1.0))


            if j.GetName() == "rShoulderX":
                # self.handles.append(self.env.plot3(p_link, pointsize=0.05, colors=array([0, 0, 0]), drawstyle=1))
                dist = la.norm(p_link - p1r)
                if print_dist:
                    print "dist shoulder : ", dist
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
            if j.GetName() == "rElbowZ":
                dist = la.norm(p_link - p2r)
                if print_dist:
                    print "dist elbow : ", dist
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
            if j.GetName() == "rWristX":
                dist = la.norm(p_link - p3r)
                self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                if print_dist:
                    print "dist wrist : ", dist

            if self.compute_left_arm:

                if j.GetName() == "lShoulderX":
                    # self.handles.append(self.env.plot3(p_link, pointsize=0.05, colors=array([0, 0, 0]), drawstyle=1))
                    dist = la.norm(p_link - p1l)
                    if print_dist:
                        print "dist shoulder : ", dist
                    self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                if j.GetName() == "lElbowZ":
                    dist = la.norm(p_link - p2l)
                    if print_dist:
                        print "dist elbow : ", dist
                    self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                if j.GetName() == "lWristX":
                    dist = la.norm(p_link - p3l)
                    self.handles.append(self.env.plot3(p_link, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
                    if print_dist:
                        print "dist wrist : ", dist

            #if j.GetName() == "rShoulderZ":
            #    self.handles.append(misc.DrawAxes(self.env, j.GetHierarchyChildLink().GetTransform(), 0.3))

        # for j in self.human.GetJoints():
        #     if j.GetName() == "TorsoX":  # j.GetName() == "rShoulderX" or
        #         t_link = j.GetHierarchyChildLink().GetTransform()
        #         self.handles.append(misc.DrawAxes(self.env, t_link, 0.3))

        # self.handles.append(self.env.plot3(p1, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        # self.handles.append(self.env.plot3(p2, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))
        # self.handles.append(self.env.plot3(p3, pointsize=0.03, colors=array([0, 0, 1]), drawstyle=1))

        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("TorsoZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rShoulderY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rElbowZ").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rWristY").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, self.human.GetJoint("rArmTrans").GetHierarchyChildLink().GetTransform(), 1))
        # self.handles.append(misc.DrawAxes(self.env, inv_pelvis * self.t_torso, 1))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))
        # self.handles.append(misc.DrawAxes(self.env, eye(4), 2))
        # self.handles.append(misc.DrawAxes(self.env, self.t_pelvis, 2))

        # print "joint : ", self.human.GetJoint("zTorsoTrans").GetHierarchyChildLink().GetTransform()[0:3, 3]

        self.handles.append(misc.DrawAxes(self.env, self.trunkEr, .2))
        self.handles.append(misc.DrawAxes(self.env, self.UAEr, .2))
        self.handles.append(misc.DrawAxes(self.env, self.LAEr, .2))
        self.handles.append(misc.DrawAxes(self.env, self.handEr, .2))

        if self.compute_left_arm:
            self.handles.append(misc.DrawAxes(self.env, self.trunkEl, .2))
            self.handles.append(misc.DrawAxes(self.env, self.UAEl, .2))
            self.handles.append(misc.DrawAxes(self.env, self.LAEl, .2))
            self.handles.append(misc.DrawAxes(self.env, self.handEl, .2))

        return dist

    def save_file(self, split_name, folder="."):

        # print "Trying to output new file"

        #  Get the out filename
        # dir, path = os.path.split(self.m_filepath)
        # name, type = path.rsplit('.', 1)
        # m_outpath = name + '_fixed.' + type
        #
        # dir, path = os.path.split(self.o_filepath)
        # name, type = path.rsplit('.', 1)
        # o_outpath = name + '_fixed.' + type

        # No need to normalize ids.  we output marker names
        # print "Trying to normalize ids"
        # self.normalize_ids()

        traj_file_human1 = folder + "/" + split_name + '_human1_.csv'
        traj_file_human2 = folder + "/" + split_name + '_human2_.csv'

        with open(traj_file_human1, 'w') as h1_file:
            with open(traj_file_human2, 'w') as h2_file:

                line_str = ''
                for q in self.traj_human1:
                    for q_i in q:
                        for q_ii in q_i:
                            line_str += str(q_ii) + ','
                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                h1_file.write(line_str)

                line_str = ''
                for q in self.traj_human2:
                    for q_i in q:
                        for q_ii in q_i:
                            line_str += str(q_ii) + ','
                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                h2_file.write(line_str)

        print "End writing !!!"
        print traj_file_human1
        print traj_file_human2

    def draw_frame(self, frame, dt=None):

        del self.handles[:]
        self.drawer.clear()
        self.drawer.draw_frame_skeleton(frame)

        humans = self.drawer.isolate_humans(frame)

        for j, h in enumerate(self.humans):

            markers_remaped = self.remap(humans[j].markers)

            markers = []
            for m in markers_remaped:
                markers.append(m.array)

            transforms = []
            for o in humans[j].objects:
                transforms.append(o.get_transform())

            t_pelvis = transforms[0] * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))
            t_head   = transforms[1]

            if self.use_elbow_pads:
                t_elbow  = transforms[2]
                t_elbow  = t_elbow * MakeTransform(rodrigues([0, 0, -pi/2]), matrix([0, 0, 0]))
                t_elbow  = t_elbow * MakeTransform(rodrigues([0, pi/2, 0]), matrix([0, 0, 0]))
            else:
                t_elbow = array(eye(4))

            # self.handles.append(misc.DrawAxes(self.env, t_elbow, .2))

            trunk_center = (markers[0] + markers[1])/2
            inv_pelvis = la.inv(t_pelvis)
            trunk_center = array(array(inv_pelvis).dot(append(trunk_center, 1)))[0:3]

            markers_in_pelvis = self.get_markers_in_pelvis_frame(markers, t_pelvis)

            if not self.compute_left_arm:
                [config, d_r_torso, d_r_shoulder_elbow, d_r_elbow_wrist] = \
                    self.compute_ik(markers_in_pelvis, la.inv(self.t_trans) * t_elbow)
                d_l_torso = 0
                d_l_shoulder_elbow = 0
                d_l_elbow_wrist = 0
            else:
                [config,
                 d_r_torso, d_r_shoulder_elbow, d_r_elbow_wrist,
                 d_l_torso, d_l_shoulder_elbow, d_l_elbow_wrist,
                 ] = self.compute_ik(markers_in_pelvis, la.inv(self.t_trans) * t_elbow)

            offset_pelvis_torso = trunk_center
            offset_pelvis_torso -= self.offset_pelvis_torso_init
            # offset_pelvis_torso += array([0.16, 0., 0.])

            self.set_human_model_sizes(h, t_pelvis, offset_pelvis_torso,
                                       d_r_torso, d_r_shoulder_elbow, d_r_elbow_wrist,
                                       d_l_torso, d_l_shoulder_elbow, d_l_elbow_wrist)

            q_cur = self.get_human_configuration(h, config)
            h.SetDOFValues(q_cur[0:h.GetDOF()])

            self.compute_dist_to_points(h, markers, t_elbow)

            # Save to current configurations
            if j == 0:
                traj = self.traj_human1
            if j == 1:
                traj = self.traj_human2

            traj.append([[dt], h.GetDOFValues()])

    def play_skeleton(self, max_frame=None):
        # for frame in self.frames:
        prev_time = self.drawer.frames[0].get_time()

        self.traj_human1 = []
        self.traj_human2 = []

        t0_prev_time = time.clock()
        t0_end = time.time()
        dt = 0.

        for i, frame in enumerate(self.drawer.frames):

            t0 = time.time()
            dt_0 = t0 - t0_prev_time
            t0_prev_time = t0

            # time.sleep(max(dt-(dt_0), 0.))

            curr_time = frame.get_time()
            dt = curr_time - prev_time
            prev_time = curr_time

            # print "dt ", dt, " dt0 : ", dt_0

            if max_frame is not None:
                if i > max_frame:
                    break

            if i % 4 == 0:
                del self.handles[:]

            self.drawer.clear()
            self.drawer.draw_frame_skeleton(frame)

            humans = self.drawer.isolate_humans(frame)

            for j, h in enumerate(self.humans):

                markers_remaped = self.remap(humans[j].markers)

                markers = []
                for m in markers_remaped:
                    markers.append(m.array)

                transforms = []
                for o in humans[j].objects:
                    transforms.append(o.get_transform())

                t_pelvis = transforms[0] * MakeTransform(rodrigues([0, 0, pi]), matrix([0, 0, 0]))
                t_head   = transforms[1]

                if self.use_elbow_pads:
                    t_elbow  = transforms[2]
                    t_elbow  = t_elbow * MakeTransform(rodrigues([0, 0, -pi/2]), matrix([0, 0, 0]))
                    t_elbow  = t_elbow * MakeTransform(rodrigues([0, pi/2, 0]), matrix([0, 0, 0]))
                else:
                    t_elbow = array(eye(4))

                # self.handles.append(misc.DrawAxes(self.env, t_elbow, .2))

                trunk_center = (markers[0] + markers[1])/2
                inv_pelvis = la.inv(t_pelvis)
                trunk_center = array(array(inv_pelvis).dot(append(trunk_center, 1)))[0:3]

                self.handles.append(misc.DrawAxes(self.env, t_pelvis, .3))

                markers_in_pelvis = self.get_markers_in_pelvis_frame(markers, t_pelvis)

                if not self.compute_left_arm:
                    [config, d_r_torso, d_r_shoulder_elbow, d_r_elbow_wrist] = \
                        self.compute_ik(markers_in_pelvis, la.inv(self.t_trans) * t_elbow)
                    d_l_torso = 0
                    d_l_shoulder_elbow = 0
                    d_l_elbow_wrist = 0
                else:
                    [config,
                     d_r_torso, d_r_shoulder_elbow, d_r_elbow_wrist,
                     d_l_torso, d_l_shoulder_elbow, d_l_elbow_wrist,
                     ] = self.compute_ik(markers_in_pelvis, la.inv(self.t_trans) * t_elbow)

                offset_pelvis_torso = trunk_center
                offset_pelvis_torso -= self.offset_pelvis_torso_init
                # offset_pelvis_torso += array([0.16, 0., 0.])

                self.set_human_model_sizes(h, t_pelvis, offset_pelvis_torso,
                                           d_r_torso, d_r_shoulder_elbow, d_r_elbow_wrist,
                                           d_l_torso, d_l_shoulder_elbow, d_l_elbow_wrist)

                q_cur = self.get_human_configuration(h, config)
                h.SetDOFValues(q_cur[0:h.GetDOF()])

                self.compute_dist_to_points(h, markers, t_elbow)

                # Save to current configurations
                if j == 0:
                    traj = self.traj_human1
                if j == 1:
                    traj = self.traj_human2

                traj.append([[dt], h.GetDOFValues()])

            self.draw_frame(frame)

#                print curr_time

            # if i % 3 == 0:
            #     print "press enter to continue"
            #     sys.stdin.readline()

            #for h in self.humans:
            #    print "robot in collision ", h.CheckSelfCollision()

    def run(self):

        self.play_skeleton()

if __name__ == "__main__":

    # folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/vacation/test/'
    # m_file = folder + '[0600-1000]markers.csv'
    # o_file = folder + '[0600-1000]objects.csv'

    data_folder = '/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/data/'

    folder = data_folder + 'vacation/four_motions/'

    # m_file = folder + '[0580-0680]markers.csv'
    # o_file = folder + '[0580-0680]objects.csv'
    #
    # m_file = folder + '[0760-0900]markers.csv'
    # o_file = folder + '[0760-0900]objects.csv'
    #
    # m_file = folder + '[1060-1180]markers.csv'
    # o_file = folder + '[1060-1180]objects.csv'

    # m_file = folder + '[1300-1420]markers.csv'
    # o_file = folder + '[1300-1420]objects.csv'

    # m_file = folder + '[2160-2280]markers.csv'
    # o_file = folder + '[2160-2280]objects.csv'

    # m_file = folder + '[1820-1960]markers.csv'
    # o_file = folder + '[1820-1960]objects.csv'

    # m_file = folder + '[1500-1680]markers.csv'
    # o_file = folder + '[1500-1680]objects.csv'

    # VACTION ------------------------------------

    name = '[0440-0580]'

    m_file = folder + name + 'markers.csv'
    o_file = folder + name + 'objects.csv'

    # name = '[0700-0860]'
    # m_file = folder + name + 'markers.csv'
    # o_file = folder + name + 'objects.csv'

    # m_file = folder + '[1460-1620]markers.csv'
    # o_file = folder + '[1460-1620]objects.csv'
    # name = '[1460-1620]'

    # m_file = folder + '[1800-1980]markers.csv'
    # o_file = folder + '[1800-1980]objects.csv'
    #
    name = '[1460-1620]'
#    m_file = folder + name + 'markers.csv'
#    o_file = folder + name + 'objects.csv'

#    m_file = folder + name + 'markers_new_smooth.csv'
#    o_file = folder + name + 'objects_new_smooth.csv'

    m_file = folder + name + 'markers_raw.csv'
    o_file = folder + name + 'objects_raw.csv'

    # LATEST ------------------------------------

    # Folder 1
#    folder_num = '1'
#    name = '[0446-0578]'
#    name = '[0780-0871]'
#    name = '[2554-2671]' # stopping

#    # Folder 2
#    folder_num = '2'
#    name = '[0525-0657]'
#    name = '[2197-2343]'
#    name = '[2711-2823]'

    # Folder 3
    folder_num = '3'
    name = '[0444-0585]'
    #name = '[1064-1140]'
    #name = '[1342-1451]'
    #name = '[1882-1981]' # regrasp
    #name = '[2172-2249]'
    #name = '[2646-2737]' # second human too noisy

    # Folder 4
    #folder_num = '4'
    #name = '[0489-0589]'
    #name = '[1537-1608]'
    #name = '[2018-2099]'

    # Folder 5
    # name = ll

    # Folder 6
    #folder_num = '6'
    #name = '[0408-0491]' # Perfect!!!
    #name = '[0889-0945]' # too short
    #name = '[1188-1256]'

    # Folder 7
    # name = ll

    # Folder 6
    folder_num = '8'
    name = '[0629-0768]' # Replan

    folder = data_folder + 'ten_runs/trials/' + folder_num + '/'
    m_file = folder + name + 'markers.csv'
    o_file = folder + name + 'objects.csv'

   #
   # m_file = folder + '[2554-2671]markers.csv'
   # o_file = folder + '[2554-2671]objects.csv'

    # folder = data_folder + 'ten_runs/video/'
    # name = ""
    # m_file = folder + name + 'markers_fixed.csv'
    # o_file = folder + name + 'objects_fixed.csv'

    # ------
    #data_folder = '/home/rafi/logging_twelve/0/'
    #folder = data_folder
    #name = ""
    #m_file = folder + name + 'markers_fixed.csv'
    #o_file = folder + name + 'objects_fixed.csv'

    print "try to load file : ", m_file
    print "try to load file : ", o_file

    test = TestBioHumanIk()
    test.initialize(m_file, o_file)

    while True:
        test.run()
        test.save_file(name)
        print "press enter to exit"
        sys.stdin.readline()

