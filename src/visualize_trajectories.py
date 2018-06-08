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

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import pinocchio as se3
import visualize_trajectories
import utils
import rospy
import time
import sys
import optparse


def load_data_from_file():
    import h5py
    with h5py.File('./data/trajectories.hdf5', 'r') as f:
        configurations = f["configurations"][:]
        points = f["points"][:]
        trajectories = []
        for i in range(len(configurations)):
            trajectories.append(f["trajectories_{:04d}".format(i)][:])
    return configurations, points, trajectories


def query_yes_no(question):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False,
             "quit": "q", "q": "q"}
    prompt = " [Y/n or q] "
    default = "y"
    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes', 'no' or 'quit' "
                             "(or 'y', 'n' or 'q').\n")


def publish_joint_state(publisher, robot, q):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time().now()
    for name in robot.model.names:
        if name == "universe":
            continue
        idx = robot.model.getJointId(name) - 1
        # print "name: {}, idx: {}".format(name, idx)
        joint_state.name.append(name)
        joint_state.position.append(q[idx])
    publisher.publish(joint_state)


def publish_target(publisher, p):
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    t = rospy.Duration()
    marker.lifetime = t
    marker.pose.position.x = p[0]
    marker.pose.position.y = p[1]
    marker.pose.position.z = p[2]
    radius = .1
    marker.scale.x = radius
    marker.scale.y = radius
    marker.scale.z = radius
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    publisher.publish(marker)


def publish_trajecories(urdf_path, configurations, trajectories, targets):
    idx = 0
    robot = se3.RobotWrapper(urdf_path)
    trajectory_pub = rospy.Publisher('/joint_states', JointState)
    target_pub = rospy.Publisher("/target", Marker, queue_size=100)

    while idx < len(trajectories):
        trajectory = trajectories[idx]
        print "targets[idx] : ", targets[idx].transpose()
        publish_target(target_pub, targets[idx])
        publish_joint_state(trajectory_pub, robot, configurations[idx])
        time.sleep(1.)  # play for 1 seconds
        query_yes_no("Continue")
        for q in trajectory:
            publish_target(target_pub, targets[idx])
            publish_joint_state(trajectory_pub, robot, q)
            time.sleep(2. / float(len(trajectory)))  # play for 2 seconds
        answer = query_yes_no("Replay trajectory")
        if answer == 'q':
            break
        if answer == False:
            idx += 1


if __name__ == '__main__':

    parser = optparse.OptionParser("usage: %prog [options] arg1 arg2")
    parser.add_option('--robot',
                      default=False, type="int", dest='robot',
                      help='Play planar robot')
    (options, args) = parser.parse_args()

    urdf_path = utils.human_urdf_path()
    if options.robot:
        urdf_path = "../urdf/r2_robot.urdf"

    rospy.init_node('human_bio_pinnochio_ik_player')

    configurations, targets, trajectories = load_data_from_file()
    print "configurations.shape : ", configurations.shape
    print "targets.shape : ", targets.shape

    for trajectory in trajectories:
        print trajectory.shape
    publish_trajecories(urdf_path, configurations, trajectories, targets)
