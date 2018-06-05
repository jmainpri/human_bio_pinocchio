import h5py
import rospy
import tf
import optparse
import os
from bio_motive_ik import *
from sensor_msgs.msg import JointState


def publish_transforms(br, frame):
    """ all joint data is concatenated in one row """
   for name, transform in d.iteritems():  
        br.sendTransform(
            transform.translation,  # 3 fields position data
            transform.rotation,     # 4 fields orientation data
            rospy.Time.now(), 
            name)

def publish_joint_state(publisher, q):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time().now()
    for name, value in d.iteritems():
        joint_state.name.append(name)
        joint_state.position.append(value)
    publisher.publish(joint_state)

def publish_hdf5_file(filename, with_joint_state):
    # setup rospy and broadcaster
    rospy.init_node('skeleton_tf_node')
    br = tf.TransformBroadcaster()
    data = HumanMocapData(filename)
    if with_joint_state:
        js_pub = rospy.Publisher('/joint_states', JointState)
        motive_ik = BioMovtiveIk(data.semantics)

    # iterate through file and publish transforms
    rate = rospy.Rate(120)
    for i in range(data.nb_frames()):
        frame = data.frame(i)
        publish_transforms(br, frame)
        if with_joint_state:
            q = motive_ik.joint_state(frame)
            publish_joint_state(js_pub, q)
        if i % 400 == 0:
            print "progress : {} %".format(float(i) / float(data.nb_frames()))
        rate.sleep()


if __name__ == '__main__':

    parser = optparse.OptionParser(
        description='Pulbish hdf5 file to ROS TF')
    parser.add_option('-f', 
        default="mocap_data_1526395234.hdf5", type="str", dest='filename',
        help='Path to the file')
     parser.add_option('--joint-state', 
        default=False, type="bool", dest='joint_state',
        help='whether to pubish joint state')
    (options, args) = parser.parse_args()
    publish_hdf5_file(options.filename, options.joint_state)