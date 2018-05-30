import h5py
import rospy
import tf
import optparse
import os

def publish_transforms(row, br):
    """ all joint data is concatenated in one row """
    for i in range(0, len(row), 7):
        pos = row[i: i + 3]             # 3 fields position data
        rot = row[i + 3: i + 7]         # 4 fields orientation data
        br.sendTransform(pos, rot, rospy.Time.now(), "joint" + str(i), "base")


def publish_hdf5_file(filename):
    # setup rospy and broadcaster
    rospy.init_node('skeleton_tf_node')
    br = tf.TransformBroadcaster()

    # read skeletondata from h5py
    file = h5py.File(filename, 'r')
    skeletonname = file['skeletons'].keys()[0]
    skeletondata = file['skeletons'][skeletonname][:]  # open as numpy array

    # iterate through file and publish transforms
    rate = rospy.Rate(120)
    for i, row in enumerate(skeletondata):
        publish_transforms(row, br)
        if i % 400 == 0:
            print "progress : {} %".format(float(i) / float(len(skeletondata)))
        rate.sleep()


if __name__ == '__main__':

    parser = optparse.OptionParser(
        description='Pulbish hdf5 file to ROS TF')
    parser.add_option('-f', 
        default="mocap_data_1526395234.hdf5", type="str", dest='filename',
        help='Path to the file')
    (options, args) = parser.parse_args()
    publish_hdf5_file(options.filename)
