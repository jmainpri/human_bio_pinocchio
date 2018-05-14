from sensor_msgs.msg import JointState
import pinocchio as se3
import visualize_trajectories
import utils
import rospy
import time
import sys

def load_data_from_file():
    import h5py
    with h5py.File('./data/trajectories.hdf5', 'r') as f:
        configurations = f["configurations"][:]
        points = f["points"][:]
        trajectories = []
        for i in range(len(configurations)):
            trajectories.append(f["trajectories_{:04d}".format(i)][:])
    return configurations, points, trajectories

def query_yes_no(question, default="yes"):
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
    if default is None:
        prompt = " [y/n or q] "
    elif default == "yes":
        prompt = " [Y/n or q] "
    elif default == "no":
        prompt = " [y/No r q] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

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
    
    print "q : ", q.shape
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time().now()
    for name in robot.model.names:
        if name == "universe":
            continue
        idx = robot.index(name)
        # print "name: {}, idx: {}".format(name, idx)
        joint_state.name.append(name)
        joint_state.position.append(q[idx])
    publisher.publish(joint_state)

def publish_trajecories(trajectories):
    rospy.init_node('human_bio_pinnochio_ik_player')
    for trajectory in trajectories:
        robot = se3.RobotWrapper(utils.human_urdf_path())
        publisher = rospy.Publisher('/joint_states', JointState)
        for q in trajectory:    
            publish_joint_state(publisher, robot, q)
            time.sleep(2./float(len(trajectory))) # play for 2 seconds
        query_yes_no("Replay trajectory")

if __name__ == '__main__':
    configurations, points, trajectories = load_data_from_file()
    print "configurations.shape : ", configurations.shape
    print "points.shape : ", points.shape
    for trajectory in trajectories:
        print trajectory.shape
    publish_trajecories(trajectories)