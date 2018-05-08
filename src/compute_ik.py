from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import rospkg

print se3
rospack = rospkg.RosPack()
human = se3.RobotWrapper(
    str(rospack.get_path('human_bio_urdf') + "/urdf/human_bio.urdf"))
q = human.q0
wrist_name = "rWristY"
wrist_index = human.index(wrist_name)
print "shape of q : ", q.shape
print "wrist_name : ", wrist_name
print "wrist_index : ", wrist_index

for i in range(20):
    q_rand=rand(q.shape)
    human.forwardKinematics(q_rand)
    J = human.jacobian(q_rand, wrist_index)
    print "Jacobian shape : ", J.shape
    np.set_printoptions(
        suppress=True, 
        formatter={'float_kind':'{:5.2f}'.format}, 
        linewidth=100000)
    print J[:, 23:37]

    # print "J : ", J.transpose()
    # p = human.data.oMi[].translation
    # print "p_{} : {}".format(i, p.transpose())
