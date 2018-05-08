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
print "shape of q : ", q.shape
print "human : ", human
for i in range(20):
    q_rand=rand(q.shape)
    human.forwardKinematics(q_rand)
    p = human.data.oMi[human.index("rWristY")].translation
    print "p_{} : {}".format(i, p.transpose())
