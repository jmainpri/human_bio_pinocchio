from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import rospkg

rospack = rospkg.RosPack()
human = se3.RobotWrapper(
    str(rospack.get_path('human_bio_urdf') + "/urdf/human_bio.urdf"))
q = human.q0
print "shape of q : ", q.shape
