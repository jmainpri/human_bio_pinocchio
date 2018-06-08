from math_utils import *
import numpy as np

T = np.eye(4)
T[0:3, 3] = np.array([2.] * 3)
A = Affine3d(T)
print A
B = A * np.array([3.] * 3)
print B

A = Affine3d(np.eye(4))
print "rotation : ", A.rotation
print A
