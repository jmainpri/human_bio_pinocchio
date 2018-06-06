# -*- coding: utf-8 -*-
# transformations.py

# Copyright (c) 2006, Christoph Gohlke
# Copyright (c) 2006-2009, The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holders nor the names of any
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

import warnings
import math
import numpy
from numpy import linalg as la

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


class Affine3d:
    """
        3D dimensional affine/homogenous transform
        The rotation is encoded using quaternions. 
        Uses the ROS convention for the quaternion (x, y, z, w)
        This should be the main interface for this module. 

        TODO: should write a test for this class
    """
    translation = None
    rotation = None
    def __init__(self, t, r=numpy.array([0., 0., 0., 1.])):
        if t.shape == (4, 4):
            self._set_matrix(t)
        elif t.shape == (3, ):
            self.translation = t
            self.rotation = r

    def _set_matrix(self, M):
        if M.shape != (4, 4):
            raise ValueError('Matrix not of the correct size')
        self.translation = numpy.squeeze(numpy.asarray(M[0:3, 3]))
        self.rotation = quaternion_from_matrix(M)

    def linear(self):
        return numpy.mat(quaternion_matrix(self.rotation))

    def matrix(self):
        return numpy.bmat([[self.linear(), 
            numpy.matrix(self.translation).transpose()],
            [numpy.mat([0., 0., 0., 1.])]])

    def __mul__(self, p):
        p_mat = numpy.mat(numpy.concatenate((p, [1]))).transpose()
        return numpy.squeeze(numpy.asarray(self.matrix() * p_mat)[:3])

    def __str__(self):
        ss = "Transform :\n"
        ss += " - translation (x = {:.4f}, y = {:.4f}, z = {:.4f})\n".format(
            self.translation[0], 
            self.translation[1], 
            self.translation[2])
        ss += " - rotation \
   (x = {:.4f}, y = {:.4f}, z = {:.4f}, w = {:.4f})\n".format(
            self.rotation[0], self.rotation[1], 
            self.rotation[2], self.rotation[3])
        return ss


def quaternion_matrix(quaternion):
    """Return rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    # We assum the ROS convention (x, y, z, w)
    quaternion_tmp = numpy.array([0.0]*4)
    quaternion_tmp[1] = quaternion[0] # x
    quaternion_tmp[2] = quaternion[1] # y
    quaternion_tmp[3] = quaternion[2] # z
    quaternion_tmp[0] = quaternion[3] # w
    q = numpy.array(quaternion_tmp, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]])


def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
    ...                    quaternion_from_matrix(R, isprecise=True))
    True
    >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
    >>> is_same_quaternion(quaternion_from_matrix(R, isprecise=False),
    ...                    quaternion_from_matrix(R, isprecise=True))
    True

    """
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    if isprecise:
        q = numpy.empty((4, ))
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = numpy.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = numpy.linalg.eigh(K)
        q = V[[3, 0, 1, 2], numpy.argmax(w)]
    if q[0] < 0.0:
        numpy.negative(q, q)

    # We assume the ROS convention (x, y, z, w)
    quaternion_tmp = numpy.array([0.0]*4)
    quaternion_tmp[3] = q[0] # w
    quaternion_tmp[0] = q[1] # x
    quaternion_tmp[1] = q[2] # y
    quaternion_tmp[2] = q[3] # z
    return quaternion_tmp


def euler_from_matrix(matrix, axes='sxyz'):

    # Return Euler angles from rotation matrix for specified axis sequence.
    #
    # axes : One of 24 axis sequences as string or encoded tuple
    #
    # Note that many Euler angle triplets can describe one matrix.
    #
    # >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    # >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    # >>> R1 = euler_matrix(al, be, ga, 'syxz')
    # >>> numpy.allclose(R0, R1)
    # True
    # >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    # >>> for axes in _AXES2TUPLE.keys():
    # ...    R0 = euler_matrix(axes=axes, *angles)
    # ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    # ...    if not numpy.allclose(R0, R1): print axes, "failed"

    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return numpy.array([ax, ay, az])


def rtocarda( R, i, j, k):

    # RTOCARDA (Spacelib): Rotation  matrix  to Cardan or Eulerian angles.
    #
    # Extracts the Cardan (or Euler) angles from a rotation matrix.
    # The parameters  i, j, k  specify the sequence of the rotation axes
    # (their value must be the constant (X,Y or Z).
    # j must be different from i and k, k could be equal to i.
    # The two solutions are stored in the  three-element vectors q1 and q2.
    # RTOCARDA performs the inverse operation than CARDATOR.
    # Usage:
    #
    #			[q1,q2]=rtocarda(R,i,j,k)
    #
    # Related functions : MTOCARDA
    #
    # (c) G.Legnani, C. Moiola 1998; adapted from: G.Legnani and R.Adamini 1993
    #___________________________________________________________________________

    #spheader
    #disp('got this far')
    # if ( i<X | i>Z | j<X | j>Z | k<X | k>Z | i==j | j==k )
    # 	error('Error in RTOCARDA: Illegal rotation axis ')
    # end

    a = numpy.array([0.0, 0.0, 0.0])
    b = numpy.array([0.0, 0.0, 0.0])

    # print "R : ", R

    if (j-i+3) % 3 == 1:
        sig = 1  # ciclic
    else:
        sig = -1  # anti ciclic

    if i != k:  # Cardanic Convention

        i -= 1
        j -= 1
        k -= 1

        a[0] = math.atan2(-sig*R[j, k], R[k, k])
        a[1] = math.asin(sig*R[i, k])
        a[2] = math.atan2(-sig*R[i, j], R[i, i])

        b[0] = math.atan2(sig*R[j, k], -R[k, k])
        b[1] = ((math.pi-math.asin(sig*R[i, k]) + math.pi) % 2*math.pi)-math.pi
        b[2] = math.atan2(sig*R[i, j], -R[i, i])

    else:  # Euleriana Convention

        l = 6-i-j

        i -= 1
        j -= 1
        k -= 1
        l -= 1

        a[0] = math.atan2(R[j, i], -sig*R[l, i])
        a[1] = math.acos(R[i, i])
        a[2] = math.atan2(R[i, j], sig*R[i, l])

        b[0] = math.atan2(-R[j, i], sig*R[l, i])
        b[1] = -math.acos(R[i, i])
        b[2] = math.atan2(-R[i, j], -sig*R[i, l])

    # report in degrees instead of radians
    a = a * 180/math.pi
    b = b * 180/math.pi

    # print "a : ", a
    # print "b : ", b

    return [a, b]


def normalize(x):

    y = numpy.matrix(numpy.eye(3))

    # y[0, :] = x[0, :] / la.norm(x[0, :])
    # y[1, :] = x[1, :] / la.norm(x[1, :])
    # y[2, :] = x[2, :] / la.norm(x[2, :])

    y[:, 0] = x[:, 0] / la.norm(x[:, 0])
    y[:, 1] = x[:, 1] / la.norm(x[:, 1])
    y[:, 2] = x[:, 2] / la.norm(x[:, 2])

    # important to export as matrix
    return numpy.matrix(y)