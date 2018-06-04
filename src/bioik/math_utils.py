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