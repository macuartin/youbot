#!/usr/bin/env python
from __future__ import division
import numpy as np
from forward_kinematic import forward_kinematic
from rot2euler import *


def jacobian(DH, q):
    '''
    Recibe la matriz DH y los valores articulares de un punto de trayectoria
    '''
    # Hay que realizar un ciclo en reversa para encontrar las derivadas parciales

    DoF = len(DH)

    J = np.matrix([[0] * DoF] * 6)

    for i in range(DoF-1, -1, -1):

        T = forward_kinematic(DH[i:DoF, :], q[i:DoF, 0])

        J[0, i] = -T[0, 0]*T[1, 3]+T[1, 0]*T[0, 3]
        J[1, i] = -T[0, 1]*T[1, 3]+T[1, 1]*T[0, 3]
        J[2, i] = -T[0, 2]*T[1, 3]+T[1, 2]*T[0, 3]
        J[3, i] = T[2, 0]
        J[4, i] = T[2, 1]
        J[5, i] = T[2, 2]

    return J
