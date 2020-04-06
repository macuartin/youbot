from __future__ import division
import numpy as np
from homogeneus_matrix import homogeneus_matrix
from inverse_dynamic import inverse_dynamic


def forward_dynamic(DH, Params, tau, q, qd):
    pt, DoF = q.shape
    eye = np.identity(DoF)
    zero = np.zeros((DoF, DoF))
    grav = np.matrix([0, 0, 0, 0, 0, -9.81])
    Fext = np.zeros((6, 1))
    qdd = np.zeros(q.shape)

    for j in range(pt):
        M = inverse_dynamic(DH, Params, np.ones((DoF, 1)) * q[j, :], zero, eye, np.matrix([0, 0, 0, 0, 0, 0]), Fext)
        Fvel = inverse_dynamic(DH, Params, q[j, :], qd[j, :], grav, grav, Fext)

        Minv = np.linalg.inv(M)
        Ft = tau[j, :].reshape(5, 1) - Fvel

        total = np.dot(Minv, Ft)
        qdd[j, :] = np.squeeze(np.asarray(total))

    return qdd, True
