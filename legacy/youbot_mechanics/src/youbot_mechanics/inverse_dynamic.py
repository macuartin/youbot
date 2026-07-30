from __future__ import division
import numpy as np
from homogeneus_matrix import homogeneus_matrix
from dynamics_operators import *


def inverse_dynamic(DH, Params, Q, Qd, Qdd, grav, Fext):
    DoF, pt = Q.shape
    V0 = np.zeros((6, 1))
    Vd0 = np.transpose(grav)
    H = np.array([[0], [0], [1], [0], [0], [0]])
    V = np.zeros((6, DoF))
    Vd = np.zeros((6, DoF))
    F = np.zeros((6, DoF))
    tau = np.zeros((DoF, pt))

    # Etapa 1: Calculo de Velocidad y Aceleracion por cuerpo.

    for i in range(DoF):
        l = np.squeeze(np.asarray(DH[i, :]))
        A = homogeneus_matrix(l, Q[i])
        r = A[0:3, 0:3]
        p = A[0:3, 3]

        R = rotation(r)
        P = trasnlation(p)

        if i == 0:
            v = np.dot(np.dot(np.transpose(R), P), V0) + np.dot(H, Qd[i])
            V[:, i] = np.squeeze(np.asarray(v))

            vd = np.dot(np.dot(np.transpose(R), P), Vd0) + np.dot(H, Qdd[i])
            Vd[:, i] = np.squeeze(np.asarray(vd))

        else:
            v = np.dot(np.dot(np.transpose(R), P), V[:, i-1].reshape(6, 1)) + np.dot(H, Qd[i])
            V[:, i] = np.squeeze(np.asarray(v))

            Want = Wskew(V[0:3, i-1])
            Wact = Wskew(V[0:3, i])

            coriolis = np.dot(np.dot(np.dot(Want, np.transpose(R)), P), V[:, i-1].reshape(6, 1))
            centripeta = np.dot(H, Qdd[i])
            centrifuga = np.dot(np.dot(Wact, H), Qd[i])

            vd = np.dot(np.dot(np.transpose(R), P), Vd[:, i-1].reshape(6, 1)) + np.dot(H, Qdd[i]) + coriolis + centripeta + centrifuga
            Vd[:, i] = np.squeeze(np.asarray(vd))

    # Etapa 2: Calculo de Fuerzas por cuerpo.
    for i in range(DoF-1, -1, -1):
        l = np.squeeze(np.asarray(DH[i, :]))
        A = homogeneus_matrix(l, Q[i])

        r = A[0:3, 0:3]
        p = A[0:3, 3]

        R = rotation(r)
        P = trasnlation(p)

        I = spatialInertia(Params[i, :])
        S = centerOfMassDistance(Params[i, :])

        Wact = Wskew(V[0:3, i])

        if i == DoF-1:
            f = np.dot(np.dot(np.transpose(P), R), Fext) + np.dot(I, Vd[:, i].reshape(6, 1)) + np.dot(np.dot(I, S) + np.dot(Wact, I), V[:, i].reshape(6, 1))
            F[:, i] = np.squeeze(np.asarray(f))
        else:
            f = np.dot(np.dot(np.transpose(P), R), F[:, i+1].reshape(6, 1)) + np.dot(I, Vd[:, i].reshape(6, 1)) + np.dot(np.dot(I, S) + np.dot(Wact, I), V[:, i].reshape(6, 1))
            F[:, i] = np.squeeze(np.asarray(f))
        tau[i, 0] = np.dot(H.reshape(1, 6), F[:, i].reshape(6, 1))

    return tau
