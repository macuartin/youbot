import numpy as np


def skew(v):
    M = np.matrix([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]])
    return M


def trasnlation(p):
    P = np.zeros((6, 6))
    P[0:3, 0:3] = np.identity(3)
    P[3:6, 3:6] = np.identity(3)
    P[0:3, 3:6] = skew(p)
    return P


def rotation(r):
    R = np.zeros((6, 6))
    R[0:3, 0:3] = r
    R[3:6, 3:6] = r
    return R


def Wskew(v):
    W = np.zeros((6, 6))
    W[0:3, 0:3] = skew(v)
    W[3:6, 3:6] = skew(v)
    return W


def spatialInertia(Params):
    J = np.zeros((3, 3))
    J[0, 0] = Params[0, 4]
    J[1, 1] = Params[0, 5]
    J[2, 2] = Params[0, 6]

    m = Params[0, 0]

    I = np.zeros((6, 6))
    I[0:3, 0:3] = J
    I[3:6, 3:6] = np.dot(m, np.identity(3))
    return I


def centerOfMassDistance(Params):
    s = np.array([Params[0, 1], Params[0, 2], Params[0, 3]])
    S = np.zeros((6, 6))
    S[0:3, 0:3] = np.identity(3)
    S[3:6, 3:6] = np.identity(3)
    S[0:3, 3:6] = skew(s)
    return S
