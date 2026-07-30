from __future__ import division

import numpy as np
import matplotlib.pyplot as plt

from homogeneus_matrix import homogeneus_matrix


def forward_kinematic(DH, qi):
    '''
    Recibe la matriz DH y los valores articulares de un punto de trayectoria
    '''

    T = np.identity(4)

    n = len(DH)

    for i in range(n):
        l = np.squeeze(np.asarray(DH[i, :]))
        A = homogeneus_matrix(l, qi[i, 0])
        T = np.dot(T, A)

    return T


def forward_inverse_comparator(DH, Q, X, Y, Z):

    pt = len(X)

    Xn = np.zeros(shape=X.shape)
    Yn = np.zeros(shape=Y.shape)
    Zn = np.zeros(shape=Z.shape)

    for i in range(pt):
        T = forward_kinematic(DH, Q[:, i])
        Xn[i] = T[0, 3]
        Yn[i] = T[1, 3]
        Zn[i] = T[2, 3]

    fig1 = plt.figure()
    plt.subplot(3, 1, 1)
    l1 = plt.plot(X, color="red")
    l2 = plt.plot(Xn, color="blue")
    plt.ylabel('X (m)')
    plt.xlabel('Puntos de Trayectoria')

    plt.subplot(3, 1, 2)
    plt.plot(Y, color="red")
    plt.plot(Yn, color="blue")
    plt.ylabel('Y (m)')
    plt.xlabel('Puntos de Trayectoria')

    plt.subplot(3, 1, 3)
    plt.plot(Z, color="red")
    plt.plot(Zn, color="blue")
    plt.ylabel('Z (m)')
    plt.xlabel('Puntos de Trayectoria')

    plt.show()
