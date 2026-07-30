#! /usr/bin/env python
from __future__ import division
import rospy
import numpy as np


class Robot(object):

    def __init__(self, dh, q0):
        self.dh = dh
        self.q0 = q0

    def inverse_kinematic(self, Xdif):
        J = jacobian()
        iJ = np.linalg.pinv(J)
        total = np.dot(iJ, Xdif) + Q0
        self.q0 = total
        return (total)

    def jacobian(self):
        '''
        Recibe la matriz DH y los valores articulares de un punto de trayectoria
        '''
        # Hay que realizar un ciclo en reversa para encontrar las derivadas parciales

        DoF = len(self.dh)

        J = np.matrix([[0] * DoF] * 6)

        for i in range(DoF-1, -1, -1):

            T = forward_kinematic(self.dh[i:DoF, :], self.q[i:DoF, 0])

            J[0, i] = -T[0, 0]*T[1, 3]+T[1, 0]*T[0, 3]
            J[1, i] = -T[0, 1]*T[1, 3]+T[1, 1]*T[0, 3]
            J[2, i] = -T[0, 2]*T[1, 3]+T[1, 2]*T[0, 3]
            J[3, i] = T[2, 0]
            J[4, i] = T[2, 1]
            J[5, i] = T[2, 2]

        return J

    def forward_kinematic(self, DH, qi):
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

    def homogeneus_matrix(self, DHi, qi):
        '''
        Recibe los parametros geometricos de una articulacion (DH)
        y los transforma en cordenadas homogeneas
        '''
        alpha = DHi[2]
        a = DHi[3]
        sigma = DHi[4]

        if(sigma == 0):
            d = DHi[1]
            theta = qi
            p = [a*np.cos(theta), a*np.sin(theta), d]
        elif(sigma == 1):
            d = qi
            theta = DHi[0]
            p = [0, 0, d]

        A = [[np.cos(theta), -np.cos(alpha)*np.sin(theta),  np.sin(alpha)*np.sin(theta), p[0]],
             [np.sin(theta),  np.cos(alpha)*np.cos(theta), -np.sin(alpha)*np.cos(theta), p[1]],
             [0, np.sin(alpha), np.cos(alpha), p[2]],
             [0, 0, 0, 1]]

        return np.matrix(A)
