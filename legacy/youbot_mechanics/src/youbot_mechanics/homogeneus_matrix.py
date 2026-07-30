from __future__ import division
import numpy as np


def homogeneus_matrix(DHi, qi):
    '''
    Recibe una fila de la matriz DH (link) y el valor de una articulacion en un punto de trayectoria.
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


if __name__ == '__main__':
    DH = [[0, 0.147, 0.033, np.pi/2, 0],
          [0, 0, 0.155, 0, 0],
          [0, 0, 0.135, 0, 0],
          [0, 0, 0, np.pi/2, 0],
          [0, 0.113, 0, 0, 0]]

    for link in DH:
        print(homogeneus_matrix(link, 0))
