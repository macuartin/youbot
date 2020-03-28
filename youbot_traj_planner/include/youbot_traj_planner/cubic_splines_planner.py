#!/usr/bin/env python

from __future__ import division

import numpy as np


def get_a_matrix(n, h):

    # Declaracion de Variable
    A = np.zeros(shape=(n, n))

    # Primer componente de diagonal principal
    A[0, 0] = 2 * h[0]

    # Paralelas a diagonal principal
    for i in range(0, n-1):
        A[i+1, i] = h[i]
        A[i, i+1] = h[i]

    # Elementos de diagonal principal entre primer y ultimo elemento
    for i in range(1, n-1):
        A[i, i] = 2 * (h[i-1] + h[i])

    # Ultimo componente de la diagonal principal
    A[n-1, n-1] = 2 * h[n-2]


def get_F_vector(n, h, checkpoints,
                 initial_velocity, final_velocity, component):

    # Vector F
    # Declaracion de Variable
    F = np.zeros(shape=(n, 1))

    # Primer elemento
    F[0] = (((3/h[0])*(checkpoints[1][component]-checkpoints[0][component]))
            - 3*initial_velocity[component])

    # Elementos del medio
    for i in range(1, n-1):
        F[i] = (((3/h[i])
                * (checkpoints[i+1][component]-checkpoints[i][component]))
                - ((3/h[i-1])
                * (checkpoints[i][component]-checkpoints[i-1][component])))

    # Ultimo elemento
    F[n-1] = (3*final_velocity[component]
              - ((3/h[n-2])
              * (checkpoints[n-1][component]-checkpoints[n-2][component])))

    return F


def get_b_coefficient(A, F):
    # Resolviendo el Sistema b = A^-1 * f
    return np.dot(np.linalg.inv(A), F)


def get_a_coefficient(n, b, h, checkpoints, component):
    a = np.zeros(shape=(n, 1))
    for i in range(0, n-1):
        a[i] = (b[i+1] - b[i]) / (3*h[i])
    return a


def get_c_coefficient(n, b, h, checkpoints, component):
    c = np.zeros(shape=(n, 1))
    for i in range(0, n-1):
        c[i] = (((1/h[i])*(checkpoints[i+1][component]
                - checkpoints[i][component]))
                - ((h[i]/3)*((2*b[i]) + b[i+1])))
    return c


def get_d_coefficient(n, b, h, checkpoints, component):
    d = np.zeros(shape=(n, 1))
    for i in range(0, n-1):
        d[i] = checkpoints[i][component]
    return d


def cubic_splines_planner(checkpoints, checkpoints_timing, initial_velocity,
                          final_velocity, sampling_time):

    '''
        checkpoints: [[x0,y0,z0], [x1,y1,z1], ... , [xn,yn,zn]].

        checkpoints_timing: [t0, t1, ... , tn ].

        velocity_vector: [[Vx0, Vy0, Vz0], [Vxn, Vyn, Vzn]].

        sampling_time: time in seconds.
    '''

    # Components
    x = 0
    y = 1
    z = 2

    # Tamano del sistema
    n = len(checkpoints_timing)

    # Vector de diferencias de tiempo.
    h = []
    for i in range(0, n-1):
        h.append(checkpoints_timing[i+1]-checkpoints_timing[i])

    # Matriz A
    A = get_a_matrix(n, h)

    # Vector F
    Fx = get_F_vector(n, h, checkpoints, initial_velocity, final_velocity, x)
    Fy = get_F_vector(n, h, checkpoints, initial_velocity, final_velocity, y)
    Fz = get_F_vector(n, h, checkpoints, initial_velocity, final_velocity, z)

    # Calculo de Coeficientes
    bx = get_b_coefficient(A, Fx)
    by = get_b_coefficient(A, Fy)
    bz = get_b_coefficient(A, Fz)

    ax = get_a_coefficient(n, bx, h, checkpoints, x)
    ay = get_a_coefficient(n, by, h, checkpoints, y)
    az = get_a_coefficient(n, bz, h, checkpoints, z)

    cx = get_c_coefficient(n, bx, h, checkpoints, x)
    cy = get_c_coefficient(n, by, h, checkpoints, y)
    cz = get_c_coefficient(n, bz, h, checkpoints, z)

    dx = get_d_coefficient(n, bx, h, checkpoints, x)
    dy = get_d_coefficient(n, by, h, checkpoints, y)
    dz = get_d_coefficient(n, bz, h, checkpoints, z)

    # Calculo de Posicion, Velocidad y Aceleracion.
    # Tamano de los vectores
    tv = 0
    for i in range(0, n-1):
        tv = tv + len(np.arange(0, h[i]+sampling_time, sampling_time))

    # Creacion de Vectores
    X = np.zeros(shape=(tv, 1))
    Y = np.zeros(shape=(tv, 1))
    Z = np.zeros(shape=(tv, 1))
    Roll = np.zeros(shape=(tv, 1))
    Pitch = np.zeros(shape=(tv, 1))
    Yaw = np.zeros(shape=(tv, 1))
    Vx = np.zeros(shape=(tv, 1))
    Vy = np.zeros(shape=(tv, 1))
    Vz = np.zeros(shape=(tv, 1))
    Ax = np.zeros(shape=(tv, 1))
    Ay = np.zeros(shape=(tv, 1))
    Az = np.zeros(shape=(tv, 1))
    T = np.zeros(shape=(tv, 1))
    # Variable Auxiliar para acumular la cantidad de tiempos
    p = 0
    for i in range(0, n-1):
        # Tiempo a evaluar en los polinomios
        # (de 0 al delta de tiempo del tramo)
        Tk = np.arange(0, h[i]+sampling_time, sampling_time)
        # Momento en el cual esta presente el robot en cada posicion
        tc = (np.arange(checkpoints_timing[i],
              checkpoints_timing[i+1] + sampling_time,
              sampling_time))

        k = len(Tk)
        for j in range(0, k):
            X[j+p] = ((ax[i] * (Tk[j]**3))
                      + (bx[i] * (Tk[j]**2))
                      + (cx[i]*Tk[j])
                      + dx[i])

            Y[j+p] = ((ay[i] * (Tk[j]**3))
                      + (by[i] * (Tk[j]**2))
                      + (cy[i]*Tk[j])
                      + dy[i])

            Z[j+p] = ((az[i] * (Tk[j]**3))
                      + (bz[i] * (Tk[j]**2))
                      + (cz[i]*Tk[j])
                      + dz[i])

            Vx[j+p] = (3 * ax[i] * (Tk[j]**2)) + (2 * bx[i] * (Tk[j])) + cx[i]
            Vy[j+p] = (3 * ay[i] * (Tk[j]**2)) + (2 * by[i] * (Tk[j])) + cy[i]
            Vz[j+p] = (3 * az[i] * (Tk[j]**2)) + (2 * bz[i] * (Tk[j])) + cz[i]

            Ax[j+p] = (6 * ax[i] * (Tk[j])) + (2 * bx[i])
            Ay[j+p] = (6 * ay[i] * (Tk[j])) + (2 * by[i])
            Az[j+p] = (6 * az[i] * (Tk[j])) + (2 * bz[i])

            Roll[j+p] = (Tk[j]*(checkpoints[i+1][3] - checkpoints[i][3])
                         + checkpoints[i][3])

            Pitch[j+p] = (Tk[j]*(checkpoints[i+1][4] - checkpoints[i][4])
                          + checkpoints[i][4])

            Yaw[j+p] = (Tk[j]*(checkpoints[i+1][5] - checkpoints[i][5])
                        + checkpoints[i][5])

            T[j+p] = tc[j]
        p = p + k

    return X, Y, Z, Vx, Vy, Vz, Roll, Pitch, Yaw
