#! /usr/bin/env python
from __future__ import division
import rospy
import numpy as np


class TrajectoryController(object):

    def __init__(self, checkpoints, checkpoints_timing, initial_velocity,
                 final_velocity, sampling_time, generator):

        self.checkpoints = checkpoints
        self.checkpoints_timing = checkpoints_timing
        self.initial_velocity = initial_velocity
        self.final_velocity = final_velocity
        self.sampling_time = sampling_time
        self.generator = generator

    def get_position_trajectory(self):
        if self.generator == 'cubic_splines':
            return self.cubic_splines_interpolator('position')

    def get_velocity_trajectory(self):
        if self.generator == 'cubic_splines':
            return self.cubic_splines_interpolator('velocity')

    def get_acceleration_trajectory(self):
        if self.generator == 'cubic_splines':
            return self.cubic_splines_interpolator('acceleration')

    def get_orientation_trajectory(self):
        if self.generator == 'cubic_splines':
            return self.cubic_splines_interpolator('position')

    def __get_a_matrix(self, n, h):

        A = np.zeros(shape=(n, n))

        A[0, 0] = 2 * h[0]

        for i in range(0, n-1):
            A[i+1, i] = h[i]
            A[i, i+1] = h[i]

        for i in range(1, n-1):
            A[i, i] = 2 * (h[i-1] + h[i])

        A[n-1, n-1] = 2 * h[n-2]

        return A

    def __get_F_vector(self, n, h, component):

        F = np.zeros(shape=(n, 1))

        F[0] = (((3/h[0])*(self.checkpoints[1][component]-self.checkpoints[0][component]))
                - 3*self.initial_velocity[component])

        for i in range(1, n-1):
            F[i] = (((3/h[i])
                    * (self.checkpoints[i+1][component]-self.checkpoints[i][component]))
                    - ((3/h[i-1])
                    * (self.checkpoints[i][component]-self.checkpoints[i-1][component])))

        F[n-1] = (3*self.final_velocity[component]
                  - ((3/h[n-2])
                  * (self.checkpoints[n-1][component]-self.checkpoints[n-2][component])))

        return F

    def __get_b_coefficient(self, A, F):
        return np.dot(np.linalg.inv(A), F)

    def __get_a_coefficient(self, n, b, h, component):
        a = np.zeros(shape=(n, 1))
        for i in range(0, n-1):
            a[i] = (b[i+1] - b[i]) / (3*h[i])
        return a

    def __get_c_coefficient(self, n, b, h, component):
        c = np.zeros(shape=(n, 1))
        for i in range(0, n-1):
            c[i] = (((1/h[i])*(self.checkpoints[i+1][component]
                    - self.checkpoints[i][component]))
                    - ((h[i]/3)*((2*b[i]) + b[i+1])))
        return c

    def __get_d_coefficient(self, n, b, h, component):
        d = np.zeros(shape=(n, 1))
        for i in range(0, n-1):
            d[i] = self.checkpoints[i][component]
        return d

    def cubic_splines_interpolator(self, component):

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
        n = len(self.checkpoints_timing)

        # Vector de diferencias de tiempo.
        h = []
        for i in range(0, n-1):
            h.append(self.checkpoints_timing[i+1] - self.checkpoints_timing[i])

        # Matriz A
        A = self.__get_a_matrix(n, h)

        # Vector F
        Fx = self.__get_F_vector(n, h, x)
        Fy = self.__get_F_vector(n, h, y)
        Fz = self.__get_F_vector(n, h, z)

        # Calculo de Coeficientes
        bx = self.__get_b_coefficient(A, Fx)
        by = self.__get_b_coefficient(A, Fy)
        bz = self.__get_b_coefficient(A, Fz)

        ax = self.__get_a_coefficient(n, bx, h, x)
        ay = self.__get_a_coefficient(n, by, h, y)
        az = self.__get_a_coefficient(n, bz, h, z)

        cx = self.__get_c_coefficient(n, bx, h, x)
        cy = self.__get_c_coefficient(n, by, h, y)
        cz = self.__get_c_coefficient(n, bz, h, z)

        dx = self.__get_d_coefficient(n, bx, h, x)
        dy = self.__get_d_coefficient(n, by, h, y)
        dz = self.__get_d_coefficient(n, bz, h, z)

        # Calculo de Posicion, Velocidad y Aceleracion.
        # Tamano de los vectores
        tv = 0
        for i in range(0, n-1):
            tv = tv + len(np.arange(0, h[i] + self.sampling_time, self.sampling_time))

        # Creacion de Vectores
        T = np.zeros(tv)
        ori = np.zeros((tv, 3))
        traj = np.zeros((tv, 3))

        # Variable Auxiliar para acumular la cantidad de tiempos
        p = 0
        for i in range(0, n-1):
            # Tiempo a evaluar en los polinomios
            # (de 0 al delta de tiempo del tramo)
            Tk = np.arange(0, h[i] + self.sampling_time, self.sampling_time)
            # Momento en el cual esta presente el robot en cada posicion
            tc = (np.arange(self.checkpoints_timing[i],
                  self.checkpoints_timing[i+1] + self.sampling_time,
                  self.sampling_time))

            k = len(Tk)
            for j in range(0, k):

                if component == 'position':
                    traj[j+p, x] = ((ax[i] * (Tk[j]**3))
                                    + (bx[i] * (Tk[j]**2))
                                    + (cx[i]*Tk[j])
                                    + dx[i])

                    traj[j+p, y] = ((ay[i] * (Tk[j]**3))
                                    + (by[i] * (Tk[j]**2))
                                    + (cy[i]*Tk[j])
                                    + dy[i])

                    traj[j+p, z] = ((az[i] * (Tk[j]**3))
                                    + (bz[i] * (Tk[j]**2))
                                    + (cz[i]*Tk[j])
                                    + dz[i])
                elif component == 'velocity':
                    traj[j+p, x] = (3 * ax[i] * (Tk[j]**2)) + (2 * bx[i] * (Tk[j])) + cx[i]
                    traj[j+p, y] = (3 * ay[i] * (Tk[j]**2)) + (2 * by[i] * (Tk[j])) + cy[i]
                    traj[j+p, z] = (3 * az[i] * (Tk[j]**2)) + (2 * bz[i] * (Tk[j])) + cz[i]
                elif component == 'acceleration':
                    traj[j+p, x] = (6 * ax[i] * (Tk[j])) + (2 * bx[i])
                    traj[j+p, y] = (6 * ay[i] * (Tk[j])) + (2 * by[i])
                    traj[j+p, z] = (6 * az[i] * (Tk[j])) + (2 * bz[i])

                # Roll[j+p] = (Tk[j]*(self.checkpoints[i+1][3] - self.checkpoints[i][3])
                #              + self.checkpoints[i][3])

                # Pitch[j+p] = (Tk[j]*(self.checkpoints[i+1][4] - self.checkpoints[i][4])
                #               + self.checkpoints[i][4])

                # Yaw[j+p] = (Tk[j]*(self.checkpoints[i+1][5] - self.checkpoints[i][5])
                #             + self.checkpoints[i][5])

                T[j+p] = tc[j]
            p = p + k

        return traj
