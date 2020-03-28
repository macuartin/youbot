#!/usr/bin/env python

from __future__ import division

import numpy as np


def cubic_splines_planner(checkpoints, checkpoints_timing, initial_velocity, final_velocity, sampling_time):

    '''
        checkpoints: [[x0,y0,z0], [x1,y1,z1], ... , [xn,yn,zn]].

        checkpoints_timing: [t0, t1, ... , tn ].

        velocity_vector: [[Vx0, Vy0, Vz0], [Vxn, Vyn, Vzn]].

        sampling_time: time in seconds.
    '''

    #Tamano del sistema
    n = len(checkpoints_timing)

    #Vector de diferencias de tiempo.
    h = []
    for i in range(0,n-1):
        h.append(checkpoints_timing[i+1]-checkpoints_timing[i] )

    #Matriz A
    #Declaracion de Variable
    A = np.zeros(shape=(n,n))

    #Primer componente de diagonal principal
    A[0,0] = 2 * h[0]

    #Paralelas a diagonal principal
    for i in range(0,n-1):
        A[i+1,i] = h[i]
        A[i,i+1] = h[i]

    #Elementos de diagonal principal entre primer y ultimo elemento
    for i in range(1,n-1):
        A[i,i] = 2 * (h[i-1] + h[i])

    #Ultimo componente de la diagonal principal
    A[n-1,n-1] = 2 * h[n-2]


    #Vector F
    #Declaracion de Variable
    Fx = np.zeros(shape=(n,1))
    Fy = np.zeros(shape=(n,1))
    Fz = np.zeros(shape=(n,1))

    #Primer elemento
    Fx[0] = ((3/h[0])*(checkpoints[1][0]-checkpoints[0][0])) - 3*initial_velocity[0]
    Fy[0] = ((3/h[0])*(checkpoints[1][1]-checkpoints[0][1])) - 3*initial_velocity[1]
    Fz[0] = ((3/h[0])*(checkpoints[1][2]-checkpoints[0][2])) - 3*initial_velocity[2]

    #Elementos del medio
    for i in range(1,n-1):
        Fx[i]=( (3/h[i]) * (checkpoints[i+1][0]-checkpoints[i][0]) ) - ( (3/h[i-1]) * (checkpoints[i][0]-checkpoints[i-1][0]) )
        Fy[i]=( (3/h[i]) * (checkpoints[i+1][1]-checkpoints[i][1]) ) - ( (3/h[i-1]) * (checkpoints[i][1]-checkpoints[i-1][1]) )
        Fz[i]=( (3/h[i]) * (checkpoints[i+1][2]-checkpoints[i][2]) ) - ( (3/h[i-1]) * (checkpoints[i][2]-checkpoints[i-1][2]) )

    #Ultimo elemento
    Fx[n-1]= 3*final_velocity[0] - ((3/h[n-2]) * (checkpoints[n-1][0]-checkpoints[n-2][0]))
    Fy[n-1]= 3*final_velocity[1] - ((3/h[n-2]) * (checkpoints[n-1][1]-checkpoints[n-2][1]))
    Fz[n-1]= 3*final_velocity[2] - ((3/h[n-2]) * (checkpoints[n-1][2]-checkpoints[n-2][2]))


    #Resolinitial_velocityendo el Sistema b = A^-1 * f
    bx = np.dot(np.linalg.inv(A),Fx)
    by = np.dot(np.linalg.inv(A),Fy)
    bz = np.dot(np.linalg.inv(A),Fz)

    #Calculo de Coeficientes
    ax=np.zeros(shape=(n,1))
    ay=np.zeros(shape=(n,1))
    az=np.zeros(shape=(n,1))
    cx=np.zeros(shape=(n,1))
    cy=np.zeros(shape=(n,1))
    cz=np.zeros(shape=(n,1))
    dx=np.zeros(shape=(n,1))
    dy=np.zeros(shape=(n,1))
    dz=np.zeros(shape=(n,1))

    for i in range(0, n-1):
        #Coeficiente a
        ax[i] = (bx[i+1] - bx[i]) / (3*h[i])
        ay[i] = (by[i+1] - by[i]) / (3*h[i])
        az[i] = (bz[i+1] - bz[i]) / (3*h[i])
        
        #Coeficiente c
        cx[i] = ((1/h[i])*(checkpoints[i+1][0]-checkpoints[i][0])) - ((h[i]/3)*((2*bx[i])+ bx[i+1]))
        cy[i] = ((1/h[i])*(checkpoints[i+1][1]-checkpoints[i][1])) - ((h[i]/3)*((2*by[i])+ by[i+1]))
        cz[i] = ((1/h[i])*(checkpoints[i+1][2]-checkpoints[i][2])) - ((h[i]/3)*((2*bz[i])+ bz[i+1]))
        
        #Coeficiente d
        dx[i] = checkpoints[i][0]
        dy[i] = checkpoints[i][1]
        dz[i] = checkpoints[i][2]

    #Calculo de Posicion, Velocidad y Aceleracion.
    #Tamano de los vectores
    tv = 0 
    for i in range(0,n-1):
        tv = tv + len(np.arange(0,h[i]+sampling_time,sampling_time))

    #Creacion de Vectores     
    X=np.zeros(shape=(tv,1))
    Y=np.zeros(shape=(tv,1))
    Z=np.zeros(shape=(tv,1))
    Roll=np.zeros(shape=(tv,1))
    Pitch=np.zeros(shape=(tv,1))
    Yaw=np.zeros(shape=(tv,1))
    Vx=np.zeros(shape=(tv,1))
    Vy=np.zeros(shape=(tv,1))
    Vz=np.zeros(shape=(tv,1))
    Ax=np.zeros(shape=(tv,1))
    Ay=np.zeros(shape=(tv,1))
    Az=np.zeros(shape=(tv,1))
    T=np.zeros(shape=(tv,1))
    #Variable Auxiliar para acumular la cantidad de tiempos
    p = 0
    for i in range(0,n-1):
        Tk = np.arange(0,h[i]+sampling_time,sampling_time) #Tiempo a evaluar en los polinomios (de 0 al delta de tiempo del tramo)
        tc = np.arange(checkpoints_timing[i],checkpoints_timing[i+1]+sampling_time, sampling_time) #Momento en el cual esta presente el robot en cada posicion
        k = len(Tk)
        for j in range(0,k):
            X[j+p] = ((ax[i] * (Tk[j]**3)) + (bx[i] * (Tk[j]**2)) + (cx[i]*Tk[j]) + dx[i])
            Y[j+p] = (ay[i] * (Tk[j]**3)) + (by[i] * (Tk[j]**2)) + (cy[i]*Tk[j]) + dy[i]
            Z[j+p] = (az[i] * (Tk[j]**3)) + (bz[i] * (Tk[j]**2)) + (cz[i]*Tk[j]) + dz[i]
            
            Vx[j+p] = (3 * ax[i] * (Tk[j]**2)) + (2 * bx[i] * (Tk[j])) + cx[i]
            Vy[j+p] = (3 * ay[i] * (Tk[j]**2)) + (2 * by[i] * (Tk[j])) + cy[i]
            Vz[j+p] = (3 * az[i] * (Tk[j]**2)) + (2 * bz[i] * (Tk[j])) + cz[i]
            
            Ax[j+p] = (6 * ax[i] * (Tk[j])) + (2 * bx[i])
            Ay[j+p] = (6 * ay[i] * (Tk[j])) + (2 * by[i])
            Az[j+p] = (6 * az[i] * (Tk[j])) + (2 * bz[i])

            Roll[j+p] = Tk[j]*(checkpoints[i+1][3] - checkpoints[i][3]) + checkpoints[i][3]
            Pitch[j+p] = Tk[j]*(checkpoints[i+1][4] - checkpoints[i][4]) + checkpoints[i][4]
            Yaw[j+p] = Tk[j]*(checkpoints[i+1][5] - checkpoints[i][5]) + checkpoints[i][5]
            
            T[j+p] = tc[j]
        p = p + k
        
    return X,Y,Z,Vx,Vy,Vz,Roll,Pitch,Yaw
