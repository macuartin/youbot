#!/usr/bin/env python

from __future__ import division

import numpy as np


def traj_planner(arg1, arg2, arg3, arg4):

    '''
        arg1: Puntos de control [[x0,y0,z0], [x1,y1,z1], ... , [xn,yn,zn]]
        arg2: Vector de Tiempos [t0, t1, ... , tn ]
        arg3: Velocidad inicial y final [[Vx0, Vy0, Vz0], [Vxn, Vyn, Vzn]]
        arg4: timepo de muestreo en segundos
    '''

    #Datos de entrada
    pc = arg1
    vt = arg2
    vi = arg3[0]
    vf = arg3[1]
    tm = arg4

    #Tamano del sistema
    n = len(vt)

    #Vector de diferencias de tiempo.
    h = []
    for i in range(0,n-1):
        h.append(vt[i+1]-vt[i] )

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
    Fx[0] = ((3/h[0])*(pc[1][0]-pc[0][0])) - 3*vi[0]
    Fy[0] = ((3/h[0])*(pc[1][1]-pc[0][1])) - 3*vi[1]
    Fz[0] = ((3/h[0])*(pc[1][2]-pc[0][2])) - 3*vi[2]

    #Elementos del medio
    for i in range(1,n-1):
        Fx[i]=( (3/h[i]) * (pc[i+1][0]-pc[i][0]) ) - ( (3/h[i-1]) * (pc[i][0]-pc[i-1][0]) )
        Fy[i]=( (3/h[i]) * (pc[i+1][1]-pc[i][1]) ) - ( (3/h[i-1]) * (pc[i][1]-pc[i-1][1]) )
        Fz[i]=( (3/h[i]) * (pc[i+1][2]-pc[i][2]) ) - ( (3/h[i-1]) * (pc[i][2]-pc[i-1][2]) )

    #Ultimo elemento
    Fx[n-1]= 3*vf[0] - ((3/h[n-2]) * (pc[n-1][0]-pc[n-2][0]))
    Fy[n-1]= 3*vf[1] - ((3/h[n-2]) * (pc[n-1][1]-pc[n-2][1]))
    Fz[n-1]= 3*vf[2] - ((3/h[n-2]) * (pc[n-1][2]-pc[n-2][2]))


    #Resolviendo el Sistema b = A^-1 * f
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
        cx[i] = ((1/h[i])*(pc[i+1][0]-pc[i][0])) - ((h[i]/3)*((2*bx[i])+ bx[i+1]))
        cy[i] = ((1/h[i])*(pc[i+1][1]-pc[i][1])) - ((h[i]/3)*((2*by[i])+ by[i+1]))
        cz[i] = ((1/h[i])*(pc[i+1][2]-pc[i][2])) - ((h[i]/3)*((2*bz[i])+ bz[i+1]))
        
        #Coeficiente d
        dx[i] = pc[i][0]
        dy[i] = pc[i][1]
        dz[i] = pc[i][2]

    #Calculo de Posicion, Velocidad y Aceleracion.
    #Tamano de los vectores
    tv = 0 
    for i in range(0,n-1):
        tv = tv + len(np.arange(0,h[i]+tm,tm))

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
        Tk = np.arange(0,h[i]+tm,tm) #Tiempo a evaluar en los polinomios (de 0 al delta de tiempo del tramo)
        tc = np.arange(vt[i],vt[i+1]+tm, tm) #Momento en el cual esta presente el robot en cada posicion
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

            Roll[j+p] = Tk[j]*(pc[i+1][3] - pc[i][3]) + pc[i][3]
            Pitch[j+p] = Tk[j]*(pc[i+1][4] - pc[i][4]) + pc[i][4]
            Yaw[j+p] = Tk[j]*(pc[i+1][5] - pc[i][5]) + pc[i][5]

            
            
            T[j+p] = tc[j]
        p = p + k
    return X,Y,Z,Vx,Vy,Vz,Roll,Pitch,Yaw
