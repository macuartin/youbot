#!/usr/bin/env python

import numpy as np
import math


def rot2euler(R):

    euler = np.matrix([[0.0]*2]*3)

    if (math.fabs(R[2, 0]) != 1):
        euler[1, 0] = -math.asin(R[2, 0])
        euler[1, 1] = math.pi-euler[1, 0]

        euler[0, 0] = math.atan2(R[2, 1]/math.cos(euler[1, 0]), R[2, 2]/math.cos(euler[1, 0]))
        euler[0, 1] = math.atan2(R[2, 1]/math.cos(euler[1, 1]), R[2, 2]/math.cos(euler[1, 1]))

        euler[2, 0] = math.atan2(R[1, 0]/math.cos(euler[1, 0]), R[0, 0]/math.cos(euler[1, 0]))
        euler[2, 1] = math.atan2(R[1, 0]/math.cos(euler[1, 1]), R[0, 0]/math.cos(euler[1, 1]))
    else:
        euler[2, 0] = 0

        if (R[2, 0] == -1):
            euler[1, 0] = math.pi/2
            euler[0, 0] = euler[2, 0]+math.atan2(R[0, 1], R[0, 2])
        else:
            euler[1, 0] = -math.pi/2
            euler[0, 0] = -euler[2, 0]+math.atan2(-R[0, 1], -R[0, 2])

    return euler
