#!/usr/bin/env python
from __future__ import division

import numpy as np
from jacobian import jacobian


def inverse_kinematic(DH, Xdif, Q0):
    J = jacobian(DH, Q0)
    iJ = np.linalg.pinv(J)
    total = np.dot(iJ, Xdif) + Q0
    return (total)
