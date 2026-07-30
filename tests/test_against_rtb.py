"""Validacion del modelo contra roboticstoolbox-python como oraculo independiente.

La pregunta que responde este archivo es la que quedo abierta en 2018: la
matematica del Informe de Avance #1 era correcta o no. Se construye el mismo
robot en roboticstoolbox (Peter Corke) a partir de la misma tabla DH y se
comparan cinematica directa, jacobianos, torques de Newton-Euler, matriz de
masa y dinamica directa sobre configuraciones aleatorias.

Si estos tests pasan, el modelo de 2018 era correcto y lo que fallo fue la
implementacion de 2020.
"""

from __future__ import annotations

import numpy as np
import pytest

import youbot
from youbot import dynamics, model

rtb = pytest.importorskip("roboticstoolbox")

SEED = 20181207  # fecha de presentacion del anteproyecto
N_TRIALS = 25
TOL = 1e-9


@pytest.fixture(scope="module")
def oracle():
    """El mismo youBot construido en roboticstoolbox desde la tabla DH."""
    links = []
    for i in range(model.DOF):
        d, a, alpha = model.DH[i]
        Ixx, Iyy, Izz = model.INERTIA_DIAG[i]
        links.append(
            rtb.RevoluteDH(
                d=d,
                a=a,
                alpha=alpha,
                m=model.MASS[i],
                r=model.COM[i],
                I=[Ixx, Iyy, Izz, 0.0, 0.0, 0.0],
                G=1.0,
                Jm=0.0,
                B=0.0,
                Tc=[0.0, 0.0],
            )
        )
    robot = rtb.DHRobot(links, name="youBot")
    # rtb usa vd_base = -robot.gravity, la misma convencion que este modelo.
    robot.gravity = model.GRAVITY
    return robot


@pytest.fixture(scope="module")
def configs():
    rng = np.random.default_rng(SEED)
    return [model.random_configuration(rng) for _ in range(N_TRIALS)]


def test_forward_kinematics(oracle, configs):
    for q in configs:
        assert np.allclose(youbot.forward_kinematics(q), oracle.fkine(q).A, atol=TOL)


def test_jacobian_base(oracle, configs):
    for q in configs:
        assert np.allclose(youbot.jacobian_base(q), oracle.jacob0(q), atol=TOL)


def test_jacobian_ee(oracle, configs):
    """El jacobiano en frame del efector, la formulacion de Paul que uso 2020."""
    for q in configs:
        assert np.allclose(youbot.jacobian_ee(q), oracle.jacobe(q), atol=TOL)


def test_gravity_torques(oracle, configs):
    for q in configs:
        assert np.allclose(youbot.gravity_torques(q), oracle.gravload(q), atol=TOL)


def test_mass_matrix(oracle, configs):
    for q in configs:
        M = youbot.mass_matrix(q)
        assert np.allclose(M, oracle.inertia(q), atol=TOL)
        assert np.allclose(M, M.T, atol=TOL), "la matriz de masa debe ser simetrica"
        assert np.all(np.linalg.eigvalsh(M) > 0), "debe ser definida positiva"


def test_inverse_dynamics(oracle, configs):
    """Newton-Euler completo, con velocidad y aceleracion no nulas."""
    rng = np.random.default_rng(SEED + 1)
    for q in configs:
        qd = rng.uniform(-1.0, 1.0, model.DOF)
        qdd = rng.uniform(-2.0, 2.0, model.DOF)
        assert np.allclose(
            youbot.inverse_dynamics(q, qd, qdd), oracle.rne(q, qd, qdd), atol=TOL
        )
