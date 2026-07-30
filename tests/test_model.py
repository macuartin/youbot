"""Comprobaciones autocontenidas del modelo, sin oraculo externo.

Corren aunque roboticstoolbox no este instalado.
"""

from __future__ import annotations

import numpy as np
import pytest

import youbot
from youbot import model

SEED = 20181207  # fecha de presentacion del anteproyecto
N_TRIALS = 25


@pytest.fixture(scope="module")
def configs():
    rng = np.random.default_rng(SEED)
    return [model.random_configuration(rng) for _ in range(N_TRIALS)]


def test_forward_dynamics_roundtrip(configs):
    """La dinamica directa debe deshacer a la inversa.

    Es el test que en 2020 no podia ni ejecutarse: la matriz de masa salia como
    vector (5,1) y el np.linalg.inv() fallaba.
    """
    rng = np.random.default_rng(SEED + 2)
    for q in configs:
        qd = rng.uniform(-1.0, 1.0, model.DOF)
        qdd = rng.uniform(-2.0, 2.0, model.DOF)
        tau = youbot.inverse_dynamics(q, qd, qdd)
        assert np.allclose(youbot.forward_dynamics(q, qd, tau), qdd, atol=1e-8)


def test_ik_step_converges():
    """La cinematica inversa diferencial cierra sobre un objetivo cercano.

    En 2020 esto no podia converger: el jacobiano estaba en dtype int64 y
    devolvia ceros.
    """
    rng = np.random.default_rng(SEED + 3)
    q = model.random_configuration(rng)
    T_start = youbot.forward_kinematics(q)

    target = T_start[0:3, 3] + np.array([0.02, -0.015, 0.01])

    for _ in range(200):
        p = youbot.forward_kinematics(q)[0:3, 3]
        err = target - p
        if np.linalg.norm(err) < 1e-6:
            break
        twist = np.concatenate([err, np.zeros(3)])
        q = youbot.ik_step(q, twist * 0.5)

    p_final = youbot.forward_kinematics(q)[0:3, 3]
    assert np.linalg.norm(target - p_final) < 1e-5


def test_legacy_jacobian_was_truncated_to_zero():
    """Reproduce el bug de 2020, para que quede en el registro y no se repita.

    np.matrix de enteros de Python da dtype int64 y trunca toda asignacion en
    coma flotante. Es la razon por la que el objetivo especifico 3 nunca
    convergio.
    """
    J_legacy = np.matrix([[0] * model.DOF] * 6)
    assert J_legacy.dtype == np.int64

    q = np.zeros(model.DOF)
    J_ok = youbot.jacobian_ee(q)
    for i in range(6):
        for j in range(model.DOF):
            J_legacy[i, j] = J_ok[i, j]

    assert np.count_nonzero(J_legacy) < np.count_nonzero(np.round(J_ok, 12))


def test_follows_a_cartesian_trajectory():
    """Objetivo especifico 3: seguir una trayectoria cartesiana en articular.

    Se genera un spline entre tres puntos alcanzables, se resuelve la
    trayectoria articular y se comprueba que el efector final la sigue por
    debajo de un milimetro de error.
    """
    q0 = np.array([0.0, 0.3, -0.6, 0.4, 0.0])
    start = youbot.forward_kinematics(q0)[0:3, 3]

    waypoints = np.array(
        [start, start + [0.05, 0.03, -0.04], start + [0.0, 0.08, 0.02]]
    )
    spl = youbot.make_trajectory(waypoints, [0.0, 2.0, 4.0])
    _, positions, _, _ = youbot.sample(spl, 0.05)

    q_traj, error = youbot.follow_cartesian_path(positions, q0)

    assert q_traj.shape == (positions.shape[0], model.DOF)
    assert error.max() < 1e-3, f"error maximo de seguimiento {error.max():.6f} m"

    # Y la trayectoria articular respeta los limites mecanicos del robot.
    assert np.all(q_traj >= model.JOINT_LIMITS[:, 0] - 1e-9)
    assert np.all(q_traj <= model.JOINT_LIMITS[:, 1] + 1e-9)
