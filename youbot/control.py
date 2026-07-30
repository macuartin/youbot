"""Control de posicion articular a partir de una trayectoria cartesiana.

Es el objetivo especifico 3 del trabajo de 2018: "implementar un control de
posicion articular para el brazo manipulador por medio de la cinematica inversa
metodo Jacobiano, para labores de Pick & Place, basado en coordenadas
cartesianas".

El lazo que en 2020 nunca se cerro. El planificador producia la trayectoria
cartesiana y la cinematica inversa estaba escrita, pero el servicio ROS que
tenia que unirlos se quedo en `return True`, y de haberse conectado habria
fallado igual: el jacobiano devolvia ceros y ademas se mezclaban dos frames.
"""

from __future__ import annotations

import numpy as np

from .kinematics import forward_kinematics, ik_step_position
from .model import JOINT_LIMITS


def follow_cartesian_path(
    positions: np.ndarray,
    q0: np.ndarray,
    gain: float = 1.0,
    damping: float = 1e-3,
    iterations: int = 8,
    clamp: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """Resuelve la trayectoria articular que sigue una secuencia de posiciones.

    Args:
        positions: (m, 3) posiciones cartesianas del efector final, en el frame
            base del brazo.
        q0: (n,) configuracion articular de partida.
        gain: ganancia proporcional sobre el error de posicion.
        damping: amortiguamiento de la pseudo-inversa.
        iterations: iteraciones de cinematica inversa por punto de trayectoria.
        clamp: si acotar cada articulacion a sus limites mecanicos.

    Returns:
        (q_trajectory, tracking_error) con formas (m, n) y (m,). El error es la
        norma de la diferencia entre la posicion pedida y la alcanzada.
    """
    positions = np.atleast_2d(np.asarray(positions, dtype=float))
    q = np.asarray(q0, dtype=float).ravel().copy()

    q_traj = np.zeros((positions.shape[0], q.size))
    error = np.zeros(positions.shape[0])

    for k, target in enumerate(positions):
        for _ in range(iterations):
            reached = forward_kinematics(q)[0:3, 3]
            q = ik_step_position(q, gain * (target - reached), damping)
            if clamp:
                q = np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

        q_traj[k] = q
        error[k] = np.linalg.norm(target - forward_kinematics(q)[0:3, 3])

    return q_traj, error
