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

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .kinematics import forward_kinematics, ik_step_position
from .model import ARM_MOUNT, JOINT_LIMITS

#: Configuracion de partida del brazo para resolver un agarre: hombro y codo
#: flexionados hacia adelante, lejos de singularidades.
READY_POSE = np.array([0.0, 0.5, -1.0, 0.5, 0.0])

#: Tolerancia de posicion para considerar el agarre viable, en metros.
#:
#: La pinza del youBot tiene una carrera de unos 23 mm entre dedos. Con una
#: pieza de 15 mm quedan unos 4 mm de holgura por lado, asi que 5 mm de error de
#: posicion del efector final es el limite razonable para que el agarre cierre
#: sobre la pieza y no contra ella o al aire.
GRASP_TOLERANCE = 0.005

#: Tolerancia angular de la direccion de aproximacion de la pinza, en radianes.
#:
#: Agarrar no es alcanzar un punto. Una pinza de dos dedos tiene que llegar
#: alineada con la cara de la pieza: si entra torcida, un dedo choca antes de que
#: el otro cierre. Diez grados es el margen tipico para una pieza prismatica.
#:
#: Esta restriccion es la que hace que el error de estacionamiento importe, y no
#: es un artificio del experimento sino la geometria del brazo. En el youBot la
#: articulacion 1 es un giro y las 2, 3 y 4 son cabeceos en un plano, asi que el
#: eje de la herramienta solo puede apuntar dentro del plano vertical que define
#: la articulacion 1, el mismo plano donde tiene que estar el punto de agarre. Si
#: la plataforma llega girada, la direccion de aproximacion que exige la pieza se
#: sale de ese plano y el agarre deja de ser alcanzable.
APPROACH_TOLERANCE = np.deg2rad(10.0)


@dataclass
class GraspResult:
    """Resultado de un intento de agarre."""

    success: bool
    position_residual: float
    approach_error: float
    configuration: np.ndarray


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


def pick_point_in_arm_frame(
    base_pose: np.ndarray, pick_point: np.ndarray
) -> np.ndarray:
    """Pasa un punto del mundo al frame de la cadena DH del brazo.

    El brazo va montado sin rotacion respecto de la plataforma (el URDF declara
    rpy="0 0 0"), asi que basta deshacer la pose de la plataforma y restar el
    offset de montaje.
    """
    x, y, theta = np.asarray(base_pose, dtype=float).ravel()
    c, s = np.cos(theta), np.sin(theta)
    R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    p = np.asarray(pick_point, dtype=float).ravel() - np.array([x, y, 0.0])
    return R_wb.T @ p - ARM_MOUNT


def attempt_grasp(
    base_pose: np.ndarray,
    pick_point: np.ndarray,
    approach_world: np.ndarray | None = None,
    q0: np.ndarray | None = None,
    tolerance: float = GRASP_TOLERANCE,
    approach_tolerance: float = APPROACH_TOLERANCE,
    iterations: int = 400,
) -> GraspResult:
    """Intenta agarrar en el punto de recogida desde una pose de la plataforma.

    Es el eslabon que conecta los objetivos especificos 5 y 4, y con el la tesis
    entera de 2018: si el AGV se estaciona mal, el manipulador no puede hacer su
    tarea. Aqui eso deja de ser una afirmacion y pasa a ser una medida.

    La tarea son cinco restricciones para cinco grados de libertad: las tres de
    la posicion del efector final y las dos de la direccion de aproximacion de la
    pinza. El giro alrededor del propio eje de la herramienta queda libre, que es
    lo correcto para una pinza de dos dedos sobre una pieza prismatica.

    Args:
        approach_world: direccion en la que debe entrar la pinza, en coordenadas
            del mundo. Por defecto horizontal desde el robot hacia la estacion.
    """
    target = pick_point_in_arm_frame(base_pose, pick_point)

    approach = (
        np.array([-1.0, 0.0, 0.0])
        if approach_world is None
        else np.asarray(approach_world, dtype=float).ravel()
    )
    approach = approach / np.linalg.norm(approach)

    # La direccion de aproximacion, vista desde el frame del brazo. La plataforma
    # solo gira alrededor de z, asi que basta deshacer su guinada.
    theta = float(np.asarray(base_pose, dtype=float).ravel()[2])
    c, s = np.cos(theta), np.sin(theta)
    approach_arm = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]]) @ approach

    # Peso que pone el error angular en las mismas unidades que el de posicion:
    # una desalineacion igual a la tolerancia angular pesa lo mismo que un error
    # de posicion igual a la tolerancia de posicion.
    weight = tolerance / approach_tolerance

    def residuals(q: np.ndarray) -> np.ndarray:
        T = forward_kinematics(q)
        # El eje de la herramienta es la z del frame del efector final.
        angle = np.arccos(np.clip(np.dot(T[0:3, 2], approach_arm), -1.0, 1.0))
        axis = np.cross(T[0:3, 2], approach_arm)
        norm = np.linalg.norm(axis)
        if norm > 1e-12:
            axis = axis * (angle / norm)
        return np.concatenate([target - T[0:3, 3], weight * axis])

    starts = [READY_POSE] if q0 is None else [np.asarray(q0, dtype=float).ravel()]
    if q0 is None:
        # Varios arranques: el problema no es convexo y con un solo punto de
        # partida el solver se queda en minimos locales que no son limites
        # fisicos del brazo sino del metodo.
        starts += [
            np.array([0.0, 1.0, -1.5, 0.5, 0.0]),
            np.array([0.0, 0.2, -0.5, 1.0, 0.0]),
            np.array([0.0, 1.3, -2.0, 1.2, 0.0]),
        ]

    best: GraspResult | None = None
    for start in starts:
        solution = least_squares(
            residuals,
            np.clip(start, JOINT_LIMITS[:, 0] + 1e-6, JOINT_LIMITS[:, 1] - 1e-6),
            bounds=(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]),
            max_nfev=iterations,
        )

        T = forward_kinematics(solution.x)
        residual = float(np.linalg.norm(target - T[0:3, 3]))
        misalignment = float(
            np.arccos(np.clip(np.dot(T[0:3, 2], approach_arm), -1.0, 1.0))
        )
        candidate = GraspResult(
            success=residual < tolerance and misalignment < approach_tolerance,
            position_residual=residual,
            approach_error=misalignment,
            configuration=solution.x,
        )
        if best is None or _cost(candidate, weight) < _cost(best, weight):
            best = candidate
        if best.success:
            break

    return best


def _cost(result: GraspResult, weight: float) -> float:
    """Escalar para comparar soluciones de agarre entre distintos arranques."""
    return result.position_residual + weight * result.approach_error
