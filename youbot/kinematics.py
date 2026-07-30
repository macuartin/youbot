"""Cinematica directa, jacobiano y cinematica inversa diferencial del youBot.

Port a Python 3 de legacy/youbot_mechanics/src/youbot_mechanics/, con los
errores de implementacion corregidos. La formulacion es la misma de 2018.
"""

from __future__ import annotations

import numpy as np

from .model import DH, DOF


def link_transform(d: float, a: float, alpha: float, theta: float) -> np.ndarray:
    """Matriz de transformacion homogenea de un eslabon rotacional (DH clasica).

    Identica en forma a homogeneus_matrix() de 2020. Lo que cambia es como se
    la alimenta: aquel codigo leia alpha del indice 2 y a del indice 3 de la
    fila DH, mientras que la tabla que le pasaban en el bloque de prueba y en
    los YAML de ROS estaba en orden (theta, d, a, alpha). Con esa transposicion
    el eslabon 1 quedaba con alpha = 0.033 rad y a = 1.5708 m: un brazo de metro
    y medio. Aqui los parametros van como argumentos con nombre y el problema
    no puede reaparecer.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def link_transforms(q: np.ndarray) -> list[np.ndarray]:
    """Transformaciones eslabon a eslabon A_1 .. A_n para una configuracion."""
    q = np.asarray(q, dtype=float).ravel()
    return [link_transform(DH[i, 0], DH[i, 1], DH[i, 2], q[i]) for i in range(DOF)]


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """Pose del efector final respecto de la base del brazo, como matriz 4x4."""
    T = np.eye(4)
    for A in link_transforms(q):
        T = T @ A
    return T


def jacobian_ee(q: np.ndarray) -> np.ndarray:
    """Jacobiano expresado en el frame del efector final (6xN, orden [v; w]).

    Formulacion de Paul, la misma que uso el codigo de 2020: para la columna i
    se toma la transformacion del frame i-1 al efector final y se combinan sus
    columnas n, o, a con el vector de posicion p.

    El bug de 2020 estaba en el contenedor, no en la formula:

        J = np.matrix([[0] * DoF] * 6)

    produce dtype int64, asi que las seis asignaciones en coma flotante que
    venian despues se truncaban a cero. El jacobiano salia practicamente nulo,
    su pseudo-inversa era ruido, y con eso la cinematica inversa del objetivo
    especifico 3 nunca pudo converger. Sin ningun error en pantalla.
    """
    q = np.asarray(q, dtype=float).ravel()
    A = link_transforms(q)

    J = np.zeros((6, DOF))
    for i in range(DOF):
        # Transformacion del frame i-1 al efector final.
        T = np.eye(4)
        for k in range(i, DOF):
            T = T @ A[k]

        p = T[0:3, 3]
        J[0:3, i] = -T[0, 0:3] * p[1] + T[1, 0:3] * p[0]
        J[3:6, i] = T[2, 0:3]
    return J


def jacobian_base(q: np.ndarray) -> np.ndarray:
    """Jacobiano expresado en el frame base (6xN, orden [v; w]).

    Necesario para cerrar el lazo contra una trayectoria cartesiana definida en
    coordenadas de la base, que es lo que produce el planificador. El codigo de
    2020 mezclaba los dos frames: alimentaba el jacobiano en frame del efector
    con incrementos cartesianos expresados en la base.
    """
    q = np.asarray(q, dtype=float).ravel()
    A = link_transforms(q)

    # Poses acumuladas: T[i] es el frame i respecto de la base.
    T = np.eye(4)
    origins = [T[0:3, 3].copy()]
    axes = [T[0:3, 2].copy()]
    for i in range(DOF):
        T = T @ A[i]
        origins.append(T[0:3, 3].copy())
        axes.append(T[0:3, 2].copy())

    p_ee = origins[-1]
    J = np.zeros((6, DOF))
    for i in range(DOF):
        z = axes[i]
        J[0:3, i] = np.cross(z, p_ee - origins[i])
        J[3:6, i] = z
    return J


def ik_step(
    q: np.ndarray, twist: np.ndarray, damping: float = 1e-3
) -> np.ndarray:
    """Un paso de cinematica inversa diferencial en el frame base.

    Resuelve dq a partir del twist deseado con minimos cuadrados amortiguados
    (Levenberg-Marquardt), no con la pseudo-inversa cruda de 2020. El brazo
    tiene 5 grados de libertad para una tarea de 6, asi que el sistema es
    sobredeterminado y el jacobiano pierde rango en configuraciones singulares.
    La pseudo-inversa sin amortiguar explota ahi; el amortiguamiento acota dq a
    cambio de un error de seguimiento pequeno.
    """
    J = jacobian_base(q)
    twist = np.asarray(twist, dtype=float).ravel()
    JT = J.T
    dq = JT @ np.linalg.solve(J @ JT + damping**2 * np.eye(6), twist)
    return np.asarray(q, dtype=float).ravel() + dq


def ik_step_position(
    q: np.ndarray, error: np.ndarray, damping: float = 1e-3
) -> np.ndarray:
    """Un paso de cinematica inversa sobre una tarea de solo posicion.

    Con 5 grados de libertad no se puede imponer posicion y orientacion a la
    vez: son 6 restricciones para 5 incognitas. Para pick & place lo que hay que
    garantizar es la posicion del efector, dejando la orientacion como grado de
    libertad del que dispone el solver. Se usan por tanto las tres primeras
    filas del jacobiano.
    """
    J = jacobian_base(q)[0:3]
    error = np.asarray(error, dtype=float).ravel()
    JT = J.T
    dq = JT @ np.linalg.solve(J @ JT + damping**2 * np.eye(3), error)
    return np.asarray(q, dtype=float).ravel() + dq


def rotation_to_rpy(R: np.ndarray) -> np.ndarray:
    """Angulos roll-pitch-yaw (convencion XYZ fija) de una matriz de rotacion.

    Reemplaza a rot2euler.py, que devolvia las dos soluciones en una matriz 3x2
    y dejaba al llamante elegir. Aqui se devuelve la rama con pitch en
    [-pi/2, pi/2], y en la singularidad se fija roll = 0.
    """
    sy = -R[2, 0]
    sy = np.clip(sy, -1.0, 1.0)
    pitch = np.arcsin(sy)
    if np.abs(np.abs(sy) - 1.0) < 1e-9:
        return np.array([0.0, pitch, np.arctan2(-R[0, 1], R[1, 1])])
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])
