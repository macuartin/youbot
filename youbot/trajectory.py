"""Planificacion de trayectorias cartesianas por splines cubicos.

De los cinco componentes que quedaron escritos en 2020, este es el unico que
estaba bien. legacy/youbot_control/src/youbot_control/trajectory.py resuelve a
mano el sistema tridiagonal del spline cubico natural con velocidades inicial y
final impuestas: monta la matriz A, el vector F, y de ahi saca los coeficientes
a, b, c, d de cada tramo. Son unas 130 lineas y la formulacion es correcta.

Es tambien, exactamente, lo que hace scipy.interpolate.CubicSpline con
bc_type=((1, v0), (1, vn)). tests/test_trajectory.py comprueba que las dos
implementaciones coinciden, o sea que aquellas 130 lineas eran correctas y
redundantes al mismo tiempo.
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline


def make_trajectory(
    waypoints: np.ndarray,
    times: np.ndarray,
    initial_velocity: np.ndarray | None = None,
    final_velocity: np.ndarray | None = None,
) -> CubicSpline:
    """Spline cubico que pasa por los waypoints en los tiempos dados.

    Args:
        waypoints: (m, k) con m puntos de k componentes (tipicamente x, y, z).
        times: (m,) instantes de paso, estrictamente crecientes.
        initial_velocity: (k,) velocidad impuesta en el primer punto. Cero si
            no se indica.
        final_velocity: (k,) velocidad impuesta en el ultimo punto. Cero si no
            se indica.

    Returns:
        Spline evaluable: spl(t) da posicion, spl(t, 1) velocidad y
        spl(t, 2) aceleracion.
    """
    waypoints = np.atleast_2d(np.asarray(waypoints, dtype=float))
    times = np.asarray(times, dtype=float).ravel()

    if waypoints.shape[0] != times.size:
        raise ValueError("waypoints y times deben tener el mismo numero de puntos")
    if np.any(np.diff(times) <= 0):
        raise ValueError("times debe ser estrictamente creciente")

    k = waypoints.shape[1]
    v0 = np.zeros(k) if initial_velocity is None else np.asarray(initial_velocity, float)
    vn = np.zeros(k) if final_velocity is None else np.asarray(final_velocity, float)

    return CubicSpline(times, waypoints, bc_type=((1, v0), (1, vn)))


def sample(spl: CubicSpline, dt: float) -> tuple[np.ndarray, ...]:
    """Muestrea el spline a paso fijo.

    Returns:
        (t, posicion, velocidad, aceleracion).
    """
    t = np.arange(spl.x[0], spl.x[-1] + dt, dt)
    t = t[t <= spl.x[-1] + 1e-12]
    return t, spl(t), spl(t, 1), spl(t, 2)
