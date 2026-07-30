"""El generador de trayectorias de 2020 contra el de scipy.

legacy/youbot_control/src/youbot_control/trajectory.py resuelve a mano el spline
cubico con velocidades de borde impuestas. Este test comprueba que coincide con
scipy.interpolate.CubicSpline, o sea que ese componente si estaba correcto.

Para importarlo hay que stubear rospy: el modulo lo importa aunque no lo use
para nada, herencia de haber vivido dentro de un paquete ROS.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import numpy as np
import pytest

from youbot import make_trajectory, sample

LEGACY = Path(__file__).resolve().parents[1] / "legacy/youbot_control/src"

WAYPOINTS = np.array(
    [
        [0.0, 0.0, 0.0],
        [5.0, 7.0, 1.0],
        [3.0, 2.0, 4.0],
    ]
)
TIMES = np.array([0.0, 4.0, 7.0])
V0 = np.array([2.0, 3.0, 6.0])
VN = np.array([4.0, 2.0, 1.0])


@pytest.fixture(scope="module")
def legacy_controller():
    """El TrajectoryController de 2020, con rospy stubeado."""
    sys.modules.setdefault("rospy", types.ModuleType("rospy"))
    sys.path.insert(0, str(LEGACY))
    try:
        from youbot_control.trajectory import TrajectoryController
    except ImportError as err:  # pragma: no cover
        pytest.skip(f"no se pudo importar el codigo legacy: {err}")
    finally:
        sys.path.remove(str(LEGACY))
    return TrajectoryController


def test_interpolation_properties():
    spl = make_trajectory(WAYPOINTS, TIMES, V0, VN)

    assert np.allclose(spl(TIMES), WAYPOINTS, atol=1e-12)
    assert np.allclose(spl(TIMES[0], 1), V0, atol=1e-12)
    assert np.allclose(spl(TIMES[-1], 1), VN, atol=1e-12)

    # Continuidad C2 en el nudo interior.
    t = TIMES[1]
    eps = 1e-7
    for order in (1, 2):
        assert np.allclose(spl(t - eps, order), spl(t + eps, order), atol=1e-5)


def test_sampling_covers_the_interval():
    spl = make_trajectory(WAYPOINTS, TIMES, V0, VN)
    t, pos, vel, acc = sample(spl, 0.05)

    assert t[0] == pytest.approx(TIMES[0])
    assert t[-1] <= TIMES[-1] + 1e-9
    assert pos.shape == (t.size, 3)
    assert vel.shape == acc.shape == pos.shape


def test_matches_legacy_implementation(legacy_controller):
    """Los coeficientes del spline de 2020 coinciden con los de scipy.

    Se comparan coeficientes y no muestras porque el bucle de muestreo de 2020
    no corre en numpy 2: hace `traj[j+p, x] = ax[i] * ...` donde `ax[i]` es un
    array de forma (1,), y asignar una secuencia a un escalar dejo de estar
    permitido. Es un detalle de la epoca, no un error de formulacion.

    El solver tridiagonal, que es la parte que importa, se ejecuta entero.
    """
    controller = legacy_controller(
        WAYPOINTS.tolist(),
        TIMES.tolist(),
        V0.tolist(),
        VN.tolist(),
        0.05,
        "cubic_splines",
    )

    n = len(TIMES)
    h = [TIMES[i + 1] - TIMES[i] for i in range(n - 1)]
    p = "_TrajectoryController__"
    A = getattr(controller, p + "get_a_matrix")(n, h)

    spl = make_trajectory(WAYPOINTS, TIMES, V0, VN)

    for component in range(WAYPOINTS.shape[1]):
        F = getattr(controller, p + "get_F_vector")(n, h, component)
        b = getattr(controller, p + "get_b_coefficient")(A, F)
        # b sale del sistema tridiagonal; a, c y d se derivan de b.
        coeffs = [
            getattr(controller, p + "get_a_coefficient")(n, b, h, component),
            b,
            getattr(controller, p + "get_c_coefficient")(n, b, h, component),
            getattr(controller, p + "get_d_coefficient")(n, b, h, component),
        ]

        for order in range(4):
            legacy = np.asarray(coeffs[order], dtype=float).ravel()[: n - 1]
            scipy_c = spl.c[order, :, component]
            assert np.allclose(legacy, scipy_c, atol=1e-9), (
                f"componente {component}, orden {order}: "
                f"legacy {legacy} vs scipy {scipy_c}"
            )
