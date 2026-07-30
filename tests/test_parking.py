"""Validacion del estacionamiento por servo visual y del agarre que depende de el.

Objetivos especificos 1, 4 y 5 del trabajo de 2018. Los criterios numericos son
los que fijo el propio anteproyecto: error espacial de estacionamiento acotado a
+-10 cm y 95 por ciento de exito en el agarre.
"""

from __future__ import annotations

import numpy as np
import pytest

from youbot import attempt_grasp, parking
from youbot.control import APPROACH_TOLERANCE, pick_point_in_arm_frame

SEED = 20181207

#: Ruido de deteccion de esquinas medido sobre el detector real de ArUco
#: (ver experiments/measure_detector_noise.py): media 0.42 px, p95 0.74 px.
PIXEL_NOISE = 0.5

#: Error relativo de ejecucion de las velocidades comandadas.
ACTUATION_NOISE = 0.02

#: Criterios del anteproyecto de 2018.
PARKING_TOLERANCE = 0.10
GRASP_SUCCESS_TARGET = 0.95


@pytest.fixture(scope="module")
def scene():
    return parking.Scene.default()


def test_converges_without_noise(scene):
    """Sin ruido el estacionamiento cierra por debajo del milimetro."""
    starts = [
        (0.75, 0.10, np.pi - 0.15),
        (0.90, -0.20, np.pi + 0.25),
        (0.60, 0.05, np.pi + 0.05),
        (0.55, 0.0, np.pi),
    ]
    for start in starts:
        result = parking.park(np.array(start), scene)
        assert result.converged, f"no convergio desde {start}"
        assert result.position_error < 1e-3, (
            f"desde {start}: error {result.position_error * 1000:.3f} mm"
        )


def test_meets_the_2018_parking_criterion(scene):
    """Con ruido medido, el error espacial se mantiene dentro de +-10 cm."""
    rng = np.random.default_rng(SEED)
    errors = []

    for _ in range(40):
        start = np.array([0.55, 0.0, np.pi]) + rng.uniform(
            [-0.10, -0.12, -0.10], [0.25, 0.12, 0.10]
        )
        result = parking.park(
            start,
            scene,
            pixel_noise=PIXEL_NOISE,
            actuation_noise=ACTUATION_NOISE,
            rng=rng,
        )
        if result.lost_marker:
            continue
        errors.append(result.position_error)

    errors = np.array(errors)
    assert errors.size >= 30, "demasiados ensayos perdieron el marcador"
    assert errors.max() < PARKING_TOLERANCE, (
        f"error maximo {errors.max() * 100:.2f} cm, criterio {PARKING_TOLERANCE * 100:.0f} cm"
    )


def test_grasp_succeeds_from_the_nominal_pose(scene):
    """Desde la pose de estacionamiento ideal el agarre es exacto."""
    result = attempt_grasp(scene.desired_base_pose, scene.pick_point)
    assert result.success, (
        f"residuo {result.position_residual * 1000:.3f} mm, "
        f"desalineacion {np.rad2deg(result.approach_error):.2f} grados"
    )


def test_grasp_degrades_with_parking_error(scene):
    """Un error de estacionamiento lateral hace inviable el agarre.

    Es la premisa del anteproyecto de 2018 convertida en comprobacion. Notese que
    lo que falla no es alcanzar el punto, que el brazo alcanza de sobra girando la
    articulacion 1, sino llegar con la pinza alineada.
    """
    bad = np.array(scene.desired_base_pose) + np.array([0.0, 0.30, 0.0])
    result = attempt_grasp(bad, scene.pick_point)

    assert not result.success
    assert result.position_residual < 0.005, (
        "el punto si es alcanzable: el brazo tiene la articulacion 1 libre"
    )
    assert result.approach_error > APPROACH_TOLERANCE, (
        "lo que falla es la alineacion de la pinza, no el alcance"
    )


def test_grasp_chain_after_visual_servoing(scene):
    """El encadenamiento completo: estacionar con ruido y luego agarrar."""
    rng = np.random.default_rng(SEED + 7)
    successes = 0
    trials = 0

    for _ in range(40):
        start = np.array([0.55, 0.0, np.pi]) + rng.uniform(
            [-0.10, -0.12, -0.10], [0.25, 0.12, 0.10]
        )
        result = parking.park(
            start,
            scene,
            pixel_noise=PIXEL_NOISE,
            actuation_noise=ACTUATION_NOISE,
            rng=rng,
        )
        if result.lost_marker:
            continue
        trials += 1
        successes += int(attempt_grasp(result.final_pose, scene.pick_point).success)

    assert trials >= 30
    rate = successes / trials
    assert rate >= GRASP_SUCCESS_TARGET, (
        f"exito de agarre {rate:.1%}, criterio {GRASP_SUCCESS_TARGET:.0%}"
    )


def test_misalignment_follows_the_analytic_law(scene):
    """La desalineacion de la pinza obedece a arctan(error_lateral / alcance).

    Es el oraculo analitico de esta fase, y explica el mecanismo: un error
    lateral obliga a la articulacion 1 a girar para alcanzar la pieza, y ese giro
    saca del plano del brazo la direccion de aproximacion que la pieza exige.

    De aqui sale el error de estacionamiento lateral maximo tolerable:
    alcance * tan(tolerancia_angular).
    """
    reach = pick_point_in_arm_frame(scene.desired_base_pose, scene.pick_point)[0]

    for lateral in (0.02, 0.04, 0.08, 0.15, 0.20):
        result = attempt_grasp(
            np.array(scene.desired_base_pose) + [0.0, lateral, 0.0], scene.pick_point
        )
        predicted = np.arctan2(lateral, reach)
        assert result.approach_error == pytest.approx(predicted, abs=np.deg2rad(0.5)), (
            f"lateral {lateral * 100:.1f} cm: medido "
            f"{np.rad2deg(result.approach_error):.3f}, predicho {np.rad2deg(predicted):.3f}"
        )

    tolerable = reach * np.tan(APPROACH_TOLERANCE)
    assert attempt_grasp(
        np.array(scene.desired_base_pose) + [0.0, tolerable * 0.9, 0.0],
        scene.pick_point,
    ).success
    assert not attempt_grasp(
        np.array(scene.desired_base_pose) + [0.0, tolerable * 1.1, 0.0],
        scene.pick_point,
    ).success
