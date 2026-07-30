"""Validacion del modelo de camara y de la matriz de interaccion.

En la Fase 1 el oraculo era roboticstoolbox. Aqui es la diferencia finita: si la
matriz de interaccion es correcta, predice el cambio de las caracteristicas de
imagen ante un desplazamiento infinitesimal de la camara. Y si el modelo de
proyeccion es correcto, el detector de OpenCV recupera de la imagen sintetizada
las mismas esquinas que la proyeccion predijo.
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.linalg import expm

from youbot import parking
from youbot.vision import (
    Camera,
    detect_corners,
    interaction_matrix,
    render_marker_view,
    world_to_camera,
)

SEED = 20181207


def _se3(twist: np.ndarray) -> np.ndarray:
    """Matriz 4x4 del algebra se(3) para un twist en orden [v; w]."""
    v, w = twist[0:3], twist[3:6]
    M = np.zeros((4, 4))
    M[0:3, 0:3] = [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]
    M[0:3, 3] = v
    return M


@pytest.fixture(scope="module")
def setup():
    scene = parking.Scene.default()
    camera = Camera()
    pose_c = parking.camera_pose_matrix(scene.desired_base_pose)
    return scene, camera, pose_c


def test_projection_is_inside_the_sensor(setup):
    """Desde la pose de estacionamiento el marcador se ve entero y centrado."""
    scene, camera, pose_c = setup
    pixels = camera.project(world_to_camera(pose_c, scene.corners_world()))

    assert camera.in_view(pixels)
    center = pixels.mean(axis=0)
    assert center[0] == pytest.approx(camera.cx, abs=1.0)
    assert center[1] == pytest.approx(camera.cy, abs=1.0)


def test_normalize_inverts_projection(setup):
    """Normalizar y volver a proyectar reproduce la direccion del rayo."""
    scene, camera, pose_c = setup
    points_c = world_to_camera(pose_c, scene.corners_world())
    normalized = camera.normalize(camera.project(points_c))

    expected = points_c[:, 0:2] / points_c[:, [2]]
    assert np.allclose(normalized, expected, atol=1e-12)


def test_interaction_matrix_against_finite_differences(setup):
    """La matriz de interaccion predice el cambio de las caracteristicas.

    Se aplica a la camara un twist pequeno en su propio frame y se compara el
    cambio medido de las coordenadas normalizadas con L @ twist. Se prueban los
    seis grados de libertad por separado y una combinacion aleatoria.
    """
    scene, camera, pose_c = setup
    corners = scene.corners_world()

    def features(pose):
        return camera.normalize(camera.project(world_to_camera(pose, corners))).ravel()

    points_c = world_to_camera(pose_c, corners)
    L = interaction_matrix(camera.normalize(camera.project(points_c)), points_c[:, 2])

    rng = np.random.default_rng(SEED)
    twists = list(np.eye(6)) + [rng.normal(0.0, 1.0, 6) for _ in range(5)]
    dt = 1e-7

    for twist in twists:
        moved = pose_c @ expm(_se3(np.asarray(twist, dtype=float)) * dt)
        measured = (features(moved) - features(pose_c)) / dt
        predicted = L @ np.asarray(twist, dtype=float)
        assert np.allclose(measured, predicted, atol=1e-5), (
            f"twist {twist}: medido {measured} vs predicho {predicted}"
        )


def test_camera_jacobian_against_finite_differences(setup):
    """L @ J predice el cambio de caracteristicas ante velocidades de la base.

    Es la matriz que usa la ley de control, asi que este es el test que importa:
    valida de una vez la matriz de interaccion y el jacobiano del montaje de la
    camara sobre la plataforma omnidireccional.
    """
    scene, camera, _ = setup
    corners = scene.corners_world()
    base = np.asarray(scene.desired_base_pose, dtype=float)

    def features(base_pose):
        pose_c = parking.camera_pose_matrix(base_pose)
        return camera.normalize(camera.project(world_to_camera(pose_c, corners))).ravel()

    pose_c = parking.camera_pose_matrix(base)
    points_c = world_to_camera(pose_c, corners)
    L = interaction_matrix(camera.normalize(camera.project(points_c)), points_c[:, 2])
    A = L @ parking.camera_jacobian()

    rng = np.random.default_rng(SEED + 1)
    dt = 1e-7

    for u in list(np.eye(3)) + [rng.normal(0.0, 1.0, 3) for _ in range(5)]:
        u = np.asarray(u, dtype=float)
        theta = base[2]
        c, s = np.cos(theta), np.sin(theta)
        moved = base + dt * np.array(
            [c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]]
        )
        measured = (features(moved) - features(base)) / dt
        assert np.allclose(measured, A @ u, atol=1e-5), (
            f"u {u}: medido {measured} vs predicho {A @ u}"
        )


def test_detector_recovers_the_projected_corners(setup):
    """El detector de ArUco sobre la imagen sintetizada devuelve las esquinas.

    Cierra el lazo del objetivo especifico 1: no es solo el modelo de proyeccion,
    es que un detector real las encuentra. Ademas mide el ruido de deteccion, que
    es el parametro que alimenta la campana Monte Carlo.
    """
    scene, camera, pose_c = setup
    expected = camera.project(world_to_camera(pose_c, scene.corners_world()))

    image = render_marker_view(camera, pose_c, scene.marker_pose, scene.marker_side)
    detected = detect_corners(image)

    assert detected is not None, "el detector no encontro el marcador"
    residual = np.linalg.norm(detected - expected, axis=1)
    assert residual.max() < 2.0, f"residuo por esquina {residual}"
