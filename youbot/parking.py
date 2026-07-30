"""Control servo visual de estacionamiento de precision para la base omnidireccional.

Objetivos especificos 1 y 5 del trabajo de 2018. El planteamiento del problema
del anteproyecto es este: con SLAM se puede asegurar que el AGV llegue a una
estacion, pero no con que orientacion y posicion respecto de ella, y de eso
depende que el manipulador pueda hacer su tarea o no.

El esquema es servo visual basado en imagen (IBVS): la referencia no es una pose
sino la imagen que el marcador de la estacion produce cuando el robot esta bien
estacionado. El error se mide en el plano imagen y se corrige con las tres
velocidades de la base omnidireccional.

Detalle de implementacion: la matriz de interaccion se evalua con las
profundidades de la pose deseada (Z*) y no con las actuales. Es la variante
clasica robusta, y evita tener que estimar profundidad en cada iteracion.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .model import CAMERA_MOUNT, CAMERA_ROTATION
from .vision import (
    Camera,
    detect_corners,
    interaction_matrix,
    marker_corners,
    render_marker_view,
    transform_points,
    world_to_camera,
)


def base_pose_matrix(pose: np.ndarray) -> np.ndarray:
    """Transformacion 4x4 del frame de la plataforma al mundo, desde (x, y, theta)."""
    x, y, theta = np.asarray(pose, dtype=float).ravel()
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[0:3, 0:3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    T[0:3, 3] = [x, y, 0.0]
    return T


def camera_pose_matrix(base_pose: np.ndarray) -> np.ndarray:
    """Pose 4x4 de la camara en el mundo, dada la pose de la plataforma."""
    T_wb = base_pose_matrix(base_pose)
    T_bc = np.eye(4)
    T_bc[0:3, 0:3] = CAMERA_ROTATION
    T_bc[0:3, 3] = CAMERA_MOUNT
    return T_wb @ T_bc


def camera_jacobian() -> np.ndarray:
    """Jacobiano de la base a la camara, (6, 3).

    Mapea las tres velocidades controlables de la plataforma omnidireccional,
    (vx, vy, omega) en el frame de la plataforma, al twist de la camara expresado
    en el frame de la camara. Es constante: la camara va rigidamente montada.
    """
    R_cb = CAMERA_ROTATION.T
    J = np.zeros((6, 3))

    # Traslacion pura de la plataforma.
    J[0:3, 0] = R_cb @ np.array([1.0, 0.0, 0.0])
    J[0:3, 1] = R_cb @ np.array([0.0, 1.0, 0.0])

    # Giro de la plataforma: arrastra la camara por estar desplazada del centro.
    omega_b = np.array([0.0, 0.0, 1.0])
    J[0:3, 2] = R_cb @ np.cross(omega_b, CAMERA_MOUNT)
    J[3:6, 2] = R_cb @ omega_b
    return J


@dataclass
class Scene:
    """Estacion de trabajo: marcador, punto de recogida y pose de estacionamiento."""

    marker_pose: np.ndarray
    marker_side: float = 0.15
    pick_point: np.ndarray = field(
        default_factory=lambda: np.array([0.10, 0.0, 0.35])
    )
    desired_base_pose: np.ndarray = field(
        default_factory=lambda: np.array([0.55, 0.0, np.pi])
    )

    @staticmethod
    def default() -> "Scene":
        """Marcador vertical en el origen, encarando hacia +x, a 35 cm del suelo.

        El robot se estaciona a 55 cm delante, girado 180 grados para quedar de
        frente. El punto de recogida esta sobre el borde de la estacion.

        Sobre la orientacion del marcador: OpenCV define el frame de un ArUco con
        x a la derecha, y hacia abajo y **z entrando en el plano del marcador**,
        o sea alejandose de quien lo mira. Poner la z apuntando hacia el robot,
        que es el error intuitivo, hace que el marcador se sintetice espejado y
        el detector no lo encuentre. Aqui z va hacia -x del mundo.
        """
        T = np.eye(4)
        T[0:3, 0:3] = [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
        T[0:3, 3] = [0.0, 0.0, 0.35]
        return Scene(marker_pose=T)

    def corners_world(self) -> np.ndarray:
        return transform_points(self.marker_pose, marker_corners(self.marker_side))


@dataclass
class ParkResult:
    """Resultado de una maniobra de estacionamiento."""

    converged: bool
    iterations: int
    final_pose: np.ndarray
    position_error: float
    heading_error: float
    feature_error: float
    lost_marker: bool = False
    trace: np.ndarray | None = None


def _features(
    camera: Camera, base_pose: np.ndarray, scene: Scene
) -> tuple[np.ndarray, np.ndarray]:
    """Pixeles de las cuatro esquinas y sus profundidades, para una pose de base."""
    pose_c = camera_pose_matrix(base_pose)
    points_c = world_to_camera(pose_c, scene.corners_world())
    return camera.project(points_c), points_c[:, 2]


def park(
    initial_pose: np.ndarray,
    scene: Scene | None = None,
    camera: Camera | None = None,
    gain: float = 0.6,
    dt: float = 0.05,
    max_iterations: int = 600,
    pixel_noise: float = 0.0,
    actuation_noise: float = 0.0,
    use_detector: bool = False,
    sensor_noise: float = 0.0,
    on_step=None,
    tolerance: float = 1e-3,
    settle_window: int = 20,
    settle_position: float = 1e-3,
    settle_heading: float = np.deg2rad(0.1),
    damping: float = 1e-4,
    rng: np.random.Generator | None = None,
    record: bool = False,
) -> ParkResult:
    """Estaciona la base por servo visual basado en imagen.

    Args:
        initial_pose: (x, y, theta) de partida, tipicamente la pose con la que
            un algoritmo de navegacion deja al robot cerca de la estacion.
        gain: ganancia proporcional de la ley de control.
        dt: paso de integracion, en segundos.
        pixel_noise: desviacion tipica del ruido de deteccion de esquinas, en
            pixeles. Modelo del ruido del detector; se ignora si use_detector.
        use_detector: si True, en cada paso se sintetiza la vista del marcador y
            las esquinas las extrae cv2.aruco de la imagen, en vez de proyectarlas
            geometricamente y anadirles ruido. Mas lento, pero es la unica forma
            de que una politica aprendida vea exactamente los mismos pixeles que
            uso el maestro, que es lo que exige un experimento de destilacion.
        sensor_noise: ruido gaussiano de sensor de la imagen, en niveles de gris.
            Solo aplica si use_detector.
        on_step: callback opcional que recibe (iteracion, pose, imagen, accion) en
            cada paso. Es el gancho con el que se graban demostraciones.
        actuation_noise: desviacion tipica del error relativo de ejecucion de las
            velocidades comandadas, adimensional. 0.02 son ruedas que se desvian
            un 2 por ciento de lo pedido.
        tolerance: norma del error de caracteristicas, en coordenadas
            normalizadas, por debajo de la cual se considera convergido. Solo es
            alcanzable sin ruido: con 0.5 px de ruido de deteccion el suelo del
            error de caracteristicas es del orden de 3.5e-3, o sea por encima de
            esta tolerancia. De ahi el criterio de asentamiento.
        settle_window: numero de pasos sobre los que se mide si el vehiculo ya
            dejo de moverse.
        settle_position: desplazamiento maximo dentro de la ventana, en metros,
            para declarar la maniobra terminada. El valor por defecto no es
            arbitrario: el jitter de la pose en regimen permanente, con 0.5 px de
            ruido de deteccion, tiene un p95 de 1.75 mm medido, asi que el umbral
            tiene que estar en ese orden o la maniobra no termina nunca. Apretarlo
            no mejora el resultado, solo agota el limite de iteraciones; aflojarlo
            termina antes pero con mas error. Con 1 mm el error final queda 15
            veces por debajo del presupuesto de la tarea.
        settle_heading: cambio de guinada maximo dentro de la ventana, en
            radianes.

    Returns:
        ParkResult con el error espacial final respecto de la pose deseada, que
        es la magnitud que el objetivo especifico 5 exige acotar a +-10 cm.
    """
    scene = Scene.default() if scene is None else scene
    camera = Camera() if camera is None else camera
    generator = np.random.default_rng() if rng is None else rng

    # Referencia: la imagen del marcador desde la pose de estacionamiento ideal.
    target_px, target_depths = _features(camera, scene.desired_base_pose, scene)
    s_target = camera.normalize(target_px).ravel()

    # Matriz de interaccion evaluada en la pose deseada, constante en el lazo.
    L = interaction_matrix(camera.normalize(target_px), target_depths)
    A = L @ camera_jacobian()
    ATA = A.T @ A + damping * np.eye(3)

    pose = np.asarray(initial_pose, dtype=float).ravel().copy()
    trace = [pose.copy()] if record else None

    converged = False
    lost = False
    error_norm = np.inf
    history: list[np.ndarray] = [pose.copy()]

    for iteration in range(1, max_iterations + 1):
        try:
            pixels, _ = _features(camera, pose, scene)
        except ValueError:
            lost = True
            break

        if not camera.in_view(pixels):
            lost = True
            break

        image = None
        if use_detector:
            image = render_marker_view(
                camera,
                camera_pose_matrix(pose),
                scene.marker_pose,
                scene.marker_side,
                noise_sigma=sensor_noise,
                rng=generator,
            )
            detected = detect_corners(image)
            if detected is None:
                lost = True
                break
            pixels = detected
        elif pixel_noise > 0:
            pixels = pixels + generator.normal(0.0, pixel_noise, pixels.shape)

        error = camera.normalize(pixels).ravel() - s_target
        error_norm = float(np.linalg.norm(error))
        if error_norm < tolerance:
            converged = True
            break

        # Criterio de asentamiento: el vehiculo ya no se mueve. Es el unico
        # criterio de parada valido cuando el ruido de deteccion pone un suelo al
        # error de caracteristicas, que es siempre que haya un detector real.
        if len(history) > settle_window:
            recent = np.array(history[-settle_window:])
            moved = np.linalg.norm(recent[:, 0:2] - pose[0:2], axis=1).max()
            turned = np.abs(
                np.arctan2(
                    np.sin(recent[:, 2] - pose[2]), np.cos(recent[:, 2] - pose[2])
                )
            ).max()
            if moved < settle_position and turned < settle_heading:
                converged = True
                break

        # Ley de control: minimos cuadrados amortiguados sobre las tres
        # velocidades de la plataforma.
        u = -gain * np.linalg.solve(ATA, A.T @ error)

        if on_step is not None:
            on_step(iteration, pose.copy(), image, u.copy())

        if actuation_noise > 0:
            u = u * (1.0 + generator.normal(0.0, actuation_noise, u.shape))

        # Integracion de la pose: las velocidades son en el frame de la base.
        theta = pose[2]
        c, s = np.cos(theta), np.sin(theta)
        pose = pose + dt * np.array(
            [c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]]
        )

        history.append(pose.copy())
        if record:
            trace.append(pose.copy())

    desired = np.asarray(scene.desired_base_pose, dtype=float).ravel()
    heading = np.arctan2(
        np.sin(pose[2] - desired[2]), np.cos(pose[2] - desired[2])
    )

    return ParkResult(
        converged=converged,
        iterations=iteration,
        final_pose=pose,
        position_error=float(np.linalg.norm(pose[0:2] - desired[0:2])),
        heading_error=float(abs(heading)),
        feature_error=error_norm,
        lost_marker=lost,
        trace=np.array(trace) if record else None,
    )
