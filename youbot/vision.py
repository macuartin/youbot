"""Modelo de camara, marcador fiducial y matriz de interaccion.

Es la base del objetivo especifico 1 del trabajo de 2018, del que no quedo
escrita ni una linea: "desarrollar e implementar un esquema de control visual,
para el AGV, basado en la extraccion de caracteristicas por medio de codigos QR
localizados en las estaciones de la FMS".

Sobre el marcador. El objetivo original decia codigos QR, elegidos porque de
ellos se puede extraer informacion codificada ademas de las caracteristicas de
la imagen. Aqui se usan marcadores ArUco, que cumplen las dos funciones (llevan
un identificador y aportan cuatro esquinas detectables) y que son la familia que
desplazo al QR para estimacion de pose. Para lo que importa al control servo
visual, las cuatro esquinas de un cuadrado plano, los dos son equivalentes.
OpenCV ademas genera imagenes ArUco de forma nativa, sin dependencias extra.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Camera:
    """Camara pinhole con parametros intrinsecos.

    Los valores por defecto corresponden a una camara industrial compacta tipo
    la uEye CP del anteproyecto, con sensor VGA y optica gran angular: el campo
    de vision horizontal es de unos 77 grados. Las camaras de docking son
    angulares a proposito, porque el marcador tiene que seguir en cuadro mientras
    el vehiculo corrige una desviacion inicial que no controla.
    """

    fx: float = 400.0
    fy: float = 400.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480

    def project(self, points_camera: np.ndarray) -> np.ndarray:
        """Proyecta puntos 3D del frame de la camara a pixeles (N, 2)."""
        p = np.atleast_2d(np.asarray(points_camera, dtype=float))
        z = p[:, 2]
        if np.any(z <= 1e-6):
            raise ValueError("hay puntos detras del plano de la camara")
        return np.column_stack(
            [self.fx * p[:, 0] / z + self.cx, self.fy * p[:, 1] / z + self.cy]
        )

    def normalize(self, pixels: np.ndarray) -> np.ndarray:
        """Pasa de pixeles a coordenadas normalizadas (N, 2)."""
        p = np.atleast_2d(np.asarray(pixels, dtype=float))
        return np.column_stack([(p[:, 0] - self.cx) / self.fx, (p[:, 1] - self.cy) / self.fy])

    def in_view(self, pixels: np.ndarray) -> bool:
        """True si todos los pixeles caen dentro del sensor."""
        p = np.atleast_2d(np.asarray(pixels, dtype=float))
        return bool(
            np.all(p[:, 0] >= 0)
            and np.all(p[:, 0] < self.width)
            and np.all(p[:, 1] >= 0)
            and np.all(p[:, 1] < self.height)
        )


def marker_corners(side: float) -> np.ndarray:
    """Las cuatro esquinas del marcador en su propio frame, (4, 3).

    El marcador vive en el plano z = 0, con x a la derecha e y hacia abajo, que
    es el orden en que OpenCV devuelve las esquinas de un ArUco.
    """
    h = side / 2.0
    return np.array(
        [
            [-h, -h, 0.0],
            [h, -h, 0.0],
            [h, h, 0.0],
            [-h, h, 0.0],
        ]
    )


def transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Aplica una transformacion homogenea 4x4 a un conjunto de puntos (N, 3)."""
    p = np.atleast_2d(np.asarray(points, dtype=float))
    return (T[0:3, 0:3] @ p.T).T + T[0:3, 3]


def world_to_camera(camera_pose: np.ndarray, points_world: np.ndarray) -> np.ndarray:
    """Pasa puntos del mundo al frame de la camara, dada la pose 4x4 de esta."""
    R = camera_pose[0:3, 0:3]
    t = camera_pose[0:3, 3]
    p = np.atleast_2d(np.asarray(points_world, dtype=float))
    return (R.T @ (p - t).T).T


def interaction_matrix(normalized: np.ndarray, depths: np.ndarray) -> np.ndarray:
    """Matriz de interaccion de un conjunto de puntos, (2N, 6).

    Relaciona el twist de la camara, expresado en el frame de la camara y en
    orden [v; w], con la velocidad de las coordenadas normalizadas de imagen:
    s_punto = L v_camara.

    Es la forma clasica de Chaumette y Hutchinson. Para cada punto (x, y) a
    profundidad Z:

        [ -1/Z    0   x/Z    x*y   -(1+x^2)   y  ]
        [   0   -1/Z  y/Z  (1+y^2)   -x*y    -x  ]
    """
    s = np.atleast_2d(np.asarray(normalized, dtype=float))
    Z = np.asarray(depths, dtype=float).ravel()
    if s.shape[0] != Z.size:
        raise ValueError("hacen falta tantas profundidades como puntos")

    L = np.zeros((2 * s.shape[0], 6))
    for i, ((x, y), z) in enumerate(zip(s, Z)):
        L[2 * i] = [-1.0 / z, 0.0, x / z, x * y, -(1.0 + x**2), y]
        L[2 * i + 1] = [0.0, -1.0 / z, y / z, 1.0 + y**2, -x * y, -x]
    return L


def marker_image(
    marker_id: int = 0, pixels: int = 200, border_modules: int = 1
) -> tuple[np.ndarray, np.ndarray]:
    """Imagen del marcador ArUco en escala de grises, con su zona silenciosa.

    Un marcador impreso necesita margen blanco alrededor para que el detector
    encuentre el contorno. Ese margen no forma parte del cuadrado que se
    proyecta, asi que la funcion devuelve tambien donde quedan las esquinas del
    marcador dentro de la plantilla. Devolverlas evita recalcular el padding en
    quien la consume, que es como se cuela un desplazamiento constante.

    Returns:
        (imagen, esquinas) con esquinas (4, 2) en pixeles de plantilla, en el
        mismo orden que marker_corners().
    """
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)

    # El diccionario 4x4 ocupa 6 modulos con su borde negro.
    pad = int(round(pixels * border_modules / 6.0))
    if pad > 0:
        marker = cv2.copyMakeBorder(
            marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255
        )

    lo, hi = float(pad), float(pad + pixels - 1)
    corners = np.array([[lo, lo], [hi, lo], [hi, hi], [lo, hi]])
    return marker, corners


def detect_corners(image: np.ndarray) -> np.ndarray | None:
    """Detecta las esquinas del marcador en una imagen, (4, 2) en pixeles.

    Devuelve None si el detector no encuentra ningun marcador.
    """
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None or len(corners) == 0:
        return None
    return np.asarray(corners[0], dtype=float).reshape(4, 2)


def render_marker_view(
    camera: Camera,
    camera_pose: np.ndarray,
    marker_pose: np.ndarray,
    marker_side: float,
    marker_id: int = 0,
    noise_sigma: float = 0.0,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Sintetiza la imagen que ve la camara de un marcador plano.

    El marcador es plano, asi que lo que ve la camara es exactamente una
    homografia de su imagen frontal. No hace falta un motor de render: warp de
    la imagen del ArUco a los cuatro pixeles proyectados y ya.

    Args:
        noise_sigma: desviacion tipica del ruido gaussiano de sensor, en niveles
            de gris.
    """
    import cv2

    corners_world = transform_points(marker_pose, marker_corners(marker_side))
    corners_px = camera.project(world_to_camera(camera_pose, corners_world))

    template, src = marker_image(marker_id)
    H = cv2.getPerspectiveTransform(
        src.astype(np.float32), corners_px.astype(np.float32)
    )
    canvas = np.full((camera.height, camera.width), 255, dtype=np.uint8)
    image = cv2.warpPerspective(
        template,
        H,
        (camera.width, camera.height),
        dst=canvas,
        borderMode=cv2.BORDER_TRANSPARENT,
    )

    if noise_sigma > 0:
        generator = np.random.default_rng() if rng is None else rng
        noisy = image.astype(float) + generator.normal(0.0, noise_sigma, image.shape)
        image = np.clip(noisy, 0, 255).astype(np.uint8)

    return image
