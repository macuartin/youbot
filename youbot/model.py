"""Parametros fisicos del brazo manipulador del KUKA youBot.

Cadena serial de 5 grados de libertad, todos rotacionales. Convencion de
Denavit-Hartenberg clasica (Craig), con la matriz de transformacion homogenea

    A_i = [[ ct, -st*ca,  st*sa, a*ct],
           [ st,  ct*ca, -ct*sa, a*st],
           [  0,     sa,     ca,    d],
           [  0,      0,      0,    1]]

Procedencia de los numeros y decisiones tomadas: ver docs/baseline-2018.md.
"""

from __future__ import annotations

import numpy as np

#: Parametros DH por eslabon: (d, a, alpha). theta es la variable articular.
#:
#: Los valores vienen del Informe de Avance #1 (2018) y quedan confirmados
#: contra la implementacion de referencia kirillin/youbot_arm_kinematics, que
#: publica DH_A = (0.033, 0.155, 0.135, 0, 0), DH_ALPHA = (pi/2, 0, 0, pi/2, 0)
#: y DH_D = (0.147, 0, 0, 0, 0.218).
#:
#: Dos discrepancias de la documentacion de 2018 se resuelven aqui:
#:
#: - alpha4: el Informe de Avance tabula 270 grados, el codigo de 2020 usaba
#:   pi/2. Gana el codigo: la referencia publica pi/2. Los 270 grados son un
#:   error de transcripcion del informe.
#: - d5: 0.2175 y no los 0.113 del bloque de prueba de homogeneus_matrix.py.
#:   Comprobacion aritmetica independiente: 0.147 + 0.155 + 0.135 + 0.2175 =
#:   0.6545, exactamente la longitud extendida de 65.45 cm que declara el
#:   propio Informe de Avance. La referencia redondea a 0.218; se conserva
#:   0.2175 por reproducir el total documentado.
DH = np.array(
    [
        # d,      a,      alpha
        [0.147, 0.033, np.pi / 2],
        [0.000, 0.155, 0.0],
        [0.000, 0.135, 0.0],
        [0.000, 0.000, np.pi / 2],
        [0.2175, 0.000, 0.0],
    ]
)

DOF = DH.shape[0]

#: Limites articulares del youBot real, en radianes.
#:
#: No intervienen en la validacion del modelo, pero acotan el espacio de
#: trabajo alcanzable en las campanas de simulacion.
JOINT_LIMITS = np.deg2rad(
    np.array(
        [
            [-169.0, 169.0],
            [-65.0, 90.0],
            [-151.0, 146.0],
            [-102.5, 102.5],
            [-167.5, 167.5],
        ]
    )
)

#: Masa de cada eslabon, en kilogramos. Informe de Avance #1.
MASS = np.array([1.390, 1.318, 0.821, 0.769, 0.687])

#: Vector del origen del eslabon a su centro de masa, expresado en el frame
#: del propio eslabon, en metros.
#:
#: El Informe de Avance tabula estas coordenadas en centimetros sin decirlo:
#: el eslabon 2 aparece con Sx = 11.397, que como metros seria un centro de
#: masa a once metros de la articulacion en un brazo de 65 cm. Leidos como
#: centimetros los cinco eslabones caen dentro de su propia geometria, que es
#: la unica lectura fisicamente posible. El codigo de 2020 los usaba crudos,
#: o sea con un factor 100 de error.
COM = (
    np.array(
        [
            [1.516, 0.359, 3.105],
            [11.397, 1.500, -1.903],
            [0.013, 10.441, 2.022],
            [0.015, 5.353, -2.464],
            [0.000, 0.120, -1.648],
        ]
    )
    / 100.0
)

#: Momentos de inercia principales (Ixx, Iyy, Izz) de cada eslabon respecto de
#: su centro de masa, en kg*m^2. Informe de Avance #1. El informe no aporta los
#: productos de inercia, asi que el tensor se toma diagonal.
INERTIA_DIAG = np.array(
    [
        [0.0029525, 0.0060091, 0.0058821],
        [0.0031145, 0.0005843, 0.0031631],
        [0.00172767, 0.00041967, 0.0018468],
        [0.0006764, 0.0010573, 0.0006610],
        [0.0001934, 0.0001602, 0.0000689],
    ]
)

#: Aceleracion de la gravedad expresada en el frame base, m/s^2.
GRAVITY = np.array([0.0, 0.0, -9.81])


def inertia_tensor(link: int) -> np.ndarray:
    """Tensor de inercia 3x3 del eslabon indicado, respecto de su centro de masa."""
    return np.diag(INERTIA_DIAG[link])


def random_configuration(rng: np.random.Generator) -> np.ndarray:
    """Configuracion articular aleatoria dentro de los limites del robot."""
    return rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
