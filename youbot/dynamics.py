"""Dinamica del brazo del youBot por formulacion recursiva de Newton-Euler.

Reimplementacion de legacy/youbot_mechanics/src/youbot_mechanics/inverse_dynamic.py
y forward_dynamic.py. La estructura de dos etapas es la que describe el Informe
de Avance #1 de 2018: propagacion de velocidades y aceleraciones desde la base
hacia el efector final, y propagacion de fuerzas desde el efector final hacia
la base.

La recursion esta escrita para DH clasica, que es la convencion de la tabla del
modelo. Importa: en DH clasica el origen del frame i queda en el extremo distal
del eslabon i y el eje de la articulacion i es Z_{i-1}, no Z_i. Eso cambia tres
cosas respecto de la version que aparece en los libros para DH modificada:

- la velocidad articular se suma a la del padre antes de rotar al frame hijo,
- el vector entre origenes se expresa en el frame hijo, como
  pstar = (a, d*sin(alpha), d*cos(alpha)),
- el torque se proyecta sobre R_i^T z, no sobre z.

Lo que cambia respecto del codigo de 2020:

1. El termino de aceleracion articular estaba sumado dos veces en la
   propagacion de aceleraciones, una vez directo y otra bajo el nombre
   `centripeta`. Los torques salian sesgados en proporcion a q'' y el error era
   silencioso.
2. La matriz de masa se construia llamando a la dinamica inversa, pero esa
   funcion solo escribia la primera columna del vector de torques, asi que
   devolvia un (n,1) donde hacia falta un (n,n). El np.linalg.inv() siguiente
   no podia funcionar. Aqui se arma por columnas de aceleracion unitaria.
3. Los operadores espaciales de 2020 (Wskew como diagonal de bloques de
   skew(w)) no son el producto cruzado espacial correcto para vectores de la
   forma [w; v], asi que el termino de sesgo w x (I w) quedaba mal. Se pasa a
   vectores de 3 componentes, que es exactamente lo que describe en palabras el
   Informe de Avance y es directamente auditable.
"""

from __future__ import annotations

import numpy as np

from .kinematics import link_transforms
from .model import COM, DH, DOF, GRAVITY, MASS, inertia_tensor

_Z = np.array([0.0, 0.0, 1.0])


def _pstar(i: int) -> np.ndarray:
    """Vector del origen del frame i-1 al origen del frame i, en el frame i."""
    d, a, alpha = DH[i]
    return np.array([a, d * np.sin(alpha), d * np.cos(alpha)])


def inverse_dynamics(
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    gravity: np.ndarray | None = None,
    wrench_ee: np.ndarray | None = None,
) -> np.ndarray:
    """Torques articulares que producen la aceleracion pedida.

    Args:
        q, qd, qdd: posicion, velocidad y aceleracion articular (n,).
        gravity: vector de gravedad en el frame base. Por defecto el del modelo.
            Pasar ceros para obtener la dinamica sin gravedad.
        wrench_ee: fuerza y momento externos en el efector final, expresados en
            el frame del ultimo eslabon, como (6,) en orden [f; n].

    Returns:
        Vector de torques (n,).
    """
    q = np.asarray(q, dtype=float).ravel()
    qd = np.asarray(qd, dtype=float).ravel()
    qdd = np.asarray(qdd, dtype=float).ravel()
    g = GRAVITY if gravity is None else np.asarray(gravity, dtype=float).ravel()
    fext = np.zeros(6) if wrench_ee is None else np.asarray(wrench_ee, dtype=float).ravel()

    A = link_transforms(q)
    R = [Ai[0:3, 0:3] for Ai in A]
    pstar = [_pstar(i) for i in range(DOF)]

    # Etapa 1: propagacion hacia el efector final.
    # El efecto de la gravedad entra como aceleracion lineal de la base.
    w = np.zeros(3)
    wd = np.zeros(3)
    vd = -g

    force = np.zeros((DOF, 3))
    moment = np.zeros((DOF, 3))

    for i in range(DOF):
        Rt = R[i].T  # del frame i-1 al frame i
        ps = pstar[i]

        wd = Rt @ (wd + _Z * qdd[i] + np.cross(w, _Z * qd[i]))
        w = Rt @ (w + _Z * qd[i])
        vd = np.cross(wd, ps) + np.cross(w, np.cross(w, ps)) + Rt @ vd

        # Fuerza y momento de inercia del eslabon, en su propio frame.
        s = COM[i]
        vd_com = np.cross(wd, s) + np.cross(w, np.cross(w, s)) + vd
        I = inertia_tensor(i)

        force[i] = MASS[i] * vd_com
        moment[i] = I @ wd + np.cross(w, I @ w)

    # Etapa 2: propagacion de fuerzas hacia la base. Los momentos se toman
    # respecto del origen del frame padre, de ahi el brazo pstar[i] + COM[i].
    f = fext[0:3].copy()
    n = fext[3:6].copy()
    tau = np.zeros(DOF)

    for i in range(DOF - 1, -1, -1):
        R_child = np.eye(3) if i == DOF - 1 else R[i + 1]

        # El brazo del momento de la fuerza que llega del hijo es pstar[i], el
        # del propio eslabon, no el del hijo.
        n = (
            R_child @ n
            + np.cross(pstar[i], R_child @ f)
            + np.cross(pstar[i] + COM[i], force[i])
            + moment[i]
        )
        f = R_child @ f + force[i]
        tau[i] = n @ (R[i].T @ _Z)

    return tau


def mass_matrix(q: np.ndarray) -> np.ndarray:
    """Matriz de masa M(q), simetrica y definida positiva (n,n).

    Se arma columna a columna: la columna j es el vector de torques que hace
    falta para una aceleracion unitaria en la articulacion j, con el robot en
    reposo y sin gravedad.
    """
    M = np.zeros((DOF, DOF))
    zero = np.zeros(DOF)
    for j in range(DOF):
        e = np.zeros(DOF)
        e[j] = 1.0
        M[:, j] = inverse_dynamics(q, zero, e, gravity=np.zeros(3))
    return M


def bias_torques(q: np.ndarray, qd: np.ndarray) -> np.ndarray:
    """Torques de Coriolis, centrifugos y de gravedad, sin aceleracion."""
    return inverse_dynamics(q, qd, np.zeros(DOF))


def gravity_torques(q: np.ndarray) -> np.ndarray:
    """Torques necesarios para sostener el brazo contra la gravedad."""
    return inverse_dynamics(q, np.zeros(DOF), np.zeros(DOF))


def forward_dynamics(
    q: np.ndarray,
    qd: np.ndarray,
    tau: np.ndarray,
    wrench_ee: np.ndarray | None = None,
) -> np.ndarray:
    """Aceleracion articular que resulta de aplicar unos torques dados.

    Resuelve M(q) q'' = tau - b(q, q'). Se usa una factorizacion de Cholesky
    en vez de invertir M explicitamente: M es simetrica definida positiva y
    resolver el sistema es mas estable numericamente que multiplicar por la
    inversa.
    """
    M = mass_matrix(q)
    b = inverse_dynamics(q, qd, np.zeros(DOF), wrench_ee=wrench_ee)
    rhs = np.asarray(tau, dtype=float).ravel() - b
    L = np.linalg.cholesky(M)
    return np.linalg.solve(L.T, np.linalg.solve(L, rhs))
