"""Genera demostraciones de estacionamiento con el controlador clasico como experto.

El bloqueante que el survey identificaba para afinar un VLA eran los datos: la
documentacion de SmolVLA recomienda del orden de 50 episodios teleoperados de la
tarea. Aqui no hace falta teleoperar nada, porque las Fases 1 y 2 dejaron un
controlador que resuelve la tarea. Un controlador que funciona es un experto que
genera demostraciones solo.

La condicion que hay que respetar para que la destilacion signifique algo es que
maestro y alumno vean **los mismos pixeles**. Por eso el experto corre con el
detector real dentro del lazo (`use_detector=True`) sobre la misma imagen que se
graba como observacion.

    uv run --group vla python experiments/generate_demonstrations.py --episodes 120
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from youbot import parking
from youbot.vision import Camera

SEED = 20181207
RESULTS = Path(__file__).resolve().parents[1] / "results"

#: Ventana de aproximacion: la dispersion con la que un algoritmo de navegacion
#: deja al robot cerca de la estacion. La misma de la campana de la Fase 2.
APPROACH_LOW = np.array([-0.10, -0.12, -0.10])
APPROACH_HIGH = np.array([0.25, 0.12, 0.10])

#: Ruido de sensor de imagen, en niveles de gris.
SENSOR_NOISE = 2.0

#: Error relativo de ejecucion de las velocidades comandadas.
ACTUATION_NOISE = 0.02

#: Camara de observacion, la misma para el experto y para la politica aprendida.
#:
#: Es una decision de diseno crucial y no una comodidad. Si el experto viera
#: 640x480 y la politica 256x192, la comparacion mediria resolucion y no
#: aprendizaje. Se mide con la misma camara para los dos.
#:
#: 256x192 sale de un barrido: la precision del experto se degrada con la
#: resolucion (1,43 mm a 640x480, 4,83 mm a 256x192) y por debajo de esa
#: resolucion deja de asentarse, porque el marcador ocupa menos de 50 px y las
#: esquinas subpixel dejan de ser fiables. Es el punto mas bajo que todavia
#: resuelve la tarea, y por tanto el mas exigente que se le puede pedir a la
#: politica sin regalarle pixeles.
OBSERVATION_CAMERA = Camera(fx=160.0, fy=160.0, cx=128.0, cy=96.0, width=256, height=192)

#: Instruccion en lenguaje natural que acompana a cada episodio.
INSTRUCTION = "dock the mobile base in front of the station marker"

#: Frecuencia del lazo de control: dt = 0.05 s.
FPS = 20

#: Nombre de la camara dentro del dataset.
CAMERA_KEY = "observation.images.front"


@dataclass
class Episode:
    """Una demostracion completa."""

    images: np.ndarray  # (T, H, W, 3) uint8
    actions: np.ndarray  # (T, 3) float32, velocidades (vx, vy, omega)
    poses: np.ndarray  # (T, 3) float32, pose de la plataforma, solo para analisis
    final_position_error: float
    final_heading_error: float
    settled: bool


def collect_episode(
    start: np.ndarray,
    scene: parking.Scene,
    camera: Camera,
    rng: np.random.Generator,
) -> Episode | None:
    """Corre el experto una vez y graba cada paso.

    Devuelve None si la maniobra pierde el marcador de vista, que es un fallo
    legitimo del experto y no debe entrar en el conjunto de entrenamiento.
    """
    import cv2

    images: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    poses: list[np.ndarray] = []

    def record(_iteration, pose, image, action):
        # La imagen ya viene a la resolucion de observacion: es la de la camara.
        images.append(cv2.cvtColor(image, cv2.COLOR_GRAY2RGB))
        actions.append(action.astype(np.float32))
        poses.append(pose.astype(np.float32))

    result = parking.park(
        start,
        scene,
        camera,
        use_detector=True,
        sensor_noise=SENSOR_NOISE,
        actuation_noise=ACTUATION_NOISE,
        on_step=record,
        rng=rng,
    )

    if result.lost_marker or not images:
        return None

    return Episode(
        images=np.array(images, dtype=np.uint8),
        actions=np.array(actions, dtype=np.float32),
        poses=np.array(poses, dtype=np.float32),
        final_position_error=result.position_error,
        final_heading_error=result.heading_error,
        settled=result.converged,
    )


def collect(episodes: int, camera: Camera | None = None) -> list[Episode]:
    scene = parking.Scene.default()
    camera = OBSERVATION_CAMERA if camera is None else camera
    rng = np.random.default_rng(SEED)

    collected: list[Episode] = []
    attempts = 0

    while len(collected) < episodes:
        attempts += 1
        start = np.asarray(scene.desired_base_pose, dtype=float) + rng.uniform(
            APPROACH_LOW, APPROACH_HIGH
        )
        episode = collect_episode(start, scene, camera, rng)
        if episode is None:
            continue
        collected.append(episode)
        if len(collected) % 10 == 0:
            print(f"  {len(collected)}/{episodes} episodios ({attempts} intentos)")

    return collected


def previous_actions(actions: np.ndarray) -> np.ndarray:
    """El estado que se le da a la politica: la accion del paso anterior.

    Deliberadamente NO se le da la pose de la plataforma. Darsela seria resolver
    la tarea sin mirar la imagen, que es justo lo contrario de lo que este
    experimento mide: el servo visual basado en imagen existe porque el robot no
    conoce su pose respecto de la estacion. La accion anterior si es informacion
    que un robot real tiene, porque es la que el mismo comando.
    """
    state = np.zeros_like(actions)
    state[1:] = actions[:-1]
    return state


def write_lerobot_dataset(episodes: list[Episode], root: Path, repo_id: str) -> Path:
    """Escribe las demostraciones en formato LeRobot, listo para afinar."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    height, width = episodes[0].images.shape[1:3]
    features = {
        CAMERA_KEY: {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (3,),
            "names": ["prev_vx", "prev_vy", "prev_omega"],
        },
        "action": {
            "dtype": "float32",
            "shape": (3,),
            "names": ["vx", "vy", "omega"],
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=repo_id, fps=FPS, features=features, root=root, robot_type="youbot"
    )

    for episode in episodes:
        state = previous_actions(episode.actions)
        for t in range(episode.actions.shape[0]):
            dataset.add_frame(
                {
                    CAMERA_KEY: episode.images[t],
                    "observation.state": state[t],
                    "action": episode.actions[t],
                    "task": INSTRUCTION,
                }
            )
        dataset.save_episode()

    return root


def summarise(episodes: list[Episode]) -> dict:
    lengths = np.array([e.actions.shape[0] for e in episodes])
    errors = np.array([e.final_position_error for e in episodes])
    headings = np.array([e.final_heading_error for e in episodes])

    return {
        "episodes": len(episodes),
        "frames": int(lengths.sum()),
        "length_mean": float(lengths.mean()),
        "length_max": int(lengths.max()),
        "settled_fraction": float(np.mean([e.settled for e in episodes])),
        "expert_position_error_mm": {
            "mean": float(errors.mean() * 1000),
            "p95": float(np.percentile(errors, 95) * 1000),
            "max": float(errors.max() * 1000),
        },
        "expert_heading_error_deg": {
            "mean": float(np.rad2deg(headings.mean())),
            "max": float(np.rad2deg(headings.max())),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=120)
    parser.add_argument(
        "--npz",
        type=Path,
        default=None,
        help="opcional: vuelca tambien las imagenes crudas a un npz. Pesa unos "
        "4 MB por episodio, cien veces mas que el dataset LeRobot, que las "
        "codifica en video.",
    )
    parser.add_argument(
        "--lerobot-root",
        type=Path,
        default=RESULTS / "docking-dataset",
        help="destino del dataset en formato LeRobot",
    )
    args = parser.parse_args()

    print(f"Recogiendo {args.episodes} demostraciones del experto clasico...")
    episodes = collect(args.episodes)
    stats = summarise(episodes)

    print(f"\nepisodios: {stats['episodes']}, fotogramas: {stats['frames']}")
    print(f"longitud media: {stats['length_mean']:.1f} pasos, maxima {stats['length_max']}")
    print(f"asentaron por criterio propio: {stats['settled_fraction']:.1%}")
    err = stats["expert_position_error_mm"]
    print(
        f"error del experto: media {err['mean']:.3f} mm, "
        f"p95 {err['p95']:.3f} mm, max {err['max']:.3f} mm"
    )

    import json

    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "demonstrations.json").write_text(json.dumps(stats, indent=2) + "\n")

    if args.npz is not None:
        np.savez_compressed(
            args.npz,
            images=np.concatenate([e.images for e in episodes]),
            actions=np.concatenate([e.actions for e in episodes]),
            poses=np.concatenate([e.poses for e in episodes]),
            episode_index=np.concatenate(
                [
                    np.full(e.actions.shape[0], i, dtype=np.int32)
                    for i, e in enumerate(episodes)
                ]
            ),
            instruction=np.array([INSTRUCTION]),
        )
        print(f"npz crudo en {args.npz} ({args.npz.stat().st_size / 1e6:.1f} MB)")

    print(f"\nEscribiendo dataset LeRobot en {args.lerobot_root}...")
    write_lerobot_dataset(episodes, args.lerobot_root, "macuartin/youbot-docking")
    print("Listo.")


if __name__ == "__main__":
    main()
