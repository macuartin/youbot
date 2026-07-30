"""Evalua en lazo cerrado la politica destilada contra su propio maestro.

Mismas poses iniciales, misma camara, mismo ruido, mismas metricas. Lo unico que
cambia entre los dos brazos es quien decide la accion: la ley de control
analitica de las Fases 1 y 2, o la red afinada sobre sus demostraciones.

Al final se encadena el agarre con `attempt_grasp`, igual que en la Fase 2, para
traducir el error de estacionamiento en lo unico que le importa a la tarea: si la
pieza se puede coger o no.

    uv run --group vla python experiments/evaluate_policy.py --trials 40
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from youbot import attempt_grasp, parking
from youbot.vision import Camera, detect_corners, render_marker_view

from generate_demonstrations import (  # noqa: E402
    ACTUATION_NOISE,
    APPROACH_HIGH,
    APPROACH_LOW,
    CAMERA_KEY,
    INSTRUCTION,
    OBSERVATION_CAMERA,
    SENSOR_NOISE,
)

RESULTS = Path(__file__).resolve().parents[1] / "results"
CHECKPOINT = RESULTS / "smolvla-docking"
DATASET = RESULTS / "docking-dataset"

SEED = 20181207
MAX_STEPS = 600
DT = 0.05

#: El mismo criterio de parada que usa el experto: la maniobra termina cuando el
#: vehiculo deja de moverse. Se aplica igual a los dos brazos; darle al experto
#: una regla de parada y a la politica no, o al reves, sesgaria la comparacion.
SETTLE_WINDOW = 20
SETTLE_POSITION = 1e-3
SETTLE_HEADING = np.deg2rad(0.1)


def sample_starts(trials: int, scene: parking.Scene, seed: int = SEED) -> np.ndarray:
    """Las mismas poses iniciales para los dos brazos del experimento."""
    rng = np.random.default_rng(seed)
    return np.array(
        [
            np.asarray(scene.desired_base_pose, dtype=float)
            + rng.uniform(APPROACH_LOW, APPROACH_HIGH)
            for _ in range(trials)
        ]
    )


def evaluate_expert(
    starts: np.ndarray,
    scene: parking.Scene,
    camera: Camera,
    max_steps: int = MAX_STEPS,
) -> list[dict]:
    rows = []
    for i, start in enumerate(starts):
        tic = time.time()
        result = parking.park(
            start,
            scene,
            camera,
            use_detector=True,
            sensor_noise=SENSOR_NOISE,
            actuation_noise=ACTUATION_NOISE,
            rng=np.random.default_rng(SEED + i),
            max_iterations=max_steps,
        )
        rows.append(
            {
                "lost_marker": bool(result.lost_marker),
                "settled": bool(result.converged),
                "steps": int(result.iterations),
                "position_error_mm": float(result.position_error * 1000),
                "heading_error_deg": float(np.rad2deg(result.heading_error)),
                "seconds": time.time() - tic,
                "grasp": bool(
                    attempt_grasp(result.final_pose, scene.pick_point).success
                )
                if not result.lost_marker
                else False,
            }
        )
    return rows


def load_policy(device: str | None):
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.policies.smolvla.processor_smolvla import (
        make_smolvla_pre_post_processors,
    )

    from train_policy import make_config

    if device is None:
        device = "mps" if torch.backends.mps.is_available() else "cpu"

    dataset = LeRobotDataset(repo_id="macuartin/youbot-docking", root=DATASET)
    config = make_config(dataset.meta, device)

    policy = SmolVLAPolicy.from_pretrained(
        CHECKPOINT, config=config, dataset_stats=dataset.meta.stats
    )
    policy.to(device)
    policy.eval()

    preprocessor, postprocessor = make_smolvla_pre_post_processors(
        config, dataset_stats=dataset.meta.stats
    )
    return policy, preprocessor, postprocessor, device


def evaluate_policy(
    starts: np.ndarray,
    scene: parking.Scene,
    camera: Camera,
    device: str | None,
    max_steps: int = MAX_STEPS,
) -> tuple[list[dict], dict]:
    import cv2
    import torch

    policy, preprocessor, postprocessor, device = load_policy(device)

    rows = []
    latencies: list[float] = []

    for i, start in enumerate(starts):
        rng = np.random.default_rng(SEED + i)
        pose = np.asarray(start, dtype=float).copy()
        previous_action = np.zeros(3, dtype=np.float32)
        policy.reset()

        lost = False
        settled = False
        history: list[np.ndarray] = [pose.copy()]
        tic = time.time()
        step = 0

        for step in range(1, max_steps + 1):
            camera_pose = parking.camera_pose_matrix(pose)
            try:
                image = render_marker_view(
                    camera,
                    camera_pose,
                    scene.marker_pose,
                    scene.marker_side,
                    noise_sigma=SENSOR_NOISE,
                    rng=rng,
                )
            except ValueError:
                lost = True
                break

            # El mismo criterio de perdida que usa el experto: si el detector no
            # encuentra el marcador, la maniobra ha fracasado. Se aplica a los dos
            # brazos por igual aunque la politica no use las esquinas.
            if detect_corners(image) is None:
                lost = True
                break

            observation = {
                CAMERA_KEY: torch.from_numpy(
                    cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
                ).permute(2, 0, 1)[None].float()
                / 255.0,
                "observation.state": torch.from_numpy(previous_action)[None],
                "task": [INSTRUCTION],
            }

            latency_start = time.time()
            with torch.no_grad():
                batch = preprocessor(observation)
                batch = {
                    k: (v.to(device) if isinstance(v, torch.Tensor) else v)
                    for k, v in batch.items()
                }
                action = policy.select_action(batch)
                action = postprocessor(action)
            latencies.append(time.time() - latency_start)

            u = np.asarray(action.squeeze().cpu().numpy(), dtype=float).ravel()[:3]
            previous_action = u.astype(np.float32)

            u_applied = u * (1.0 + rng.normal(0.0, ACTUATION_NOISE, u.shape))
            theta = pose[2]
            c, s = np.cos(theta), np.sin(theta)
            pose = pose + DT * np.array(
                [
                    c * u_applied[0] - s * u_applied[1],
                    s * u_applied[0] + c * u_applied[1],
                    u_applied[2],
                ]
            )
            history.append(pose.copy())

            if len(history) > SETTLE_WINDOW:
                recent = np.array(history[-SETTLE_WINDOW:])
                moved = np.linalg.norm(recent[:, 0:2] - pose[0:2], axis=1).max()
                turned = np.abs(
                    np.arctan2(
                        np.sin(recent[:, 2] - pose[2]), np.cos(recent[:, 2] - pose[2])
                    )
                ).max()
                if moved < SETTLE_POSITION and turned < SETTLE_HEADING:
                    settled = True
                    break

        desired = np.asarray(scene.desired_base_pose, dtype=float)
        heading = np.arctan2(
            np.sin(pose[2] - desired[2]), np.cos(pose[2] - desired[2])
        )
        rows.append(
            {
                "lost_marker": bool(lost),
                "settled": bool(settled),
                "steps": int(step),
                "position_error_mm": float(
                    np.linalg.norm(pose[0:2] - desired[0:2]) * 1000
                ),
                "heading_error_deg": float(abs(np.rad2deg(heading))),
                "seconds": time.time() - tic,
                "grasp": bool(attempt_grasp(pose, scene.pick_point).success)
                if not lost
                else False,
            }
        )
        print(
            f"  ensayo {i + 1}/{len(starts)}: "
            f"{rows[-1]['position_error_mm']:.2f} mm, "
            f"agarre {'si' if rows[-1]['grasp'] else 'no'}"
        )

    inference = {
        "device": device,
        "latency_mean_ms": float(np.mean(latencies) * 1000),
        "latency_p95_ms": float(np.percentile(latencies, 95) * 1000),
        "effective_hz": float(1.0 / np.mean(latencies)),
    }
    return rows, inference


def summarise(rows: list[dict]) -> dict:
    valid = [r for r in rows if not r["lost_marker"]]
    if not valid:
        return {"trials": len(rows), "lost_marker": len(rows), "completed": 0}

    errors = np.array([r["position_error_mm"] for r in valid])
    headings = np.array([r["heading_error_deg"] for r in valid])
    return {
        "trials": len(rows),
        "lost_marker": sum(r["lost_marker"] for r in rows),
        "completed": len(valid),
        "position_error_mm": {
            "mean": float(errors.mean()),
            "p95": float(np.percentile(errors, 95)),
            "max": float(errors.max()),
        },
        "heading_error_deg": {
            "mean": float(headings.mean()),
            "max": float(headings.max()),
        },
        "steps_mean": float(np.mean([r["steps"] for r in valid])),
        "settled_fraction": float(np.mean([r["settled"] for r in valid])),
        "grasp_success_rate": float(np.mean([r["grasp"] for r in valid])),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--max-steps", type=int, default=MAX_STEPS)
    args = parser.parse_args()

    scene = parking.Scene.default()
    camera = OBSERVATION_CAMERA
    starts = sample_starts(args.trials, scene)

    print(f"Experto clasico sobre {args.trials} ensayos...")
    expert = summarise(evaluate_expert(starts, scene, camera, args.max_steps))

    print(f"\nPolitica destilada sobre los mismos {args.trials} ensayos...")
    policy_rows, inference = evaluate_policy(
        starts, scene, camera, args.device, args.max_steps
    )
    student = summarise(policy_rows)

    print("\n" + "=" * 62)
    print(f"{'':22s} {'experto':>18s} {'destilada':>18s}")
    print("-" * 62)

    def row(label: str, a, b, fmt: str = "{:.3f}"):
        left = fmt.format(a) if a is not None else "-"
        right = fmt.format(b) if b is not None else "-"
        print(f"{label:22s} {left:>18s} {right:>18s}")

    row("ensayos completados", expert["completed"], student["completed"], "{:d}")
    row("asentaron", expert.get("settled_fraction", 0) * 100, student.get("settled_fraction", 0) * 100, "{:.0f}%")
    row("marcador perdido", expert["lost_marker"], student["lost_marker"], "{:d}")
    if expert.get("position_error_mm") and student.get("position_error_mm"):
        row("error medio (mm)", expert["position_error_mm"]["mean"], student["position_error_mm"]["mean"])
        row("error p95 (mm)", expert["position_error_mm"]["p95"], student["position_error_mm"]["p95"])
        row("error max (mm)", expert["position_error_mm"]["max"], student["position_error_mm"]["max"])
        row("guinada media (deg)", expert["heading_error_deg"]["mean"], student["heading_error_deg"]["mean"])
        row("exito de agarre", expert["grasp_success_rate"] * 100, student["grasp_success_rate"] * 100, "{:.1f}%")

    print("-" * 62)
    print(
        f"latencia de inferencia: {inference['latency_mean_ms']:.1f} ms de media "
        f"({inference['effective_hz']:.1f} Hz efectivos) en {inference['device']}"
    )

    RESULTS.mkdir(exist_ok=True)
    payload = {
        "expert": expert,
        "distilled": student,
        "inference": inference,
        "task_budget_mm": 54.1,
        "trials_raw": {"policy": policy_rows},
    }
    out = RESULTS / "distillation.json"
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"\nGuardado en {out}")


if __name__ == "__main__":
    main()
