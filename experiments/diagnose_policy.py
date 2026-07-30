"""Diagnostico en lazo abierto de la politica destilada.

Antes de interpretar cualquier numero de lazo cerrado hay que responder una
pregunta mas basica: **la politica reproduce las acciones del experto sobre los
fotogramas con los que se entreno?**

Si no lo hace, el error en lazo cerrado no informa de nada sobre acumulacion de
error ni sobre control: informa de que el modelo no aprendio la tarea. Y para
distinguir "no puede aprenderla" de "no se entreno bastante" se repite el
diagnostico sobre los checkpoints intermedios y se mira si la curva mejora o esta
plana.

    uv run --group vla python experiments/diagnose_policy.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

RESULTS = Path(__file__).resolve().parents[1] / "results"
DATASET = RESULTS / "docking-dataset"
SAMPLES = 64
SEED = 20181207


def evaluate_checkpoint(checkpoint: Path, indices: np.ndarray) -> dict:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.policies.smolvla.processor_smolvla import (
        make_smolvla_pre_post_processors,
    )

    from train_policy import make_config

    device = "mps" if torch.backends.mps.is_available() else "cpu"
    dataset = LeRobotDataset(repo_id="macuartin/youbot-docking", root=DATASET)
    config = make_config(dataset.meta, device)

    policy = SmolVLAPolicy.from_pretrained(
        checkpoint, config=config, dataset_stats=dataset.meta.stats
    )
    policy.to(device)
    policy.eval()

    preprocessor, postprocessor = make_smolvla_pre_post_processors(
        config, dataset_stats=dataset.meta.stats
    )

    predicted, actual = [], []
    for i in indices:
        sample = dataset[int(i)]
        observation = {
            "observation.images.front": sample["observation.images.front"][None],
            "observation.state": sample["observation.state"][None],
            "task": [sample["task"]],
        }
        policy.reset()
        with torch.no_grad():
            batch = preprocessor(observation)
            batch = {
                k: (v.to(device) if isinstance(v, torch.Tensor) else v)
                for k, v in batch.items()
            }
            action = postprocessor(policy.select_action(batch))
        predicted.append(action.squeeze().cpu().numpy()[:3])
        truth = sample["action"].numpy()
        actual.append((truth[0] if truth.ndim > 1 else truth)[:3])

    predicted = np.array(predicted)
    actual = np.array(actual)
    errors = np.linalg.norm(predicted - actual, axis=1)
    magnitude = np.linalg.norm(actual, axis=1)

    # Correlacion por componente: mide si la politica sigue la forma de la
    # accion aunque falle la escala. Una correlacion cercana a cero significa
    # que no aprendio nada util, no que aprendio con imprecision.
    correlation = [
        float(np.corrcoef(predicted[:, k], actual[:, k])[0, 1]) for k in range(3)
    ]

    return {
        "checkpoint": checkpoint.name,
        "samples": len(indices),
        "action_magnitude_mean": float(magnitude.mean()),
        "open_loop_error_mean": float(errors.mean()),
        "open_loop_error_max": float(errors.max()),
        "relative_error": float(errors.mean() / magnitude.mean()),
        "correlation_per_axis": correlation,
        "baseline_predict_zero": float(magnitude.mean()),
        "baseline_predict_dataset_mean": float(
            np.linalg.norm(actual - actual.mean(axis=0), axis=1).mean()
        ),
    }


def main() -> None:
    checkpoints = sorted(
        [p for p in RESULTS.glob("smolvla-docking*") if p.is_dir()],
        key=lambda p: (len(p.name), p.name),
    )
    rng = np.random.default_rng(SEED)

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id="macuartin/youbot-docking", root=DATASET)
    indices = rng.choice(dataset.num_frames, SAMPLES, replace=False)

    rows = [evaluate_checkpoint(c, indices) for c in checkpoints]

    print(f"\n{'checkpoint':28s} {'err rel':>9s} {'corr vx':>9s} {'corr vy':>9s} {'corr w':>9s}")
    print("-" * 68)
    for row in rows:
        c = row["correlation_per_axis"]
        print(
            f"{row['checkpoint']:28s} {row['relative_error'] * 100:8.1f}% "
            f"{c[0]:9.3f} {c[1]:9.3f} {c[2]:9.3f}"
        )

    print("\nReferencias de comparacion:")
    print(
        f"  predecir siempre cero:            error relativo 100,0%\n"
        f"  predecir la media del dataset:    error relativo "
        f"{rows[0]['baseline_predict_dataset_mean'] / rows[0]['action_magnitude_mean'] * 100:.1f}%"
    )

    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "open_loop_diagnosis.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(f"\nGuardado en {RESULTS / 'open_loop_diagnosis.json'}")


if __name__ == "__main__":
    main()
