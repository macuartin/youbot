"""Afina SmolVLA sobre las demostraciones del controlador clasico.

Es la destilacion: el experto de las Fases 1 y 2 genero las demostraciones y aqui
una politica aprendida de 450M de parametros intenta reproducirlo. La pregunta no
es si el VLA generaliza mejor, que este banco no lo prueba, sino **cuanta
precision se pierde al sustituir una ley de control analitica por una red**, con
maestro y alumno viendo exactamente los mismos pixeles.

    uv run --group vla python experiments/train_policy.py --steps 3000

El benchmark de velocidad es lo primero que conviene correr, porque decide si el
entrenamiento cabe en local o hay que alquilar GPU:

    uv run --group vla python experiments/train_policy.py --benchmark
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import torch

RESULTS = Path(__file__).resolve().parents[1] / "results"
DATASET = RESULTS / "docking-dataset"
CHECKPOINT = RESULTS / "smolvla-docking"

#: Modelo base del que se parte. SmolVLA es el unico de la familia que la
#: documentacion declara ejecutable en hardware de consumo.
BASE_MODEL = "lerobot/smolvla_base"


def pick_device(requested: str | None = None) -> torch.device:
    if requested:
        return torch.device(requested)
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def build(device: torch.device, batch_size: int):
    """Carga el dataset y la politica, ya emparejados."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import dataset_to_policy_features
    from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from lerobot.policies.smolvla.processor_smolvla import (
        make_smolvla_pre_post_processors,
    )
    from lerobot.configs.types import FeatureType

    metadata_only = LeRobotDataset(repo_id="macuartin/youbot-docking", root=DATASET)
    features = dataset_to_policy_features(metadata_only.meta.features)
    output_features = {k: v for k, v in features.items() if v.type is FeatureType.ACTION}
    input_features = {k: v for k, v in features.items() if k not in output_features}

    config = SmolVLAConfig(
        input_features=input_features,
        output_features=output_features,
        device=str(device),
    )

    # El dataset tiene que servir un chunk de acciones por muestra, no una sola.
    delta_timestamps = {
        "action": [i / metadata_only.fps for i in range(config.chunk_size)]
    }
    dataset = LeRobotDataset(
        repo_id="macuartin/youbot-docking",
        root=DATASET,
        delta_timestamps=delta_timestamps,
    )

    policy = SmolVLAPolicy.from_pretrained(
        BASE_MODEL, config=config, dataset_stats=dataset.meta.stats
    )
    policy.to(device)
    policy.train()

    # El preprocesador es el que tokeniza la instruccion en lenguaje natural y
    # normaliza estado y accion con las estadisticas del dataset. Sin el, la
    # politica no recibe observation.language.tokens y no puede correr.
    preprocessor, postprocessor = make_smolvla_pre_post_processors(
        config, dataset_stats=dataset.meta.stats
    )

    loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=0,
        drop_last=True,
    )
    return dataset, policy, loader, preprocessor, postprocessor


def run(
    steps: int,
    batch_size: int,
    learning_rate: float,
    device: torch.device,
    benchmark: bool,
    checkpoint_every: int = 500,
) -> dict:
    dataset, policy, loader, preprocessor, _ = build(device, batch_size)
    print(
        f"dataset: {dataset.num_episodes} episodios, {dataset.num_frames} fotogramas"
    )
    trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"politica: {trainable / 1e6:.1f}M parametros entrenables en {device}")

    optimizer = torch.optim.AdamW(policy.parameters(), lr=learning_rate)

    losses: list[float] = []
    durations: list[float] = []
    step = 0
    start = time.time()

    while step < steps:
        for batch in loader:
            batch = preprocessor(batch)
            batch = {
                k: (v.to(device) if isinstance(v, torch.Tensor) else v)
                for k, v in batch.items()
            }
            tic = time.time()
            loss, _ = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()
            if device.type == "mps":
                torch.mps.synchronize()
            durations.append(time.time() - tic)

            losses.append(float(loss.detach()))
            step += 1

            if step % 10 == 0 or step == 1:
                recent = sum(durations[-10:]) / len(durations[-10:])
                print(
                    f"  paso {step:5d}/{steps} | loss {sum(losses[-10:]) / len(losses[-10:]):.4f} "
                    f"| {recent:.2f} s/paso"
                )
            if benchmark and step >= 20:
                break
            if checkpoint_every and step % checkpoint_every == 0:
                CHECKPOINT.mkdir(parents=True, exist_ok=True)
                policy.save_pretrained(CHECKPOINT)
                print(f"    checkpoint guardado en el paso {step}")
            if step >= steps:
                break
        if benchmark and step >= 20:
            break

    elapsed = time.time() - start
    seconds_per_step = sum(durations) / len(durations)

    report = {
        "device": str(device),
        "batch_size": batch_size,
        "steps_run": step,
        "seconds_per_step": seconds_per_step,
        "elapsed_s": elapsed,
        "final_loss_mean_last10": sum(losses[-10:]) / len(losses[-10:]),
        "trainable_parameters_m": trainable / 1e6,
        "projected_hours_for_20k_steps": seconds_per_step * 20000 / 3600,
    }

    if benchmark:
        print("\n--- benchmark ---")
        print(f"{seconds_per_step:.2f} s/paso con batch {batch_size} en {device}")
        print(
            f"proyeccion para 20.000 pasos: "
            f"{report['projected_hours_for_20k_steps']:.1f} horas"
        )
        RESULTS.mkdir(exist_ok=True)
        (RESULTS / "train_benchmark.json").write_text(json.dumps(report, indent=2) + "\n")
        return report

    CHECKPOINT.mkdir(parents=True, exist_ok=True)
    policy.save_pretrained(CHECKPOINT)
    (CHECKPOINT / "training_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nCheckpoint en {CHECKPOINT}")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--checkpoint-every", type=int, default=500)
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="corre 20 pasos y proyecta cuanto costaria un entrenamiento completo",
    )
    args = parser.parse_args()

    run(
        args.steps,
        args.batch_size,
        args.learning_rate,
        pick_device(args.device),
        args.benchmark,
        args.checkpoint_every,
    )


if __name__ == "__main__":
    main()
