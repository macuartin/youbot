"""Campana de validacion de los objetivos especificos 1, 4 y 5 del trabajo de 2018.

Tres experimentos:

1. Monte Carlo del estacionamiento por servo visual con ruido medido, para
   contrastar contra el criterio de +-10 cm del anteproyecto.
2. Barrido de sensibilidad del agarre al error de estacionamiento, lateral y de
   guinada por separado, para obtener el error maximo tolerable.
3. Cuenca de atraccion: desde donde puede el servo visual siquiera empezar.

    uv run python experiments/parking_campaign.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from youbot import attempt_grasp, parking
from youbot.control import APPROACH_TOLERANCE, pick_point_in_arm_frame

SEED = 20181207
RESULTS = Path(__file__).resolve().parents[1] / "results"

#: Ruido de deteccion medido con experiments/measure_detector_noise.py.
PIXEL_NOISE = 0.5

#: Error relativo de ejecucion de las velocidades de rueda comandadas.
ACTUATION_NOISE = 0.02

#: Criterios numericos del anteproyecto de 2018.
PARKING_TOLERANCE = 0.10
GRASP_SUCCESS_TARGET = 0.95

#: Dispersion de la pose con la que un algoritmo de navegacion deja al robot
#: cerca de la estacion, antes de que entre el servo visual.
APPROACH_SPREAD_LOW = np.array([-0.10, -0.12, -0.10])
APPROACH_SPREAD_HIGH = np.array([0.25, 0.12, 0.10])


def monte_carlo(trials: int = 500) -> dict:
    """Estacionamiento con ruido, y agarre encadenado desde la pose alcanzada."""
    scene = parking.Scene.default()
    rng = np.random.default_rng(SEED)

    position_errors: list[float] = []
    heading_errors: list[float] = []
    iterations: list[int] = []
    grasps = 0
    lost = 0

    for _ in range(trials):
        start = np.asarray(scene.desired_base_pose, dtype=float) + rng.uniform(
            APPROACH_SPREAD_LOW, APPROACH_SPREAD_HIGH
        )
        result = parking.park(
            start,
            scene,
            pixel_noise=PIXEL_NOISE,
            actuation_noise=ACTUATION_NOISE,
            rng=rng,
        )
        if result.lost_marker:
            lost += 1
            continue

        position_errors.append(result.position_error)
        heading_errors.append(result.heading_error)
        iterations.append(result.iterations)
        grasps += int(attempt_grasp(result.final_pose, scene.pick_point).success)

    position = np.array(position_errors)
    heading = np.array(heading_errors)
    completed = position.size

    return {
        "trials": trials,
        "completed": completed,
        "lost_marker": lost,
        "position_error_m": {
            "mean": float(position.mean()),
            "p95": float(np.percentile(position, 95)),
            "max": float(position.max()),
        },
        "heading_error_deg": {
            "mean": float(np.rad2deg(heading.mean())),
            "p95": float(np.rad2deg(np.percentile(heading, 95))),
            "max": float(np.rad2deg(heading.max())),
        },
        "iterations_mean": float(np.mean(iterations)),
        "meets_parking_criterion": bool(position.max() < PARKING_TOLERANCE),
        "grasp_success_rate": grasps / completed,
        "meets_grasp_criterion": bool(grasps / completed >= GRASP_SUCCESS_TARGET),
    }


def sensitivity() -> dict:
    """Como se degrada el agarre al imponer error de estacionamiento."""
    scene = parking.Scene.default()
    reach = float(pick_point_in_arm_frame(scene.desired_base_pose, scene.pick_point)[0])

    lateral = []
    for error in np.linspace(0.0, 0.20, 41):
        result = attempt_grasp(
            np.asarray(scene.desired_base_pose) + [0.0, error, 0.0], scene.pick_point
        )
        lateral.append(
            {
                "error_m": float(error),
                "approach_error_deg": float(np.rad2deg(result.approach_error)),
                "predicted_deg": float(np.rad2deg(np.arctan2(error, reach))),
                "position_residual_mm": float(result.position_residual * 1000),
                "success": bool(result.success),
            }
        )

    yaw = []
    for error in np.deg2rad(np.linspace(0.0, 40.0, 41)):
        result = attempt_grasp(
            np.asarray(scene.desired_base_pose) + [0.0, 0.0, error], scene.pick_point
        )
        yaw.append(
            {
                "error_deg": float(np.rad2deg(error)),
                "approach_error_deg": float(np.rad2deg(result.approach_error)),
                "success": bool(result.success),
            }
        )

    def threshold(rows: list[dict], key: str) -> float | None:
        for row in rows:
            if not row["success"]:
                return row[key]
        return None

    return {
        "reach_m": reach,
        "approach_tolerance_deg": float(np.rad2deg(APPROACH_TOLERANCE)),
        "analytic_lateral_limit_m": reach * float(np.tan(APPROACH_TOLERANCE)),
        "measured_lateral_limit_m": threshold(lateral, "error_m"),
        "measured_yaw_limit_deg": threshold(yaw, "error_deg"),
        "lateral": lateral,
        "yaw": yaw,
    }


def basin_of_attraction(steps: int = 17) -> dict:
    """Desde que poses iniciales el servo visual llega a converger."""
    scene = parking.Scene.default()
    lateral_axis = np.linspace(-0.30, 0.30, steps)
    yaw_axis = np.deg2rad(np.linspace(-30.0, 30.0, steps))

    grid = np.zeros((steps, steps), dtype=int)
    for i, lateral in enumerate(lateral_axis):
        for j, yaw in enumerate(yaw_axis):
            start = np.asarray(scene.desired_base_pose) + [0.0, lateral, yaw]
            result = parking.park(start, scene)
            grid[i, j] = 1 if result.converged else 0

    return {
        "lateral_axis_m": lateral_axis.tolist(),
        "yaw_axis_deg": np.rad2deg(yaw_axis).tolist(),
        "converged": grid.tolist(),
        "converged_fraction": float(grid.mean()),
    }


def plot(campaign: dict, sweep: dict, basin: dict) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))

    ax = axes[0]
    lateral = sweep["lateral"]
    errors = [row["error_m"] * 100 for row in lateral]
    ax.plot(errors, [row["approach_error_deg"] for row in lateral], label="medido")
    ax.plot(
        errors,
        [row["predicted_deg"] for row in lateral],
        "--",
        label="arctan(e / alcance)",
    )
    ax.axhline(
        sweep["approach_tolerance_deg"], color="crimson", ls=":", label="tolerancia pinza"
    )
    ax.axvline(
        sweep["analytic_lateral_limit_m"] * 100,
        color="gray",
        ls=":",
        label="limite lateral",
    )
    ax.axvline(PARKING_TOLERANCE * 100, color="orange", ls="-.", label="criterio 2018")
    ax.set_xlabel("error lateral de estacionamiento (cm)")
    ax.set_ylabel("desalineacion de la pinza (grados)")
    ax.set_title("El agarre no tolera lo que el criterio de 2018 permitia")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    yaw = sweep["yaw"]
    ax.plot(
        [row["error_deg"] for row in yaw],
        [row["approach_error_deg"] for row in yaw],
    )
    ax.axhline(sweep["approach_tolerance_deg"], color="crimson", ls=":")
    ax.set_xlabel("error de guinada al estacionar (grados)")
    ax.set_ylabel("desalineacion de la pinza (grados)")
    ax.set_title("Sensibilidad a la guinada")
    ax.grid(alpha=0.3)

    ax = axes[2]
    grid = np.array(basin["converged"], dtype=float)
    extent = [
        basin["yaw_axis_deg"][0],
        basin["yaw_axis_deg"][-1],
        basin["lateral_axis_m"][0] * 100,
        basin["lateral_axis_m"][-1] * 100,
    ]
    ax.imshow(grid, origin="lower", extent=extent, aspect="auto", cmap="Greens", vmin=0, vmax=1)
    ax.set_xlabel("guinada inicial (grados)")
    ax.set_ylabel("desviacion lateral inicial (cm)")
    ax.set_title(f"Cuenca de atraccion ({basin['converged_fraction']:.0%} converge)")

    fig.tight_layout()
    RESULTS.mkdir(exist_ok=True)
    path = RESULTS / "parking_campaign.png"
    fig.savefig(path, dpi=140)
    return path


def main() -> None:
    print("Monte Carlo del estacionamiento...")
    campaign = monte_carlo()
    print("Barrido de sensibilidad del agarre...")
    sweep = sensitivity()
    print("Cuenca de atraccion...")
    basin = basin_of_attraction()

    pos = campaign["position_error_m"]
    head = campaign["heading_error_deg"]

    print("\n--- Objetivo especifico 5: estacionamiento de precision ---")
    print(f"ensayos: {campaign['trials']}, completados: {campaign['completed']}, "
          f"marcador perdido: {campaign['lost_marker']}")
    print(f"error de posicion: media {pos['mean'] * 1000:.3f} mm, "
          f"p95 {pos['p95'] * 1000:.3f} mm, max {pos['max'] * 1000:.3f} mm")
    print(f"error de guinada:  media {head['mean']:.4f} deg, max {head['max']:.4f} deg")
    print(f"criterio +-10 cm: {'CUMPLE' if campaign['meets_parking_criterion'] else 'NO CUMPLE'}")

    print("\n--- Objetivo especifico 4: agarre ---")
    print(f"exito de agarre: {campaign['grasp_success_rate']:.1%}")
    print(f"criterio 95%: {'CUMPLE' if campaign['meets_grasp_criterion'] else 'NO CUMPLE'}")

    print("\n--- Sensibilidad ---")
    print(f"alcance horizontal del brazo: {sweep['reach_m'] * 100:.2f} cm")
    print(f"limite lateral analitico:  {sweep['analytic_lateral_limit_m'] * 100:.2f} cm")
    print(f"limite lateral medido:     {sweep['measured_lateral_limit_m'] * 100:.2f} cm")
    print(f"limite de guinada medido:  {sweep['measured_yaw_limit_deg']:.2f} deg")
    print(
        f"\nEl criterio de +-10 cm del anteproyecto es {PARKING_TOLERANCE / sweep['analytic_lateral_limit_m']:.1f}"
        " veces mas laxo que lo que la tarea de pick & place admite en lateral."
    )

    RESULTS.mkdir(exist_ok=True)
    payload = {"monte_carlo": campaign, "sensitivity": sweep, "basin": basin}
    out = RESULTS / "parking_campaign.json"
    out.write_text(json.dumps(payload, indent=2) + "\n")
    figure = plot(campaign, sweep, basin)
    print(f"\nGuardado en {out} y {figure}")


if __name__ == "__main__":
    main()
