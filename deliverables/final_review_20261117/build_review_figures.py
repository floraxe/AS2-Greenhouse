"""Generate review figures from immutable saved simulation traces."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNS = ["four_aisle_slam_20261116_180100", "four_aisle_slam_20261116_194958"]
fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
for index, name in enumerate(RUNS):
    folder = ROOT / "outputs" / name
    report = json.loads((folder / "full_loop_delivery_acceptance.json").read_text("utf-8"))
    rows = json.loads((folder / "场景静止轨迹.json").read_text("utf-8"))
    done_time = next(row["t"] for row in rows if row["state"] == "DONE")
    rows = [row for row in rows if row["t"] <= done_time]
    truth = np.array([row["position"][:2] for row in rows])
    valid = [row for row in rows if row.get("navigation", {}).get("valid")
             and row.get("navigation_position") is not None]
    nav = np.array([row["navigation_position"][:2] for row in valid])
    trajectory = axes[index, 0]
    trajectory.plot(truth[:, 0], truth[:, 1], label="PhysX reference", linewidth=2)
    trajectory.plot(nav[:, 0], nav[:, 1], label="Online navigation", linewidth=0.9)
    trajectory.scatter([-3], [-0.825], marker="x", color="black", label="Known start")
    trajectory.set(xlabel="Task X (m)", ylabel="Task Y (m)",
                   title=f"Run {index + 1}: complete four-aisle route")
    trajectory.legend(fontsize=8, loc="upper right")
    trajectory.grid(alpha=0.2)

    moving = [row for row in valid if any(abs(float(row["command"].get(key, 0))) > 1e-5
                                         for key in ("vx", "vy", "omega"))]
    time = [row["t"] for row in moving]
    error = [100 * row["slam_vs_reference_error_m"] for row in moving]
    metrics = report["online_motion_metrics"]
    chart = axes[index, 1]
    chart.plot(time, error, linewidth=0.7, label="Moving online position error")
    chart.axhline(5, color="black", linestyle="--", linewidth=1,
                  label="P95 acceptance limit (5 cm)")
    chart.set(xlabel="Simulation time (s)", ylabel="Position error (cm)",
              title=f"Run {index + 1}: mean {100*metrics['position_mean_m']:.2f} cm, "
                    f"P95 {100*metrics['position_p95_m']:.2f} cm")
    chart.legend(fontsize=8, loc="upper left")
    chart.grid(alpha=0.2)
    print(name, "samples", len(moving), "mean_cm", np.mean(error),
          "p95_cm", np.percentile(error, 95))
fig.suptitle("AS2 greenhouse simulation: two independent online runs", fontsize=15)
fig.supxlabel("Source: saved trajectories and database audits, 2026-11-16. "
               "Position scoring uses simulator reference; no trajectory fitting.", fontsize=9)
fig.savefig(HERE / "two_online_runs.png", dpi=180)
plt.close(fig)
