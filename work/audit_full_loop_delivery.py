"""Summarize one saved full-route online SLAM run and its database audit."""
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

from audit_full_map_database import evaluate as evaluate_database


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


capture = Path(sys.argv[1])
database = Path(sys.argv[2])
report = json.loads((capture / "场景检查.json").read_text(encoding="utf-8"))
rows = json.loads((capture / "场景静止轨迹.json").read_text(encoding="utf-8"))
graph = evaluate_database(database, capture / "场景静止轨迹.json")
done = next((r for r in reversed(rows) if r["state"] == "DONE"), None)
if done is None:
    raise RuntimeError("No DONE row found in saved trajectory")

errors = []
yaw_errors_deg = []
for row in rows:
    nav = row.get("navigation") or {}
    command = row.get("command") or {}
    moving = any(abs(float(command.get(k, 0.0))) > 1e-5 for k in ("vx", "vy", "omega"))
    error = row.get("slam_vs_reference_error_m")
    if not moving or not nav.get("valid") or error is None:
        continue
    errors.append(float(error))
    yaw_errors_deg.append(abs(math.degrees(wrap(float(nav["yaw"]) - float(row["yaw"])))) )

if not errors:
    raise RuntimeError("No valid moving online SLAM samples found")

start = report["full_map"]["config"]["start_xy_m"]
start_yaw = 0.0
final_nav = done["navigation"]
final_truth_xy = done["position"][:2]
final_nav_xy = final_nav["position"][:2]
control_xy = (final_nav.get("control_position") or final_nav["position"])[:2]
home_truth_m = math.dist(final_truth_xy, start)
home_nav_m = math.dist(final_nav_xy, start)
home_control_m = math.dist(control_xy, start)
truth_heading_error_deg = abs(math.degrees(wrap(float(done["yaw"]) - start_yaw)))
nav_heading_error_deg = abs(math.degrees(wrap(float(final_nav["yaw"]) - start_yaw)))
control_heading_error_deg = abs(math.degrees(wrap(float(final_nav.get("control_yaw", final_nav["yaw"])) - start_yaw)))
coverage = report.get("full_map", {}).get("coverage", {})

online = {
    "moving_samples": len(errors),
    "position_mean_m": statistics.mean(errors),
    "position_p95_m": float(np.percentile(errors, 95)),
    "position_max_m": max(errors),
    "yaw_abs_mean_deg": statistics.mean(yaw_errors_deg),
    "yaw_abs_p95_deg": float(np.percentile(yaw_errors_deg, 95)),
    "yaw_abs_max_deg": max(yaw_errors_deg),
}
home = {
    "truth_position_xy_m": final_truth_xy,
    "truth_error_m": home_truth_m,
    "online_navigation_xy_m": final_nav_xy,
    "online_navigation_error_m": home_nav_m,
    "control_fused_xy_m": control_xy,
    "control_fused_error_m": home_control_m,
    "truth_heading_error_deg": truth_heading_error_deg,
    "online_navigation_heading_error_deg": nav_heading_error_deg,
    "control_fused_heading_error_deg": control_heading_error_deg,
}
criteria = {
    "route_completed_without_fault_or_attention": bool(report["completed"] and not report["fault"] and not report["attention_reason"]),
    "all_selected_aisles_covered": bool(coverage.get("all_selected_aisles_covered")),
    "database_full_slam_verified": bool(graph.get("full_slam_verified")),
    "online_moving_mean_le_2_5_cm": online["position_mean_m"] <= 0.025,
    "online_moving_p95_le_5_cm": online["position_p95_m"] <= 0.05,
    "online_navigation_home_le_3_cm": home_nav_m <= 0.03,
    "truth_home_le_3_cm": home_truth_m <= 0.03,
    "truth_home_heading_le_2_deg": truth_heading_error_deg <= 2.0,
}
result = {
    "scope": "Full density online RGB+LiDAR SLAM and estimated-pose route; four selected aisles 2-5; truth pose used only for independent scoring; simulated sensors, not hardware acceptance",
    "capture": str(capture),
    "database": str(database),
    "completed": report["completed"],
    "fault": report["fault"],
    "attention": report["attention_reason"],
    "simulation_time_s": report["simulation_time_s"],
    "travel_m": report["travel_m"],
    "selected_aisles": report["route23"]["aisles_from_right"],
    "excluded_aisles": report["route23"]["excluded_aisles"],
    "coverage": coverage,
    "online_motion_metrics": online,
    "home_metrics": home,
    "online_global_loop_event_count": report["online_planning"]["global_loop_events"],
    "online_active_loop_events": report["online_planning"]["global_loops_after_travel_and_return"],
    "database_geometry_audit": graph,
    "criteria": criteria,
    "delivery_gate_passed": all(criteria.values()),
    "notes": [
        "Database audit verifies graph connectivity, saved loop constraints, and optimized pose consistency against recorded simulator reference.",
        "Online-navigation and control-fused endpoint estimates are reported separately from simulator truth.",
        "Excluded edge aisles 1 and 6 are not part of the configured crop-route mission.",
    ],
}
out = capture / "full_loop_delivery_acceptance.json"
out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k != "database_geometry_audit"}, ensure_ascii=False, indent=2))
print("DATABASE_GEOMETRY", json.dumps({k: v for k, v in graph.items() if k not in ("structural", "constraints")}, ensure_ascii=False))
