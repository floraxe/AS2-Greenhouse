from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = json.loads(Path(__file__).with_name("manifest.json").read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


required = [
    "work/run_full_map_greenhouse.py",
    "work/as2_ros_minimal.kit",
    "work/slam_navigation_alignment.json",
    "work/full_map_near_field_memory.py",
    "work/full_map_progress_monitor.py",
    "work/full_map_row_tracking.py",
    "work/full_map_visual_foliage.py",
    "work/start_fusion_slam.sh",
    "launch/four-aisle-slam.bat",
    "launch/two-aisle-slam.bat",
    "launch/slam-screen/live_slam_screen.py"
]
for relative in required:
    assert (ROOT / relative).is_file(), f"missing required file: {relative}"

for source in (ROOT / "work").glob("*.py"):
    ast.parse(source.read_text(encoding="utf-8-sig"), filename=str(source))

for run in MANIFEST["runs"]:
    folder = ROOT / run["folder"]
    database = folder / run["database"]
    report = folder / "full_loop_delivery_acceptance.json"
    assert sha256(database) == run["database_sha256"], f"database hash mismatch: {database}"
    assert sha256(report) == run["acceptance_sha256"], f"acceptance hash mismatch: {report}"
    result = json.loads(report.read_text(encoding="utf-8"))
    assert result["completed"] is True, f"mission incomplete: {folder.name}"
    assert result["delivery_gate_passed"] is True, f"acceptance failed: {folder.name}"
    assert set(result["selected_aisles"]) == {2, 3, 4, 5}, f"wrong route scope: {folder.name}"
    assert result["database_geometry_audit"]["full_slam_verified"] is True

capture = ROOT / MANIFEST["hardware_capture"]["path"]
assert sha256(capture) == MANIFEST["hardware_capture"]["sha256"], "hardware capture hash mismatch"
archive = ROOT / MANIFEST["source_archive"]
assert sha256(archive) == MANIFEST["source_archive_sha256"], "source archive hash mismatch"

print("RELEASE_VERIFY_OK")
print(f"accepted_runs={len(MANIFEST['runs'])}; python_sources={len(list((ROOT / 'work').glob('*.py')))}")

