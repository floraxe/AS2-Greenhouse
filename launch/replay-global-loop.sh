#!/usr/bin/env bash
# Run inside the project Ubuntu VM. All outputs go into a fresh directory.
set -e
source /opt/ros/humble/setup.bash
src=${1:-"$HOME/as2_slam/greenhouse_fusion_20261115_031910.db"}
test -s "$src" || { echo "Missing database: $src"; exit 1; }
exe=$(command -v rtabmap-reprocess || find /opt/ros/humble/lib -name rtabmap-reprocess -type f -print -quit)
test -x "$exe" || { echo "Missing rtabmap-reprocess"; exit 1; }
out=$(mktemp -d "$HOME/as2_slam/loop-replay-XXXXXX")
need=$((3 * $(stat -c %s "$src") + 536870912))
free=$(df -PB1 "$out" | awk 'NR==2 {print $4}')
((free >= need)) || { echo "Need $need bytes free for both outputs"; exit 1; }
echo "Output: $out"
trials=(baseline no-spatial)
if [[ ${2:-} == one-path ]]; then
    trials=(one-path-1 one-path-2)
fi
for trial in "${trials[@]}"; do
    options=()
    if [[ $trial == no-spatial ]]; then
        options+=(--RGBD/ProximityBySpace false)
    elif [[ $trial == one-path-* ]]; then
        options+=(--RGBD/ProximityBySpace true --RGBD/ProximityMaxPaths 1)
    fi
    "$exe" --uwarn "${options[@]}" "$src" "$out/$trial.db" >"$out/$trial.log" 2>&1 || { tail -30 "$out/$trial.log"; exit 1; }
    python3 - "$out/$trial.db" <<'PY'
import sqlite3, sys
from pathlib import Path
p = Path(sys.argv[1])
with sqlite3.connect(p.resolve().as_uri() + "?mode=ro", uri=True) as db:
    print(p.name, "nodes", db.execute("SELECT COUNT(*) FROM Node").fetchone()[0],
          "edges", db.execute("SELECT type, COUNT(*) FROM Link GROUP BY type").fetchall())
PY
done
echo "Copy both databases and logs from $out for geometric verification."
