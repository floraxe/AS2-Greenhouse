#!/usr/bin/env bash
# Launch the installed online pipeline with the validated replay setting.
set -eu
dir="$HOME/as2_slam/precision"
src="$dir/start_precision_live.sh"
old='    RGBD/ProximityMaxPaths: "2"'
if [[ $(grep -Fxc "$old" "$src") != 1 ]]; then
    echo "Expected exactly one ProximityMaxPaths=2 setting in $src; inspect installed script."
    exit 1
fi
trial=$(mktemp "$dir/start_one_path_XXXXXX.sh")
sed 's/^    RGBD\/ProximityMaxPaths: "2"$/    RGBD\/ProximityMaxPaths: "1"/' "$src" > "$trial"
bash -n "$trial"
echo "Online one-path trial: $trial"
exec bash "$trial"
