#!/usr/bin/env bash
# Receive-only check of the installed physical ridge detector and state feedback.
set -e
root="$HOME/as2_field/patrol_ascii"
iface=${1:-ens37}
cd "$root"
source /opt/ros/humble/setup.bash
source unitree_msgs_ws/install/setup.bash
mkdir -p runs
dds="$root/runs/preflight_dds.xml"
[[ -d /sys/class/net/"$iface" && "$iface" =~ ^[[:alnum:]_.:-]+$ ]] || exit 1
sed "s/name=\"eno1\"/name=\"$iface\"/" config/as2_eno1.xml > "$dds"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI="file://$dds"
out=$(mktemp -d "$root/runs/preflight-XXXXXX")
# ridge_patrol.py creates its output directory itself.
echo "Receive-only detector check. Output: $out/check"
exec /usr/bin/python3 src/ridge_patrol.py --output "$out/check" --max-seconds 30
