#!/usr/bin/env bash
# Run from the extracted patrol bundle. Receives ROS data only.
set -e
iface=${1:?Usage: bash indoor-record.sh ROBOT_INTERFACE}
[[ -e /sys/class/net/"$iface" && "$iface" =~ ^[[:alnum:]_.:-]+$ ]] || { echo 'Invalid interface'; exit 1; }
source /opt/ros/humble/setup.bash
source unitree_msgs_ws/install/setup.bash
mkdir -p runs
dir=$(mktemp -d "$PWD/runs/indoor-$(date +%Y%m%d-%H%M%S)-XXXX")
sed "s/name=\"eno1\"/name=\"$iface\"/" config/as2_eno1.xml > "$dir/dds.xml"
export CYCLONEDDS_URI="file://$dir/dds.xml" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ip -br addr show "$iface" | tee "$dir/interface.txt"
ros2 topic list --no-daemon --spin-time 5 -t | tee "$dir/topics.txt"
df -h "$dir"
echo 'Enter space-separated sensor topic names from this list (empty to exit):'
read -r -a topics
(( ${#topics[@]} )) || exit 0
for t in "${topics[@]}"; do
    awk -v t="$t" '$1==t {found=1} END {exit !found}' "$dir/topics.txt" || { echo "Unknown topic: $t"; exit 1; }
done
echo "Saving $dir/bag ; Ctrl+C to finish"
ros2 bag record -o "$dir/bag" "${topics[@]}"
