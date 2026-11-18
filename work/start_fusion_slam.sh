#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
as2_icp_voxel_size="${AS2_ICP_VOXEL_SIZE:-0.02}"
case "$as2_icp_voxel_size" in
  0.01|0.02) ;;
  *) echo "Unsupported ICP voxel test value: $as2_icp_voxel_size"; exit 1 ;;
esac
mkdir -p "$HOME/as2_slam"
cd "$HOME/as2_slam"
python3 - <<'AS2_SPACE'
import shutil
from pathlib import Path
p=Path.home()/'as2_slam'
free=shutil.disk_usage(p).free
print(f'SLAM storage preflight: {free/1024**3:.3f} GiB free',flush=True)
if free<2*1024**3:
    raise SystemExit('At least 2 GiB free required before starting a new map. Preserve/archive existing databases first.')
AS2_SPACE
for as2_pkg in rtabmap_slam rtabmap_odom imu_filter_madgwick; do
  ros2 pkg prefix "$as2_pkg" >/dev/null || { echo "Missing ROS package: $as2_pkg"; exit 1; }
done
as2_existing=$(ros2 node list 2>/dev/null || true)
if printf '%s\n' "$as2_existing" | grep -Eq '^/(icp_odometry|imu_filter_madgwick|as2/slam/rtabmap|as2_rgb_sync|as2_fusion_state_bridge)$'; then
  # A disconnected helper can leave its new-session pipeline alive. Stop
  # only another same-user instance of this exact project startup script;
  # its existing SIGINT trap saves the database and stops its own children.
  python3 - <<'PY'
import os,signal,time
from pathlib import Path
script=str(Path.home()/'as2_slam/precision/start_precision_live.sh')
targets=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    pid=int(entry.name)
    if pid in (os.getpid(),os.getppid()):continue
    try:
        if entry.stat().st_uid!=os.getuid():continue
        argv=(entry/'cmdline').read_bytes().split(b'\0')
        if script.encode() not in argv or not argv or Path(argv[0].decode()).name!='bash':continue
        print('Saving stale owned AS2 pipeline:',pid,flush=True)
        os.killpg(pid,signal.SIGINT) if os.getpgid(pid)==pid else os.kill(pid,signal.SIGINT)
        targets.append(pid)
    except (FileNotFoundError,ProcessLookupError,PermissionError):pass
end=time.monotonic()+45
while targets and time.monotonic()<end:
    targets=[p for p in targets if Path('/proc',str(p)).exists()]
    if targets:time.sleep(.2)
if targets:raise SystemExit('Old AS2 pipeline has not exited; refusing parallel ROS startup')
PY
  as2_existing=$(ros2 node list 2>/dev/null || true)
fi
if printf '%s\n' "$as2_existing" | grep -Eq '^/(icp_odometry|imu_filter_madgwick|as2/slam/rtabmap|as2_rgb_sync|as2_fusion_state_bridge)$'; then
  echo "Old ROS programs still running. Ctrl+C their startup terminal, then retry."
  exit 1
fi
# Fresh database every run. Never erase or append to the accepted old map.
as2_database="$HOME/as2_slam/greenhouse_fusion_$(date +%Y%m%d_%H%M%S).db"
cat > fusion_odom.yaml <<'YAML'
/**:
  ros__parameters:
    use_sim_time: true
    frame_id: base_stabilized
    odom_frame_id: odom
    publish_tf: true
    wait_imu_to_init: true
    # Identity initialization avoids the previously observed stale-guess loss.
    Odom/GuessMotion: "false"
    # Reinitialize from the last valid pose after one failed ICP update instead
    # of remaining permanently lost with ratio=0.
    Odom/ResetCountdown: "1"
    # Keep the validated local-map density; denser maps regressed replay.
    # Raw lidar sampling density is unchanged; feature context is measured.
    OdomF2M/ScanSubtractRadius: "0.05"
    OdomF2M/ScanMaxSize: "15000"
    qos: 2
    qos_imu: 2
    deskewing: false
    Reg/Force3DoF: "true"
    Icp/PointToPlane: "true"
    Icp/PointToPlaneK: "10"
    Icp/VoxelSize: "0.02"
    Icp/OutlierRatio: "0.70"
    Icp/MaxCorrespondenceDistance: "0.3"
YAML
sed -i "s/^    Icp\/VoxelSize: \"0.02\"$/    Icp\/VoxelSize: \"$as2_icp_voxel_size\"/" fusion_odom.yaml
echo "ICP odometry profile: Icp/VoxelSize=$as2_icp_voxel_size; default remains 0.02 m."
cat > fusion_map.yaml <<'YAML'
/**:
  ros__parameters:
    use_sim_time: true
    frame_id: base_stabilized
    map_frame_id: map
    qos: 2
    qos_odom: 2
    qos_image: 2
    qos_camera_info: 2
    approx_sync: true
    approx_sync_max_interval: 0.05
    topic_queue_size: 20
    sync_queue_size: 20
    subscribe_rgb: true
    subscribe_depth: false
    subscribe_rgbd: false
    subscribe_scan_cloud: true
    subscribe_odom_info: false
    gen_depth: true
    gen_depth_decimation: 2
    gen_depth_fill_holes_size: 1
    pub_loc_pose_only_when_localizing: false
    latch: false
    map_always_update: true
    Mem/IncrementalMemory: "true"
    Mem/InitWMWithAllNodes: "false"
    Mem/SaveDepth16Format: "true"
    Mem/DepthAsMask: "false"
    Vis/DepthAsMask: "false"
    Mem/UseOdomFeatures: "false"
    Mem/STMSize: "10"
    Kp/MaxFeatures: "600"
    Reg/Strategy: "2"
    Reg/Force3DoF: "true"
    Rtabmap/DetectionRate: "1"
    Rtabmap/PublishStats: "true"
    RGBD/OptimizeFromGraphEnd: "false"
    RGBD/ProximityBySpace: "true"
    RGBD/ProximityOdomGuess: "true"
    RGBD/ProximityMaxGraphDepth: "0"
    RGBD/ProximityPathMaxNeighbors: "5"
    RGBD/ProximityMaxPaths: "2"
    RGBD/ProximityAngle: "180"
    RGBD/ProximityPathFilteringRadius: "0.3"
    RGBD/LocalRadius: "3"
    RGBD/LinearUpdate: "0"
    RGBD/AngularUpdate: "0"
    Icp/PointToPlane: "true"
    Icp/PointToPlaneK: "10"
    Icp/VoxelSize: "0.02"
    Icp/MaxCorrespondenceDistance: "0.3"
    Icp/MaxTranslation: "0.10"
    Grid/Sensor: "0"
    Grid/3D: "false"
    Grid/CellSize: "0.05"
    Grid/RangeMax: "2"
    Grid/RayTracing: "true"
    Grid/NormalsSegmentation: "false"
    Grid/MaxGroundHeight: "0.08"
    Grid/MaxObstacleHeight: "0.6"
    Grid/MinClusterSize: "3"
YAML
cat > fusion_state_bridge.py <<'AS2_BRIDGE'
"""Ubuntu bridge: real RTAB-Map graph and feature statistics to standard ROS types.
No commands, no invented localization covariance, no simulator pose input.
"""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from rtabmap_msgs.msg import Info, MapGraph


class Bridge(Node):
    def __init__(self):
        super().__init__('as2_fusion_state_bridge')
        self.correction = self.create_publisher(TransformStamped, '/as2/slam/map_correction', 10)
        self.state = self.create_publisher(String, '/as2/slam/online_state', 10)
        self.graphs = 0
        self.global_events=set()
        self.proximity_events=set()
        self.create_subscription(MapGraph, '/as2/slam/mapGraph', self.graph, qos_profile_sensor_data)
        self.create_subscription(Info, '/as2/slam/info', self.info, qos_profile_sensor_data)

    def graph(self, msg):
        if msg.header.frame_id != 'map' or not msg.poses_id:
            return
        self.graphs += 1
        out = TransformStamped()
        out.header = msg.header
        out.child_frame_id = 'odom'
        out.transform = msg.map_to_odom
        self.correction.publish(out)

    def info(self, msg):
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        if msg.loop_closure_id>0:self.global_events.add((msg.ref_id,msg.loop_closure_id))
        if msg.proximity_detection_id>0:self.proximity_events.add((msg.ref_id,msg.proximity_detection_id))
        data = dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                    ref_id=msg.ref_id, loop_closure_id=msg.loop_closure_id,
                    proximity_detection_id=msg.proximity_detection_id, graphs=self.graphs,
                    global_loop_events=len(self.global_events),proximity_events=len(self.proximity_events),
                    visual_words=int(stats.get('Keypoint/Current_frame/words', 0)),
                    dictionary_words=int(stats.get('Keypoint/Dictionary_size/words', 0)))
        out = String()
        out.data = json.dumps(data)
        self.state.publish(out)
        if self.graphs % 10 == 0 or msg.loop_closure_id > 0:
            self.get_logger().info('FUSION_STATUS '+out.data)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
AS2_BRIDGE
as2_groups=()
cat > audit_rtabmap_database.py <<'AS2_AUDIT'
"""Read-only DB structural/temporal audit; not proof of geometric loop correctness.
RTAB-Map DBDriverSqlite3 stores Transform as 12 native floats (3x4).
Node.pose is odometry pose, NOT optimized graph pose.
"""
import json,sqlite3,struct,math,sys
from pathlib import Path
from contextlib import closing

def pose(blob):
    if not blob or len(blob)!=48:return None
    a=struct.unpack('<12f',blob)
    if not all(math.isfinite(v) for v in a):return None
    r=[a[0:3],a[4:7],a[8:11]]
    err=max(abs(sum(r[k][i]*r[k][j] for k in range(3))-(i==j)) for i in range(3) for j in range(3))
    det=r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0])
    if err>.01 or abs(det-1)>.01:return None
    return (a[3],a[7],a[11],math.atan2(a[4],a[0]))

def audit(path):
    if not path.exists():raise FileNotFoundError(path)
    with closing(sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        integrity=db.execute('PRAGMA quick_check').fetchone()[0]
        nodes={i:dict(stamp=t,pose=pose(p),map_id=m) for i,t,p,m in db.execute('SELECT id,stamp,pose,map_id FROM Node')}
        links=list(db.execute('SELECT from_id,to_id,type,transform FROM Link'))
    kinds={};unique={};bad=[];adj={i:set() for i in nodes}
    for a,b,k,tr in links:
        kinds[k]=kinds.get(k,0)+1
        unique.setdefault((min(a,b),max(a,b),k),(a,b,tr))
        if a not in nodes or b not in nodes or a==b:bad.append([a,b,k]);continue
        adj[a].add(b);adj[b].add(a)
    reached=set();todo=[min(nodes)] if nodes else []
    while todo:
        i=todo.pop()
        if i in reached:continue
        reached.add(i);todo.extend(adj[i]-reached)
    # Travel separation derives only from raw Node odometry and stays diagnostic.
    prefix={};travel=0.;prev=None
    for i,n in sorted(nodes.items(),key=lambda it:it[1]['stamp']):
        if prev and n['pose'] and prev['map_id']==n['map_id'] and prev['pose']:
            travel+=math.dist(n['pose'][:2],prev['pose'][:2])
        prefix[i]=travel;prev=n
    events=[];invalid=[]
    for (lo,hi,k),(a,b,tr) in unique.items():
        if k not in (0,1,2,3):continue
        if pose(tr) is None:invalid.append([a,b,k])
        if k!=1 or a not in nodes or b not in nodes:continue
        age=abs(nodes[a]['stamp']-nodes[b]['stamp']);distance=abs(prefix[a]-prefix[b])
        events.append(dict(nodes=[a,b],stamp_a=nodes[a]['stamp'],stamp_b=nodes[b]['stamp'],age_s=age,raw_odometry_travel_separation_m=distance,long_baseline_candidate=bool(age>=30 and distance>=5),transform_valid=pose(tr) is not None))
    return dict(database=str(path),integrity=integrity,node_count=len(nodes),directed_link_counts=kinds,unique_link_counts={str(k):sum(key[2]==k for key in unique) for k in kinds},connected_nodes=len(reached),invalid_endpoints=bad,invalid_transforms=invalid,invalid_node_poses=[i for i,n in nodes.items() if n['pose'] is None],global_events=events,structural_checks_passed=bool(nodes and integrity=='ok' and len(reached)==len(nodes) and not bad and not invalid and all(n['pose'] for n in nodes.values())),geometric_loop_accuracy_verified=False,limits=['Node.pose is raw odometry; not optimized ground-truth error','Long temporal/travel separation is necessary evidence, not proof of a correct loop','Match trajectory event time to active mission; reject post-DONE repeats'])

if __name__=='__main__':
    p=Path(sys.argv[1]).expanduser()
    result=audit(p)
    output=p.with_suffix('.audit.json');output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('DATABASE_AUDIT '+json.dumps({k:result[k] for k in ('node_count','unique_link_counts','structural_checks_passed','geometric_loop_accuracy_verified')}));print('Long-baseline global candidates: '+str(sum(e['long_baseline_candidate'] for e in result['global_events'])));print('Saved audit: '+str(output))
AS2_AUDIT
as2_cleanup() {
  trap - EXIT INT TERM
  for as2_group in "${as2_groups[@]}"; do kill -TERM -- "-$as2_group" 2>/dev/null || true; done
  wait 2>/dev/null || true
  if [ -s "$as2_database" ]; then python3 audit_rtabmap_database.py "$as2_database" || echo "DB audit incomplete; inspect database manually."; fi
}
trap as2_cleanup EXIT
trap 'exit 0' INT TERM
setsid ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_sim_time:=true -p use_mag:=false -p publish_tf:=false -p world_frame:=enu -p remove_gravity_vector:=false -r imu/data_raw:=/as2/imu/data -r imu/data:=/as2/imu/filtered > fusion_imu.log 2>&1 &
as2_groups+=("$!")
setsid ros2 run rtabmap_odom icp_odometry --ros-args --params-file fusion_odom.yaml -r scan_cloud:=/as2/lidar/odometry_points -r imu:=/as2/imu/filtered -r odom:=/as2/odom > fusion_odom.log 2>&1 &
as2_groups+=("$!")
# Mapping needs measured ground returns and longer-range visual depth. Only ICP
# odometry uses the near-field, floor-separated structural subset.
setsid ros2 run rtabmap_slam rtabmap --ros-args --params-file fusion_map.yaml -r __ns:=/as2/slam -p database_path:="$as2_database" -r scan_cloud:=/as2/lidar/points -r odom:=/as2/odom -r rgb/image:=/as2/camera/image_raw -r rgb/camera_info:=/as2/camera/camera_info > fusion_map.log 2>&1 &
as2_groups+=("$!")
setsid python3 fusion_state_bridge.py --ros-args -p use_sim_time:=true > fusion_state.log 2>&1 &
as2_groups+=("$!")
printf '\nONLINE RGB + LIDAR MAPPING STARTED\nNew map: %s\n' "$as2_database"
echo 'PRECISION CANDIDATE: measured-floor base_stabilized; Windows requires --level-odometry. No simulator pose used for stabilization.'
echo 'ICP RECOVERY: identity initialization + automatic reset after one failed registration.'
echo 'Windows ROUTE23: fixed aisle 2 outbound -> far transfer -> aisle 3 inbound -> entrance closure. No A*. Keep this terminal open.'
echo 'Ctrl+C saves the map and stops only the four programs started here.'
if wait -n "${as2_groups[@]}"; then
  echo 'A ROS program exited; stopping the group.'
else
  echo 'Startup failed. Recent logs:'
  tail -n 8 fusion_imu.log fusion_odom.log fusion_map.log fusion_state.log
  exit 1
fi
