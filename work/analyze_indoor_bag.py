"""Read-only audit of the first AS2 indoor ROS bag on Windows."""
import hashlib
import json
from pathlib import Path
import sqlite3
import struct
import sys
import zipfile

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "runtime/bag_analysis_deps"))
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def summary(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return None
    return dict(count=len(a), minimum=float(a.min()), median=float(np.median(a)),
                mean=float(a.mean()), p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def timing(stamps):
    a = np.asarray(stamps)
    dt = np.diff(a) / 1e9
    return dict(first_ns=int(a[0]), last_ns=int(a[-1]), interval_s=summary(dt),
                non_increasing=int(np.count_nonzero(dt <= 0)),
                span_rate_hz=float((len(a)-1)/(dt.sum())) if dt.sum() > 0 else None)


def stamp(header):
    return int(header.stamp.sec)*1000000000 + int(header.stamp.nanosec)


def main():
    bag = next((ROOT / "field_data/indoor_20261117").rglob("*.db3"))
    out = bag.parent.parent / "analysis"
    out.mkdir(exist_ok=True)
    store = get_typestore(Stores.ROS2_HUMBLE)
    bundle = zipfile.ZipFile(ROOT / "AS2_温室垄沟巡航程序.zip")
    types = {}
    for name in bundle.namelist():
        if "/unitree_go/msg/" in name and name.endswith(".msg"):
            types.update(get_types_from_msg(bundle.read(name).decode("utf8"),
                                           "unitree_go/msg/" + Path(name).stem))
    store.register(types)
    report = dict(bag=str(bag), size_bytes=bag.stat().st_size, topics={})
    with bag.open("rb") as stream:
        report["sha256"] = hashlib.file_digest(stream, "sha256").hexdigest()
    imu_acc, imu_gyro, sport_pos, sport_vel, sport_acc = [], [], [], [], []
    cloud_sizes, cloud_bounds, cloud_good = [], [], []
    video_times, video_sizes = [], []
    video_path = out / "frontvideo720.bin"
    with sqlite3.connect(bag.resolve().as_uri()+"?mode=ro", uri=True) as db, video_path.open("wb") as video:
        report["sqlite_integrity"] = db.execute("PRAGMA quick_check").fetchone()[0]
        topics = db.execute("SELECT id,name,type FROM topics").fetchall()
        for topic_id, name, kind in topics:
            received, sampled, latency = [], [], []
            unparsed_bytes = []
            frames = set()
            first = None
            for t, data in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id", (topic_id,)):
                if name == '/frontvideostream':
                    received.append(t)
                    if data[1] != 1:
                        raise ValueError('Video endianness changed')
                    frame_time, resolution, length = struct.unpack_from('<QII', data, 4)
                    if resolution not in (720, 360) or not 0 <= len(data)-20-length <= 3:
                        raise ValueError('Inferred video layout does not fit all packets')
                    video_times.append(frame_time)
                    video_sizes.append([length, resolution])
                    if resolution == 720:
                        video.write(data[20:20+length])
                    continue
                definition = store.get_msgdef(kind)
                decoder = definition.deserialize_cdr_le if data[1] else definition.deserialize_cdr_be
                msg, used = decoder(data[4:], 0, definition.cls, store)
                unparsed_bytes.append(len(data)-4-used)
                if first is None:
                    first = msg
                received.append(t)
                if hasattr(msg, "header"):
                    h = stamp(msg.header)
                    sampled.append(h)
                    latency.append((t-h)/1e9)
                    frames.add(msg.header.frame_id)
                if name.endswith("imu_livox_mid360"):
                    imu_acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                    imu_gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
                elif kind.endswith("PointCloud2"):
                    fields = {f.name: f for f in msg.fields}
                    dtype = np.dtype(dict(names=["x","y","z"],
                        formats=[">f4" if msg.is_bigendian else "<f4"]*3,
                        offsets=[fields[k].offset for k in ("x","y","z")],itemsize=msg.point_step))
                    raw = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=msg.data,
                                     strides=(msg.row_step,msg.point_step))
                    xyz = np.column_stack([raw[k].ravel() for k in ("x","y","z")])
                    good = xyz[np.isfinite(xyz).all(axis=1)]
                    cloud_sizes.append(len(xyz));cloud_good.append(len(good))
                    if len(good):
                        cloud_bounds.append(np.percentile(good,[1,50,99],axis=0).tolist())
                    if len(cloud_sizes) == 1:
                        np.savez_compressed(out/"first_cloud.npz",xyz=good)
                elif name == "/lf/sportmodestate":
                    sport_pos.append(msg.position);sport_vel.append(msg.velocity)
                    sport_acc.append(msg.imu_state.accelerometer)
                    sampled.append(int(msg.stamp.sec)*1000000000+int(msg.stamp.nanosec))
                elif name == "/frontvideostream":
                    video_times.append(int(msg.time_frame))
                    video_sizes.append([len(msg.video720p),len(msg.video360p),len(msg.video180p)])
                    video.write(msg.video720p.tobytes())
            result = dict(type=kind, messages=len(received), receive_timing=timing(received),
                          sample_timing=timing(sampled) if sampled else None,
                          frames=sorted(frames), receive_minus_header_s=summary(latency))
            result['unparsed_trailing_bytes'] = summary(unparsed_bytes)
            result['schema_size_matches'] = max(unparsed_bytes) <= 3 if unparsed_bytes else False
            if name == '/frontvideostream':
                result['decode_note'] = 'Inferred Q timestamp + I resolution + byte sequence; all packet lengths verified; differs from bundled .msg'
            if kind.endswith("PointCloud2"):
                result["layout"] = dict(width=int(first.width),height=int(first.height),point_step=int(first.point_step),
                    fields=[dict(name=f.name,offset=int(f.offset),datatype=int(f.datatype)) for f in first.fields])
            report["topics"][name]=result
    for label, values in [("imu_acceleration_mps2",imu_acc),("imu_angular_velocity_rps",imu_gyro),
                          ("sport_position_m",sport_pos),("sport_velocity_mps",sport_vel),
                          ("sport_imu_acceleration_mps2",sport_acc)]:
        a=np.asarray(values)
        report[label]=dict(axis_mean=a.mean(axis=0).tolist(),axis_std=a.std(axis=0).tolist(),
                          norm=summary(np.linalg.norm(a,axis=1)))
    pos=np.asarray(sport_pos)
    report["sport_displacement_from_first_m"]=summary(np.linalg.norm(pos-pos[0],axis=1))
    report["point_count_per_scan"]=summary(cloud_sizes)
    report["finite_point_count_per_scan"]=summary(cloud_good)
    report["point_xyz_percentiles_1_50_99_first_scan"]=cloud_bounds[0]
    report["video"]=dict(field_sizes_first=video_sizes[:5],time_frame_first=video_times[:5],
                        time_frame_difference=summary(np.diff(video_times)),output=str(video_path),
                        first_bytes=video_path.read_bytes()[:32].hex())
    (out/"audit.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),"utf8")
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
