"""Read-only ROS 2 display for an Isaac Sim / SLAM recording."""
import argparse
import math
import time

import numpy as np
from PIL import Image, ImageDraw


def cloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields and fields[name].datatype == 7 for name in ("x", "y", "z")):
        raise ValueError("Expected FLOAT32 x/y/z")
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": [">f4" if msg.is_bigendian else "<f4"] * 3,
                      "offsets": [fields[name].offset for name in ("x", "y", "z")],
                      "itemsize": msg.point_step})
    values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([values[name].ravel() for name in ("x", "y", "z")])
    return xyz[np.isfinite(xyz).all(axis=1)]


def lidar_image(points, size=(440, 248)):
    """Forward perspective at the lidar origin, X forward / Y left / Z up."""
    width, height = size
    image = Image.new("RGB", size, (20, 25, 31))
    pitch = math.radians(10)
    x, y, z = points.T
    depth = x * math.cos(pitch) - z * math.sin(pitch)
    up = x * math.sin(pitch) + z * math.cos(pitch)
    good = (depth > 0.15) & (depth < 9)
    depth, up, side, raw_z = depth[good], up[good], y[good], z[good]
    focal = width / (2 * math.tan(math.radians(50)))
    u = (width / 2 - focal * side / depth).astype(int)
    v = (height / 2 - focal * up / depth).astype(int)
    good = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, raw_z = u[good], v[good], raw_z[good]
    pixels = np.asarray(image).copy()
    shade = np.clip((raw_z + 0.65) / 1.7, 0, 1)
    colors = np.column_stack((40 + 210 * shade, 210 - 110 * shade,
                              235 - 185 * shade)).astype(np.uint8)
    # 2x2 pixels improve readability; all received points remain available.
    for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
        pixels[np.minimum(v + dv, height - 1), np.minimum(u + du, width - 1)] = colors
    return Image.fromarray(pixels)


def fit(image, size):
    copy = image.copy()
    copy.thumbnail(size)
    canvas = Image.new("RGB", size, (28, 30, 34))
    canvas.paste(copy, ((size[0] - copy.width) // 2, (size[1] - copy.height) // 2))
    return canvas


def map_image(msg, graph, robot, size=(896, 410)):
    width, height = msg.info.width, msg.info.height
    if width * height != len(msg.data) or not width or not height:
        raise ValueError("Invalid occupancy dimensions")
    cells = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
    pixels = np.full((height, width, 3), 80, dtype=np.uint8)
    pixels[cells == 0] = (230, 232, 233)
    pixels[cells > 0] = (20, 22, 25)
    transpose = getattr(Image, "Transpose", Image)
    image = Image.fromarray(pixels).transpose(transpose.FLIP_TOP_BOTTOM)
    draw = ImageDraw.Draw(image)
    origin = msg.info.origin
    q = origin.orientation
    angle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    c, s = math.cos(angle), math.sin(angle)

    def pixel(x, y):
        dx, dy = x - origin.position.x, y - origin.position.y
        return ((c*dx + s*dy) / msg.info.resolution,
                height - 1 - (-s*dx + c*dy) / msg.info.resolution)

    if len(graph) > 1:
        draw.line([pixel(x, y) for x, y in graph], fill=(30, 145, 225), width=2)
    if robot:
        x, y, yaw = robot
        px, py = pixel(x, y)
        end = pixel(x + 0.8*math.cos(yaw), y + 0.8*math.sin(yaw))
        draw.ellipse((px-5, py-5, px+5, py+5), fill=(240, 70, 50))
        draw.line([(px, py), end], fill=(240, 70, 50), width=3)
    # Fit the visited route rather than the entire grid allocated by SLAM.
    # Keep square map cells and allow unknown space outside grid boundaries.
    anchors = [pixel(x, y) for x, y in graph]
    if robot:
        anchors.append(pixel(robot[0], robot[1]))
    if not anchors:
        known_y, known_x = np.nonzero(cells >= 0)
        if len(known_x):
            anchors = [(float(known_x.min()), float(height-1-known_y.max())),
                       (float(known_x.max()), float(height-1-known_y.min()))]
        else:
            anchors = [(width/2, height/2)]
    bounds = np.asarray(anchors)
    low, high = bounds.min(axis=0), bounds.max(axis=0)
    padding = 1.5 / msg.info.resolution
    view_width = max(6 / msg.info.resolution, high[0]-low[0]+2*padding)
    view_height = max(3 / msg.info.resolution, high[1]-low[1]+2*padding)
    ratio = size[0] / size[1]
    view_width = max(view_width, view_height*ratio)
    view_height = view_width / ratio
    center = (low + high) / 2
    left, top = np.floor(center - [view_width/2, view_height/2]).astype(int)
    viewport = Image.new("RGB", (math.ceil(view_width), math.ceil(view_height)), (80, 80, 80))
    viewport.paste(image, (-int(left), -int(top)))
    sampling = getattr(Image, "Resampling", Image)
    return viewport.resize(size, sampling.NEAREST)


def self_test():
    from types import SimpleNamespace as S
    fields = [S(name=n, datatype=7, offset=i*4) for i, n in enumerate(("x", "y", "z"))]
    p = np.array([[2, 0, 0], [3, 0.5, -0.4], [1, -0.2, 0.2]], dtype="<f4")
    msg = S(fields=fields, point_step=12, row_step=36, height=1, width=3,
            is_bigendian=False, data=p.tobytes())
    assert np.allclose(cloud_xyz(msg), p)
    assert lidar_image(p).size == (440, 248)
    q = S(x=0, y=0, z=0, w=1)
    m = S(info=S(width=50, height=30, resolution=0.1,
                 origin=S(position=S(x=0, y=0), orientation=q)), data=[0]*1500)
    assert map_image(m, [(0.5, 1), (2, 1)], (2, 1, 0)).size == (896, 410)
    print("Rendering / PointCloud2 decoding checks passed")


def main():
    import tkinter as tk
    from PIL import ImageTk
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image as RosImage, PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from rtabmap_msgs.msg import MapGraph
    from tf2_ros import Buffer, TransformListener, TransformException
    import json

    rclpy.init()
    node = rclpy.create_node("as2_recording_screen")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    window = tk.Tk()
    window.title("AS2 SLAM Live Screen")
    window.configure(bg="#1c1e22")
    window.geometry("920x790")
    window.minsize(650, 570)
    state = {"map": None, "rgb": None, "cloud": None, "graph": [],
             "sim": None, "loops": 0}
    stamps = {}
    photos = {}

    header = tk.Label(window, text="Waiting for ROS data", bg="#1c1e22",
                      fg="white", font=("DejaVu Sans", 13))
    header.pack(fill="x", padx=8, pady=5)
    map_label = tk.Label(window, bg="#1c1e22")
    map_label.pack(fill="both", expand=True)
    lower = tk.Frame(window, bg="#1c1e22")
    lower.pack(fill="both", expand=True)
    boxes, captions = {}, {}
    for name in ("rgb", "cloud"):
        frame = tk.Frame(lower, bg="#1c1e22")
        frame.pack(side="left", fill="both", expand=True)
        captions[name] = tk.Label(frame, fg="white", bg="#1c1e22")
        captions[name].pack(fill="x")
        boxes[name] = tk.Label(frame, bg="#1c1e22")
        boxes[name].pack(fill="both", expand=True)
    footer = tk.Label(window, text="Map: online occupancy | Blue: optimized graph | "
                      "Red: TF robot | LiDAR: forward view, height color",
                      fg="#c4c7cc", bg="#1c1e22", font=("DejaVu Sans", 9))
    footer.pack(fill="x", pady=4)

    def received(key, msg):
        state[key] = msg
        stamps[key] = time.monotonic()

    def graph(msg):
        state["graph"] = [(p.position.x, p.position.y) for p in msg.poses]

    def clock(msg):
        state["sim"] = msg.clock.sec + msg.clock.nanosec / 1e9

    def info(msg):
        try:
            state["loops"] = json.loads(msg.data).get("global_loop_events", 0)
        except (ValueError, TypeError):
            pass

    subscriptions = [
        node.create_subscription(OccupancyGrid, "/as2/slam/map", lambda m: received("map", m), qos),
        node.create_subscription(RosImage, "/as2/camera/image_raw", lambda m: received("rgb", m), qos),
        node.create_subscription(PointCloud2, "/as2/lidar/points", lambda m: received("cloud", m), qos),
        node.create_subscription(MapGraph, "/as2/slam/mapGraph", graph, qos),
        node.create_subscription(Clock, "/clock", clock, qos),
        node.create_subscription(String, "/as2/slam/online_state", info, qos),
    ]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    last = {}

    def show(key, target, image):
        photos[key] = ImageTk.PhotoImage(image)
        target.configure(image=photos[key])

    def refresh():
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0)
        width = max(620, window.winfo_width() - 24)
        map_size = (width, max(250, int(window.winfo_height()*0.52)))
        tile_size = (width//2 - 6, max(180, int(window.winfo_height()*0.32)))
        sim = "--" if state["sim"] is None else f"{state['sim']:.1f}"
        header.configure(text=f"SIM {sim} s   |   Graph nodes {len(state['graph'])}"
                              f"   |   Global events {state['loops']} (pending audit)")
        robot = None
        try:
            tf = buffer.lookup_transform("map", "base_stabilized", rclpy.time.Time())
            t, q = tf.transform.translation, tf.transform.rotation
            robot = (t.x, t.y, math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        except TransformException:
            pass
        for key in ("map", "rgb", "cloud"):
            msg = state[key]
            age = "--" if key not in stamps else f"{time.monotonic()-stamps[key]:.1f}"
            if key != "map":
                title = "ONBOARD RGB" if key == "rgb" else "LIDAR FORWARD VIEW"
                stamp = "--" if msg is None else f"{msg.header.stamp.sec+msg.header.stamp.nanosec/1e9:.1f}"
                captions[key].configure(text=f"{title} | t={stamp}s | received {age}s ago")
            if msg is None:
                continue
            try:
                if key == "map":
                    show(key, map_label, map_image(msg, state["graph"], robot, map_size))
                elif last.get(key) != stamps[key] or last.get("size") != tile_size:
                    if key == "cloud":
                        image = lidar_image(cloud_xyz(msg), tile_size)
                    else:
                        if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
                            raise ValueError("RGB display expects RGB/BGR 8-bit encoding")
                        channels = 4 if "a8" in msg.encoding else 3
                        raw = np.ndarray((msg.height, msg.width, channels), dtype=np.uint8,
                                         buffer=msg.data, strides=(msg.step, channels, 1))[:, :, :3]
                        if msg.encoding.startswith("bgr"):
                            raw = raw[:, :, ::-1]
                        image = fit(Image.fromarray(raw), tile_size)
                    show(key, boxes[key], image)
                    last[key] = stamps[key]
            except (ValueError, TypeError, BufferError) as exc:
                node.get_logger().error(f"{key}: {exc}")
        last["size"] = tile_size
        window.after(100, refresh)

    def close():
        node.destroy_node()
        rclpy.shutdown()
        window.destroy()

    window.protocol("WM_DELETE_WINDOW", close)
    refresh()
    window.mainloop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test() if args.self_test else main()
