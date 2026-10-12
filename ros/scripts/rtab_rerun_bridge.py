#!/usr/bin/env python3
"""RTAB-Map (+ RGB-D + TF) ROS 2 -> Rerun bridge: one 3-D view of the SLAM map cloud, trajectory,
loop closures, and the live camera frustum moving through it, plus 2-D RGB / depth / grid panels.
Targets rerun 0.26 (classic entity-path transform tree)."""
from __future__ import annotations

import argparse
import threading
from collections import defaultdict
from typing import Optional

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import Path, OccupancyGrid
from tf2_msgs.msg import TFMessage

try:
    from rtabmap_msgs.msg import MapGraph
    _HAVE_MAPGRAPH = True
except Exception:
    _HAVE_MAPGRAPH = False

try:
    from nvblox_msgs.msg import Mesh as NvbloxMesh
    _HAVE_NVBLOX = True
except Exception:
    _HAVE_NVBLOX = False

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAVE_PC2 = True
except Exception:
    _HAVE_PC2 = False

try:
    from scipy.spatial.transform import Rotation as Rot
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

CAM = "camera"           # fixed entity for the 3D frustum + 2D RGB panel
PANEL_DEPTH = "panels/depth"
PANEL_GRID = "panels/grid"


def stamp_to_dt64(stamp) -> np.datetime64:
    return np.datetime64(Time.from_msg(stamp).nanoseconds, "ns")


def height_colors(z: np.ndarray, lo: float = 0.0, hi: float = 4.0) -> np.ndarray:
    t = np.clip((z - lo) / max(1e-6, hi - lo), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def dist_colors(d: np.ndarray, near: float = 0.2, far: float = 2.0) -> np.ndarray:
    """ESDF proximity heatmap: red = close to an obstacle (danger), through green, to blue = far (safe)."""
    t = np.clip((d - near) / max(1e-6, far - near), 0.0, 1.0)
    r = np.clip(1.0 - t, 0, 1)
    g = np.clip(1.0 - np.abs(2 * t - 1), 0, 1)
    b = np.clip(t, 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def transient_local(depth: int) -> QoSProfile:
    return QoSProfile(depth=depth, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


def reliable_volatile(depth: int) -> QoSProfile:
    return QoSProfile(depth=depth, durability=DurabilityPolicy.VOLATILE,
                      reliability=ReliabilityPolicy.RELIABLE)


class RtabRerunBridge(Node):
    def __init__(self, args) -> None:
        super().__init__("rtab_rerun_bridge")
        self._root = args.root_frame
        self._cam_frame = args.camera_frame
        self._tf_lock = threading.Lock()
        self._parent: dict[str, str] = {}                      # child -> parent
        self._local: dict[str, tuple[np.ndarray, np.ndarray]] = {}  # child -> (t, quat xyzw)
        self._K: Optional[np.ndarray] = None
        self._counts: dict[str, int] = defaultdict(int)

        cg = ReentrantCallbackGroup()
        self.create_subscription(TFMessage, "/tf", self._on_tf, qos_profile_sensor_data, callback_group=cg)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf, transient_local(100), callback_group=cg)
        self.create_subscription(CameraInfo, args.camera_info, self._on_camera_info, qos_profile_sensor_data, callback_group=cg)
        self.create_subscription(Image, args.rgb, self._on_rgb, qos_profile_sensor_data, callback_group=cg)
        self.create_subscription(Image, args.depth, self._on_depth, qos_profile_sensor_data, callback_group=cg)
        for topic in (args.cloud_map, args.octomap):
            self.create_subscription(PointCloud2, topic, self._on_cloud, transient_local(1), callback_group=cg)
        self.create_subscription(Path, args.map_path, self._on_path, reliable_volatile(5), callback_group=cg)
        self.create_subscription(OccupancyGrid, args.grid_map, self._on_grid, transient_local(1), callback_group=cg)
        if _HAVE_MAPGRAPH:
            self.create_subscription(MapGraph, args.map_graph, self._on_graph, transient_local(1), callback_group=cg)
        else:
            self.get_logger().warn("rtabmap_msgs not importable -> loop-closure graph view disabled")
        if _HAVE_NVBLOX:
            self.create_subscription(NvbloxMesh, args.nvblox_mesh, self._on_mesh, reliable_volatile(2), callback_group=cg)
        else:
            self.get_logger().warn("nvblox_msgs not importable -> nvblox 3D mesh view disabled")
        self.create_subscription(PointCloud2, args.nvblox_esdf, self._on_esdf, qos_profile_sensor_data, callback_group=cg)
        if not _HAVE_PC2:
            self.get_logger().warn("sensor_msgs_py not importable -> map cloud view disabled")
        if not _HAVE_SCIPY:
            self.get_logger().warn("scipy not importable -> 3D camera frustum disabled (2D RGB still works)")

        self.create_timer(3.0, self._report)
        self.get_logger().info(f"RTAB->Rerun bridge up (root={self._root}, cam={self._cam_frame})")

    def _report(self) -> None:
        c = self._counts
        self.get_logger().info(
            "rx  tf=%d rgb=%d depth=%d cloud=%d path=%d graph=%d grid=%d mesh=%d esdf=%d"
            % (c["tf"], c["rgb"], c["depth"], c["cloud"], c["path"], c["graph"], c["grid"], c["mesh"], c["esdf"]))

    # ---- TF: classic entity-path tree ----
    def _epath(self, frame: str) -> str:
        chain, f, seen = [frame], frame, set()
        while f in self._parent and f not in seen:
            seen.add(f)
            f = self._parent[f]
            chain.append(f)
        return "/".join(reversed(chain))

    def _on_tf(self, msg: TFMessage) -> None:
        self._counts["tf"] += 1
        for tr in msg.transforms:
            t = np.array([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z])
            q = np.array([tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w])
            with self._tf_lock:
                self._parent[tr.child_frame_id] = tr.header.frame_id
                self._local[tr.child_frame_id] = (t, q)
            rr.set_time("ros_time", timestamp=stamp_to_dt64(tr.header.stamp))
            rr.log(self._epath(tr.child_frame_id),
                   rr.Transform3D(translation=t, quaternion=rr.Quaternion(xyzw=q), axis_length=0.25))

    def _root_T(self, frame: str):
        """Pose of *frame* in the root frame as (pos, Rot), or None if the chain is incomplete."""
        if not _HAVE_SCIPY:
            return None
        chain, f, seen = [], frame, set()
        with self._tf_lock:
            while f in self._local and f not in seen:
                seen.add(f)
                chain.append((self._local[f]))   # (t, q), leaf-first
                f = self._parent.get(f, "")
            root_reached = (f == self._root)
        if not root_reached:
            return None
        pos, rot = np.zeros(3), Rot.identity()
        for t, q in reversed(chain):             # near-root -> leaf
            pos = pos + rot.apply(t)
            rot = rot * Rot.from_quat(q)
        return pos, rot

    # ---- Camera ----
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        rr.log(CAM, rr.Pinhole(image_from_camera=self._K, width=msg.width, height=msg.height,
                               camera_xyz=rr.ViewCoordinates.RDF, image_plane_distance=0.8), static=True)

    def _on_rgb(self, msg: Image) -> None:
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding.lower().startswith("bgr"):
            arr = arr[:, :, ::-1]
        self._counts["rgb"] += 1
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        pose = self._root_T(self._cam_frame)
        if pose is not None:
            pos, rot = pose
            rr.log(CAM, rr.Transform3D(translation=pos, quaternion=rr.Quaternion(xyzw=rot.as_quat())))
        rr.log(CAM, rr.Image(arr))

    def _on_depth(self, msg: Image) -> None:
        enc = msg.encoding.lower()
        if "32fc1" in enc:
            d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        elif "16uc1" in enc:
            d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
        else:
            return
        self._counts["depth"] += 1
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        rr.log(PANEL_DEPTH, rr.DepthImage(d, meter=1.0))

    # ---- RTAB-Map map cloud ----
    def _on_cloud(self, msg: PointCloud2) -> None:
        if not _HAVE_PC2:
            return
        names = [f.name for f in msg.fields]
        has_rgb = "rgb" in names
        fields = ("x", "y", "z", "rgb") if has_rgb else ("x", "y", "z")
        pts = pc2.read_points(msg, field_names=fields, skip_nans=True)
        if not isinstance(pts, np.ndarray) or pts.size == 0:
            return
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32)
        if has_rgb:
            packed = np.ascontiguousarray(pts["rgb"]).view(np.uint8).reshape(-1, 4)
            rgb = packed[:, :3][:, ::-1].copy()
        else:
            rgb = height_colors(xyz[:, 2])
        self._counts["cloud"] += 1
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        rr.log(self._epath(msg.header.frame_id or self._root) + "/cloud", rr.Points3D(xyz, colors=rgb, radii=0.02))

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses], np.float32)
        self._counts["path"] += 1
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        rr.log(self._epath(msg.header.frame_id or self._root) + "/trajectory",
               rr.LineStrips3D([pts], colors=[0, 200, 255], radii=0.03))

    def _on_grid(self, msg: OccupancyGrid) -> None:
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        g = np.array(msg.data, np.int8).reshape(h, w)
        img = np.full((h, w, 3), 128, np.uint8)
        img[g == 0] = (240, 240, 240)
        img[g > 0] = (20, 20, 20)
        img[g < 0] = (90, 90, 90)
        self._counts["grid"] += 1
        rr.log(PANEL_GRID, rr.Image(img))

    # ---- Pose graph + loop closures ----
    def _on_graph(self, msg: MapGraph) -> None:
        base = self._epath(msg.header.frame_id or self._root) + "/graph"
        id_to_xyz = {pid: [p.position.x, p.position.y, p.position.z] for pid, p in zip(msg.poses_id, msg.poses)}
        if not id_to_xyz:
            return
        nodes = np.array(list(id_to_xyz.values()), np.float32)
        self._counts["graph"] += 1
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        rr.log(base + "/nodes", rr.Points3D(nodes, colors=[255, 180, 0], radii=0.05))
        neigh, loops = [], []
        for lk in msg.links:
            a, b = id_to_xyz.get(lk.from_id), id_to_xyz.get(lk.to_id)
            if a is None or b is None:
                continue
            (neigh if lk.type == 0 else loops).append([a, b])
        if neigh:
            rr.log(base + "/neighbor_links", rr.LineStrips3D(neigh, colors=[120, 120, 120], radii=0.01))
        if loops:
            rr.log(base + "/loop_closures", rr.LineStrips3D(loops, colors=[255, 0, 120], radii=0.025))

    # ---- nvblox 3D mesh reconstruction (per-block, incremental) ----
    def _on_mesh(self, msg: "NvbloxMesh") -> None:
        base = self._epath(msg.header.frame_id or self._root) + "/nvblox_mesh"
        bs = float(msg.block_size_m)
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        if msg.clear:                                          # full republish -> drop the old mesh first
            rr.log(base, rr.Clear(recursive=True))
        self._counts["mesh"] += 1
        for idx, blk in zip(msg.block_indices, msg.blocks):
            ent = f"{base}/{idx.x}_{idx.y}_{idx.z}"
            n = len(blk.vertices)
            if n == 0 or len(blk.triangles) < 3:
                rr.log(ent, rr.Clear(recursive=False))        # block emptied -> remove it
                continue
            verts = np.array([[v.x, v.y, v.z] for v in blk.vertices], np.float32)
            if verts.size and np.abs(verts).max() <= bs * 1.5:   # block-local coords -> shift to global
                verts = verts + np.array([idx.x, idx.y, idx.z], np.float32) * bs
            tris = np.array(blk.triangles, np.int32).reshape(-1, 3)
            if blk.colors and len(blk.colors) == n:
                cols = np.array([[c.r, c.g, c.b] for c in blk.colors], np.float32)
                cols = (np.clip(cols, 0, 1) * 255).astype(np.uint8)
            else:
                cols = height_colors(verts[:, 2])             # depth-only mesh -> colour by height
            rr.log(ent, rr.Mesh3D(vertex_positions=verts, triangle_indices=tris, vertex_colors=cols))

    # ---- nvblox ESDF distance field as a proximity heatmap ----
    def _on_esdf(self, msg: PointCloud2) -> None:
        if not _HAVE_PC2:
            return
        pts = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        if not isinstance(pts, np.ndarray) or pts.size == 0:
            return
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32)
        self._counts["esdf"] += 1
        rr.set_time("ros_time", timestamp=stamp_to_dt64(msg.header.stamp))
        rr.log(self._epath(msg.header.frame_id or self._root) + "/nvblox_esdf",
               rr.Points3D(xyz, colors=dist_colors(np.asarray(pts["intensity"], np.float32)), radii=0.03))


def build_blueprint() -> "rrb.Blueprint":
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="RTAB-Map SLAM", origin="/"),
            rrb.Vertical(
                rrb.Spatial2DView(name="RGB", origin=CAM),
                rrb.Spatial2DView(name="Depth", origin=PANEL_DEPTH),
                rrb.Spatial2DView(name="2D Grid", origin=PANEL_GRID),
                row_shares=[1, 1, 1],
            ),
            column_shares=[3, 1],
        ),
        auto_layout=False, auto_views=False,
    )


def main() -> None:
    p = argparse.ArgumentParser(description="RTAB-Map ROS2 -> Rerun bridge")
    p.add_argument("--rgb", default="/quadrotor/camera/rgb")
    p.add_argument("--depth", default="/quadrotor/camera/depth")
    p.add_argument("--camera-info", default="/quadrotor/camera/camera_info")
    p.add_argument("--camera-frame", default="camera_optical_link")
    p.add_argument("--cloud-map", default="/rtabmap/cloud_map")
    p.add_argument("--octomap", default="/rtabmap/octomap_occupied_space")
    p.add_argument("--map-path", default="/rtabmap/mapPath")
    p.add_argument("--map-graph", default="/rtabmap/mapGraph")
    p.add_argument("--grid-map", default="/rtabmap/map")
    p.add_argument("--nvblox-mesh", default="/nvblox_node/mesh")
    p.add_argument("--nvblox-esdf", default="/nvblox_node/static_esdf_pointcloud")
    p.add_argument("--root-frame", default="map")
    p.add_argument("--web-port", type=int, default=9090)
    p.add_argument("--grpc-port", type=int, default=9876)
    p.add_argument("--save", default="", help="write the recording to this .rrd file instead of serving a viewer")
    args, unknown = p.parse_known_args()

    rr.init("x500_city_flight")
    if args.save:
        rr.save(args.save)
        print(f"[rtab_rerun] recording to {args.save}", flush=True)
    else:
        server_uri = rr.serve_grpc(grpc_port=args.grpc_port)
        rr.serve_web_viewer(web_port=args.web_port, open_browser=False, connect_to=server_uri)
        print(f"[rtab_rerun] web viewer on http://0.0.0.0:{args.web_port}  (grpc {args.grpc_port})", flush=True)
    rr.send_blueprint(build_blueprint())

    rclpy.init(args=unknown)
    node = RtabRerunBridge(args)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
