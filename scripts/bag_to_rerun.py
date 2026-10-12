"""Convert a recorded flight (ROS 2 bag from run_autonomy.sh OUT_DIR=...) into a Rerun recording (.rrd).

    uv run --with rerun-sdk==0.26.2 --with rosbags --with numpy python scripts/bag_to_rerun.py runs/flight1/bag runs/flight1/flight.rrd

Open it with `rerun runs/flight1/flight.rrd`: drone camera and depth, flown track, Ego-Planner's obstacle grid and
planned path, and the RTAB-Map point cloud, all on the simulation-time timeline.
"""
import sys
from pathlib import Path

import numpy as np
import rerun as rr
import rosbags.highlevel.anyreader as _ar
import rosbags.typesys.msg as _rmsg
from rosbags.highlevel import AnyReader

# some message definitions in the bag (px4_msgs, quadrotor_msgs) have Windows line endings
_parse = _rmsg.get_types_from_msg
_ar.get_types_from_msg = _rmsg.get_types_from_msg = lambda text, name: _parse(text.replace("\r", ""), name)

bag, out = Path(sys.argv[1]), sys.argv[2]
CAM, DEP = "/quadrotor/camera/rgb", "/quadrotor/camera/depth"
rr.init("x500_city_flight")
rr.save(out)
rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
track = []
with AnyReader([bag]) as rd:
    topics = {CAM, DEP, "/px4_odom", "/rtabmap/cloud_map"}
    conns = [c for c in rd.connections
             if c.topic in topics or c.topic.endswith(("grid_map/occupancy_inflate", "optimal_list"))]
    k_rgb = 0
    for c, t, raw in rd.messages(connections=conns):
        try:
            m = rd.deserialize(raw, c.msgtype)
        except Exception:
            continue
        rr.set_time("sim_time", timestamp=t * 1e-9)
        if c.topic == CAM:
            k_rgb += 1
            if k_rgb % 2 == 0:
                rr.log("camera/rgb", rr.Image(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)).compress(85))
        elif c.topic == DEP:
            if k_rgb % 2 == 0:
                d = np.nan_to_num(np.frombuffer(m.data, np.float32).reshape(m.height, m.width), posinf=0.0)
                rr.log("camera/depth", rr.DepthImage(d, meter=1.0))
        elif c.topic == "/px4_odom":
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            track.append([p.x, p.y, p.z])
            rr.log("world/drone", rr.Transform3D(translation=[p.x, p.y, p.z], quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))
            if len(track) % 10 == 0:
                rr.log("world/track", rr.LineStrips3D([track], colors=[[255, 184, 76]], radii=0.15))
        elif c.topic.endswith("occupancy_inflate"):
            pts = np.frombuffer(m.data, np.float32).reshape(-1, m.point_step // 4)[:, :3]
            rr.log("world/obstacles", rr.Points3D(pts, colors=[[120, 140, 170]], radii=0.12))
        elif c.topic.endswith("optimal_list") and m.points:
            rr.log("world/plan", rr.LineStrips3D([[[q.x, q.y, q.z] for q in m.points]], colors=[[70, 216, 196]], radii=0.2))
        elif c.topic == "/rtabmap/cloud_map":
            f = {fl.name: fl.offset for fl in m.fields}
            raw_pts = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
            xyz = np.stack([raw_pts[:, f[a]:f[a] + 4].copy().view(np.float32)[:, 0] for a in "xyz"], 1)
            col = None
            if "rgb" in f:
                rgb = raw_pts[:, f["rgb"]:f["rgb"] + 4]
                col = rgb[:, [2, 1, 0]]
            rr.log("world/map", rr.Points3D(xyz, colors=col, radii=0.06))
print("saved", out)
