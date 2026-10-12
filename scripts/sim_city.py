#!/usr/bin/env python3
"""
Isaac Sim 6.1 side of the X500 city flight: the city scene (scaled to metres) with a sky and sun, and a PX4-controlled
Holybro X500 (Pegasus Simulator MAVLink backend) carrying a forward RGB-D camera. Published to ROS 2 Jazzy:
  /clock                                sim time
  /quadrotor/camera/rgb, depth          rgb8 and 32FC1 depth (metres), frame camera_optical_link
  /quadrotor/camera/camera_info
PX4 SITL is started by Pegasus and talks to ROS 2 over uXRCE-DDS (agent in the autonomy container).

Environment variables:
  CITY_USD, SKY_HDR          city USD and lat-long sky HDR (without CITY_USD: bare ground plane)
  X500_USD                   vehicle USD (default assets/x500/x500.usda, built by scripts/build_x500_usd.py)
  SPAWN_X, SPAWN_Y, SPAWN_Z  spawn position (metres, ENU)
  CAM_W, CAM_H, CAM_FOV      camera resolution and horizontal FOV (degrees)
  CAM_SKIP                   publish every (CAM_SKIP+1)-th rendered frame (default 2: 60 Hz -> 20 Hz)
  VIEWPORT_OFF=1, CHASE_CAM  turn the GUI viewport off / chase camera on or off
  RECORD=path.npy            save sim time, position and orientation (wxyz) of the drone every step
  PX4_LOCKSTEP=1             PX4 lockstep (off by default: the camera rendering cannot hold the lockstep rate)
  PX4_DIR, PX4_AIRFRAME      PX4-Autopilot directory and airframe
"""

import os
import sys
import time
import math
import numpy as np

from isaacsim import SimulationApp

HEADLESS = os.environ.get("ISAAC_HEADLESS", "0") == "1"
# Large city scenes on a 16 GB GPU: stream textures and cap their resolution.
_TEX_MIPS = {1024: 11, 2048: 12, 4096: 13}[int(os.environ.get("MAX_TEXTURE", "2048"))]
simulation_app = SimulationApp({
    "headless": HEADLESS,
    "extra_args": ["--/rtx-transient/resourcemanager/enableTextureStreaming=true",
                   f"--/rtx-transient/resourcemanager/texturestreaming/memoryBudget={os.environ.get('TEXTURE_BUDGET', '0.4')}",
                   f"--/rtx-transient/resourcemanager/maxMipCount={_TEX_MIPS}"],
})

import carb
import omni.timeline
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.core.nodes")
for _ in range(30):
    simulation_app.update()

# Isaac 6.0: register the forked Pegasus extension path before importing it (5.1 has it installed).
_PEG_EXT = os.environ.get("PEGASUS_EXT_PATH", "")
if _PEG_EXT:
    import omni.kit.app
    _mgr = omni.kit.app.get_app().get_extension_manager()
    _mgr.add_path(_PEG_EXT)
    for _ in range(20):
        simulation_app.update()
    _mgr.set_extension_enabled_immediate("pegasus.simulator", True)
    for _ in range(40):
        simulation_app.update()

try:
    from isaacsim.core.api.world import World      # Isaac Sim 6.0
except ImportError:
    from omni.isaac.core.world import World        # Isaac Sim 5.1
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
try:
    from pegasus.simulator.logic.graphs.ros2_camera_graph import ROS2CameraGraph  # 6.0 camera publisher
except ImportError:
    ROS2CameraGraph = None
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation

CAM_W = int(os.environ.get("CAM_W", "640"))
CAM_H = int(os.environ.get("CAM_H", "480"))
CAM_FOV = float(os.environ.get("CAM_FOV", "87.0"))   # RealSense D435 depth HFOV
RGB_TOPIC = "/camera/image"
DEPTH_TOPIC = "/camera/depth"
CAMERA_FRAME = "camera_optical_link"

PX4_DIR = os.path.expanduser(os.environ.get("PX4_DIR", "~/px4_jazzy_ws/PX4-Autopilot"))
PX4_AIRFRAME = os.environ.get("PX4_AIRFRAME", "gazebo-classic_x500")
PX4_ROBOT = os.environ.get("PX4_ROBOT", "x500")
X500_USD = os.path.expanduser(os.environ.get("X500_USD", os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "x500", "x500.usda")))   # build_x500_usd.py
WORLD_USD = os.path.expanduser(os.environ.get("WORLD_USD", ""))
CITY_USD = os.path.expanduser(os.environ.get("CITY_USD", ""))
SKY_HDR = os.path.expanduser(os.environ.get("SKY_HDR", ""))
CAM_SKIP = int(os.environ.get("CAM_SKIP", "2"))   # publish every (CAM_SKIP+1)-th rendered frame (60 Hz -> 20 Hz)
SPAWN = [float(os.environ.get("SPAWN_X", "0.0")),
         float(os.environ.get("SPAWN_Y", "0.0")),
         float(os.environ.get("SPAWN_Z", "0.3"))]


def load_city(world, usd_path):
    """Reference a city USD under /World/City, converting its units to metres, swap its sky for SKY_HDR and add an
    invisible collision ground plane (the city meshes carry no colliders; the drone only needs the ground)."""
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux
    stage = omni.usd.get_context().get_stage()
    ref = Usd.Stage.Open(usd_path, Usd.Stage.LoadNone)
    mpu = UsdGeom.GetStageMetersPerUnit(ref) or 1.0
    city = stage.DefinePrim("/World/City", "Xform")
    city.GetReferences().AddReference(usd_path)
    xf = UsdGeom.Xformable(city)
    xf.ClearXformOpOrder()
    xf.AddScaleOp().Set(Gf.Vec3d(mpu, mpu, mpu))
    for _ in range(10):
        simulation_app.update()
    from isaacsim.core.utils.stage import is_stage_loading
    while is_stage_loading():
        simulation_app.update()
    # The city's own lights sit outside its default prim, so they do not come with the reference: add a sky dome
    # (HDRI) and a sun.
    if SKY_HDR and os.path.exists(SKY_HDR):
        dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
        dome.CreateTextureFileAttr().Set(Sdf.AssetPath(SKY_HDR))
        dome.CreateTextureFormatAttr().Set("latlong")
        dome.CreateIntensityAttr().Set(float(os.environ.get("SKY_INTENSITY", "1000")))
        print(f"[world] sky {os.path.basename(SKY_HDR)}", flush=True)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr().Set(float(os.environ.get("SUN_INTENSITY", "3000")))
    sun.CreateAngleAttr().Set(0.6)
    sun.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.96, 0.9))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(float(os.environ.get("SUN_TILT", "50")), 0.0, float(os.environ.get("SUN_AZIMUTH", "35"))))
    world.scene.add_default_ground_plane(z_position=float(os.environ.get("GROUND_Z", "0.0")))
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/defaultGroundPlane")):
        if prim.IsA(UsdGeom.Mesh):
            UsdGeom.Imageable(prim).MakeInvisible()
    print(f"[world] city {os.path.basename(usd_path)} (metersPerUnit {mpu}) + invisible collision ground", flush=True)


def build_clock_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ClockGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                ("PublishClock.inputs:topicName", "/clock"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("SimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    print("[ros] /clock publisher ready")


def add_realsense_visual(body_path):
    """Intel RealSense D455 model on the front mount (visual only), from the Isaac Sim asset library.
    REALSENSE_USD overrides the path, e.g. with a local copy of the asset pack."""
    import omni.usd
    from pxr import Gf, UsdGeom
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(body_path + "/realsense").IsValid():
        return
    path = os.environ.get("REALSENSE_USD", "")
    if not path:
        try:
            from isaacsim.storage.native import get_assets_root_path
            root = get_assets_root_path()
        except Exception:
            root = None
        if not root:
            print("[veh] Isaac Sim asset library not reachable, no RealSense model", flush=True)
            return
        path = root + "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
    rs = stage.DefinePrim(body_path + "/realsense", "Xform")
    rs.GetReferences().AddReference(path)
    UsdGeom.Xformable(rs).AddTranslateOp().Set(Gf.Vec3d(0.13, 0.0, -0.05))
    print(f"[veh] RealSense model: {path}", flush=True)


def build_camera_graph(camera_prim_path):
    """Isaac 6.0: forward-facing camera -> OFFSCREEN render product -> ROS2CameraHelper.
    Offscreen (IsaacCreateRenderProduct, not a GUI viewport) so the main viewport stays free for the
    chase camera, and the camera looks along body +X (not down) so the planner sees obstacles ahead.
    Publishes /quadrotor/camera/{rgb,depth,camera_info}."""
    import omni.usd
    import carb
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.prims import set_targets

    _rtx = carb.settings.get_settings()
    _old_aa = _rtx.get("/rtx/post/aa/op")
    _rtx.set_int("/rtx/post/aa/op", 2)
    print(f"[ros] rtx aa/op {_old_aa} -> 2 (FXAA, DLSS disabled for sensor render product)")

    cam_pos = [float(v) for v in os.environ.get("CAM_POS", "0.2,0.0,0.0").split(",")]
    is_realsense = "realsense" in camera_prim_path.lower()
    cam = Camera(prim_path=camera_prim_path, frequency=30.0, resolution=(CAM_W, CAM_H))
    cam.initialize()
    if not is_realsense:
        # legacy standalone camera: place it forward (+X) and up (+Z) so the planner looks ahead.
        cam.set_local_pose(np.array(cam_pos), np.array([1.0, 0.0, 0.0, 0.0]), camera_axes="world")
    # Match the real RealSense D455 depth FOV (CAM_FOV=87deg -> fx~337) so the sim intrinsics equal a
    # real sensor AND Ego's grid_map fx/fy. The D455 prim is already mounted+oriented by the asset.
    _ha = 20.955
    cam.set_horizontal_aperture(_ha)
    cam.set_vertical_aperture(_ha * CAM_H / CAM_W)
    cam.set_focal_length(_ha / (2.0 * math.tan(math.radians(CAM_FOV) / 2.0)))
    cam.set_clipping_range(0.05, 1.0e5)
    print(f"[ros] camera prim={camera_prim_path.split('/')[-1]} HFOV={CAM_FOV}deg fx={(CAM_W/2)/math.tan(math.radians(CAM_FOV)/2):.0f}")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/CameraGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("createRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("camRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("camDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("camInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                ("createRP.inputs:width", CAM_W),
                ("createRP.inputs:height", CAM_H),
                ("camRgb.inputs:type", "rgb"),
                ("camRgb.inputs:topicName", "/quadrotor/camera/rgb"),
                ("camRgb.inputs:frameId", CAMERA_FRAME),
                ("camDepth.inputs:type", "depth"),
                ("camDepth.inputs:topicName", "/quadrotor/camera/depth"),
                ("camDepth.inputs:frameId", CAMERA_FRAME),
                ("camInfo.inputs:topicName", "/quadrotor/camera/camera_info"),
                ("camInfo.inputs:frameId", CAMERA_FRAME),
                ("camRgb.inputs:frameSkipCount", CAM_SKIP),
                ("camDepth.inputs:frameSkipCount", CAM_SKIP),
                ("camInfo.inputs:frameSkipCount", CAM_SKIP),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createRP.inputs:execIn"),
                ("createRP.outputs:execOut", "camRgb.inputs:execIn"),
                ("createRP.outputs:execOut", "camDepth.inputs:execIn"),
                ("createRP.outputs:execOut", "camInfo.inputs:execIn"),
                ("createRP.outputs:renderProductPath", "camRgb.inputs:renderProductPath"),
                ("createRP.outputs:renderProductPath", "camDepth.inputs:renderProductPath"),
                ("createRP.outputs:renderProductPath", "camInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "camRgb.inputs:context"),
                ("Context.outputs:context", "camDepth.inputs:context"),
                ("Context.outputs:context", "camInfo.inputs:context"),
            ],
        },
    )
    stage = omni.usd.get_context().get_stage()
    set_targets(prim=stage.GetPrimAtPath("/World/CameraGraph/createRP"),
                attribute="inputs:cameraPrim", target_prim_paths=[camera_prim_path])
    print("[ros] camera graph (offscreen RP, forward cam): /quadrotor/camera/{rgb,depth,camera_info}")


def setup_chase_camera():
    """Free (un-parented) third-person follower as the active viewport camera, wide clip so framing
    the small drone never clips to black. Pose is driven every frame by update_chase_camera()."""
    try:
        import omni.usd
        from pxr import UsdGeom, Gf
        stage = omni.usd.get_context().get_stage()
        path = "/World/ChaseCam"
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateFocalLengthAttr(20.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1.0e6))
        UsdGeom.Xformable(cam.GetPrim()).AddTransformOp()
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = path
        print(f"[chase] free follower camera active at {path}")
        return True
    except Exception as e:
        print(f"[chase] follower camera failed: {e}")
        return False


def update_chase_camera(dist=5.5, height=2.2):
    """Place /World/ChaseCam behind+above the drone (yaw-follow only, no roll/pitch banking) and aim
    it at the drone. Runs every frame so the view tracks the flight smoothly."""
    try:
        import omni.usd
        from pxr import UsdGeom, Gf
        stage = omni.usd.get_context().get_stage()
        body = stage.GetPrimAtPath("/World/quadrotor/body")
        if not body.IsValid():
            return
        wm = omni.usd.get_world_transform_matrix(body)
        p = wm.ExtractTranslation()
        fwd = wm.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        yaw = math.atan2(fwd[1], fwd[0])
        cam_pos = Gf.Vec3d(p[0] - dist * math.cos(yaw), p[1] - dist * math.sin(yaw), p[2] + height)
        forward = (p - cam_pos).GetNormalized()
        right = Gf.Cross(forward, Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
        up = Gf.Cross(right, forward).GetNormalized()
        M = Gf.Matrix4d(1.0)
        M.SetRow(0, Gf.Vec4d(right[0], right[1], right[2], 0.0))
        M.SetRow(1, Gf.Vec4d(up[0], up[1], up[2], 0.0))
        M.SetRow(2, Gf.Vec4d(-forward[0], -forward[1], -forward[2], 0.0))
        M.SetRow(3, Gf.Vec4d(cam_pos[0], cam_pos[1], cam_pos[2], 1.0))
        cam = stage.GetPrimAtPath("/World/ChaseCam")
        cam.GetAttribute("xformOp:transform").Set(M)
    except Exception:
        pass


def main():
    timeline = omni.timeline.get_timeline_interface()

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world

    if CITY_USD and os.path.exists(CITY_USD):
        load_city(world, CITY_USD)
    elif WORLD_USD and os.path.exists(WORLD_USD):
        print(f"[world] loading {WORLD_USD}")
        pg.load_environment(WORLD_USD)
    else:
        if CITY_USD:
            print(f"[world] WARNING: CITY_USD not found: {CITY_USD}", flush=True)
        print("[world] bare ground plane (set CITY_USD to fly in the city)", flush=True)
        world.scene.add_default_ground_plane()

    config = MultirotorConfig()
    if os.environ.get("WIND", "0") == "1":
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from wind_drag import WindDrag
        dcoef = [float(v) for v in os.environ.get("WIND_DRAG", "0.6,0.6,0.3").split(",")]
        config.drag = WindDrag(drag_coefficients=dcoef)
        print(f"[wind] WindDrag enabled (coeffs {dcoef}); wind from {os.environ.get('WIND_FILE', '/tmp/px4_wind.json')}")
    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": 0,
        "px4_autolaunch": os.environ.get("PX4_AUTOLAUNCH", "1") == "1",
        "px4_dir": PX4_DIR,
        "px4_vehicle_model": PX4_AIRFRAME,
        # The camera rendering in the city cannot sustain the lockstep sensor rate -> PX4
        # poll timeouts. Off by default so PX4 runs on wall-clock (real-time-paced sim keeps it close).
        "enable_lockstep": os.environ.get("PX4_LOCKSTEP", "0") == "1",
    })
    config.backends = [PX4MavlinkBackend(mavlink_config)]

    # Camera is published from an offscreen render product (build_camera_graph, after world.reset) so
    # the GUI viewport stays free for the chase cam and the camera looks forward (not down).
    use_cam_graph = os.environ.get("ISAAC_CAMERA", "1") == "1"

    if PX4_ROBOT.lower() == "x500" and os.path.exists(X500_USD):
        robot_usd = X500_USD
        print(f"[veh] Holybro X500 V2: {robot_usd}")
    elif PX4_ROBOT.lower() == "x500":
        raise SystemExit(f"[veh] {X500_USD} not found: build it with scripts/build_x500_usd.py (see README)")
    else:
        robot_usd = ROBOTS[PX4_ROBOT]
        print(f"[veh] {PX4_ROBOT}: {robot_usd}")

    Multirotor(
        "/World/quadrotor",
        robot_usd,
        0,
        SPAWN,
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config,
    )
    add_realsense_visual("/World/quadrotor/body")

    world.reset()

    build_clock_graph()
    if use_cam_graph:
        try:
            build_camera_graph(os.environ.get("CAMERA_PRIM", "/World/quadrotor/body/camera"))
        except Exception as e:
            import traceback
            print(f"[cam] camera graph failed: {e}")
            traceback.print_exc()

    timeline.play()
    print("[run] simulation playing - PX4 SITL launching, ROS topics live")

    winddraw = None
    if os.environ.get("WIND", "0") == "1":
        try:
            enable_extension("isaacsim.util.debug_draw")
            from wind_draw import WindDraw
            winddraw = WindDraw()
            print("[wind] viewport wind arrows enabled (debug_draw)")
        except Exception as e:
            print(f"[wind] viewport draw unavailable: {e}")

    # PX4 lockstep needs sim-time to track wall-clock or the uXRCE time sync resets and arming fails;
    # pace to real-time (default on, since this scene drives PX4). Harmless when GUI already runs ~1x.
    realtime = os.environ.get("REALTIME", "1") == "1"
    phys_dt = world.get_physics_dt()
    wall0 = time.time()
    frame = 0
    chase_on = (not HEADLESS) and os.environ.get("CHASE_CAM", "1") == "1"
    chase_setup = False
    if os.environ.get("VIEWPORT_OFF", "0") == "1":     # free the GPU for the sensor camera during flights
        try:
            from omni.kit.viewport.utility import get_active_viewport
            get_active_viewport().updates_enabled = False
            print("[run] viewport updates off", flush=True)
        except Exception as e:
            print(f"[run] viewport off failed: {e}", flush=True)
    # Flight recorder for replay renders: sim time, body position and orientation (wxyz) every frame.
    rec_path = os.environ.get("RECORD", "")
    rec, rec_body = [], None
    if rec_path:
        from isaacsim.core.prims import XFormPrim
        rec_body = XFormPrim("/World/quadrotor/body")
    while simulation_app.is_running():
        world.step(render=True)
        if rec_body is not None:
            pos, quat = rec_body.get_world_poses()
            rec.append(np.concatenate([[world.current_time], np.asarray(pos)[0], np.asarray(quat)[0]]))
            if len(rec) % 30 == 0:
                np.save(rec_path, np.asarray(rec))
        if realtime:
            slack = (wall0 + (frame + 1) * phys_dt) - time.time()
            if slack > 0:
                time.sleep(slack)
        if chase_on:
            if not chase_setup and frame == 30:         # viewport ready by now
                setup_chase_camera()
                chase_setup = True
            elif chase_setup:
                update_chase_camera()                   # follow the drone every frame
        if winddraw is not None and frame % 2 == 0:     # frequent redraw -> smooth flowing gust
            winddraw.update()
        frame += 1

    if rec:
        np.save(rec_path, np.asarray(rec))
    carb.log_warn("closing")
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
