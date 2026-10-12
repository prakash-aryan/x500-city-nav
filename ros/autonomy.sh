#!/usr/bin/env bash
# Autonomy stack, run inside the x500-city-nav container (network host) against the Isaac Sim side:
#   PX4 SITL + MicroXRCEAgent -> px4_tf_pub (PX4 EKF pose -> TF + /px4_odom)
#   RTAB-Map (RGB-D mapping, PX4 odometry) + Ego-Planner (local planning from the depth image)
#   mission_bridge (route points -> Ego-Planner goals, Ego trajectory -> PX4 offboard setpoints)
set +e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/ego_ws/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
X=/x500
CAM=/quadrotor/camera
ALT=$(python3 -c "import sys; print(float(sys.argv[1]))" "${ALT:-12}")   # ROS 2 needs a double
ROUTE=${ROUTE:-"0,40,12"}
log() { echo "[autonomy] $*"; }

log "MicroXRCEAgent (udp 8888)"
MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &

log "PX4 SITL (gazebo-classic_x500 airframe, MAVLink to Isaac Sim on tcp 4560)"
( mkdir -p /tmp/px4run && cd /tmp/px4run && \
  PX4_SIM_MODEL=gazebo-classic_x500 /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
    /opt/PX4-Autopilot/ROMFS/px4fmu_common/ -s /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i 0 -d \
    > /tmp/px4.log 2>&1 ) &
sleep 14

log "px4_tf_pub (PX4 EKF -> /px4_odom + TF odom->base_link)"
/usr/bin/python3 -u $X/scripts/px4_tf_pub.py --ros-args -p use_sim_time:=true > /tmp/px4_tf.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --x 0.2 --y 0.0 --z 0.0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
  --frame-id base_link --child-frame-id camera_optical_link --ros-args -p use_sim_time:=true > /tmp/cam_tf.log 2>&1 &
/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args -p use_sim_time:=true \
  -p robot_description:="$(cat $X/config/x500_gz_container.urdf)" > /tmp/rsp.log 2>&1 &
/usr/bin/python3 -u $X/config/spin_props.py --ros-args -p use_sim_time:=true > /tmp/spin.log 2>&1 &
sleep 2

log "RTAB-Map (RGB-D map from the forward camera, PX4 odometry)"
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="--delete_db_on_start --Grid/RangeMax ${MAP_RANGE:-25} --Grid/CellSize 0.2 --Grid/ClusterRadius 0.5 --Grid/MinClusterSize 5 --Rtabmap/DetectionRate 2" \
  visual_odometry:="false" odom_topic:="/px4_odom" frame_id:="base_link" odom_frame_id:="odom" map_frame_id:="map" \
  rgb_topic:="$CAM/rgb" depth_topic:="$CAM/depth" camera_info_topic:="$CAM/camera_info" \
  approx_sync:="true" approx_sync_max_interval:="0.06" use_sim_time:="true" qos:="2" \
  rviz:="false" rtabmap_viz:="false" publish_tf_map:="true" > /tmp/rtabmap.log 2>&1 &
sleep 4

log "Ego-Planner (depth $CAM/depth + /px4_odom)"
ros2 launch $X/launch/ego_city.launch.py > /tmp/ego.log 2>&1 &
sleep 5

if [ "${RVIZ:-0}" = "1" ]; then rviz2 -d $X/config/ego.rviz > /tmp/rviz.log 2>&1 & fi
if [ "${RERUN:-0}" = "1" ]; then
  /usr/bin/python3 -u $X/scripts/rtab_rerun_bridge.py --web-port ${RERUN_WEB_PORT:-9090} --grpc-port ${RERUN_GRPC_PORT:-9876} \
    $([ -d /out ] && echo "--save /out/flight.rrd") \
    > /tmp/rtab_rerun.log 2>&1 &
fi

if [ -d /out ]; then
  log "recording camera, odometry and plan topics to /out/bag"
  rm -rf /out/bag
  ros2 bag record -o /out/bag --use-sim-time -s mcap \
    --regex "^($CAM/.*|/px4_odom|/tf|/tf_static|/position_cmd|/move_base_simple/goal|/rtabmap/cloud_map|/rtabmap/mapPath|.*grid_map/occupancy_inflate|.*optimal_list|.*planning/bspline)$" \
    > /tmp/bag.log 2>&1 &
  BAG_PID=$!
fi

# stop the recorders cleanly (bag index, .rrd) when the mission ends or the container is stopped
finish() {
  log "stopping recorders"
  [ -n "${BAG_PID:-}" ] && kill -INT $BAG_PID 2>/dev/null
  pkill -INT -f rtab_rerun_bridge.py 2>/dev/null
  for _ in $(seq 120); do      # the recorder flushes its queue and writes the index; this can take a while
    { [ -n "${BAG_PID:-}" ] && kill -0 $BAG_PID 2>/dev/null; } || pgrep -f rtab_rerun_bridge.py >/dev/null || break
    sleep 1
  done
  [ -d /out ] && cp /tmp/*.log /out/ 2>/dev/null
  exit 0
}
trap finish INT TERM

log "mission_bridge: route=$ROUTE takeoff alt=$ALT"
/usr/bin/python3 -u $X/scripts/mission_bridge.py --ros-args -p use_sim_time:=true \
  -p takeoff_alt:=$ALT -p route:="$ROUTE" -p reach_radius:=${REACH:-4.0} &
wait $!
log "mission finished"
finish
