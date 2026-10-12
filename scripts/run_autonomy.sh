#!/usr/bin/env bash
# Start the PX4 + ROS 2 autonomy container (needs the Isaac Sim side from run_sim.sh running).
#   ROUTE="E,N,U;E,N,U;..."  route points in metres relative to the take-off point
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker rm -f x500_autonomy >/dev/null 2>&1 || true
xhost +local:root >/dev/null 2>&1 || true
exec docker run --rm --name x500_autonomy --network host --ipc=host --gpus all \
  -e ROUTE="${ROUTE:-0,40,12}" -e ALT="${ALT:-12.0}" -e REACH="${REACH:-4.0}" \
  -e RVIZ="${RVIZ:-0}" -e RERUN="${RERUN:-0}" -e MAP_RANGE="${MAP_RANGE:-25}" \
  -e EGO_MAX_VEL="${EGO_MAX_VEL:-2.5}" -e EGO_MAX_ACC="${EGO_MAX_ACC:-2.5}" -e EGO_HORIZON="${EGO_HORIZON:-16}" \
  -e MAP_X="${MAP_X:-260}" -e MAP_Y="${MAP_Y:-260}" -e MAP_Z="${MAP_Z:-50}" -e MAP_RES="${MAP_RES:-0.3}" \
  -e DEPTH_RANGE="${DEPTH_RANGE:-15}" -e DEPTH_TIMEOUT="${DEPTH_TIMEOUT:-3.0}" \
  -e DISPLAY="${DISPLAY:-:1}" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$HERE/ros":/x500:ro \
  ${OUT_DIR:+-v "$OUT_DIR":/out} \
  "${IMAGE:-x500-city-nav:latest}" bash /x500/autonomy.sh
