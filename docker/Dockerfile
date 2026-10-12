# PX4 SITL + ROS 2 Jazzy + RTAB-Map + Ego-Planner, used by scripts/run_autonomy.sh.
#   docker build -t x500-city-nav:latest docker/
FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        git wget curl ca-certificates gnupg lsb-release sudo cmake ninja-build build-essential ccache \
        python3-pip python3-colcon-common-extensions python3-rosdep python3-scipy python3-empy \
        libasio-dev libtinyxml2-dev libeigen3-dev libpcl-dev \
        ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-launch ros-jazzy-rviz2 ros-jazzy-tf2-tools \
        ros-jazzy-cv-bridge ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs ros-jazzy-visualization-msgs ros-jazzy-robot-state-publisher \
        ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# uXRCE-DDS agent (PX4 <-> ROS 2)
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/Micro-XRCE-DDS-Agent \
    && cd /opt/Micro-XRCE-DDS-Agent && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DUAGENT_USE_SYSTEM_FASTDDS=OFF && make -j"$(nproc)" && make install && ldconfig

# px4_msgs
RUN mkdir -p /root/ros2_ws/src && git clone -b release/1.16 https://github.com/PX4/px4_msgs.git /root/ros2_ws/src/px4_msgs \
    && source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws \
    && colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

# PX4 SITL v1.16.2 with the X500 airframe used by the Isaac Sim side
RUN git clone -b v1.16.2 --recursive https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
    && cd /opt/PX4-Autopilot && bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
    && make px4_sitl_default && rm -rf build/px4_sitl_default/.px4-sitl
COPY airframes/4500_gazebo-classic_x500 /opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4500_gazebo-classic_x500
COPY airframes/4500_gazebo-classic_x500 /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4500_gazebo-classic_x500
RUN chmod +x /opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4500_gazebo-classic_x500

# Ego-Planner (ros2_version) with a small patch for Jazzy and one that runs its trajectories on the node clock
# (simulation time with use_sim_time)
COPY patches/ego_planner_jazzy.patch patches/ego_planner_simtime.patch /tmp/
RUN source /opt/ros/jazzy/setup.bash && mkdir -p /root/ego_ws/src && cd /root/ego_ws/src \
    && git clone --depth 1 -b ros2_version https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git \
    && cd ego-planner-swarm && git apply --reject /tmp/ego_planner_jazzy.patch && git apply /tmp/ego_planner_simtime.patch \
    && cd /root/ego_ws && colcon build --packages-up-to ego_planner \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

# Rerun logging (Python packages installed with uv)
COPY --from=ghcr.io/astral-sh/uv:0.11.23 /uv /usr/local/bin/uv
RUN uv pip install --system --break-system-packages --no-deps rerun-sdk==0.26.2 pyarrow==17.0.0
WORKDIR /root
CMD ["bash"]
