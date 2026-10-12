import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # X500 RealSense D435 (Isaac): 640x480 depth, hfov 87deg -> fx=fy=337, cx=320, cy=240
    cx, cy, fx, fy = 320.0, 240.0, 337.0, 337.0
    env = os.environ.get
    max_vel, max_acc = float(env("EGO_MAX_VEL", "2.5")), float(env("EGO_MAX_ACC", "2.5"))
    planning_horizon = float(env("EGO_HORIZON", "16.0"))
    # The grid map is centred on the take-off point; it must cover the whole route.
    map_x, map_y, map_z = float(env("MAP_X", "260")), float(env("MAP_Y", "260")), float(env("MAP_Z", "50"))
    res = float(env("MAP_RES", "0.3"))
    ray = float(env("DEPTH_RANGE", "15.0"))

    ego = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', '/px4_odom'),
            ('grid_map/odom', '/px4_odom'),
            ('grid_map/depth', '/quadrotor/camera/depth'),
        ],
        parameters=[{
            'use_sim_time': True,                  # trajectories and timeouts in simulation time
            'fsm/flight_type': 1,                  # 1 = manual target via /move_base_simple/goal
            'fsm/thresh_replan_time': 1.0,
            'fsm/thresh_no_replan_meter': 1.0,
            'fsm/planning_horizon': planning_horizon,
            'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': False,
            'fsm/fail_safe': True,
            'fsm/waypoint_num': 0,

            'grid_map/resolution': res,
            'grid_map/map_size_x': map_x,
            'grid_map/map_size_y': map_y,
            'grid_map/map_size_z': map_z,
            'grid_map/local_update_range_x': ray + 3.0,
            'grid_map/local_update_range_y': ray + 3.0,
            'grid_map/local_update_range_z': 10.0,
            'grid_map/obstacles_inflation': 0.7,
            'grid_map/local_map_margin': 10,
            'grid_map/ground_height': -1.0,
            'grid_map/cx': cx, 'grid_map/cy': cy, 'grid_map/fx': fx, 'grid_map/fy': fy,
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.15,
            'grid_map/depth_filter_maxdist': ray,
            'grid_map/depth_filter_mindist': 0.2,
            'grid_map/depth_filter_margin': 2,
            'grid_map/k_depth_scaling_factor': 1000.0,   # 32FC1 m -> mm uint16 round-trip
            'grid_map/skip_pixel': 2,
            'grid_map/p_hit': 0.65, 'grid_map/p_miss': 0.35,
            'grid_map/p_min': 0.12, 'grid_map/p_max': 0.90, 'grid_map/p_occ': 0.80,
            'grid_map/min_ray_length': 0.1,
            'grid_map/max_ray_length': ray,
            'grid_map/virtual_ceil_height': map_z - 2.0,
            'grid_map/visualization_truncate_height': map_z - 2.0,
            'grid_map/show_occ_time': False,
            'grid_map/pose_type': 2,
            'grid_map/odom_depth_timeout': float(env('DEPTH_TIMEOUT', '3.0')),                 # 2 = depth image + odometry
            'grid_map/frame_id': 'odom',

            'manager/max_vel': max_vel,
            'manager/max_acc': max_acc,
            'manager/max_jerk': 4.0,
            'manager/control_points_distance': 0.6,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': planning_horizon,
            'manager/use_distinctive_trajs': True,
            'manager/drone_id': 0,

            'optimization/lambda_smooth': 1.0,
            'optimization/lambda_collision': 0.5,
            'optimization/lambda_feasibility': 0.1,
            'optimization/lambda_fitness': 1.0,
            'optimization/dist0': 1.0,
            'optimization/swarm_clearance': 0.5,
            'optimization/max_vel': max_vel,
            'optimization/max_acc': max_acc,

            'bspline/limit_vel': max_vel,
            'bspline/limit_acc': max_acc,
            'bspline/limit_ratio': 1.1,

            'prediction/obj_num': 0,
            'prediction/lambda': 1.0,
            'prediction/predict_rate': 1.0,
        }],
    )

    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'traj_server/time_forward': 1.0}],
    )

    return LaunchDescription([ego, traj_server])
