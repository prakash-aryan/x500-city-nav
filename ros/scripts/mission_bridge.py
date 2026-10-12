#!/usr/bin/env python3
"""
Bridge Ego-Planner's trajectory command to PX4 offboard control.

Ego-Planner (plan_manage/traj_server) publishes quadrotor_msgs/PositionCommand on /position_cmd:
a smooth B-spline trajectory sample (position + velocity + acceleration + yaw) in the ENU world
frame. This node converts each sample to a PX4 NED TrajectorySetpoint and streams it over
uXRCE-DDS, and runs the takeoff state machine that Ego-Planner itself does not do:

  INIT  -> stream offboard mode + hover setpoint at the home XY, climb to takeoff_alt
  ARMED -> once stable at altitude, publish the first route point on /move_base_simple/goal
  TRACK -> follow /position_cmd (pos+vel+acc+yaw feedforward); if it goes stale, hold last pose.
           When the drone is within reach_radius of the current route point, the next one is sent.
           After the last one the drone hovers there, then lands if land_at_end is set.

The route is a short list of points along the streets (parameter "route", "E,N,U;E,N,U;..." in metres
relative to the take-off point). Ego-Planner plans the actual collision-free path between them from the
depth image.

ENU (E=x, N=y, U=z) -> PX4 NED (N, E, D) = (y, x, -z); yaw_ned = pi/2 - yaw_enu.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

PX4_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
NAN = float("nan")


class EgoPx4Bridge(Node):
    def __init__(self):
        super().__init__("ego_px4_bridge")
        self.declare_parameter("takeoff_alt", 1.6)
        self.declare_parameter("goal_e", 0.0)       # East (m, ENU)
        self.declare_parameter("goal_n", 9.0)       # North (m, ENU)
        self.declare_parameter("goal_alt", 1.6)     # Up (m, ENU)
        self.declare_parameter("send_goal", True)
        self.declare_parameter("cmd_timeout", 0.4)  # s before /position_cmd is "stale"
        self.declare_parameter("route", "")          # "E,N,U;E,N,U;..." (overrides goal_*)
        self.declare_parameter("reach_radius", 3.0)
        self.declare_parameter("land_at_end", True)
        g = self.get_parameter
        self.takeoff_alt = float(g("takeoff_alt").value)
        self.goal_e = float(g("goal_e").value)
        self.goal_n = float(g("goal_n").value)
        self.goal_alt = float(g("goal_alt").value)
        self.send_goal = bool(g("send_goal").value)
        self.cmd_timeout = float(g("cmd_timeout").value)
        route = str(g("route").value).strip()
        self.route = ([tuple(float(v) for v in pt.split(",")) for pt in route.split(";") if pt.strip()]
                      or [(self.goal_e, self.goal_n, self.goal_alt)])
        self.wp = 0
        self.goal_e, self.goal_n, self.goal_alt = self.route[0]
        self.reach_radius = float(g("reach_radius").value)
        self.land_at_end = bool(g("land_at_end").value)
        self.done_since = None
        self.landing = False
        self.landed_since = None

        self.pose = None            # (E,N,U)
        self.home = None            # (E,N) captured at first odom
        self.cmd = None             # last PositionCommand
        self.cmd_stamp = 0.0
        self.k = 0
        self.armed = False
        self.goal_sent = False
        self.at_alt_since = None

        self.ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.vcmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 1)
        self.create_subscription(Odometry, "/px4_odom", self._on_odom, PX4_QOS)
        self.create_subscription(PositionCommand, "/position_cmd", self._on_cmd, 10)
        self.create_timer(0.05, self.tick)          # 20 Hz offboard stream
        self.get_logger().info(
            f"ego_px4_bridge takeoff_alt={self.takeoff_alt} goal=(E{self.goal_e},N{self.goal_n},U{self.goal_alt})")

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y, p.z)
        if self.home is None:
            self.home = (p.x, p.y)

    def _on_cmd(self, m):
        self.cmd = m
        self.cmd_stamp = self.now_s()

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def send_vehicle_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = command
        m.param1, m.param2 = p1, p2
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 255, 1
        m.from_external = True
        self.vcmd.publish(m)

    def publish_setpoint(self, pos_ned, vel_ned, acc_ned, yaw_ned):
        t = int(self.get_clock().now().nanoseconds / 1000)
        o = OffboardControlMode()
        o.timestamp = t
        o.position = True               # position tracking with vel/acc feedforward
        self.ocm.publish(o)
        s = TrajectorySetpoint()
        s.timestamp = t
        s.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        s.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])] if vel_ned else [NAN, NAN, NAN]
        s.acceleration = [float(acc_ned[0]), float(acc_ned[1]), float(acc_ned[2])] if acc_ned else [NAN, NAN, NAN]
        s.yaw = float(yaw_ned)
        self.sp.publish(s)

    def tick(self):
        if self.pose is None or self.home is None:
            # still stream offboard mode so PX4 will accept the mode switch later
            o = OffboardControlMode(); o.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            o.position = True; self.ocm.publish(o)
            return
        pe, pn, pu = self.pose
        he, hn = self.home
        if self.landing:            # PX4 lands on its own; stop streaming offboard setpoints
            if pu < 0.3:
                self.landed_since = self.landed_since or self.now_s()
                if self.now_s() - self.landed_since > 5.0:
                    self.get_logger().info(f"LANDED at (E{pe:.1f},N{pn:.1f}), mission finished")
                    raise SystemExit
            return

        # Only follow Ego once we have climbed and fired the goal; otherwise Ego's premature hover-at-
        # origin /position_cmd would be tracked and the drone never leaves the ground (takeoff race).
        fresh = self.cmd is not None and (self.now_s() - self.cmd_stamp) < self.cmd_timeout
        if fresh and self.goal_sent:
            c = self.cmd
            pos_ned = (c.position.y, c.position.x, -c.position.z)
            vel_ned = (c.velocity.y, c.velocity.x, -c.velocity.z)
            acc_ned = (c.acceleration.y, c.acceleration.x, -c.acceleration.z)
            yaw_ned = math.pi / 2.0 - c.yaw
            self.publish_setpoint(pos_ned, vel_ned, acc_ned, yaw_ned)
        else:
            # hover/takeoff hold at home XY + target altitude, face the goal
            yaw_enu = math.atan2(self.goal_n - hn, self.goal_e - he)
            yaw_ned = math.pi / 2.0 - yaw_enu
            self.publish_setpoint((hn, he, -self.takeoff_alt), None, None, yaw_ned)

        # (Re)send offboard+arm until the drone is actually airborne. A single arm at k==30 can be lost
        # if it lands in the sim's poll-timeout / EKF-not-ready startup window, leaving it stuck on the
        # ground; retrying every ~3 s recovers from that race.
        if pu < 0.6 and (self.k == 30 or (self.k > 30 and self.k % 60 == 0)):
            self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if not self.armed:
                self.armed = True
                self.get_logger().info("arming + offboard (retry until airborne)")
        self.k += 1

        # once stable at altitude, fire the goal to Ego-Planner once
        if self.send_goal and not self.goal_sent and self.armed:
            if abs(pu - self.takeoff_alt) < 0.25:
                if self.at_alt_since is None:
                    self.at_alt_since = self.now_s()
                elif self.now_s() - self.at_alt_since > 2.0:
                    self.fire_goal()
            else:
                self.at_alt_since = None

        # advance along the route
        if self.goal_sent and not self.landing:
            d = math.hypot(pe - (he + self.goal_e), pn - (hn + self.goal_n))
            if d < self.reach_radius and abs(pu - self.goal_alt) < 2.0:
                if self.wp + 1 < len(self.route):
                    self.wp += 1
                    self.goal_e, self.goal_n, self.goal_alt = self.route[self.wp]
                    self.fire_goal()
                elif self.done_since is None:
                    self.done_since = self.now_s()
                    self.get_logger().info(f"ROUTE DONE at (E{pe:.1f},N{pn:.1f},U{pu:.1f})")
                elif self.land_at_end and self.now_s() - self.done_since > 3.0:
                    self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.landing = True
                    self.get_logger().info("LAND")

        if self.k % 40 == 0:
            st = "TRACK" if (fresh and self.goal_sent) else ("HOVER" if self.goal_sent else "CLIMB")
            self.get_logger().info(
                f"[{st}] pos(E{pe:.1f},N{pn:.1f},U{pu:.1f}) goal_sent={self.goal_sent} "
                f"cmd={'fresh' if fresh else 'stale'}")

    def fire_goal(self):
        g = PoseStamped()
        g.header.frame_id = "world"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = self.goal_e      # ENU x=East (relative to take-off point)
        g.pose.position.y = self.goal_n      # ENU y=North
        g.pose.position.z = self.goal_alt
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)
        self.goal_sent = True
        self.get_logger().info(f"GOAL {self.wp + 1}/{len(self.route)} sent to Ego-Planner: "
                               f"(E{self.goal_e},N{self.goal_n},U{self.goal_alt})")


def main():
    rclpy.init()
    n = EgoPx4Bridge()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
