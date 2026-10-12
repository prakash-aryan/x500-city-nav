#!/usr/bin/env python3
"""Publish /joint_states spinning the X500's 4 rotor joints so the props rotate in RViz's RobotModel."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = ["j_rotor_0", "j_rotor_1", "j_rotor_2", "j_rotor_3"]
DIRS = [1.0, 1.0, -1.0, -1.0]


class Spinner(Node):
    def __init__(self):
        super().__init__("prop_spinner")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ang = [0.0, 0.0, 0.0, 0.0]
        self.create_timer(0.03, self.tick)

    def tick(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = NAMES
        for i in range(4):
            self.ang[i] += DIRS[i] * 2.5
        m.position = list(self.ang)
        self.pub.publish(m)


def main():
    rclpy.init()
    n = Spinner()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
