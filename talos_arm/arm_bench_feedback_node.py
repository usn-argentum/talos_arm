#!/usr/bin/env python3
"""
Bench-only stand-in for the Teensy feedback link (see
docs/teensy_interface_spec.md — "Bench testing without hardware").

Tracks the latest /arm_joint_cmd target and republishes it on
/arm_joint_states on a timer, stamped with the current time on every
publish so robot_state_publisher never sees a stale/duplicate stamp.
Simulates perfect, instant joint tracking — good enough to exercise the
teleop -> tf -> rviz chain without hardware.

Do not run this alongside the real Teensy bridge: both publish
/arm_joint_states and would race on the topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PUBLISH_RATE_HZ = 20.0


class ArmBenchFeedbackNode(Node):
    def __init__(self):
        super().__init__("arm_bench_feedback_node")

        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4", "gripper_joint"])
        # Matches arm_teleop_node's assumed home pose (base 0 deg,
        # shoulder/elbow 90 deg, wrist 15 deg — wrist changed 2026-07-23
        # along with its hinge-range correction, see arm_params.yaml) plus
        # gripper open (0.02 m) — keep in sync manually with
        # arm_params.yaml's home_position / gripper_open_position until
        # these come from one shared source.
        self.declare_parameter(
            "home_position", [0.0, 1.57079633, 1.57079633, 0.26179939, 0.02]
        )
        self.joint_names = self.get_parameter("joint_names").value
        self.positions = list(self.get_parameter("home_position").value)[: len(self.joint_names)]

        self.pub = self.create_publisher(JointState, "/arm_joint_states", 10)
        self.create_subscription(JointState, "/arm_joint_cmd", self.cmd_callback, 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_state)

        self.get_logger().info(
            "arm_bench_feedback_node up — echoing /arm_joint_cmd as /arm_joint_states "
            "(bench stand-in, not real hardware feedback)"
        )

    def cmd_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                self.positions[self.joint_names.index(name)] = msg.position[i]

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmBenchFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
