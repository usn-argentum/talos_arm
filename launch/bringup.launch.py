import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Production/auto-start launch file - Jetson only.

    Starts just robot_state_publisher + arm_teleop_node. Does NOT start:
      - joy_node: shared with rover_drive via joy_teleop.service, both
        teleop_twist_joy and arm_teleop_node subscribe to the same /joy.
      - arm_bench_feedback_node: fakes /arm_joint_states as if hardware
        were perfectly tracking commands. Must not run once the real
        Teensy micro-ROS client is publishing /arm_joint_states - both
        would race on that topic (see arm_bench_feedback_node's own
        docstring). Real feedback comes from Teensy once connected.
      - rviz2: dev/debug visualization tool, needs a display, not
        appropriate for a headless boot-time service.

    Usage: ros2 launch talos_arm bringup.launch.py
    """
    pkg_share = get_package_share_directory("talos_arm")
    urdf_path = os.path.join(pkg_share, "urdf", "talos_arm.urdf.xacro")
    config_path = os.path.join(pkg_share, "config", "arm_params.yaml")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}],
            remappings=[("/joint_states", "/arm_joint_states")],
        ),
        Node(
            package="talos_arm",
            executable="arm_teleop_node",
            name="arm_teleop_node",
            output="screen",
            parameters=[config_path],
        ),
    ])
