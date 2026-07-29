import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("talos_arm")
    urdf_path = os.path.join(pkg_share, "urdf", "talos_arm.urdf.xacro")
    config_path = os.path.join(pkg_share, "config", "arm_params.yaml")
    rviz_config_path = os.path.join(pkg_share, "config", "talos_arm.rviz")

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
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        Node(
            package="talos_arm",
            executable="arm_teleop_node",
            name="arm_teleop_node",
            output="screen",
            parameters=[config_path],
        ),
        # Required for RViz to show anything past the base link: without a
        # publisher on /arm_joint_states, robot_state_publisher never gets
        # joint positions and never broadcasts TF for the moving links.
        Node(
            package="talos_arm",
            executable="arm_bench_feedback_node",
            name="arm_bench_feedback_node",
            output="screen",
            parameters=[config_path],
        ),
        # -d loads the saved display config (RobotModel/TF/Grid) — without it
        # RViz opens with a blank session and shows nothing.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
        ),
    ])
