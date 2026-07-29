import os
from glob import glob
from setuptools import setup

package_name = "talos_arm"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # *.rviz must be globbed too, not just *.yaml — otherwise talos_arm.rviz
        # never reaches install/share and arm.launch.py's `-d` path silently
        # points at a nonexistent file (RViz falls back to a blank session).
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    # ikpy is pip-only (not a rosdep/apt package) — install manually with
    # `pip install ikpy --break-system-packages` before running this node.
    install_requires=["setuptools", "ikpy", "numpy"],
    zip_safe=True,
    maintainer="Shati",
    maintainer_email="you@example.com",
    description="Arm control — joint-space + cartesian teleop, FK/IK, Teensy interface",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_teleop_node = talos_arm.arm_teleop_node:main",
            "arm_bench_feedback_node = talos_arm.arm_bench_feedback_node:main",
        ],
    },
)
