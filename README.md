# talos_arm

ROS2 (ament_python) package for the Talos arm: joystick teleop (joint-space
and cartesian), forward/inverse kinematics, and the interface contract for
the arm's onboard controller (Teensy).

This repo covers the **Jetson-side** ROS2 nodes only. The arm itself is
driven by a Teensy microcontroller, bridged to ROS2 via a **micro-ROS
client** running on the Teensy — that firmware doesn't exist yet and is
the next piece of this project (see [Building the Teensy micro-ROS
client](#building-the-teensy-micro-ros-client) below).

## Architecture

```
   [DualSense controller]
            |  /joy
            v
   [arm_teleop_node]  (this repo, runs on Jetson)
      - joint-space or cartesian teleop
      - FK/IK via ikpy, using urdf/talos_arm.urdf.xacro
      - clamps to joint limits from config/arm_params.yaml
            |  /arm_joint_cmd  (sensor_msgs/JointState)
            v
   [micro-ROS client on Teensy]   <-- TO BE WRITTEN, this is the next step
      - subscribes /arm_joint_cmd, drives motors/servos
      - publishes /arm_joint_states back with measured/last-commanded angles
            ^
            |  /arm_joint_states (sensor_msgs/JointState)
   [robot_state_publisher] -> TF -> RViz
```

`arm_bench_feedback_node` is a software stand-in for that Teensy link (see
below) so the Jetson side can be developed and tested with no hardware
attached.

## Nodes

- **arm_teleop_node** — reads `/joy`, gated on the R1 deadman. Toggles
  between joint-space and cartesian modes (Share/Create button). Publishes
  `/arm_joint_cmd`. All kinematic numbers (joint limits, link lengths, home
  pose, control mapping) come from `config/arm_params.yaml` — nothing is
  hardcoded in the node.
- **arm_bench_feedback_node** — bench-only stand-in for the Teensy feedback
  link. Echoes the latest `/arm_joint_cmd` back out as `/arm_joint_states`
  at 20 Hz, simulating perfect/instant joint tracking. **Do not run this
  alongside the real Teensy bridge** — both publish `/arm_joint_states` and
  will race on the topic.

## Running (bench, no hardware)

```bash
ros2 launch talos_arm arm.launch.py
```

This brings up `robot_state_publisher`, `joy_node`, `arm_teleop_node`,
`arm_bench_feedback_node`, and RViz (pre-loaded display config). Hold R1 and
drive the joints with the DualSense controller — see the docstring in
`talos_arm/arm_teleop_node.py` for the full control layout.

`ikpy` is pip-only, install manually first:
```bash
pip install ikpy --break-system-packages
```

## ROS2 <-> Teensy interface

Full spec: [`docs/teensy_interface_spec.md`](docs/teensy_interface_spec.md).
Summary:

| Topic | Direction | Type | Notes |
|---|---|---|---|
| `/arm_joint_cmd` | Jetson -> Teensy | `sensor_msgs/msg/JointState` | `name[]` + `position[]` (radians) only, ~50 Hz. Treat as "latest target," not a queued stream — don't block waiting for every message. |
| `/arm_joint_states` | Teensy -> Jetson | `sensor_msgs/msg/JointState` | Measured angle per joint where real feedback exists; if a joint is open-loop, publish the last commanded value and say so clearly in firmware comments — don't fabricate a sensor reading. |

Units are radians throughout, matching the rest of the codebase.

## Building the Teensy micro-ROS client

This is the piece that turns the two topics above into real motor motion.
It hasn't been written yet — this section is a starting checklist for
whoever picks it up.

1. **Toolchain**: use [`micro_ros_arduino`](https://github.com/micro-ROS/micro_ros_arduino)
   (prebuilt library, easiest path for Teensy/Arduino-framework boards) —
   pick the release that matches this workspace's ROS2 distro. Alternatively,
   `micro_ros_platformio` if the firmware build already uses PlatformIO.
2. **Agent**: micro-ROS boards don't join the ROS graph directly — they talk
   to a `micro-ROS agent` (usually running on the Jetson, over USB serial)
   which bridges DDS for them. Bring it up with:
   ```bash
   ros2 run micro_ros_setup create_agent_ws.sh   # one-time
   ros2 run micro_ros_setup build_agent.sh       # one-time
   ros2 run micro_ros_setup run_agent.sh serial --dev /dev/ttyACM0
   ```
   (adjust the serial device to whatever the Teensy enumerates as).
3. **Firmware skeleton**: init `rcl`/`rclc` support, then create exactly one
   subscriber (`/arm_joint_cmd`) and one publisher (`/arm_joint_states`),
   both `sensor_msgs/msg/JointState`. micro-ROS's default `colcon.meta`
   entity/memory limits are small — bump them if the executor init fails
   with an entity-count error.
4. **Command handling**: on each `/arm_joint_cmd` message, match `name[]`
   against this arm's joint names and drive to the corresponding
   `position[]` (radians). Don't queue commands — always act on the latest
   message received, per the interface spec.
5. **Feedback**: publish `/arm_joint_states` with real encoder positions
   where available. For any joint without position feedback (open-loop
   stepper), publish the last commanded value and flag it in a firmware
   comment — see the open question in the interface spec about which
   joints actually have encoders.
6. **Homing** (currently unimplemented anywhere in the stack — needed
   before this is safe to run on real hardware): drive each joint to a
   physical limit switch on boot, zero from that known position, then move
   to the shared home pose in `config/arm_params.yaml`
   (`home_position`). The Jetson side currently just *assumes* the arm
   powers up at that pose with no way to verify it — real homing on the
   Teensy side is what closes that gap. Decide and implement some way for
   the Jetson to know homing actually completed (e.g. a `homed` flag on
   first boot, or an agreed convention for the first `/arm_joint_states`
   message).
7. **Bench-test firmware without the Jetson side running real hardware
   logic**: publish fake `/arm_joint_cmd` values by hand and confirm the
   firmware reacts correctly, before ever connecting it to
   `arm_teleop_node`:
   ```bash
   ros2 topic pub /arm_joint_cmd sensor_msgs/msg/JointState \
     "{name: ['joint1','joint2','joint3','joint4'], position: [0.3, -0.2, 0.1, 0.0]}"
   ros2 topic echo /arm_joint_states
   ```
8. **Integration**: once firmware is bench-verified, launch
   `arm.launch.py` **without** `arm_bench_feedback_node` running (kill it,
   or drop it from the launch file for real-hardware runs) so the real
   Teensy feedback isn't racing the bench stand-in on `/arm_joint_states`.

## Known placeholders (confirm before trusting on real hardware)

Everything in `config/arm_params.yaml` is flagged inline, but the
significant ones:
- **DOF count (4)** — pending machine team CAD.
- **Joint limits** — placeholder ranges, not yet confirmed against real
  hardware stops.
- **Home position** — assumed, not verified by any real homing sequence
  (see above).
- **Which joints have real position feedback vs. open-loop** — open
  question for the Teensy/firmware side.

## License

MIT — see [`LICENSE`](LICENSE).
