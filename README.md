# talos_arm — Teensy interface

Quick reference for the micro-ROS client on Teensy. Full spec:
[`docs/teensy_interface_spec.md`](docs/teensy_interface_spec.md).

## Topics

| Topic | Direction | Type |
|---|---|---|
| `/arm_joint_cmd` | Jetson → Teensy (subscribe) | `sensor_msgs/msg/JointState` |
| `/arm_joint_states` | Teensy → Jetson (publish) | `sensor_msgs/msg/JointState` |

Joint names: `joint1`, `joint2`, `joint3`, `joint4` (+ `gripper_joint` on
`/arm_joint_cmd` only). Units: radians throughout.

## `/arm_joint_cmd` (subscribe)

Only `name[]` and `position[]` are populated. Published at ~50 Hz. Treat
each message as the **latest target**, not a queue — don't block waiting
for every message.

## `/arm_joint_states` (publish)

Publish measured angle per joint where you have real feedback (encoder).
For any open-loop joint, publish the last commanded value instead and flag
that in a firmware comment — don't fabricate a sensor reading.

## Bench test without the Jetson teleop running

```bash
ros2 topic pub /arm_joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint1','joint2','joint3','joint4'], position: [0.3, -0.2, 0.1, 0.0]}"
ros2 topic echo /arm_joint_states
```

## License

MIT — see [`LICENSE`](LICENSE).
