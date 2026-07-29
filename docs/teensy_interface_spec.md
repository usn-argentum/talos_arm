# Arm ROS2 <-> Teensy Interface Spec (v0.1 — draft)

Same pattern as the drive interface (Hermes). Two topics, both `sensor_msgs/msg/JointState`.

## Topic: `/arm_joint_cmd`  (Jetson -> Teensy)
Commanded joint targets. Teensy subscribes.

| Field | Type | Meaning |
|---|---|---|
| `header.stamp` | Time | when this command was generated |
| `name[]` | string[] | joint names, e.g. `["joint1","joint2","joint3","joint4"]` |
| `position[]` | float64[] | target angle per joint, **radians** |

Only `position` is populated for now (no velocity/effort commands yet — flag if
you need those, adding fields now is cheap, later isn't).

Publish rate: matches `/joy` rate, ~50 Hz. Teensy should NOT block waiting for
every message — treat as "latest target," not a queued command stream.

## Topic: `/arm_joint_states`  (Teensy -> Jetson)
Actual joint angles. Teensy publishes.

| Field | Type | Meaning |
|---|---|---|
| `header.stamp` | Time | when this reading was taken |
| `name[]` | string[] | same joint names as above |
| `position[]` | float64[] | measured angle per joint, **radians** |

**Open question for Teensy team:** which joints actually have position
feedback (encoder) vs open-loop steppers? If a joint has no real feedback,
publish the last commanded value for it and flag that clearly in your
firmware comments — don't silently fabricate a sensor reading.

## Units
Radians throughout, matching the servo-angle convention already used on
Hermes (`std_msgs/Float32`). Confirm with Teensy team if any joint's native
driver works in different units (steps, degrees) — conversion happens in
firmware, not on the Jetson side.

## Bench testing without hardware
Jetson side can be tested standalone right now:
```bash
ros2 launch talos_arm arm.launch.py
ros2 topic pub /arm_joint_states sensor_msgs/msg/JointState \
  "{name: ['joint1','joint2','joint3','joint4'], position: [0.3, -0.2, 0.1, 0.0]}"
ros2 topic echo /arm_joint_cmd
```
Teensy team can do the same on their end once firmware exists — publish fake
`/arm_joint_cmd` values by hand and confirm their code reacts correctly,
before ever touching the Jetson side.

## Not yet decided (flag if this affects your firmware design)
- DOF count is currently a **placeholder (4)** — pending machine team CAD
- Whether cartesian-mode teleop ever reaches Teensy any differently than
  joint-space mode (answer: no — Teensy always receives joint angles either
  way; IK happens Jetson-side)

## Homing (needed — not yet implemented anywhere)
Jetson side currently just *assumes* the arm powers up at a fixed pose
(base 0 deg, shoulder/elbow 90 deg, wrist 15 deg, gripper open — see
`home_position` in `arm_params.yaml`) and seeds its internal joint-angle
estimate with that assumption. That is **not** real homing — there is no
sensor confirming the arm is actually there, so if it powers up anywhere
else, the Jetson side has a silently wrong estimate.

Needed from Teensy/machine team:
- Physical limit switches per joint (or equivalent — confirm what's
  actually feasible per joint).
- Firmware homing routine: drive each joint to its switch, zero/calibrate
  from that known position, then move to the shared home pose above.
- Some way for Jetson to know homing actually completed (e.g. a homed=true
  field, or just trust the first `/arm_joint_states` reading after boot —
  flag which is more feasible).

Joint limits are also currently placeholders pending real mechanical
confirmation (kept in sync with `arm_params.yaml`, which is the source of
truth — flag here if this drifts out of sync again): base -180..180 deg,
shoulder 0..120 deg (not symmetric — 0 is one end of travel; tightened
from an earlier 0..200 deg placeholder, see `arm_params.yaml` for why),
elbow 0..180 deg, wrist 0..30 deg (hinge/pivot, not a twist joint — see
`arm_params.yaml` and the urdf). Flag if any of these don't match real
hardware stops.
