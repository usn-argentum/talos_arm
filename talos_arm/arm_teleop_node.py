#!/usr/bin/env python3
"""
talos_arm teleop node.

Reads /joy, gates on R1 (arm deadman — drive uses L1, kept separate so both
can be held at once), and publishes joint commands to /arm_joint_cmd.

Two modes, toggled with mode_toggle_button (edge-triggered). Deliberately
DIFFERENT stick/button layouts between the two — joint-space doesn't mirror
cartesian's layout, they're independent schemes:

  - joint-space: right stick Y -> joint2 (shoulder), up = shoulder up (sign
                 flipped 2026-07-23 — it used to be inverted: stick down
                 raised the shoulder). Right stick X is unused in this mode.
                 L2/R2 -> joint3 (elbow, paired +/-, fully bidirectional: L2
                 negative, R2 positive — do NOT go back to one-trigger-per-
                 joint, that let a joint ratchet to its limit with no way
                 back). D-pad left/right -> joint1 (base/yaw, direct jog;
                 moved here 2026-07-23, was previously on right stick X).
                 D-pad up/down -> joint4 (wrist, direct jog — same physical
                 control as before, but the wrist itself changed from a
                 twist joint to a hinge, see below).

  - cartesian:   Full 4-joint (yaw+shoulder+elbow+wrist) coupled Jacobian
                 against FIXED world-frame X/Y/Z targets — right stick X ->
                 world X (forward/back), right stick Y -> world Y
                 (sideways), left stick Y -> world Z (height, push down to
                 lower the gripper). Left stick X and D-pad are unused in
                 this mode; base is NOT on D-pad here, it's part of the
                 Jacobian solve alongside the other three joints.

                 This is a deliberate reversal (2026-07-23) of an earlier
                 decoupled, base-relative design (D-pad-jogs-yaw +
                 2-DOF-shoulder/elbow-IK) that had been built specifically
                 to eliminate base/shoulder/elbow cross-talk. That decoupled
                 version was correct and bug-free, but after directly
                 comparing both by feel by temporarily standing up the
                 unified coupled solve as an isolated test node, the unified
                 feel was preferred and asked to be made permanent instead.
                 Base cross-talk with the stick (and D-pad no longer owning
                 the base exclusively) is therefore expected behavior now,
                 not a bug — this file no longer tries to prevent it.

                 Known consequence of keeping this permanently: the
                 near-yaw-axis singularity band diagnosed earlier this
                 project (Jacobian conditioning collapses as the wrist
                 approaches the vertical yaw axis, since yaw's column
                 magnitude is proportional to radius from that axis) is
                 back, confirmed numerically to be essentially unchanged in
                 severity even with wrist now contributing as a real 4th
                 DOF (smallest singular value near that pose is ~0.004
                 either way — the wrist's small ~30 deg range doesn't
                 provide enough of an alternate direction to meaningfully
                 help). A conditioning guard is reinstated for it (see
                 _cartesian_step): blocks steps that keep closing in on the
                 band (radius from the yaw axis not increasing) while still
                 allowing steps that climb back out. Same mechanism as
                 before removal — the guard doesn't care how many other
                 joints are in the solve, only the radius, so re-adding it
                 needed no new logic beyond the radius check itself.

                 joint4 (wrist) is now a normal active joint in the Jacobian
                 (previously excluded entirely): the wrist was originally
                 modeled as a twist joint (axis collinear with the next
                 link's offset, so it had exactly zero effect on
                 end-effector position by construction) — machine team
                 corrected this (2026-07-23) to a hinge/pivot, same style as
                 shoulder/elbow, with a much smaller ~30 deg range. See the
                 urdf's joint4 axis and arm_params.yaml for the paired
                 change. Its Jacobian column is genuinely nonzero now, just
                 small (matches its small range) — it contributes real, if
                 minor, reach the same way shoulder/elbow do.

                 The remaining mechanical singularity (shoulder+elbow
                 collinear, theta3 at exactly 0 or pi) still applies —
                 confirmed the wrist's column is exactly zero at the
                 shoulder=elbow=0 pose specifically (verified across
                 several wrist angles), so it doesn't rescue that case
                 either; the one-time nudge off that exact pose is kept.

Gripper (placeholder prismatic slide, not mode-dependent): Circle opens,
Square closes, held level (not edge-triggered) — release either and the
gripper holds its last commanded position. Published as an extra joint
in the same /arm_joint_cmd JointState alongside joint1-4. Unaffected by
any of today's changes — this logic doesn't look at cartesian_mode at all.

All kinematic numbers come from arm_params.yaml — nothing here is hardcoded.
"""

import math
import os

import numpy as np
import ikpy.chain
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Point

# DualSense indices, confirmed live via `ros2 topic echo /joy` on this
# controller/driver combo. Note this driver's buttons array is only 13
# elements (0-12) — there is no touchpad-click button in it, so mode
# toggle uses Share/Create (8) instead.
AXIS_LEFT_X = 0   # confirmed live 2026-07-23 (used only incidentally so far)
AXIS_LEFT_Y = 1   # confirmed live 2026-07-23
AXIS_L2 = 2
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_R2 = 5
# AXIS_DPAD_X is not independently live-captured — it's determined by
# elimination from data that IS live-confirmed: this driver's axes array has
# exactly 8 slots (0-7, confirmed via capture), and every other slot
# (0,1,2,3,4,5,7) is independently confirmed above/below, leaving 6 as the
# only remaining candidate for the hat's X axis. A hat's X/Y are always
# reported as a consecutive pair by this driver, so this isn't a blind guess
# about ordering, but flagging it since it wasn't captured by directly
# wiggling D-pad left/right the way every other axis here was.
AXIS_DPAD_X = 6  # hat axis: +1.0 = right, -1.0 = left, 0.0 = released
AXIS_DPAD_Y = 7  # hat axis, not a button: +1.0 = up, -1.0 = down, 0.0 = released
BUTTON_R1 = 5
BUTTON_MODE_TOGGLE = 8  # Share/Create
BUTTON_CIRCLE = 1
BUTTON_SQUARE = 3


class ArmTeleopNode(Node):
    def __init__(self):
        super().__init__("arm_teleop_node")

        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("num_joints", 4)
        self.declare_parameter("joint_limits_lower", [-1.57, -1.57, -1.57, -1.57])
        self.declare_parameter("joint_limits_upper", [1.57, 1.57, 1.57, 1.57])
        self.declare_parameter("home_position", [0.0, 1.57079633, 1.57079633, 1.57079633])
        self.declare_parameter("link_lengths", [0.15, 0.20, 0.20, 0.10])
        self.declare_parameter("mount_offset", [0.0, 0.0, 0.10])
        self.declare_parameter("joint_jog_speed", 0.8)
        self.declare_parameter("cartesian_speed", 0.15)
        self.declare_parameter("gripper_joint_name", "gripper_joint")
        self.declare_parameter("gripper_open_position", 0.02)
        self.declare_parameter("gripper_close_position", 0.0)

        self.joint_names = self.get_parameter("joint_names").value
        self.num_joints = self.get_parameter("num_joints").value
        self.lower = self.get_parameter("joint_limits_lower").value
        self.upper = self.get_parameter("joint_limits_upper").value
        self.home_position = self.get_parameter("home_position").value
        self.link_lengths = self.get_parameter("link_lengths").value
        self.mount_offset = self.get_parameter("mount_offset").value
        self.jog_speed = self.get_parameter("joint_jog_speed").value
        self.cart_speed = self.get_parameter("cartesian_speed").value
        self.gripper_joint_name = self.get_parameter("gripper_joint_name").value
        self.gripper_open_position = self.get_parameter("gripper_open_position").value
        self.gripper_close_position = self.get_parameter("gripper_close_position").value

        # Assumed startup pose — base 0 deg, shoulder/elbow 90 deg, wrist
        # 15 deg (wrist value changed 2026-07-23 along with its hinge-range
        # correction — see arm_params.yaml), gripper open. This is what the
        # Jetson side ASSUMES on boot; it is not a verified physical homing
        # result until Teensy firmware actually implements limit-switch
        # homing (see docs/teensy_interface_spec.md) and confirms the arm is
        # really here.
        self.current_angles = list(self.home_position)[: self.num_joints]
        self.gripper_position = self.gripper_open_position
        self.cartesian_mode = False
        self._prev_mode_button = False
        self._ik_unreachable_warned = False
        self._singularity_warned = False

        # ikpy loads the URDF and provides forward_kinematics, which is all
        # that's needed here — the Jacobian used for cartesian mode is
        # derived numerically from it (see _position_jacobian), rather than
        # calling ikpy's own inverse_kinematics(). joint4 is now active
        # (2026-07-23, wrist corrected from twist to hinge — see module
        # docstring) since it genuinely affects position now. gripper_joint
        # stays masked inactive: it's controlled separately via
        # Circle/Square, not by cartesian mode. The mask doesn't affect
        # forward_kinematics, only ikpy's own optimizer (which we no longer
        # call) — FK still uses whatever value is passed for every link.
        urdf_path = os.path.join(
            get_package_share_directory("talos_arm"), "urdf", "talos_arm.urdf.xacro"
        )
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=[
                "base_link", "mount_joint", "arm_base_link",
                "joint1", "link1", "joint2", "link2", "joint3", "link3",
                "joint4", "link4",
            ],
            active_links_mask=[False, False, True, True, True, True, False],
        )

        self.cmd_pub = self.create_publisher(JointState, "/arm_joint_cmd", 10)
        # Debug-only: end-effector position resulting from this cycle's
        # velocity step, so it can be observed directly instead of inferred
        # by re-deriving FK from the published joint commands.
        self.target_debug_pub = self.create_publisher(Point, "/arm_cartesian_target_debug", 10)
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        # Feedback from Teensy (or ros2 topic pub during bench testing)
        self.create_subscription(JointState, "/arm_joint_states", self.feedback_callback, 10)

        self.get_logger().info("talos_arm teleop node up — joint-space mode active")

    def feedback_callback(self, msg: JointState):
        # Keep a running estimate of actual angles — cartesian mode integrates
        # directly from this every cycle, and joint-space mode does the same.
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                if idx < len(msg.position):
                    self.current_angles[idx] = msg.position[i] if i < len(msg.position) else self.current_angles[idx]

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) <= BUTTON_R1 or not msg.buttons[BUTTON_R1]:
            return  # arm deadman not held — publish nothing, stay put

        # Edge-triggered mode toggle
        mode_button = bool(msg.buttons[BUTTON_MODE_TOGGLE]) if len(msg.buttons) > BUTTON_MODE_TOGGLE else False
        if mode_button and not self._prev_mode_button:
            self.cartesian_mode = not self.cartesian_mode
            # No seeding needed: cartesian mode integrates joint velocities
            # directly from current_angles every cycle (same pattern
            # joint-space mode already uses), so there's no separate target
            # state to initialize on entry.
            self.get_logger().info(f"mode -> {'cartesian' if self.cartesian_mode else 'joint-space'}")
        self._prev_mode_button = mode_button

        if self.cartesian_mode:
            targets = self._cartesian_step(msg)
        else:
            targets = self._joint_space_step(msg)

        self._update_gripper(msg)
        self._publish(targets)

    def _update_gripper(self, msg: Joy):
        circle = bool(msg.buttons[BUTTON_CIRCLE]) if len(msg.buttons) > BUTTON_CIRCLE else False
        square = bool(msg.buttons[BUTTON_SQUARE]) if len(msg.buttons) > BUTTON_SQUARE else False
        if circle:
            self.gripper_position = self.gripper_open_position
        elif square:
            self.gripper_position = self.gripper_close_position
        # neither held -> hold last commanded position

    @staticmethod
    def _trigger_press(raw: float) -> float:
        # This DualSense/joy_node combo rests L2/R2 at 1.0 and bottoms out at
        # -1.0 (confirmed via `ros2 topic echo /joy`), not the 0-at-rest
        # convention the rest of this file assumes. Remap to a 0 (rest) ->
        # 1 (full press) fraction before it's used as a delta.
        return (1.0 - raw) / 2.0

    def _joint_space_step(self, msg: Joy):
        dt = 0.02  # matches typical /joy publish rate; refine once measured
        deltas = [0.0] * self.num_joints
        if self.num_joints > 0:
            # Base/yaw moved here from right stick X (2026-07-23) — right
            # stick is shoulder-only in this mode now (see below).
            deltas[0] = msg.axes[AXIS_DPAD_X] * self.jog_speed * dt
        if self.num_joints > 1:
            # Sign flipped (2026-07-23): this was inverted before (stick
            # down raised the shoulder) — negated so stick up = shoulder up.
            deltas[1] = -msg.axes[AXIS_RIGHT_Y] * self.jog_speed * dt
        if self.num_joints > 2:
            # Paired trigger control: L2 presses negative, R2 presses positive.
            # Both are 0 at rest (via _trigger_press) so they cancel out when
            # neither — or both — is held, and add when only one is pressed.
            deltas[2] = (
                self._trigger_press(msg.axes[AXIS_R2]) - self._trigger_press(msg.axes[AXIS_L2])
            ) * self.jog_speed * dt
        if self.num_joints > 3:
            # D-pad up/down is a hat axis, already 0 at rest with a signed
            # range — no remap needed like the triggers. Unchanged control
            # (D-pad up/down already drove joint4 before) — only the joint's
            # own mechanical type changed (twist -> hinge, see module
            # docstring), not which input drives it.
            deltas[3] = msg.axes[AXIS_DPAD_Y] * self.jog_speed * dt

        targets = [self.current_angles[i] + deltas[i] for i in range(self.num_joints)]
        return [max(self.lower[i], min(self.upper[i], targets[i])) for i in range(self.num_joints)]

    def _forward_kinematics(self, thetas):
        # thetas = (theta1, theta2, theta3, theta4) -- all 4 are active
        # variables in cartesian mode now that the wrist is a hinge with a
        # real (if small) effect on position (see module docstring).
        theta1, theta2, theta3, theta4 = thetas
        full = [0.0, 0.0, theta1, theta2, theta3, theta4, self.gripper_position]
        return tuple(self.chain.forward_kinematics(full)[:3, 3])

    def _position_jacobian(self, thetas, eps=1e-6):
        # 3x4 positional Jacobian via central-ish finite differences on
        # ikpy's forward_kinematics, same technique as before, now over all
        # 4 joints instead of 3.
        base = np.array(self._forward_kinematics(thetas))
        J = np.zeros((3, 4))
        th = np.array(thetas)
        for i in range(4):
            perturbed = th.copy()
            perturbed[i] += eps
            J[:, i] = (np.array(self._forward_kinematics(tuple(perturbed))) - base) / eps
        return J

    def _cartesian_step(self, msg: Joy):
        # Full 4-joint (yaw+shoulder+elbow+wrist) coupled Jacobian against
        # fixed world-frame X/Y/Z targets. See module docstring for why this
        # unified design is the permanent choice (tested directly against a
        # decoupled base-relative alternative, this feel was preferred) and
        # for the singularity-band consequence of that choice.
        dt = 0.02  # matches typical /joy publish rate; refine once measured
        theta1, theta2, theta3, theta4 = self.current_angles[:4]

        # Exact rank-deficient pose: shoulder AND elbow both at exactly 0
        # (both links pointing straight up) — verified the wrist's own
        # column is ALSO exactly zero here regardless of its own angle (see
        # module docstring), so it doesn't rescue this case. One real nudge
        # off first, same reasoning as previous versions of this code.
        if theta2 < 1e-3 and theta3 < 1e-3:
            new_theta3 = theta3 + 0.05
            targets = [theta1, theta2, new_theta3, theta4][: self.num_joints]
            return [max(self.lower[i], min(self.upper[i], targets[i])) for i in range(self.num_joints)]

        v = np.array([
            msg.axes[AXIS_RIGHT_X] * self.cart_speed,
            msg.axes[AXIS_RIGHT_Y] * self.cart_speed,
            msg.axes[AXIS_LEFT_Y] * self.cart_speed,
        ])

        thetas = (theta1, theta2, theta3, theta4)
        x0, y0, _ = self._forward_kinematics(thetas)
        r0 = math.hypot(x0, y0)

        J = self._position_jacobian(thetas)
        # Damped least-squares — same damping value already validated live
        # for this arm's singularities.
        damping = 0.08
        J_pinv = J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(3))
        joint_vel = J_pinv @ v

        # Magnitude-preserving velocity cap, not per-component clipping —
        # np.clip per-component would not preserve the vector's direction.
        max_vel = 2.0 * self.jog_speed
        speed = float(np.linalg.norm(joint_vel))
        if speed > max_vel:
            if not self._ik_unreachable_warned:
                self.get_logger().warn(
                    "cartesian joint velocity hit the safety cap (likely near a "
                    "kinematic singularity) — motion is being deliberately slowed here"
                )
                self._ik_unreachable_warned = True
            joint_vel = joint_vel * (max_vel / speed)
        else:
            self._ik_unreachable_warned = False

        new_thetas = [thetas[i] + float(joint_vel[i]) * dt for i in range(4)]

        new_x, new_y, new_z = self._forward_kinematics(tuple(new_thetas))

        # Singularity conditioning guard, reinstated (2026-07-23) now that
        # the unified/coupled solve is permanent and yaw shares a Jacobian
        # with the rest again. Same mechanism as before: joint1's axis is
        # z, so its Jacobian column magnitude is EXACTLY the radius from
        # that axis, r = hypot(x, y) — a structural fact independent of how
        # many other joints are in the solve, so adding wrist as a 4th
        # column doesn't change it (confirmed numerically: smallest
        # singular value near this band is ~0.004 either way). Block steps
        # that keep closing in on the band (r not increasing) while still
        # allowing steps that climb back out (r increasing).
        SINGULARITY_RADIUS = 0.05
        r1 = math.hypot(new_x, new_y)
        if r0 < SINGULARITY_RADIUS and r1 <= r0:
            if not self._singularity_warned:
                self.get_logger().warn(
                    f"cartesian mode: near yaw-axis singularity (radius={r0:.3f}m < "
                    f"{SINGULARITY_RADIUS}m) — blocking motion that closes in further; "
                    "move the stick the other way to climb back out"
                )
                self._singularity_warned = True
            targets = list(self.current_angles[: self.num_joints])
            return [max(self.lower[i], min(self.upper[i], targets[i])) for i in range(self.num_joints)]
        else:
            self._singularity_warned = False

        self.target_debug_pub.publish(Point(x=new_x, y=new_y, z=new_z))

        targets = new_thetas[: self.num_joints]
        return [max(self.lower[i], min(self.upper[i], targets[i])) for i in range(self.num_joints)]

    def _publish(self, targets):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names) + [self.gripper_joint_name]
        msg.position = list(targets) + [self.gripper_position]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
