#!/usr/bin/env python3
"""G1 exercise: walk forward a few short steps.

Open-loop: a DCM/LIPM footstep-and-CoM plan (see gait.py) is converted to
leg joint angles offline via inverse kinematics, then played back as a fixed
time-indexed trajectory. There is no balance feedback — the walk only stays
upright because:

  1. the plan's zero-moment point sits inside the support foot the whole
     time (open-loop dynamic feasibility), and
  2. `simulation.launch.py walk:=true` raises the leg position-actuator
     gains (`leg_kp_scale`/`leg_kd_scale`), which keeps the real tracking
     error small enough that assumption (1) stays valid.

This has only been validated for short, slow steps (default parameters
below). Larger step_length/step_time or more steps WILL likely fall over —
there is no sensing here to catch deviation. Real balance would need a state
estimator (fusing the IMU with leg kinematics) and closed-loop foot
placement, which is out of scope for this node.

Assumes `simulation.py` is running with default leg gains raised (see
`simulation.launch.py`'s `leg_kp_scale`/`leg_kd_scale` args) and the robot
standing (the default `initial_pose`).
"""
import math
import os

import mujoco
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from unitree_g1_interfaces.msg import JointCommand

from .gait import LEG_JOINTS, WalkGenerator

MJCF_RELATIVE_PATH = os.path.join(
    'mjcf', 'unitree_g1', 'mjcf', 'scene_g1_tcri.mjcf'
)

# Validated defaults (headless MuJoCo, leg_kp_scale=4, leg_kd_scale=3): 6
# steps of 6 cm at 0.5 s/step completes without falling. Treat any other
# combination as untested.
DEFAULT_STEP_LENGTH = 0.06
DEFAULT_STEP_TIME = 0.5
DEFAULT_N_STEPS = 6
DEFAULT_HALF_WIDTH = 0.09

PREP_HOLD_S = 0.5     # time to just hold the standing pose before moving
PREP_EASE_S = 2.0      # time to ease from standing into the crouched gait pose
SETTLE_S = 1.0          # extra time the plan holds the final double-support pose
COMMAND_RATE_HZ = 100.0


class G1Walk(Node):

    def __init__(self):
        super().__init__('g1_exercise_walk')

        self.declare_parameter('step_length', DEFAULT_STEP_LENGTH)
        self.declare_parameter('step_time', DEFAULT_STEP_TIME)
        self.declare_parameter('n_steps', DEFAULT_N_STEPS)
        self.declare_parameter('half_width', DEFAULT_HALF_WIDTH)

        step_length = self.get_parameter('step_length').value
        step_time = self.get_parameter('step_time').value
        n_steps = self.get_parameter('n_steps').value
        half_width = self.get_parameter('half_width').value

        if (step_length, step_time, n_steps) != (
            DEFAULT_STEP_LENGTH, DEFAULT_STEP_TIME, DEFAULT_N_STEPS
        ):
            self.get_logger().warn(
                'Using non-default walk parameters '
                f'(step_length={step_length}, step_time={step_time}, '
                f'n_steps={n_steps}); only the defaults have been validated '
                'not to fall.'
            )

        mjcf_path = os.path.join(
            get_package_share_directory('unitree_g1_mujoco'), MJCF_RELATIVE_PATH
        )
        self.generator = WalkGenerator(
            mjcf_path,
            step_length=step_length,
            step_time=step_time,
            n_steps=n_steps,
            half_width=half_width,
            rest_time=1.0,
            shift_time=0.4,
            hold_time=0.0,
            settle_time=SETTLE_S,
        )
        self.joint_names = LEG_JOINTS['left'] + LEG_JOINTS['right']
        self.q_ready = self._concat(*self.generator.joint_targets(0.0)[:2])

        self.publisher = self.create_publisher(JointCommand, 'joint_command', 10)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / COMMAND_RATE_HZ, self._update)

        total_s = PREP_HOLD_S + PREP_EASE_S + self.generator.duration
        self.get_logger().info(
            f'G1 walk exercise started: {n_steps} steps, '
            f'~{total_s:.1f}s total.'
        )

    @staticmethod
    def _concat(left_q, right_q):
        return list(left_q) + list(right_q)

    def _update(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if t < PREP_HOLD_S:
            q_cmd = [0.0] * 12
        elif t < PREP_HOLD_S + PREP_EASE_S:
            a = (t - PREP_HOLD_S) / PREP_EASE_S
            a = 0.5 * (1.0 - math.cos(math.pi * a))  # smooth ease-in
            q_cmd = [(1.0 - a) * 0.0 + a * q for q in self.q_ready]
        else:
            tp = min(t - PREP_HOLD_S - PREP_EASE_S, self.generator.duration)
            left_q, right_q, _info = self.generator.joint_targets(tp)
            q_cmd = self._concat(left_q, right_q)

        msg = JointCommand()
        msg.joint_names = self.joint_names
        msg.positions = q_cmd
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = G1Walk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
