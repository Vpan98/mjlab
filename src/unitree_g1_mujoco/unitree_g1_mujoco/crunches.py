#!/usr/bin/env python3
"""G1 exercise: crunches.

The G1_23dof model has no torso/spine joint (`waist_yaw_joint` only yaws,
it does not pitch), so a classic sit-up-style curl of the torso toward the
legs is not physically possible with this model. What IS achievable with
the hip and knee joints is a **reverse crunch**: lying on the back, pulling
both knees up toward the chest and extending back out, which is a standard
bodyweight ab exercise and exercises the same muscles.

This node only commands the hip_pitch/knee joints; it assumes
`simulation.py` was started with `initial_pose:=supine` (see
`crunches.launch.py`) so the robot is lying on its back before this runs.
There is no balance feedback — in headless testing the pelvis rocks by
roughly +/-0.15 m through each cycle but stays on the ground and does not
flip over, for the default amplitude/frequency below. Larger amplitude or
a higher frequency is untested.
"""
import math

import rclpy
from rclpy.node import Node

from unitree_g1_interfaces.msg import JointCommand

CRUNCH_JOINTS = [
    'left_hip_pitch_joint', 'right_hip_pitch_joint',
    'left_knee_joint', 'right_knee_joint',
]

HIP_TARGET = -0.7       # rad, hip flexion at full crunch
KNEE_TARGET = 1.1        # rad, knee flexion at full crunch
REP_DURATION_S = 2.5      # one full curl-and-extend cycle
N_REPS = 6
RAMP_REPS = 1.0            # ease the first rep in instead of starting at speed

COMMAND_RATE_HZ = 50.0


class G1Crunches(Node):

    def __init__(self):
        super().__init__('g1_exercise_crunches')
        self.publisher = self.create_publisher(JointCommand, 'joint_command', 10)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / COMMAND_RATE_HZ, self._update)
        total_s = N_REPS * REP_DURATION_S
        self.get_logger().info(
            f'G1 crunches exercise started: {N_REPS} reps, ~{total_s:.1f}s. '
            'Requires simulation.py to have been launched with '
            'initial_pose:=supine.'
        )

    def _update(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        total_s = N_REPS * REP_DURATION_S

        if t >= total_s:
            hip, knee = 0.0, 0.0
        else:
            freq = 1.0 / REP_DURATION_S
            ramp = min(t / (RAMP_REPS * REP_DURATION_S), 1.0)
            phase = 0.5 * (1.0 - math.cos(2.0 * math.pi * freq * t))  # 0..1..0
            amount = ramp * phase
            hip = HIP_TARGET * amount
            knee = KNEE_TARGET * amount

        msg = JointCommand()
        msg.joint_names = CRUNCH_JOINTS
        msg.positions = [hip, hip, knee, knee]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = G1Crunches()
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
