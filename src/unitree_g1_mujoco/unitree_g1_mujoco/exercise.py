#!/usr/bin/env python3
"""G1 exercise: wave hello.

Publishes JointCommand messages on `joint_command` that drive the right arm
through a raise -> wave -> lower sequence. Assumes `simulation.py` is already
running and subscribed to `joint_command`; all joints not commanded here stay
at their last commanded position (the standing pose from the MJCF keyframe).

All target angles are within the joint ranges declared in
g1_23dof_rev_1_0_tcri.mjcf.
"""
import math

import rclpy
from rclpy.node import Node

from unitree_g1_interfaces.msg import JointCommand

WAVE_JOINTS = [
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
]

RAISE_DURATION_S = 2.0    # time to lift the arm into waving position
WAVE_DURATION_S = 4.0     # time spent waving
LOWER_DURATION_S = 2.0    # time to return the arm to the side

SHOULDER_PITCH_TARGET = -1.2   # rad, arm raised forward/up
SHOULDER_ROLL_TARGET = -0.3    # rad, arm out slightly to the side
ELBOW_TARGET = -1.0            # rad, elbow bent so the hand points up
WAVE_AMPLITUDE = 0.5           # rad, wrist swing amplitude
WAVE_FREQUENCY_HZ = 1.0

COMMAND_RATE_HZ = 30.0


class G1WaveHello(Node):

    def __init__(self):
        super().__init__('g1_exercise_wave_hello')
        self.publisher = self.create_publisher(JointCommand, 'joint_command', 10)
        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / COMMAND_RATE_HZ, self._update)
        self.get_logger().info('G1 wave-hello exercise started.')

    def _update(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        shoulder_pitch, shoulder_roll, elbow, wrist = self._trajectory(t)

        msg = JointCommand()
        msg.joint_names = WAVE_JOINTS
        msg.positions = [shoulder_pitch, shoulder_roll, elbow, wrist]
        self.publisher.publish(msg)

    def _trajectory(self, t: float):
        wave_start = RAISE_DURATION_S
        wave_end = wave_start + WAVE_DURATION_S
        lower_end = wave_end + LOWER_DURATION_S

        if t < wave_start:
            frac = t / RAISE_DURATION_S
            return (
                SHOULDER_PITCH_TARGET * frac,
                SHOULDER_ROLL_TARGET * frac,
                ELBOW_TARGET * frac,
                0.0,
            )
        if t < wave_end:
            wave_t = t - wave_start
            wrist = WAVE_AMPLITUDE * math.sin(2 * math.pi * WAVE_FREQUENCY_HZ * wave_t)
            return (SHOULDER_PITCH_TARGET, SHOULDER_ROLL_TARGET, ELBOW_TARGET, wrist)
        if t < lower_end:
            frac = (t - wave_end) / LOWER_DURATION_S
            return (
                SHOULDER_PITCH_TARGET * (1.0 - frac),
                SHOULDER_ROLL_TARGET * (1.0 - frac),
                ELBOW_TARGET * (1.0 - frac),
                0.0,
            )
        return (0.0, 0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = G1WaveHello()
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
