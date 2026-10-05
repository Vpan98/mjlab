#!/usr/bin/env python3
"""MuJoCo simulation node for the Unitree G1.

Steps the G1 MJCF model in MuJoCo, opens a passive viewer, and bridges the
simulation to ROS 2:

  Publishes:
    joint_states  (sensor_msgs/JointState)   - position/velocity/effort of
                                                the 23 actuated joints
    imu/pelvis    (sensor_msgs/Imu)          - pelvis IMU (gyro + accel)
    imu/torso     (sensor_msgs/Imu)          - torso IMU (gyro + accel)

  Subscribes:
    joint_command (unitree_g1_interfaces/JointCommand) - target positions
                                                          for named joints
"""
import os

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from unitree_g1_interfaces.msg import JointCommand

# The 23 actuated joints of the G1_23dof model, in an arbitrary but fixed
# order used for the joint_states message. Actual qpos/qvel/actuator indices
# are looked up by name below, so this list does not need to match MJCF
# declaration order.
JOINT_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
]

MJCF_RELATIVE_PATH = os.path.join(
    'mjcf', 'unitree_g1', 'mjcf', 'scene_g1_tcri.mjcf'
)
STAND_KEYFRAME = 'stand'
STATE_PUBLISH_RATE_HZ = 50.0

# Pelvis quaternion (w, x, y, z) for lying on the back, used by the
# crunches exercise. 180 deg rotation about the world Y axis.
SUPINE_QUAT = (0.0, 0.0, 1.0, 0.0)
SUPINE_START_Z = 0.22   # dropped from here and allowed to settle on the floor

LEG_JOINT_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
]


class G1Simulation(Node):

    def __init__(self):
        super().__init__('g1_simulation')

        self.declare_parameter('initial_pose', 'stand')  # 'stand' | 'supine'
        self.declare_parameter('leg_kp_scale', 1.0)
        self.declare_parameter('leg_kd_scale', 1.0)
        initial_pose = self.get_parameter('initial_pose').value
        leg_kp_scale = self.get_parameter('leg_kp_scale').value
        leg_kd_scale = self.get_parameter('leg_kd_scale').value

        mjcf_path = os.path.join(
            get_package_share_directory('unitree_g1_mujoco'), MJCF_RELATIVE_PATH
        )
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)

        if leg_kp_scale != 1.0 or leg_kd_scale != 1.0:
            self._scale_leg_gains(leg_kp_scale, leg_kd_scale)

        if initial_pose == 'supine':
            self._reset_supine()
        else:
            if initial_pose != 'stand':
                self.get_logger().warn(
                    f"Unknown initial_pose '{initial_pose}', using 'stand'."
                )
            key_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_KEY, STAND_KEYFRAME
            )
            if key_id >= 0:
                mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            else:
                mujoco.mj_resetData(self.model, self.data)

        # Resolve qpos/qvel/actuator indices for each joint once, by name.
        self._qpos_adr = {}
        self._qvel_adr = {}
        self._actuator_id = {}
        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self._qpos_adr[name] = self.model.jnt_qposadr[joint_id]
            self._qvel_adr[name] = self.model.jnt_dofadr[joint_id]
            self._actuator_id[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pelvis_pub = self.create_publisher(Imu, 'imu/pelvis', 10)
        self.imu_torso_pub = self.create_publisher(Imu, 'imu/torso', 10)

        self.create_subscription(
            JointCommand, 'joint_command', self._on_joint_command, 10
        )

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.create_timer(self.model.opt.timestep, self._step_physics)
        self.create_timer(1.0 / STATE_PUBLISH_RATE_HZ, self._publish_state)

        self.get_logger().info(
            f'G1 MuJoCo simulation running (dt={self.model.opt.timestep:.4f}s, '
            f"initial_pose='{initial_pose}')."
        )

    def _scale_leg_gains(self, kp_scale, kd_scale):
        """Scale the position-actuator P and D gains of the 12 leg
        actuators only (arms/waist untouched). Used by the walk exercise,
        whose open-loop tracking needs stiffer legs than the default
        kp=500 gains to stay upright; see gait.py / walk.py for why."""
        for name in LEG_JOINT_NAMES:
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            kp = self.model.actuator_gainprm[actuator_id, 0]
            kd = -self.model.actuator_biasprm[actuator_id, 2]
            self.model.actuator_gainprm[actuator_id, 0] = kp * kp_scale
            self.model.actuator_biasprm[actuator_id, 1] = -kp * kp_scale
            self.model.actuator_biasprm[actuator_id, 2] = -kd * kd_scale

    def _reset_supine(self):
        """Lay the robot on its back: drop it from just above the floor
        with the pelvis pitched 180 deg and let contact settle it, rather
        than hand-placing an exact pose (simpler and more robust than
        computing a penetration-free pose by hand)."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[2] = SUPINE_START_Z
        self.data.qpos[3:7] = SUPINE_QUAT
        mujoco.mj_forward(self.model, self.data)
        settle_steps = int(round(1.5 / self.model.opt.timestep))
        for _ in range(settle_steps):
            mujoco.mj_step(self.model, self.data)

    def _on_joint_command(self, msg: JointCommand):
        for name, position in zip(msg.joint_names, msg.positions):
            actuator_id = self._actuator_id.get(name)
            if actuator_id is None:
                self.get_logger().warn(f'Ignoring unknown joint in command: {name}')
                continue
            self.data.ctrl[actuator_id] = position

    def _step_physics(self):
        if not self.viewer.is_running():
            self.get_logger().info('Viewer closed, shutting down.')
            rclpy.shutdown()
            return
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def _publish_state(self):
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = [self.data.qpos[self._qpos_adr[n]] for n in JOINT_NAMES]
        joint_state.velocity = [self.data.qvel[self._qvel_adr[n]] for n in JOINT_NAMES]
        joint_state.effort = [self.data.actuator_force[self._actuator_id[n]] for n in JOINT_NAMES]
        self.joint_state_pub.publish(joint_state)

        self._publish_imu(
            self.imu_pelvis_pub, stamp,
            'imu-pelvis-angular-velocity', 'imu-pelvis-linear-acceleration',
        )
        self._publish_imu(
            self.imu_torso_pub, stamp,
            'imu-torso-angular-velocity', 'imu-torso-linear-acceleration',
        )

    def _publish_imu(self, publisher, stamp, gyro_sensor_name, accel_sensor_name):
        msg = Imu()
        msg.header.stamp = stamp
        gyro = self.data.sensor(gyro_sensor_name).data
        accel = self.data.sensor(accel_sensor_name).data
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        publisher.publish(msg)

    def destroy_node(self):
        if self.viewer.is_running():
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = G1Simulation()
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
