"""Launches the walk exercise, standalone.

This only starts the `walk` node. The simulation itself must already be
running with the leg gains raised, e.g.:

    ros2 launch unitree_g1_mujoco simulation.launch.py \
        leg_kp_scale:=4.0 leg_kd_scale:=3.0

The default parameters here (step_length=0.06, step_time=0.5, n_steps=6)
are the only combination validated in headless MuJoCo not to fall; see
walk.py for details.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('step_length', default_value='0.06'),
        DeclareLaunchArgument('step_time', default_value='0.5'),
        DeclareLaunchArgument('n_steps', default_value='6'),
        Node(
            package='unitree_g1_mujoco',
            executable='walk',
            name='g1_exercise_walk',
            output='screen',
            parameters=[{
                'step_length': ParameterValue(
                    LaunchConfiguration('step_length'), value_type=float),
                'step_time': ParameterValue(
                    LaunchConfiguration('step_time'), value_type=float),
                'n_steps': ParameterValue(
                    LaunchConfiguration('n_steps'), value_type=int),
            }],
        ),
    ])
