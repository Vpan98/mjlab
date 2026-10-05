"""Launches the crunches exercise, standalone.

This only starts the `crunches` node. The simulation itself must already
be running with the robot lying on its back, e.g.:

    ros2 launch unitree_g1_mujoco simulation.launch.py initial_pose:=supine
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_g1_mujoco',
            executable='crunches',
            name='g1_exercise_crunches',
            output='screen',
        ),
    ])
