from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_g1_mujoco',
            executable='exercise',
            name='g1_exercise_wave_hello',
            output='screen',
        ),
    ])
