from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # 'stand' (default) or 'supine' (required before running crunches)
        DeclareLaunchArgument('initial_pose', default_value='stand'),
        # Raise these (e.g. 4.0 / 3.0) before running the walk exercise;
        # leave at 1.0 for wave/crunches. See walk.py for why.
        DeclareLaunchArgument('leg_kp_scale', default_value='1.0'),
        DeclareLaunchArgument('leg_kd_scale', default_value='1.0'),
        Node(
            package='unitree_g1_mujoco',
            executable='simulation',
            name='g1_simulation',
            output='screen',
            parameters=[{
                'initial_pose': LaunchConfiguration('initial_pose'),
                'leg_kp_scale': ParameterValue(
                    LaunchConfiguration('leg_kp_scale'), value_type=float),
                'leg_kd_scale': ParameterValue(
                    LaunchConfiguration('leg_kd_scale'), value_type=float),
            }],
        ),
    ])
