import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mirte_slam')
    slam_config = os.path.join(pkg, 'config', 'slam_toolbox_config.yaml')
    rviz_config = os.path.join(pkg, 'config', 'slam_rviz.rviz')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': False}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=LaunchConfigurationEquals('use_rviz', 'true'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz for map visualisation (set true only when running on a machine with a display)',
        ),
        slam,
        rviz,
    ])
