import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WS = os.path.join(os.path.expanduser('~'), 'ro47007_mirte_ws')

SLAM_CONFIG = os.path.join(WS, 'src', 'mirte_slam', 'config', 'slam_toolbox_config.yaml')
RVIZ_CONFIG = os.path.join(WS, 'src', 'mirte_slam', 'config', 'slam_rviz.rviz')


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mdp_greenhouse_gazebo'),
                'launch', 'greenhouse_sim.launch.py',
            )
        )
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[SLAM_CONFIG, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', RVIZ_CONFIG],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo,
        # Delay SLAM and RViz so Gazebo + robot have time to initialise
        TimerAction(period=8.0, actions=[slam]),
        TimerAction(period=9.0, actions=[rviz]),
    ])
