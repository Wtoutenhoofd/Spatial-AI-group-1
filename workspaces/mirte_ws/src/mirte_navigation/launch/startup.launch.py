import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mirte_slam'),
                'launch', 'slam.launch.py',
            )
        ),
        # Show the SLAM map in RViz by default (override with use_rviz:=false on a headless robot)
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    state_manager = Node(
        package='mirte_statemachine',
        executable='StateManager',
        name='StateManager',
        output='screen'
    )

    detector = Node(
        package='mirte_perception',
        executable='GoalGenerator',
        name='GoalGenerator',
        output='screen'
    )

    white_board_tracker = Node(
        package='mirte_navigation',
        executable='WhiteBoardTracker',
        name='WhiteBoardTracker',
        output='screen'
    )

    sandpit_tracker = Node(
        package='mirte_perception',
        executable='ArucoDetector',
        name='ArucoDetector',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the SLAM map (set false on a headless robot)',
        ),
        slam,
        state_manager,
        detector,
        sandpit_tracker,
        white_board_tracker
    ])