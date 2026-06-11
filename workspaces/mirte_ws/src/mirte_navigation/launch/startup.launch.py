import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mirte_slam'),
                'launch', 'slam.launch.py',
            )
        )
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
        slam,
        state_manager,
        detector,
        sandpit_tracker,
        white_board_tracker
    ])