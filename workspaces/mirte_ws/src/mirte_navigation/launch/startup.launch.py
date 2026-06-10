from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    state_manager = Node(
        package='mirte_statemachine',
        executable='StateManager',
        name='StateManager',
        output='screen'
    )

    detector = Node(
        package='mirte_perception',
        executable='ArucoDetector',
        name='ArucoDetector',
        output='screen'
    )

    text_detector = Node(
        package='mirte_perception',
        executable='TextDetector',
        name='TextDetector',
        output='screen'
    )

    white_board_tracker = Node(
        package='mirte_navigation',
        executable='WhiteBoardTracker',
        name='WhiteBoardTracker',
        output='screen'
    )

    return LaunchDescription([
        state_manager,
        detector,
        text_detector,
        white_board_tracker
    ])
