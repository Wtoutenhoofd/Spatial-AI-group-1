import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_map_path = os.path.join(os.path.expanduser('~'), 'mirte_maps', 'map')

    use_rviz = LaunchConfiguration('use_rviz')
    autosave = LaunchConfiguration('autosave')
    map_path = LaunchConfiguration('map_path')
    save_period = LaunchConfiguration('save_period')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mirte_slam'),
                'launch', 'slam.launch.py',
            )
        ),
        # Show the SLAM map in RViz by default (override with use_rviz:=false on a headless robot);
        # forward the autosave settings so the map is periodically written to disk.
        launch_arguments={
            'use_rviz': use_rviz,
            'autosave': autosave,
            'map_path': map_path,
            'save_period': save_period,
        }.items(),
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
        DeclareLaunchArgument(
            'autosave',
            default_value='true',
            description='Periodically save the SLAM map to disk (set false to disable)',
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value=default_map_path,
            description='Base path for the saved map; writes <map_path>.pgm and <map_path>.yaml',
        ),
        DeclareLaunchArgument(
            'save_period',
            default_value='10.0',
            description='Seconds between map autosaves',
        ),
        slam,
        state_manager,
        detector,
        sandpit_tracker,
        white_board_tracker
    ])