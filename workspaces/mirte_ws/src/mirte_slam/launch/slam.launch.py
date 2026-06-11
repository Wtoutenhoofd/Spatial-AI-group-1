import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('mirte_slam')
    slam_config = os.path.join(pkg, 'config', 'slam_toolbox_config.yaml')
    rviz_config = os.path.join(pkg, 'config', 'slam_rviz.rviz')

    default_map_path = os.path.join(os.path.expanduser('~'), 'mirte_maps', 'map')

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

    # Periodically persist the live map to <map_path>.pgm/.yaml via slam_toolbox/save_map.
    map_autosaver = Node(
        package='mirte_slam',
        executable='map_autosaver',
        name='map_autosaver',
        output='screen',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'save_period': ParameterValue(
                LaunchConfiguration('save_period'), value_type=float
            ),
        }],
        condition=IfCondition(LaunchConfiguration('autosave')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz for map visualisation (set true only when running on a machine with a display)',
        ),
        DeclareLaunchArgument(
            'autosave',
            default_value='true',
            description='Periodically save the map to disk via slam_toolbox/save_map',
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
        rviz,
        map_autosaver,
    ])
