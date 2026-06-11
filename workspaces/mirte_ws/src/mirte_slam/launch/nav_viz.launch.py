"""
Laptop visualisation launch — run this on your laptop while the robot is running.
Works for both mapping mode and navigation mode.

Usage:
    ros2 launch mirte_slam nav_viz.launch.py

What you get:
  - RViz with map, laser scan, Nav2 costmaps, planned path, AMCL particles
  - RViz bridge: '2D Pose Estimate' and '2D Nav Goal' buttons forward to robot
    via rosbridge (bypasses the DDS publishing limitation)

RViz toolbar shortcuts:
  P  = 2D Pose Estimate (set robot starting position for AMCL)
  G  = 2D Nav Goal      (send navigation goal to Nav2)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('mirte_slam'), 'config', 'nav_rviz.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    bridge = Node(
        package='mirte_teleop_gui',
        executable='rviz_bridge',
        name='rviz_bridge',
        output='screen',
    )

    return LaunchDescription([rviz, bridge])
