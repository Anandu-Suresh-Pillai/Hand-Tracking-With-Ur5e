"""
gesture_control.launch.py
==========================
Launches the full UR5e gesture control simulation:
  1. robot_state_publisher  — reads the URDF and publishes TF transforms
  2. gesture_control_node   — webcam hand tracking + IK + joint publisher
  3. rviz2                  — 3D visualisation with pre-loaded config

Mesh path resolution
--------------------
The provided URDF uses relative paths like:
    ../meshes/ur5e/visual/base.dae

After installation via colcon, the package meshes live at:
    share/gesture_control/meshes/ur5e/...

We dynamically rewrite all mesh filename attributes in the URDF to use
the canonical ROS package:// URI scheme before passing to robot_state_publisher.
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _load_urdf_with_fixed_paths(pkg_share: str) -> str:
    """
    Read the installed URDF and replace relative mesh paths with
    package:// URIs that ROS 2 can resolve correctly.
    """
    urdf_path = os.path.join(pkg_share, 'urdf', 'ur5e.urdf')
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    # Replace: filename="../meshes/ur5e/..." → filename="package://gesture_control/meshes/ur5e/..."
    # This regex matches any relative path containing "meshes/ur5e"
    urdf_content = re.sub(
        r'filename=["\'](?:\.\./)+meshes/ur5e/([^"\']+)["\']',
        r'filename="package://gesture_control/meshes/ur5e/\1"',
        urdf_content,
    )

    return urdf_content


def generate_launch_description():
    pkg_share = get_package_share_directory('gesture_control')
    rviz_config = os.path.join(pkg_share, 'rviz', 'ur5e.rviz')

    # Build the URDF string with corrected mesh paths
    robot_description = _load_urdf_with_fixed_paths(pkg_share)

    # ---- robot_state_publisher ----
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
    )

    # ---- gesture_control_node ----
    gesture_node = Node(
        package='gesture_control',
        executable='gesture_control_node',
        name='gesture_control_node',
        output='screen',
        parameters=[{
            'camera_index': 0,
            'loop_hz': 20,
        }],
    )

    # ---- RViz2 ---- (delayed 2s to allow robot_state_publisher to start first)
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        ],
    )

    return LaunchDescription([
        rsp_node,
        gesture_node,
        rviz_node,
    ])
