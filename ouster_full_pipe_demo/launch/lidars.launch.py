# Copyright 2020, Andreas Lebherz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Demonstration of Ouster Driver integration. """

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os

context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(Path(FindPackage(package_name).perform(context)), 'share', package_name)


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Ouster Driver integration.
    """
    ouster_demo_pkg_prefix = get_package_share_directory('ouster_full_pipe_demo')
    lidar_top_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/lidar_top.param.yaml')
    lidar_front_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/lidar_front.param.yaml')
    lidar_middle_left_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/lidar_middle_left.param.yaml')
    lidar_middle_right_param_file = os.path.join(
        ouster_demo_pkg_prefix, 'param/lidar_middle_right.param.yaml')

    # Argumentsavp_
    lidar_top_param = DeclareLaunchArgument(
        'lidar_top_param_file',
        default_value=lidar_top_param_file,
        description='Path to config file for Ouster OS2 Top'
    )
    lidar_front_param = DeclareLaunchArgument(
        'lidar_front_param_file',
        default_value=lidar_front_param_file,
        description='Path to config file for Ouster OS1 Front'
    )
    lidar_middle_left_param = DeclareLaunchArgument(
        'lidar_middle_left_param_file',
        default_value=lidar_middle_left_param_file,
        description='Path to config file for Ouster OS1 Middle Left'
    )
    lidar_middle_right_param = DeclareLaunchArgument(
        'lidar_middle_right_param_file',
        default_value=lidar_middle_right_param_file,
        description='Path to config file for Ouster OS1 Middle Right'
    )


    # Nodes
    lidar_top = Node(
        package='ouster_node_128',
        node_executable='ouster_cloud_node_128_exe',
        node_namespace='lidar_top',
        parameters=[LaunchConfiguration('lidar_top_param_file')]
    )
    lidar_front = Node(
        package='ouster_node',
        node_executable='ouster_cloud_node_exe',
        node_namespace='lidar_front',
        parameters=[LaunchConfiguration('lidar_front_param_file')]
    )
    lidar_middle_left = Node(
        package='ouster_node',
        node_executable='ouster_cloud_node_exe',
        node_namespace='lidar_middle_left',
        parameters=[LaunchConfiguration('lidar_middle_left_param_file')]
    )
    lidar_middle_right = Node(
        package='ouster_node',
        node_executable='ouster_cloud_node_exe',
        node_namespace='lidar_middle_right',
        parameters=[LaunchConfiguration('lidar_middle_right_param_file')]
    )

    return LaunchDescription([
        lidar_top_param,
        lidar_front_param,
        lidar_middle_left_param,
        lidar_middle_right_param,
        lidar_top,
        lidar_front,
        lidar_middle_left,
        lidar_middle_right
    ])
