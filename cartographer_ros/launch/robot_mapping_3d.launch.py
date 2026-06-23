"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')
    
    pkg_share = get_package_share_directory('cartographer_ros')
    
    cartographer_config_dir_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value = os.path.join(pkg_share, 'configuration_files'),
        description = 'Full path to config directory')
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value = 'robot_3d.lua',
        description = 'Name of the lua configuration file')
    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename',
        default_value = '',
        description = 'Full path to a pbstream file to load a saved state.')

    ## ***** File paths ******
    urdf_file = os.path.join(pkg_share, 'urdf', 'MRO_ROVER.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## Robot State Publisher
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output = 'screen'
        )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Static TF for base_footprint
    static_baselink_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '-0.06', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('load_state_filename'),
            '--ros-args',
            '--remap', 'points2:=/sensor/lidar32/points_raw',
            '--remap', 'imu:=/sensor/imu/data_raw'],
        output = 'screen',
        # Defense in depth: if Cartographer ever dies despite the in-process
        # fault tolerance, restart it automatically without taking down the
        # rest of the robot's nodes.
        respawn = True,
        respawn_delay = 2.0
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05}],
        )

    return LaunchDescription([
        use_sim_time_arg,
        cartographer_config_dir_arg,
        configuration_basename_arg,
        load_state_filename_arg,
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_baselink_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
