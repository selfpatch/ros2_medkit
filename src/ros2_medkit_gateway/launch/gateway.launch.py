# Copyright 2025 mfaferek93
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_medkit_gateway')
    default_config = os.path.join(pkg_dir, 'config', 'gateway_params.yaml')

    # Resolve graph provider plugin path (optional - gateway starts without it)
    graph_provider_path = ''
    try:
        graph_provider_prefix = get_package_prefix('ros2_medkit_graph_provider')
        graph_provider_path = os.path.join(
            graph_provider_prefix, 'lib', 'ros2_medkit_graph_provider',
            'libros2_medkit_graph_provider_plugin.so')
    except PackageNotFoundError:
        print('[gateway.launch.py] ros2_medkit_graph_provider not installed '
              '- graph endpoints will not be available')

    declare_host_arg = DeclareLaunchArgument(
        'server_host', default_value='127.0.0.1',
        description='Host to bind REST server (127.0.0.1 or 0.0.0.0)')

    declare_port_arg = DeclareLaunchArgument(
        'server_port', default_value='8080',
        description='Port for REST API')

    declare_refresh_arg = DeclareLaunchArgument(
        'refresh_interval_ms', default_value='2000',
        description='Cache refresh interval in milliseconds')

    # Build parameter overrides - only inject plugin path if found
    param_overrides = {
        'server.host': LaunchConfiguration('server_host'),
        'server.port': LaunchConfiguration('server_port'),
        'refresh_interval_ms': LaunchConfiguration('refresh_interval_ms'),
    }
    if graph_provider_path:
        param_overrides['plugins'] = ['graph_provider']
        param_overrides['plugins.graph_provider.path'] = graph_provider_path

    gateway_node = Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[default_config, param_overrides],
        arguments=['--ros-args', '--log-level', 'info'])

    return LaunchDescription([
        declare_host_arg,
        declare_port_arg,
        declare_refresh_arg,
        gateway_node,
    ])
