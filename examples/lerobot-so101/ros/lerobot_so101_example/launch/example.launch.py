# Copyright 2026 ahmedsleem109
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

"""Launch the LeRobot SO-101 example with plain ros2_medkit.

Robot side: SO-101 simulation, LeRobot policy, camera monitor, episode runner.
ros2_medkit side, unchanged packages only: gateway with the graph watchdog
plugin, fault manager, diagnostic bridge, and ros2_medkit_policy_bridge when it
is installed.

Arguments:
  num_episodes  attempts per campaign (default 10)
  autostart     start a campaign at launch (default false)
  medkit        start ros2_medkit (default true)
  policy_bridge start ros2_medkit_policy_bridge if installed (default true)
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lerobot_so101_example')
    config = os.path.join(share, 'config')
    gateway_share = get_package_share_directory('ros2_medkit_gateway')
    with_medkit = IfCondition(LaunchConfiguration('medkit'))
    try:
        get_package_share_directory('ros2_medkit_policy_bridge')
        has_policy_bridge = True
    except PackageNotFoundError:
        has_policy_bridge = False

    medkit = [
        Node(
            package='ros2_medkit_gateway',
            executable='gateway_node',
            name='ros2_medkit_gateway',
            output='screen',
            parameters=[
                os.path.join(gateway_share, 'config', 'gateway_params.yaml'),
                os.path.join(config, 'gateway.yaml'),
            ],
            condition=with_medkit,
        ),
        Node(
            package='ros2_medkit_fault_manager',
            executable='fault_manager_node',
            name='fault_manager',
            output='screen',
            parameters=[
                os.path.join(config, 'fault_manager.yaml'),
                {
                    'fault_thresholds.config_file': os.path.join(config, 'fault_thresholds.yaml'),
                },
            ],
            condition=with_medkit,
        ),
        Node(
            package='ros2_medkit_diagnostic_bridge',
            executable='diagnostic_bridge_node',
            name='diagnostic_bridge',
            output='screen',
            parameters=[os.path.join(config, 'diagnostic_bridge.yaml')],
            condition=with_medkit,
        ),
    ]
    if has_policy_bridge:
        medkit.append(
            Node(
                package='ros2_medkit_policy_bridge',
                executable='policy_bridge_node',
                name='policy_bridge',
                output='screen',
                parameters=[os.path.join(config, 'policy_bridge.yaml')],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration('medkit'),
                            "' == 'true' and '",
                            LaunchConfiguration('policy_bridge'),
                            "' == 'true'",
                        ]
                    )
                ),
            )
        )

    robot = [
        Node(
            package='lerobot_so101_example',
            executable='so101_sim_node',
            name='so101_sim',
            namespace='so101',
            output='screen',
        ),
        Node(
            package='lerobot_so101_example',
            executable='lerobot_policy_node',
            name='lerobot_policy',
            output='screen',
            # Restarted if it dies: ./scripts/kill-policy.sh tests GRAPH_NODE_DISAPPEARED
            respawn=True,
            respawn_delay=5.0,
        ),
        Node(
            package='lerobot_so101_example',
            executable='camera_monitor',
            name='camera_monitor',
            output='screen',
        ),
        Node(
            package='lerobot_so101_example',
            executable='episode_runner',
            name='episode_runner',
            output='screen',
            parameters=[
                {
                    'num_episodes': ParameterValue(
                        LaunchConfiguration('num_episodes'), value_type=int
                    ),
                    'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                    'checkpoint': os.environ.get('POLICY_REPO', ''),
                }
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument('num_episodes', default_value='10'),
            DeclareLaunchArgument('autostart', default_value='false'),
            DeclareLaunchArgument('medkit', default_value='true'),
            DeclareLaunchArgument('policy_bridge', default_value='true'),
            *medkit,
            *robot,
        ]
    )
