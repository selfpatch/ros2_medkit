# Copyright 2026 mfaferek93
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

"""Single-command bringup for the local medkit stack."""

# Composes the gateway, the fault_manager and the generic fault bridges via
# IncludeLaunchDescription so a user gets a fault-diagnosable stack with one
# command and no manual topic/service wiring. Each component is individually
# toggleable, and a shared params file turns on healing and black-box capture
# that the conservative per-node defaults leave off.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, enable_arg=None, launch_arguments=None):
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', launch_file))
    kwargs = {}
    if launch_arguments is not None:
        kwargs['launch_arguments'] = launch_arguments.items()
    if enable_arg is not None:
        kwargs['condition'] = IfCondition(LaunchConfiguration(enable_arg))
    return IncludeLaunchDescription(source, **kwargs)


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config',
        'bringup_params.yaml',
    )
    # The same file gateway.launch.py defaults to. Named here so the argument
    # below always carries a real path: gateway.launch.py loads this value as a
    # parameters entry, and an empty string is not a file.
    gateway_default_config = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config',
        'gateway_params.yaml',
    )

    params_file = LaunchConfiguration('params_file')
    server_host = LaunchConfiguration('server_host')
    server_port = LaunchConfiguration('server_port')
    cors_allowed_origins = LaunchConfiguration('cors_allowed_origins')
    tls_enabled = LaunchConfiguration('tls_enabled')
    cert_file = LaunchConfiguration('cert_file')
    key_file = LaunchConfiguration('key_file')
    auth_enabled = LaunchConfiguration('auth_enabled')
    jwt_secret = LaunchConfiguration('jwt_secret')
    auth_clients = LaunchConfiguration('auth_clients')
    config_file = LaunchConfiguration('config_file')

    args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file applied to the fault_manager (turns on healing + '
                        'black-box rosbag). The gateway and bridges run with their own configs; '
                        'tune those via their launch args.'),
        DeclareLaunchArgument(
            'server_host', default_value='127.0.0.1',
            description='Host to bind the gateway REST server (127.0.0.1 or 0.0.0.0).'),
        DeclareLaunchArgument(
            'server_port', default_value='8080',
            description='Gateway REST API port.'),
        DeclareLaunchArgument(
            'cors_allowed_origins',
            default_value='http://localhost:3000,http://localhost:5173',
            description='Comma-separated CORS origins allowed to call the gateway from a browser, '
                        'so the web UI works out of the box. Empty disables CORS.'),
        # Forwarded to gateway.launch.py, along with config_file. Without them
        # bringup can only run the profile gateway.launch.py defaults to and
        # has nowhere to receive a certificate or a signing secret, so
        # `ros2 launch ... bringup.launch.py config_file:=<secure profile>
        # jwt_secret:=... cert_file:=...` is a complete command rather than a
        # dead end.
        DeclareLaunchArgument(
            'config_file', default_value=gateway_default_config,
            description='Path to a gateway YAML config. Defaults to the package '
                        'config/gateway_params.yaml, the profile with auth and TLS '
                        'off. Point it at config/gateway_params.secure.yaml in the '
                        'same directory for the closed profile. An empty value is '
                        'not accepted: gateway.launch.py loads this path, so it has '
                        'to name a file.'),
        DeclareLaunchArgument(
            'tls_enabled', default_value='',
            description='Serve HTTPS. Empty leaves it to the config file, which has it '
                        'off in the default profile and on in the secure one. Needs '
                        'cert_file and key_file when on.'),
        DeclareLaunchArgument(
            'cert_file', default_value='',
            description='PEM certificate for HTTPS. Required while tls_enabled is true.'),
        DeclareLaunchArgument(
            'key_file', default_value='',
            description='PEM private key matching cert_file.'),
        DeclareLaunchArgument(
            'auth_enabled', default_value='',
            description='Require a credential. Empty leaves it to the config file, '
                        'which has it off in the default profile and on in the secure '
                        'one.'),
        DeclareLaunchArgument(
            'jwt_secret', default_value='',
            description='HS256 signing secret, at least 32 characters. Required while '
                        'auth_enabled is true.'),
        DeclareLaunchArgument(
            'auth_clients', default_value='',
            description='Comma-separated "client_id:client_secret:role" triples.'),
        DeclareLaunchArgument(
            'enable_fault_manager', default_value='true',
            description='Start the fault_manager node.'),
        DeclareLaunchArgument(
            'enable_log_bridge', default_value='true',
            description='Start the log_bridge (/rosout -> faults). Cost scales with log volume; '
                        'raise log_bridge severity_floor on chatty stacks.'),
        DeclareLaunchArgument(
            'enable_action_status_bridge', default_value='true',
            description='Start the action_status_bridge (aborted action goals -> faults). '
                        'Rescans the ROS graph every rescan_period_sec (raise on large graphs).'),
        DeclareLaunchArgument(
            'enable_diagnostic_bridge', default_value='false',
            description='Start the diagnostic_bridge (/diagnostics -> faults). Opt-in: for legacy '
                        'diagnostic_updater publishers; new code should report faults natively.'),
    ]

    gateway = _include(
        'ros2_medkit_gateway', 'gateway.launch.py',
        launch_arguments={'server_host': server_host, 'server_port': server_port,
                          'cors_allowed_origins': cors_allowed_origins,
                          'tls_enabled': tls_enabled, 'cert_file': cert_file,
                          'key_file': key_file, 'auth_enabled': auth_enabled,
                          'jwt_secret': jwt_secret, 'auth_clients': auth_clients,
                          'config_file': config_file})
    fault_manager = _include(
        'ros2_medkit_fault_manager', 'fault_manager.launch.py',
        enable_arg='enable_fault_manager',
        launch_arguments={'params_file': params_file})
    log_bridge = _include(
        'ros2_medkit_log_bridge', 'log_bridge.launch.py',
        enable_arg='enable_log_bridge')
    action_status_bridge = _include(
        'ros2_medkit_action_status_bridge', 'action_status_bridge.launch.py',
        enable_arg='enable_action_status_bridge')
    diagnostic_bridge = _include(
        'ros2_medkit_diagnostic_bridge', 'diagnostic_bridge.launch.py',
        enable_arg='enable_diagnostic_bridge')

    return LaunchDescription(
        args + [gateway, fault_manager, log_bridge, action_status_bridge, diagnostic_bridge])
