# Copyright 2026 bburda
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

"""Factory functions for building launch descriptions in integration tests.

Replaces ~500 lines of inline launch boilerplate previously duplicated across
9+ test files. Each test file's ``generate_test_description()`` reduces to a
single ``create_test_launch(...)`` call.

Node names, namespaces, and parameters match ``demo_nodes.launch.py`` exactly.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions

from ros2_medkit_test_utils.constants import DEFAULT_PORT
from ros2_medkit_test_utils.coverage import get_coverage_env

# ---------------------------------------------------------------------------
# Demo node registry
# ---------------------------------------------------------------------------
# Maps a short node key to the (executable, ros_name, namespace) triple.
# - executable: CMake target name (installed by ros2_medkit_integration_tests)
# - ros_name: ROS 2 node name (the ``name=`` parameter)
# - namespace: ROS 2 namespace (leading ``/`` is added by launch_ros)
#
# These MUST match demo_nodes.launch.py exactly.
# ---------------------------------------------------------------------------

DEMO_NODE_REGISTRY = {
    # Sensors
    'temp_sensor': ('demo_engine_temp_sensor', 'temp_sensor', '/powertrain/engine'),
    'rpm_sensor': ('demo_rpm_sensor', 'rpm_sensor', '/powertrain/engine'),
    'pressure_sensor': ('demo_brake_pressure_sensor', 'pressure_sensor', '/chassis/brakes'),
    'status_sensor': ('demo_door_status_sensor', 'status_sensor', '/body/door/front_left'),
    'lidar_sensor': ('demo_lidar_sensor', 'lidar_sensor', '/perception/lidar'),
    # Actuators
    'actuator': ('demo_brake_actuator', 'actuator', '/chassis/brakes'),
    'controller': ('demo_light_controller', 'controller', '/body/lights'),
    # Operations (services / actions)
    'calibration': ('demo_calibration_service', 'calibration', '/powertrain/engine'),
    'calibration_service': ('demo_calibration_service', 'calibration', '/powertrain/engine'),
    'long_calibration': ('demo_long_calibration_action', 'long_calibration', '/powertrain/engine'),
    'long_calibration_action': (
        'demo_long_calibration_action', 'long_calibration', '/powertrain/engine',
    ),
}

# Convenience groupings for callers that want subsets of demo nodes.
SENSOR_NODES = ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'status_sensor', 'lidar_sensor']
ACTUATOR_NODES = ['actuator', 'controller']
SERVICE_NODES = ['calibration', 'long_calibration']
ALL_DEMO_NODES = SENSOR_NODES + ACTUATOR_NODES + SERVICE_NODES

# Default lidar parameters that trigger deterministic faults:
# - min_range > max_range -> LIDAR_RANGE_INVALID
# - scan_frequency > 20.0 -> LIDAR_FREQ_UNSUPPORTED
# - is_calibrated=false -> LIDAR_CALIBRATION_REQUIRED
LIDAR_FAULTY_PARAMS = {
    'min_range': 10.0,
    'max_range': 5.0,
    'scan_frequency': 25.0,
    'angular_resolution': 0.5,
}


# ---------------------------------------------------------------------------
# Factory: gateway node
# ---------------------------------------------------------------------------

def create_gateway_node(*, port=DEFAULT_PORT, extra_params=None, coverage=True):
    """Create a ``gateway_node`` launch action with standard config.

    Parameters
    ----------
    port : int
        HTTP server port (default: 8080).
    extra_params : dict or None
        Additional ROS parameters merged into the node config.
    coverage : bool
        If True, set GCOV_PREFIX env vars for code coverage collection.

    Returns
    -------
    launch_ros.actions.Node
        Ready-to-use gateway node launch action.

    """
    params = {'refresh_interval_ms': 1000}
    if port != DEFAULT_PORT:
        params['server_port'] = port
    if extra_params:
        params.update(extra_params)

    return launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[params],
        additional_env=get_coverage_env() if coverage else {},
    )


# ---------------------------------------------------------------------------
# Factory: fault manager node
# ---------------------------------------------------------------------------

def create_fault_manager_node(
    *,
    storage_type='memory',
    rosbag_enabled=True,
    rosbag_topics=None,
    coverage=True,
):
    """Create a ``fault_manager_node`` with test-friendly defaults.

    Parameters
    ----------
    storage_type : str
        Storage backend: 'memory' (default, avoids filesystem issues in CI)
        or 'sqlite'.
    rosbag_enabled : bool
        Enable rosbag snapshot capture (default: True).
    rosbag_topics : list of str or None
        Explicit topic list for rosbag capture. Defaults to
        ``['/perception/lidar/scan']``.
    coverage : bool
        If True, set GCOV_PREFIX env vars for code coverage collection.

    Returns
    -------
    launch_ros.actions.Node
        Ready-to-use fault manager node launch action.

    """
    if rosbag_topics is None:
        rosbag_topics = ['/perception/lidar/scan']

    params = {
        'storage_type': storage_type,
        'snapshots.rosbag.enabled': rosbag_enabled,
        'snapshots.rosbag.duration_sec': 2.0,
        'snapshots.rosbag.duration_after_sec': 0.5,
        'snapshots.rosbag.topics': 'explicit',
        'snapshots.rosbag.include_topics': rosbag_topics,
    }

    return launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=get_coverage_env() if coverage else {},
        parameters=[params],
    )


# ---------------------------------------------------------------------------
# Factory: demo nodes
# ---------------------------------------------------------------------------

def create_demo_nodes(nodes=None, *, lidar_faulty=True, coverage=True):
    """Create demo node launch actions.

    Parameters
    ----------
    nodes : list of str or None
        Node keys from ``DEMO_NODE_REGISTRY``. ``None`` means
        ``ALL_DEMO_NODES``.
    lidar_faulty : bool
        If True (default), launch ``lidar_sensor`` with invalid parameters
        that trigger deterministic faults. If False, launch with defaults.
    coverage : bool
        If True, set GCOV_PREFIX env vars for code coverage collection.

    Returns
    -------
    list of launch_ros.actions.Node
        Ordered list of demo node launch actions.

    Raises
    ------
    KeyError
        If a node key is not in ``DEMO_NODE_REGISTRY``.

    """
    if nodes is None:
        nodes = ALL_DEMO_NODES

    coverage_env = get_coverage_env() if coverage else {}
    actions = []

    for key in nodes:
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[key]

        node_kwargs = dict(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=ros_name,
            namespace=namespace,
            output='screen',
            additional_env=coverage_env,
        )

        # Apply faulty parameters for lidar_sensor
        if key == 'lidar_sensor' and lidar_faulty:
            node_kwargs['parameters'] = [LIDAR_FAULTY_PARAMS]

        actions.append(launch_ros.actions.Node(**node_kwargs))

    return actions


# ---------------------------------------------------------------------------
# Factory: complete test launch description
# ---------------------------------------------------------------------------

def create_test_launch(
    *,
    port=DEFAULT_PORT,
    demo_nodes=None,
    gateway_params=None,
    fault_manager=True,
    lidar_faulty=True,
    demo_delay=2.0,
):
    """Build a complete ``LaunchDescription`` for an integration test.

    This is the primary entry point. Each test file's
    ``generate_test_description()`` delegates to this function.

    Parameters
    ----------
    port : int
        Gateway HTTP port (default: 8080).
    demo_nodes : list of str or None
        Node keys from ``DEMO_NODE_REGISTRY``. ``None`` means all nodes.
    gateway_params : dict or None
        Extra ROS parameters for the gateway node.
    fault_manager : bool
        If True (default), include the fault manager node.
    lidar_faulty : bool
        If True (default), launch lidar_sensor with fault-triggering params.
    demo_delay : float
        Seconds to delay demo nodes after gateway start (default: 2.0).

    Returns
    -------
    tuple[LaunchDescription, dict]
        ``(launch_description, context_dict)`` tuple expected by
        ``launch_testing``. ``context_dict`` maps ``'gateway_node'`` to the
        gateway ``Node`` action.

    """
    gateway_node = create_gateway_node(
        port=port,
        extra_params=gateway_params,
    )

    delayed_actions = create_demo_nodes(
        demo_nodes,
        lidar_faulty=lidar_faulty,
    )

    if fault_manager:
        delayed_actions.append(create_fault_manager_node())

    delayed = TimerAction(
        period=demo_delay,
        actions=delayed_actions,
    )

    launch_description = LaunchDescription([
        gateway_node,
        delayed,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': gateway_node},
    )
