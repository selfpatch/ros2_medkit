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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch all ROS 2 Medkit Gateway demo nodes with proper namespaces.

    This creates the automotive area hierarchy:
    - /powertrain (engine sensors + calibration)
    - /chassis (brake sensors + actuators)
    - /body (door sensors + light controller)

    Sensors:
    - Engine temperature sensor → /powertrain/engine
    - Engine RPM sensor → /powertrain/engine
    - Brake pressure sensor → /chassis/brakes
    - Door status sensor → /body/door/front_left

    Actuators:
    - Brake actuator → /chassis/brakes
    - Light controller → /body/lights

    Operations:
    - Calibration service (sync) → /powertrain/engine
    - Long calibration action (async) → /powertrain/engine
    """
    return LaunchDescription([
        # === POWERTRAIN AREA ===

        Node(
            package='ros2_medkit_gateway',
            executable='demo_engine_temp_sensor',
            name='temp_sensor',
            namespace='powertrain/engine',
            output='log',
            emulate_tty=True,
        ),

        Node(
            package='ros2_medkit_gateway',
            executable='demo_rpm_sensor',
            name='rpm_sensor',
            namespace='powertrain/engine',
            output='log',
            emulate_tty=True,
        ),

        Node(
            package='ros2_medkit_gateway',
            executable='demo_calibration_service',
            name='calibration',
            namespace='powertrain/engine',
            output='log',
            emulate_tty=True,
        ),

        Node(
            package='ros2_medkit_gateway',
            executable='demo_long_calibration_action',
            name='long_calibration',
            namespace='powertrain/engine',
            output='log',
            emulate_tty=True,
        ),

        # === CHASSIS AREA ===

        Node(
            package='ros2_medkit_gateway',
            executable='demo_brake_pressure_sensor',
            name='pressure_sensor',
            namespace='chassis/brakes',
            output='log',
            emulate_tty=True,
        ),

        Node(
            package='ros2_medkit_gateway',
            executable='demo_brake_actuator',
            name='actuator',
            namespace='chassis/brakes',
            output='log',
            emulate_tty=True,
        ),

        # === BODY AREA ===

        Node(
            package='ros2_medkit_gateway',
            executable='demo_door_status_sensor',
            name='status_sensor',
            namespace='body/door/front_left',
            output='log',
            emulate_tty=True,
        ),

        Node(
            package='ros2_medkit_gateway',
            executable='demo_light_controller',
            name='controller',
            namespace='body/lights',
            output='log',
            emulate_tty=True,
        ),
    ])
