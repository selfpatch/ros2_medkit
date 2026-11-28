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

"""Camera monitor: reports a frozen camera on the standard `/diagnostics` topic.

One diagnostic status per camera, named "camera_monitor: <camera> frozen":
ERROR when the same image arrived `frozen_frames` times in a row, OK otherwise.
ros2_medkit's diagnostic bridge turns ERROR into a fault and OK into its
healing, so this node needs no ros2_medkit code and works the same on a real
camera driver's topic.

A status goes out when it changes; OK is repeated every `diagnostics_period_sec`.
ERROR is not repeated: the bridge forwards every message, so an ERROR repeated
each second would need as many OKs to heal as the fault lasted seconds.
"""

import hashlib

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

CAMERAS = ('wrist', 'overhead')


class FrozenCheck:
    """Counts identical frames in a row."""

    def __init__(self):
        self.last = None
        self.repeats = 0

    def update(self, data):
        digest = hashlib.blake2b(data, digest_size=16).digest()
        self.repeats = self.repeats + 1 if digest == self.last else 0
        self.last = digest
        return self.repeats + 1  # identical frames in a row, this one included


class CameraMonitor(Node):

    def __init__(self):
        super().__init__('camera_monitor')
        self.declare_parameter('robot_ns', '/so101/so101_sim')
        self.declare_parameter('frozen_frames', 10)
        self.declare_parameter('diagnostics_period_sec', 5.0)
        ns = self.get_parameter('robot_ns').value
        self._checks = {c: FrozenCheck() for c in CAMERAS}
        self._frozen = dict.fromkeys(CAMERAS, False)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        for camera in CAMERAS:
            self.create_subscription(
                Image,
                f'{ns}/camera/{camera}/image_raw',
                lambda m, c=camera: self._on_image(c, m),
                10,
            )
        self.create_timer(
            float(self.get_parameter('diagnostics_period_sec').value),
            lambda: self._publish([c for c in CAMERAS if not self._frozen[c]]),
        )

    def _on_image(self, camera, msg):
        same = self._checks[camera].update(bytes(msg.data))
        frozen = same >= int(self.get_parameter('frozen_frames').value)
        if frozen != self._frozen[camera]:
            self._frozen[camera] = frozen
            if frozen:
                self.get_logger().warning(f'{camera} camera: the last {same} frames are identical')
            else:
                self.get_logger().info(f'{camera} camera: frames change again')
            self._publish([camera])

    def _publish(self, cameras):
        if not cameras:
            return
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for camera in cameras:
            status = DiagnosticStatus()
            status.name = f'camera_monitor: {camera} frozen'
            status.hardware_id = self.get_fully_qualified_name()
            repeats = self._checks[camera].repeats + 1 if self._checks[camera].last else 0
            if self._frozen[camera]:
                status.level = DiagnosticStatus.ERROR
                status.message = f'the last {repeats} {camera} camera frames are identical'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'frames change'
            status.values = [KeyValue(key='identical_frames', value=str(repeats))]
            array.status.append(status)
        self._diag_pub.publish(array)


def main():
    rclpy.init()
    node = CameraMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
