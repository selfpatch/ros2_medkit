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

"""
The pre-fault history a rosbag capture holds, seen from a second subscriber.

The capture keeps the messages of the last duration_sec in its ring buffer and stamps
each with its receipt time. A bag starts at the oldest buffered message. A confirmation
empties the buffer and sends the next duration_after_sec of messages to the bag, and a
publisher that stalls leaves the buffer short of old messages.
"""

import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import FaultEvent

# Timer jitter allowed at the end of a post-fault window.
WINDOW_JITTER_SEC = 0.2


class RecordedTopicWatch:
    """Receipt times of a recorded topic, and the post-fault windows that divert it."""

    def __init__(self, node, topic, msg_type, qos, duration_sec, duration_after_sec):
        self.node = node
        self.duration_sec = duration_sec
        self.duration_after_sec = duration_after_sec
        self.receipts = []
        self.buffering_since = 0.0
        node.create_subscription(msg_type, topic, self._on_message, qos)
        node.create_subscription(
            FaultEvent, '/fault_manager/events', self._on_event,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))

    def _on_message(self, _msg):
        now = time.monotonic()
        self.receipts.append(now)
        self.receipts = [t for t in self.receipts if now - t <= 2 * self.duration_sec]

    def _on_event(self, msg):
        if msg.event_type == FaultEvent.EVENT_CONFIRMED:
            self.buffering_since = (
                time.monotonic() + self.duration_after_sec + WINDOW_JITTER_SEC)

    def holds_history(self, history_sec, latency_sec):
        """
        Return True when a fault confirmed within <latency_sec> gets <history_sec> of history.

        That needs a message received after the last post-fault window, at least
        <history_sec> old now and still inside the buffer when the flush runs.
        """
        now = time.monotonic()
        oldest_kept = now + latency_sec - self.duration_sec
        return any(max(self.buffering_since, oldest_kept) <= t <= now - history_sec
                   for t in self.receipts)

    def wait_for_history(self, history_sec, timeout_sec, latency_sec=0.3):
        """Spin until holds_history(); False after <timeout_sec>."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.holds_history(history_sec, latency_sec):
                return True
        return False
