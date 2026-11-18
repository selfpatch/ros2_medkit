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

"""A second fault source for the OPC-UA tests: one node in the test process."""

import time

import rclpy
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults, ReportFault

ALL_STATUSES = ['PREFAILED', 'PREPASSED', 'CONFIRMED', 'HEALED', 'CLEARED']


class FaultManagerClient:
    """
    Reports and lists faults. rclpy takes ROS_DOMAIN_ID from the environment.

    A reply can be lost while a fault manager that just started has not matched this
    client, so a call with no answer after <answer_wait_sec> is sent again.
    """

    def __init__(self, name, answer_wait_sec, timeout_sec):
        rclpy.init()
        self.node = rclpy.create_node(name)
        self.report_client = self.node.create_client(ReportFault, '/fault_manager/report_fault')
        self.list_client = self.node.create_client(ListFaults, '/fault_manager/list_faults')
        self.answer_wait_sec = answer_wait_sec
        self.timeout_sec = timeout_sec

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def call(self, client, request):
        """Return the answer to <request>, sending it again while the reply is lost."""
        end = time.monotonic() + self.timeout_sec
        while time.monotonic() < end:
            if not client.wait_for_service(timeout_sec=self.answer_wait_sec):
                continue
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.answer_wait_sec)
            if future.done():
                return future.result()
            client.remove_pending_request(future)
        raise AssertionError(f'{client.srv_name} never answered')

    def report(self, code, source):
        """Report <code> FAILED from <source>. Return whether the fault manager accepted it."""
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'second source'
        request.source_id = source
        return self.call(self.report_client, request).accepted

    def faults(self):
        """Return every fault the fault manager holds, in any status."""
        request = ListFaults.Request()
        request.statuses = ALL_STATUSES
        return self.call(self.list_client, request).faults
