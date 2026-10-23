#!/usr/bin/env python3
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

"""Report FAILED then PASSED events to a running fault manager and print the resulting status."""

import sys
import time

import rclpy
from ros2_medkit_msgs.srv import ListFaults, ReportFault


def main():
    failed = int(sys.argv[1])
    passed = int(sys.argv[2])
    settle = float(sys.argv[3])
    time.sleep(2.0)
    rclpy.init()
    node = rclpy.create_node('sweep_probe')
    report = node.create_client(ReportFault, '/fault_manager/report_fault')
    listing = node.create_client(ListFaults, '/fault_manager/list_faults')
    if not (report.wait_for_service(timeout_sec=20.0) and
            listing.wait_for_service(timeout_sec=20.0)):
        print('probe: service not available')
        return 2

    def send(event_type):
        req = ReportFault.Request()
        req.fault_code = 'SWEEP_FAULT'
        req.event_type = event_type
        req.severity = 2
        req.description = 'sweep'
        req.source_id = '/sweep_source'
        fut = report.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        return fut.result().accepted if fut.result() else None

    def status(label):
        req = ListFaults.Request()
        req.statuses = ['PREFAILED', 'PREPASSED', 'CONFIRMED', 'HEALED', 'CLEARED']
        fut = listing.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        faults = fut.result().faults if fut.result() else []
        rows = [(f.fault_code, f.status, f.occurrence_count) for f in faults]
        print('probe %s: %s' % (label, rows))

    for _ in range(failed):
        send(ReportFault.Request.EVENT_FAILED)
    time.sleep(settle)
    status('after %d FAILED' % failed)
    for _ in range(passed):
        send(ReportFault.Request.EVENT_PASSED)
    status('after %d PASSED' % passed)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
