#!/usr/bin/env python3
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

"""
Stand-in for the fault manager's services that answers on the test's terms.

Offers /fault_manager/report_fault, clear_fault and get_fault. A ReportFault
for a code given with --late is answered after --late-seconds, one for a code
given with --never is not answered while the stub runs. Every request and every
answer is written to the log as one line: ``<monotonic seconds> <event> <code>``
with the events report_received, report_answered and clear_received.

Usage: fault_service_stub.py <log> [--late CODE] [--never CODE] [--late-seconds S]
"""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros2_medkit_msgs.srv import ClearFault, GetFault, ReportFault


class FaultServiceStub(Node):

    def __init__(self, args):
        super().__init__('fault_manager')
        self._late = set(args.late)
        self._never = set(args.never)
        self._late_seconds = args.late_seconds
        self._log = open(args.log, 'a', buffering=1)
        self._log_lock = threading.Lock()
        self.stopping = threading.Event()
        # Reentrant: a clear must be able to arrive while a report is held.
        group = ReentrantCallbackGroup()
        self.create_service(ReportFault, '/fault_manager/report_fault', self._report,
                            callback_group=group)
        self.create_service(ClearFault, '/fault_manager/clear_fault', self._clear,
                            callback_group=group)
        self.create_service(GetFault, '/fault_manager/get_fault', self._get,
                            callback_group=group)
        self._write('ready', '-')

    def _write(self, event, code):
        with self._log_lock:
            self._log.write(f'{time.monotonic():.6f} {event} {code}\n')

    def _report(self, request, response):
        self._write('report_received', request.fault_code)
        if request.fault_code in self._never:
            self.stopping.wait()
        elif request.fault_code in self._late:
            self.stopping.wait(self._late_seconds)
        response.accepted = True
        self._write('report_answered', request.fault_code)
        return response

    def _clear(self, request, response):
        self._write('clear_received', request.fault_code)
        response.success = True
        return response

    def _get(self, request, response):
        response.success = False
        response.error_message = 'not found'
        return response


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('log')
    parser.add_argument('--late', action='append', default=[])
    parser.add_argument('--never', action='append', default=[])
    parser.add_argument('--late-seconds', type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = FaultServiceStub(args)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stopping.set()
    return 0


if __name__ == '__main__':
    sys.exit(main())
