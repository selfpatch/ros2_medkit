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

"""A fault manager that refuses to clear one of its records.

Companion process for test_faults_entity_clear_failure.test.py. It serves the
fault manager's services under /fault_manager. ListFaults answers the records it
holds. ClearFault refuses REFUSED_CODE with the answer the fault manager gives
for a record it does not hold, and clears any other record it holds.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetFault, ListFaults, ReportFault

ACCEPTED_CODE = 'CLEAR_ACCEPTED'
REFUSED_CODE = 'CLEAR_REFUSED'
OWNER_A = 'owner-a'
OWNER_B = 'owner-b'


def _record(node, code, owner):
    fault = Fault()
    fault.fault_code = code
    fault.severity = Fault.SEVERITY_ERROR
    fault.description = f'{code} of {owner}'
    fault.first_occurred = node.get_clock().now().to_msg()
    fault.last_occurred = fault.first_occurred
    fault.occurrence_count = 1
    fault.status = Fault.STATUS_CONFIRMED
    fault.reporting_sources = [owner]
    fault.owner = owner
    return fault


class RefusingFaultManager(Node):

    def __init__(self):
        super().__init__('fault_manager')
        self.records = {
            (ACCEPTED_CODE, OWNER_A): _record(self, ACCEPTED_CODE, OWNER_A),
            (REFUSED_CODE, OWNER_B): _record(self, REFUSED_CODE, OWNER_B),
        }
        self.create_service(ListFaults, '/fault_manager/list_faults', self.on_list)
        self.create_service(ClearFault, '/fault_manager/clear_fault', self.on_clear)
        self.create_service(GetFault, '/fault_manager/get_fault', self.on_get)
        self.create_service(ReportFault, '/fault_manager/report_fault', self.on_report)

    def on_list(self, request, response):
        response.faults = [record for record in self.records.values()
                           if not request.statuses or record.status in request.statuses]
        return response

    def on_clear(self, request, response):
        key = (request.fault_code, request.source_id)
        if request.fault_code == REFUSED_CODE or key not in self.records:
            response.success = False
            response.message = (f'No fault record for code {request.fault_code} '
                                f'and source {request.source_id}')
            return response
        self.records[key].status = Fault.STATUS_CLEARED
        response.success = True
        response.message = 'Fault cleared'
        return response

    def on_get(self, request, response):
        record = self.records.get((request.fault_code, request.source_id))
        if record is None:
            response.success = False
            response.error_message = f'Fault not found: {request.fault_code}'
            return response
        response.success = True
        response.fault = record
        return response

    def on_report(self, request, response):
        response.accepted = False
        return response


def main():
    rclpy.init()
    node = RefusingFaultManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
