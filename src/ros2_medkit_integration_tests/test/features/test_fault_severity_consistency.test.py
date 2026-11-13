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

"""A fault record shows one severity on every gateway surface.

Real fault manager (SQLite, confirmation threshold -3), real gateway, reports
through the ReportFault service. Both codes are per_source, so a record belongs
to one owner and the gateway's code-addressed routes reach it. A CRITICAL fault is acknowledged
and then reported again as WARN. The record keeps CRITICAL, so the WARN report
confirms it at once, and GET /faults, the fault detail, the /faults/stream
frame and the payload of an OnChange trigger all carry the same severity and
status.
"""

import json
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, FAULT_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


APP_ID = 'temp_sensor'
OWNER = '/powertrain/engine/temp_sensor'
CODE = 'SEV_CONSISTENCY'
PRIME_CODE = 'SEV_CONSISTENCY_PRIME'
DB_DIR = tempfile.mkdtemp(prefix='medkit_severity_e2e_')
THRESHOLDS_PATH = os.path.join(DB_DIR, 'fault_thresholds.yaml')


def generate_test_description():
    with open(THRESHOLDS_PATH, 'w') as f:
        for code in (CODE, PRIME_CODE):
            f.write(f'{code}:\n  per_source: true\n')
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        fault_manager_params={
            'storage_type': 'sqlite',
            'database_path': os.path.join(DB_DIR, 'faults.db'),
            'confirmation_threshold': -3,
            'fault_thresholds.config_file': THRESHOLDS_PATH,
            'snapshots.rosbag.enabled': False,
        },
        gateway_params={
            # Two SSE streams at once: /faults/stream and one trigger stream.
            'sse.max_clients': 4,
            'server.http_thread_pool_size': 10,
            # A stream reader only sees its stop flag when a line arrives.
            'sse.keepalive_interval_sec': 1,
        },
    )


class SseCollector:
    """Collect SSE frames from one stream on a background thread."""

    def __init__(self, url):
        self.frames = []
        self._stop = threading.Event()
        self._response = requests.get(url, stream=True, timeout=(5, 120))
        self.status_code = self._response.status_code
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self):
        current = {}
        try:
            for line in self._response.iter_lines(decode_unicode=True):
                if self._stop.is_set():
                    break
                if line is None:
                    continue
                if line == '':
                    if current:
                        self.frames.append(current)
                        current = {}
                    continue
                if line.startswith(':'):
                    continue
                key, _, value = line.partition(':')
                current[key.strip()] = value.strip()
        except Exception:  # noqa: BLE001 - closed socket on teardown
            pass

    def payloads(self):
        return [json.loads(f['data']) for f in list(self.frames) if 'data' in f]

    def close(self):
        self._stop.set()
        self._response.close()
        self._thread.join(timeout=5)


class TestFaultSeverityConsistency(GatewayTestCase):
    """Subscribers are told the severity that GET shows."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {APP_ID}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._node = Node('fault_severity_consistency_reporter')
        cls._report_client = cls._node.create_client(ReportFault, '/fault_manager/report_fault')
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        rclpy.shutdown()

    def _failed(self, code, severity, description):
        req = ReportFault.Request()
        req.fault_code = code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = severity
        req.description = description
        req.source_id = OWNER
        future = self._report_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self.assertTrue(future.done(), 'report_fault timed out')
        self.assertTrue(future.result().accepted)

    def _wait_for(self, predicate, what):
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            found = predicate()
            if found is not None:
                return found
            time.sleep(0.2)
        self.fail(f'timed out waiting for {what}')

    def _prime_fault_stream(self, stream):
        """Report a sacrificial CRITICAL fault until its frame arrives on ``stream``."""
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            self._failed(PRIME_CODE, Fault.SEVERITY_CRITICAL, 'prime')
            settle = min(time.monotonic() + 1.0, deadline)
            while time.monotonic() < settle:
                if any(p.get('fault', {}).get('fault_code') == PRIME_CODE
                       for p in stream.payloads()):
                    return
                time.sleep(0.1)
        self.fail(f'no frame for {PRIME_CODE} on /faults/stream within {FAULT_TIMEOUT}s')

    def _list_item(self):
        r = requests.get(f'{self.BASE_URL}/faults', params={'status': 'all'}, timeout=10)
        self.assertEqual(r.status_code, 200, r.text)
        return next((i for i in r.json()['items'] if i['fault_code'] == CODE), None)

    def _detail(self):
        r = requests.get(f'{self.BASE_URL}/apps/{APP_ID}/faults/{CODE}', timeout=10)
        self.assertEqual(r.status_code, 200, r.text)
        return r.json()

    def _create_trigger(self):
        body = {
            'resource': f'{API_BASE_PATH}/apps/{APP_ID}/faults/{CODE}',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 300,
        }
        r = requests.post(f'{self.BASE_URL}/apps/{APP_ID}/triggers', json=body, timeout=5)
        self.assertEqual(r.status_code, 201, r.text)
        return r.json()

    def test_one_severity_on_every_surface(self):
        trig = self._create_trigger()
        trigger_stream = SseCollector(
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}')
        fault_stream = SseCollector(f'{self.BASE_URL}/faults/stream')
        try:
            self._check_surfaces(trigger_stream, fault_stream)
        finally:
            # Deleting the trigger ends its stream on the server side.
            requests.delete(f'{self.BASE_URL}/apps/{APP_ID}/triggers/{trig["id"]}', timeout=5)
            trigger_stream.close()
            fault_stream.close()

    def _check_surfaces(self, trigger_stream, fault_stream):
        self.assertEqual(trigger_stream.status_code, 200)
        self.assertEqual(fault_stream.status_code, 200)
        self._prime_fault_stream(fault_stream)

        def frame(event, occurrence):
            def find():
                for f in list(fault_stream.frames):
                    if f.get('event') != event or 'data' not in f:
                        continue
                    fault = json.loads(f['data'])['fault']
                    if fault['fault_code'] == CODE and fault['occurrence_count'] == occurrence:
                        return fault
                return None
            return find

        def trigger_payload(event_type, occurrence):
            def find():
                for frame_data in trigger_stream.payloads():
                    p = frame_data.get('payload', {})
                    key = (p.get('event_type'), p.get('occurrence_count'))
                    if key == (event_type, occurrence):
                        return p
                return None
            return find

        # Occurrence 1: CRITICAL confirms at once. The owner acknowledges it.
        self._failed(CODE, Fault.SEVERITY_CRITICAL, 'occurrence 1')
        self._wait_for(frame('fault_confirmed', 1), 'fault_confirmed of occurrence 1')
        r = requests.delete(f'{self.BASE_URL}/apps/{APP_ID}/faults/{CODE}', timeout=10)
        self.assertEqual(r.status_code, 204, r.text)
        self._wait_for(frame('fault_cleared', 1), 'fault_cleared of occurrence 1')

        # Occurrence 2 is reported as WARN; one report confirms it.
        self._failed(CODE, Fault.SEVERITY_WARN, 'occurrence 2')
        surfaces = {}
        with self.subTest(surface='/faults/stream'):
            surfaces['stream'] = self._wait_for(
                frame('fault_confirmed', 2), 'fault_confirmed of occurrence 2')
        with self.subTest(surface='OnChange trigger'):
            surfaces['trigger'] = self._wait_for(
                trigger_payload('fault_confirmed', 2), 'trigger event of occurrence 2')
        with self.subTest(surface='GET /faults'):
            surfaces['list'] = self._list_item()
            self.assertIsNotNone(surfaces['list'])
            self.assertEqual(surfaces['list']['occurrence_count'], 2)
        with self.subTest(surface='fault detail'):
            detail = self._detail()
            surfaces['detail'] = {
                'severity': detail['item']['severity'],
                'severity_label': detail['x-medkit']['severity_label'],
                'status': detail['x-medkit']['status_raw'],
            }

        for name, shown in surfaces.items():
            with self.subTest(agree=name):
                self.assertIsNotNone(shown)
                self.assertEqual(shown['severity'], Fault.SEVERITY_CRITICAL, shown)
                self.assertEqual(shown['severity_label'], 'CRITICAL', shown)
                self.assertEqual(shown['status'], 'CONFIRMED', shown)

        # A lower report in the occurrence updates the fault; every surface still shows CRITICAL.
        self._failed(CODE, Fault.SEVERITY_ERROR, 'occurrence 2, lower')
        updated = self._wait_for(frame('fault_updated', 2), 'fault_updated of occurrence 2')
        fired = self._wait_for(
            trigger_payload('fault_updated', 2), 'trigger event for the update')
        item = self._list_item()
        for name, shown in {'stream': updated, 'trigger': fired, 'list': item}.items():
            with self.subTest(after_update=name):
                self.assertEqual(shown['severity'], Fault.SEVERITY_CRITICAL, shown)
                self.assertEqual(shown['description'], 'occurrence 2, lower', shown)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        shutil.rmtree(DB_DIR, ignore_errors=True)
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
