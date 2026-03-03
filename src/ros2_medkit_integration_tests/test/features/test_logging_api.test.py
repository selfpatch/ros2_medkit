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

"""Feature tests for communication log REST API endpoints.

Validates GET /logs, GET /logs/configuration, and PUT /logs/configuration
for both App and Component entities.

- App log queries use exact FQN matching.
- Component log queries use namespace prefix matching (all nodes under
  the component namespace are included).

"""

import unittest

import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


class TestLoggingApi(GatewayTestCase):
    """Communication log endpoint feature tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    # ------------------------------------------------------------------
    # GET /apps/{id}/logs
    # ------------------------------------------------------------------

    def test_app_get_logs_returns_200(self):
        """GET /apps/{app_id}/logs returns 200 with items array.

        # @verifies REQ_INTEROP_061
        """
        data = self.get_json('/apps/temp_sensor/logs')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_app_get_logs_has_items_after_startup(self):
        """GET /apps/{app_id}/logs eventually contains log entries.

        /rosout messages from demo node startup populate the ring buffer.
        Poll until at least one entry is present.

        # @verifies REQ_INTEROP_061
        """
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/logs',
            condition=lambda d: len(d.get('items', [])) > 0,
            timeout=15.0,
        )
        items = data['items']
        self.assertGreater(len(items), 0, 'Expected at least one log entry')

    def test_app_log_entry_has_required_fields(self):
        """Each log entry has id, timestamp, severity, message, context fields.

        # @verifies REQ_INTEROP_061
        """
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/logs',
            condition=lambda d: len(d.get('items', [])) > 0,
            timeout=15.0,
        )
        entry = data['items'][0]
        self.assertIn('id', entry)
        self.assertIn('timestamp', entry)
        self.assertIn('severity', entry)
        self.assertIn('message', entry)
        self.assertIn('context', entry)
        # id format: "log_N"
        self.assertTrue(
            entry['id'].startswith('log_'),
            f"id should start with 'log_', got: {entry['id']}"
        )
        # timestamp is ISO 8601 with Z suffix
        ts = entry['timestamp']
        self.assertTrue(
            ts.endswith('Z') and 'T' in ts,
            f"timestamp should be ISO 8601 with Z suffix, got: {ts}"
        )
        # severity is one of the valid values
        self.assertIn(
            entry['severity'],
            {'debug', 'info', 'warning', 'error', 'fatal'},
            f"unexpected severity: {entry['severity']}"
        )
        # context.node is present
        self.assertIn('node', entry['context'])

    def test_app_get_logs_severity_filter(self):
        """GET /apps/{id}/logs?severity=error returns only entries at error level or above.

        # @verifies REQ_INTEROP_061
        """
        data = self.get_json('/apps/temp_sensor/logs?severity=error')
        self.assertIn('items', data)
        for entry in data['items']:
            self.assertIn(
                entry['severity'],
                {'error', 'fatal'},
                f"severity filter did not exclude lower levels: {entry['severity']}"
            )

    def test_app_get_logs_invalid_entity_returns_404(self):
        """GET /apps/{id}/logs returns 404 for unknown entity.

        # @verifies REQ_INTEROP_061
        """
        self.get_json('/apps/nonexistent_app_xyz/logs', expected_status=404)

    # ------------------------------------------------------------------
    # GET /apps/{id}/logs/configuration
    # ------------------------------------------------------------------

    def test_app_get_logs_configuration_returns_200(self):
        """GET /apps/{id}/logs/configuration returns 200 with config fields.

        # @verifies REQ_INTEROP_063
        """
        data = self.get_json('/apps/temp_sensor/logs/configuration')
        self.assertIn('severity_filter', data)
        self.assertIn('max_entries', data)

    def test_app_get_logs_configuration_defaults(self):
        """GET /apps/{id}/logs/configuration returns default values for unconfigured entity.

        # @verifies REQ_INTEROP_063
        """
        data = self.get_json('/apps/temp_sensor/logs/configuration')
        self.assertEqual(data['severity_filter'], 'debug')
        self.assertGreater(data['max_entries'], 0)

    # ------------------------------------------------------------------
    # PUT /apps/{id}/logs/configuration
    # ------------------------------------------------------------------

    def test_app_put_logs_configuration_updates_severity_filter(self):
        """PUT /apps/{id}/logs/configuration returns 204 and update is visible via GET.

        # @verifies REQ_INTEROP_064
        """
        self.put_raw(
            '/apps/temp_sensor/logs/configuration',
            {'severity_filter': 'warning'},
            expected_status=204,
        )
        data = self.get_json('/apps/temp_sensor/logs/configuration')
        self.assertEqual(data['severity_filter'], 'warning')

    def test_app_put_logs_configuration_updates_max_entries(self):
        """PUT /apps/{id}/logs/configuration returns 204 and update is visible via GET.

        # @verifies REQ_INTEROP_064
        """
        self.put_raw(
            '/apps/temp_sensor/logs/configuration',
            {'max_entries': 500},
            expected_status=204,
        )
        data = self.get_json('/apps/temp_sensor/logs/configuration')
        self.assertEqual(data['max_entries'], 500)

    def test_app_put_logs_configuration_invalid_severity_returns_400(self):
        """PUT /apps/{id}/logs/configuration returns 400 for invalid severity.

        # @verifies REQ_INTEROP_064
        """
        resp = self.put_raw(
            '/apps/temp_sensor/logs/configuration',
            {'severity_filter': 'verbose'},
            expected_status=400,
        )
        body = resp.json()
        self.assertIn('error_code', body)

    def test_app_put_logs_configuration_zero_max_entries_returns_400(self):
        """PUT /apps/{id}/logs/configuration returns 400 for max_entries=0.

        # @verifies REQ_INTEROP_064
        """
        resp = self.put_raw(
            '/apps/temp_sensor/logs/configuration',
            {'max_entries': 0},
            expected_status=400,
        )
        body = resp.json()
        self.assertIn('error_code', body)

    # ------------------------------------------------------------------
    # GET /components/{id}/logs  (prefix match)
    # ------------------------------------------------------------------

    def test_component_get_logs_returns_200(self):
        """GET /components/{id}/logs returns 200 with items array.

        # @verifies REQ_INTEROP_061
        """
        components = self.get_json('/components')['items']
        self.assertGreater(len(components), 0, 'At least one component required')
        comp_id = components[0]['id']

        data = self.get_json(f'/components/{comp_id}/logs')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_component_get_logs_configuration_returns_200(self):
        """GET /components/{id}/logs/configuration returns 200 with config.

        # @verifies REQ_INTEROP_063
        """
        components = self.get_json('/components')['items']
        self.assertGreater(len(components), 0, 'At least one component required')
        comp_id = components[0]['id']

        data = self.get_json(f'/components/{comp_id}/logs/configuration')
        self.assertIn('severity_filter', data)
        self.assertIn('max_entries', data)

    # ------------------------------------------------------------------
    # Capability advertisement
    # ------------------------------------------------------------------

    def test_app_detail_includes_logs_capability(self):
        """GET /apps/{id} response includes 'logs' in capabilities list.

        # @verifies REQ_INTEROP_061
        """
        data = self.get_json('/apps/temp_sensor')
        self.assertIn('capabilities', data)
        cap_names = [cap['name'] for cap in data['capabilities']]
        self.assertIn('logs', cap_names, f'Expected "logs" in capabilities, got: {cap_names}')

    def test_component_detail_includes_logs_capability(self):
        """GET /components/{id} response includes 'logs' in capabilities list.

        # @verifies REQ_INTEROP_061
        """
        components = self.get_json('/components')['items']
        self.assertGreater(len(components), 0, 'At least one component required')
        comp_id = components[0]['id']

        data = self.get_json(f'/components/{comp_id}')
        self.assertIn('capabilities', data)
        cap_names = [cap['name'] for cap in data['capabilities']]
        self.assertIn('logs', cap_names, f'Expected "logs" in capabilities, got: {cap_names}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
