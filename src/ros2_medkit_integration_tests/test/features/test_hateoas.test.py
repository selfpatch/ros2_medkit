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

"""Feature tests for HATEOAS compliance (hrefs, capability URIs, links).

Validates that list responses include href fields, entity details include
capability URIs, subareas/subcomponents/contains/hosts/depends-on endpoints
return proper link structures, and x-medkit extensions are present.

Migrated from:
- test_70 through test_81 (hrefs, capability URIs, subareas, subcomponents,
  contains, hosts, depends-on, functions)
- test_83 (x-medkit extension in list responses)
"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    ACTUATOR_NODES,
    create_test_launch,
    SENSOR_NODES,
    SERVICE_NODES,
)


def generate_test_description():
    return create_test_launch(
        demo_nodes=SENSOR_NODES + ACTUATOR_NODES + SERVICE_NODES,
        fault_manager=False,
    )


class TestHateoas(GatewayTestCase):
    """HATEOAS compliance tests for hrefs, links, and capability URIs."""

    MIN_EXPECTED_APPS = 8
    REQUIRED_AREAS = {'powertrain', 'chassis', 'body'}

    # ------------------------------------------------------------------
    # List hrefs (test_70-72)
    # ------------------------------------------------------------------

    def test_components_list_has_href(self):
        """GET /components returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertGreater(len(components), 0, 'Should have at least one component')

        for component in components:
            self.assertIn('id', component, "Component should have 'id'")
            self.assertIn('name', component, "Component should have 'name'")
            self.assertIn('href', component, "Component should have 'href'")
            self.assertTrue(
                component['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {component['href']}"
            )
            self.assertIn(component['id'], component['href'])

    def test_apps_list_has_href(self):
        """GET /apps returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/apps')
        self.assertIn('items', data)
        apps = data['items']
        self.assertGreater(len(apps), 0, 'Should have at least one app')

        for app in apps:
            self.assertIn('id', app, "App should have 'id'")
            self.assertIn('name', app, "App should have 'name'")
            self.assertIn('href', app, "App should have 'href'")
            self.assertTrue(
                app['href'].startswith('/api/v1/apps/'),
                f"href should start with /api/v1/apps/, got: {app['href']}"
            )
            self.assertIn(app['id'], app['href'])

    def test_areas_list_has_href(self):
        """GET /areas returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertGreater(len(areas), 0, 'Should have at least one area')

        for area in areas:
            self.assertIn('id', area, "Area should have 'id'")
            self.assertIn('name', area, "Area should have 'name'")
            self.assertIn('href', area, "Area should have 'href'")
            self.assertTrue(
                area['href'].startswith('/api/v1/areas/'),
                f"href should start with /api/v1/areas/, got: {area['href']}"
            )
            self.assertIn(area['id'], area['href'])

    # ------------------------------------------------------------------
    # Capability URIs (test_73-74)
    # ------------------------------------------------------------------

    def test_component_detail_has_capability_uris(self):
        """GET /components/{id} returns capability URIs at top level.

        SOVD requires entity details to have flat capability URIs.

        @verifies REQ_INTEROP_003
        """
        components = self.get_json('/components')['items']
        self.assertGreater(len(components), 0)

        component_id = components[0]['id']
        data = self.get_json(f'/components/{component_id}')

        # Verify required fields
        self.assertIn('id', data)
        self.assertEqual(data['id'], component_id)
        self.assertIn('name', data)

        # Verify SOVD capability URIs at top level
        self.assertIn('data', data, 'Component should have data URI')
        self.assertIn('operations', data, 'Component should have operations URI')
        self.assertIn('configurations', data, 'Component should have configurations URI')
        self.assertIn('faults', data, 'Component should have faults URI')

        # Verify URIs are correct format
        base = f'/api/v1/components/{component_id}'
        self.assertEqual(data['data'], f'{base}/data')
        self.assertEqual(data['operations'], f'{base}/operations')
        self.assertEqual(data['configurations'], f'{base}/configurations')
        self.assertEqual(data['faults'], f'{base}/faults')

    def test_app_detail_has_capability_uris(self):
        """GET /apps/{id} returns capability URIs at top level.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/apps/temp_sensor')

        # Verify required fields
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'temp_sensor')
        self.assertIn('name', data)

        # Verify SOVD capability URIs at top level
        self.assertIn('data', data, 'App should have data URI')
        self.assertIn('operations', data, 'App should have operations URI')
        self.assertIn('configurations', data, 'App should have configurations URI')

        # Verify URIs are correct format
        base = '/api/v1/apps/temp_sensor'
        self.assertEqual(data['data'], f'{base}/data')
        self.assertEqual(data['operations'], f'{base}/operations')
        self.assertEqual(data['configurations'], f'{base}/configurations')

    # ------------------------------------------------------------------
    # Subareas and subcomponents (test_75-76)
    # ------------------------------------------------------------------

    def test_subareas_list_has_href(self):
        """GET /areas/{id}/subareas returns items with href field.

        @verifies REQ_INTEROP_004
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/root/subareas',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        for subarea in data.get('items', []):
            self.assertIn('id', subarea, "Subarea should have 'id'")
            self.assertIn('name', subarea, "Subarea should have 'name'")
            self.assertIn('href', subarea, "Subarea should have 'href'")
            self.assertTrue(
                subarea['href'].startswith('/api/v1/areas/'),
                f"href should start with /api/v1/areas/, got: {subarea['href']}"
            )

    def test_subcomponents_list_has_href(self):
        """GET /components/{id}/subcomponents returns items with href field.

        @verifies REQ_INTEROP_005
        """
        components = self.get_json('/components')['items']
        self.assertGreater(len(components), 0)
        component_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{component_id}/subcomponents',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        for subcomp in data.get('items', []):
            self.assertIn('id', subcomp, "Subcomponent should have 'id'")
            self.assertIn('name', subcomp, "Subcomponent should have 'name'")
            self.assertIn('href', subcomp, "Subcomponent should have 'href'")
            self.assertTrue(
                subcomp['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {subcomp['href']}"
            )

    # ------------------------------------------------------------------
    # Contains and hosts (test_77b-77c)
    # ------------------------------------------------------------------

    def test_contains_list_has_href(self):
        """GET /areas/{id}/contains returns items with href field.

        @verifies REQ_INTEROP_006
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/root/contains',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('_links', data)
        self.assertEqual(data['_links']['self'], '/api/v1/areas/root/contains')
        self.assertEqual(data['_links']['area'], '/api/v1/areas/root')

        for comp in data.get('items', []):
            self.assertIn('id', comp, "Contained component should have 'id'")
            self.assertIn('name', comp, "Contained component should have 'name'")
            self.assertIn('href', comp, "Contained component should have 'href'")
            self.assertTrue(
                comp['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {comp['href']}"
            )

    def test_hosts_list_has_href(self):
        """GET /components/{id}/hosts returns items with href field.

        @verifies REQ_INTEROP_007
        """
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/hosts',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('_links', data)
        self.assertEqual(data['_links']['self'], '/api/v1/components/powertrain/hosts')
        self.assertEqual(data['_links']['component'], '/api/v1/components/powertrain')

        for app in data.get('items', []):
            self.assertIn('id', app, "Hosted app should have 'id'")
            self.assertIn('name', app, "Hosted app should have 'name'")
            self.assertIn('href', app, "Hosted app should have 'href'")
            self.assertTrue(
                app['href'].startswith('/api/v1/apps/'),
                f"href should start with /api/v1/apps/, got: {app['href']}"
            )

    # ------------------------------------------------------------------
    # Depends-on (test_78-80)
    # ------------------------------------------------------------------

    def test_depends_on_components_has_href(self):
        """GET /components/{id}/depends-on returns items with href field.

        @verifies REQ_INTEROP_008
        """
        components = self.get_json('/components')['items']
        self.assertGreater(len(components), 0)
        component_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{component_id}/depends-on',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        for dep in data.get('items', []):
            self.assertIn('id', dep, "Dependency should have 'id'")
            self.assertIn('name', dep, "Dependency should have 'name'")
            self.assertIn('href', dep, "Dependency should have 'href'")
            self.assertTrue(
                dep['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {dep['href']}"
            )

    def test_depends_on_apps_has_href(self):
        """GET /apps/{id}/depends-on returns items with href field.

        @verifies REQ_INTEROP_009
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/depends-on',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('_links', data)
        self.assertEqual(data['_links']['self'], '/api/v1/apps/temp_sensor/depends-on')
        self.assertEqual(data['_links']['app'], '/api/v1/apps/temp_sensor')

        for dep in data.get('items', []):
            self.assertIn('id', dep, "Dependency should have 'id'")
            self.assertIn('name', dep, "Dependency should have 'name'")
            self.assertIn('href', dep, "Dependency should have 'href'")
            self.assertTrue(
                dep['href'].startswith('/api/v1/apps/'),
                f"href should start with /api/v1/apps/, got: {dep['href']}"
            )

    def test_depends_on_apps_nonexistent(self):
        """GET /apps/{id}/depends-on returns 404 for unknown app.

        @verifies REQ_INTEROP_009
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/depends-on',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'App not found')
        self.assertIn('parameters', data)
        self.assertIn('app_id', data['parameters'])
        self.assertEqual(data['parameters'].get('app_id'), 'nonexistent_app')

    # ------------------------------------------------------------------
    # Functions (test_81)
    # ------------------------------------------------------------------

    def test_functions_list_has_href(self):
        """GET /functions returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions')
        self.assertIn('items', data)
        functions = data['items']

        # Functions may be empty if no manifest is loaded
        for func in functions:
            self.assertIn('id', func, "Function should have 'id'")
            self.assertIn('name', func, "Function should have 'name'")
            self.assertIn('href', func, "Function should have 'href'")
            self.assertTrue(
                func['href'].startswith('/api/v1/functions/'),
                f"href should start with /api/v1/functions/, got: {func['href']}"
            )

    # ------------------------------------------------------------------
    # x-medkit extension (test_83)
    # ------------------------------------------------------------------

    def test_x_medkit_extension_in_list_responses(self):
        """List responses have x-medkit at item and response level.

        ROS2-specific data should be in x-medkit extension, not at top level.

        @verifies REQ_INTEROP_003
        """
        # Test components list
        data = self.get_json('/components')
        self.assertIn('items', data)
        self.assertIn('x-medkit', data, 'Response should have x-medkit')
        self.assertIn('total_count', data['x-medkit'], 'Response x-medkit should have total_count')

        for component in data['items']:
            self.assertIn('x-medkit', component, 'Item should have x-medkit')
            x_medkit = component['x-medkit']
            self.assertIn('source', x_medkit, 'x-medkit should have source')

        # Test apps list
        data = self.get_json('/apps')
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)

        for app in data['items']:
            self.assertIn('x-medkit', app, 'Item should have x-medkit')
            x_medkit = app['x-medkit']
            self.assertIn('source', x_medkit, 'x-medkit should have source')
            self.assertIn('is_online', x_medkit, 'x-medkit should have is_online')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'Process {info.process_name} exited with code {info.returncode}'
            )
