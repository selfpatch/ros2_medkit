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

"""Base test case for ros2_medkit gateway integration tests.

Provides health-check polling, discovery waiting, HTTP helpers, wait helpers,
and discovery assertion methods. Replaces ~400 lines of duplicated boilerplate
in every test file.

Subclasses override class constants to control discovery waiting:

    class TestMyFeature(GatewayTestCase):
        MIN_EXPECTED_APPS = 3
        REQUIRED_AREAS = {'powertrain', 'chassis'}
        REQUIRED_APPS = {'temp_sensor', 'actuator'}
"""

import time
import unittest
from urllib.parse import quote

import requests

from ros2_medkit_test_utils.constants import (
    ACTION_TIMEOUT,
    DEFAULT_BASE_URL,
    DISCOVERY_INTERVAL,
    DISCOVERY_TIMEOUT,
    FAULT_TIMEOUT,
    GATEWAY_STARTUP_INTERVAL,
    GATEWAY_STARTUP_TIMEOUT,
    ROSBAG_TIMEOUT,
)


class GatewayTestCase(unittest.TestCase):
    """Base class for gateway integration tests.

    Class Attributes
    ----------------
    PORT : int
        Gateway HTTP port. Override for tests that use a non-default port.
    BASE_URL : str
        Full base URL including API path prefix.
    MIN_EXPECTED_APPS : int
        Minimum number of apps that must be discovered before tests run.
        Set to 0 to skip discovery waiting entirely.
    REQUIRED_AREAS : set of str
        Area IDs that must be discovered. Empty set skips area checking.
    REQUIRED_APPS : set of str
        App IDs that must be discovered. Empty set skips app checking.

    """

    BASE_URL = DEFAULT_BASE_URL

    # Override in subclass for discovery waiting.
    # When all three are falsy, setUpClass skips discovery polling.
    MIN_EXPECTED_APPS = 0
    REQUIRED_AREAS: set = set()
    REQUIRED_APPS: set = set()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    @classmethod
    def setUpClass(cls):
        """Wait for gateway health, then optionally wait for discovery."""
        cls._wait_for_gateway_health()
        if cls.MIN_EXPECTED_APPS > 0 or cls.REQUIRED_AREAS or cls.REQUIRED_APPS:
            cls._wait_for_discovery()

    @classmethod
    def _wait_for_gateway_health(cls):
        """Poll GET /health until the gateway responds with 200.

        Uses ``time.monotonic()`` for a reliable, monotonic clock.

        Raises
        ------
        AssertionError
            If the gateway does not respond within
            ``GATEWAY_STARTUP_TIMEOUT`` seconds.

        """
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(GATEWAY_STARTUP_INTERVAL)
        raise AssertionError(
            f'Gateway not responding after {GATEWAY_STARTUP_TIMEOUT}s'
        )

    @classmethod
    def _wait_for_discovery(cls):
        """Poll /apps and /areas until required entities are discovered.

        Checks:
        1. Number of apps >= ``MIN_EXPECTED_APPS``
        2. All ``REQUIRED_APPS`` IDs are present
        3. All ``REQUIRED_AREAS`` IDs are present

        Raises
        ------
        AssertionError
            If the deadline is reached without satisfying all conditions.

        """
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        while time.monotonic() < deadline:
            try:
                apps_response = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                areas_response = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
                if apps_response.status_code == 200 and areas_response.status_code == 200:
                    apps = apps_response.json().get('items', [])
                    areas = areas_response.json().get('items', [])
                    app_ids = {a.get('id', '') for a in apps}
                    area_ids = {a.get('id', '') for a in areas}

                    missing_areas = cls.REQUIRED_AREAS - area_ids
                    missing_apps = cls.REQUIRED_APPS - app_ids
                    apps_ok = len(apps) >= cls.MIN_EXPECTED_APPS and not missing_apps
                    areas_ok = not missing_areas

                    if apps_ok and areas_ok:
                        print(
                            f'Discovery complete: {len(apps)} apps, '
                            f'{len(areas)} areas'
                        )
                        return

                    print(
                        f'  Waiting: {len(apps)}/{cls.MIN_EXPECTED_APPS} apps, '
                        f'{len(areas)} areas. '
                        f'Missing areas: {missing_areas}, '
                        f'Missing apps: {missing_apps}'
                    )
            except requests.exceptions.RequestException:
                # Connection errors expected while nodes are starting; retry.
                pass
            time.sleep(DISCOVERY_INTERVAL)

        discovered_apps = set()
        discovered_areas = set()
        try:
            r = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
            if r.status_code == 200:
                discovered_apps = {
                    a.get('id', '') for a in r.json().get('items', [])
                }
            r = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
            if r.status_code == 200:
                discovered_areas = {
                    a.get('id', '') for a in r.json().get('items', [])
                }
        except requests.exceptions.RequestException:
            pass
        raise AssertionError(
            f'Discovery incomplete after {DISCOVERY_TIMEOUT}s - '
            f'found {len(discovered_apps)} apps, need {cls.MIN_EXPECTED_APPS}. '
            f'Missing apps: {cls.REQUIRED_APPS - discovered_apps}, '
            f'Missing areas: {cls.REQUIRED_AREAS - discovered_areas}'
        )

    # ------------------------------------------------------------------
    # HTTP helpers
    # ------------------------------------------------------------------

    def get_json(self, endpoint, *, timeout=10, expected_status=200):
        """Send GET and return parsed JSON.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL`` (e.g. ``'/apps'``).
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code. Assertion fails otherwise.

        Returns
        -------
        dict
            Parsed JSON response body.

        """
        response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=timeout)
        self.assertEqual(
            response.status_code,
            expected_status,
            f'GET {endpoint} returned {response.status_code}: {response.text}',
        )
        return response.json()

    def post_json(self, endpoint, data, *, timeout=10, expected_status=200):
        """Send POST with JSON body and return parsed JSON.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        data : dict
            JSON-serializable request body.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code. Assertion fails otherwise.

        Returns
        -------
        dict
            Parsed JSON response body.

        """
        response = requests.post(
            f'{self.BASE_URL}{endpoint}', json=data, timeout=timeout
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'POST {endpoint} returned {response.status_code}: {response.text}',
        )
        return response.json()

    def put_json(self, endpoint, data, *, timeout=10, expected_status=200):
        """Send PUT with JSON body and return parsed JSON.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        data : dict
            JSON-serializable request body.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code. Assertion fails otherwise.

        Returns
        -------
        dict
            Parsed JSON response body.

        """
        response = requests.put(
            f'{self.BASE_URL}{endpoint}', json=data, timeout=timeout
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'PUT {endpoint} returned {response.status_code}: {response.text}',
        )
        return response.json()

    def delete_request(self, endpoint, *, timeout=10, expected_status=204):
        """Send DELETE and return the raw ``Response``.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code (default 204 No Content).

        Returns
        -------
        requests.Response
            The raw response object.

        """
        response = requests.delete(
            f'{self.BASE_URL}{endpoint}', timeout=timeout
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'DELETE {endpoint} returned {response.status_code}: {response.text}',
        )
        return response

    def get_raw(self, endpoint, *, timeout=10, expected_status=200, **kwargs):
        """Send GET and return the raw ``Response`` (no JSON parsing).

        Useful for binary downloads, streaming, or inspecting headers.

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        timeout : int
            Request timeout in seconds.
        expected_status : int
            Expected HTTP status code.
        **kwargs
            Extra keyword arguments forwarded to ``requests.get()``
            (e.g. ``stream=True``).

        Returns
        -------
        requests.Response
            The raw response object.

        """
        response = requests.get(
            f'{self.BASE_URL}{endpoint}', timeout=timeout, **kwargs
        )
        self.assertEqual(
            response.status_code,
            expected_status,
            f'GET {endpoint} returned {response.status_code}: {response.text}',
        )
        return response

    # ------------------------------------------------------------------
    # Waiters
    # ------------------------------------------------------------------

    def poll_endpoint(self, endpoint, *, timeout=10.0, interval=0.2):
        """Poll GET endpoint until it returns 200 and return parsed JSON.

        Useful for waiting on entities or resources that may take time to
        become available (e.g. after discovery, service-only nodes that
        can transiently disappear from the ROS 2 graph between cycles).

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL`` (e.g. ``'/apps/calibration/data'``).
        timeout : float
            Maximum time to wait in seconds.
        interval : float
            Sleep between retries in seconds.

        Returns
        -------
        dict
            Parsed JSON response body.

        Raises
        ------
        AssertionError
            If the deadline is reached without a 200 response.

        """
        return self.poll_endpoint_until(
            endpoint,
            condition=None,
            timeout=timeout,
            interval=interval,
        )

    def poll_endpoint_until(
        self,
        endpoint,
        condition=None,
        *,
        timeout=10.0,
        interval=0.2,
    ):
        """Poll GET endpoint until it returns 200 and a condition is met.

        When *condition* is ``None`` (the default), any 200 response succeeds
        and the full parsed JSON is returned -- equivalent to
        :meth:`poll_endpoint`.

        When *condition* is a callable, it receives the parsed JSON on each
        200 response.  If it returns a truthy value, that value is returned
        to the caller.  If it returns a falsy value, polling continues.
        This allows both boolean checks and value extraction::

            # Boolean -- wait for non-empty items list
            data = self.poll_endpoint_until(
                '/apps/lidar_sensor/faults',
                lambda d: d.get('items'),
            )

            # Extraction -- return the first fault item directly
            fault = self.poll_endpoint_until(
                '/apps/lidar_sensor/faults',
                lambda d: next(iter(d.get('items', [])), None),
            )

        Parameters
        ----------
        endpoint : str
            Path relative to ``BASE_URL``.
        condition : callable or None
            ``condition(response_json) -> truthy_value``.  When ``None``,
            any 200 response is accepted.
        timeout : float
            Maximum time to wait in seconds.
        interval : float
            Sleep between retries in seconds.

        Returns
        -------
        object
            Parsed JSON (when *condition* is ``None``) or whatever
            *condition* returned.

        Raises
        ------
        AssertionError
            If the deadline is reached without satisfying the condition.

        """
        start_time = time.monotonic()
        last_error = None
        while time.monotonic() - start_time < timeout:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{endpoint}', timeout=2
                )
                if response.status_code == 200:
                    data = response.json()
                    if condition is None:
                        return data
                    result = condition(data)
                    if result:
                        return result
                    last_error = 'Condition not met'
                else:
                    last_error = f'Status {response.status_code}'
            except requests.exceptions.RequestException as e:
                last_error = str(e)
            time.sleep(interval)
        self.fail(
            f'GET {endpoint} not available after {timeout}s. '
            f'Last error: {last_error}'
        )

    def wait_for_fault(self, entity_endpoint, fault_code, *, max_wait=FAULT_TIMEOUT):
        """Poll until a specific fault appears on an entity.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL`` (e.g. ``'/apps/lidar_sensor'``).
        fault_code : str
            The ``fault_code`` value to wait for.
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The fault object from the ``items`` list when found.

        Raises
        ------
        AssertionError
            If the fault is not found within *max_wait*.

        """
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/faults', timeout=5
                )
                if response.status_code == 200:
                    data = response.json()
                    for fault in data.get('items', []):
                        if fault.get('fault_code') == fault_code:
                            return fault
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'Fault {fault_code} not found at {entity_endpoint}/faults '
            f'within {max_wait}s'
        )

    def wait_for_fault_with_rosbag(self, entity_endpoint, *, max_wait=ROSBAG_TIMEOUT):
        """Poll until a rosbag snapshot is available for an entity.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL`` (e.g. ``'/apps/lidar_sensor'``).
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        str or None
            The rosbag ``id`` if found, ``None`` on timeout.

        """
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/bulk-data/rosbags',
                    timeout=5,
                )
                if response.status_code == 200:
                    data = response.json()
                    items = data.get('items', [])
                    if len(items) > 0:
                        return items[0].get('id')
            except requests.exceptions.RequestException:
                pass
            time.sleep(1)
        return None

    def create_execution(self, entity_endpoint, operation_id, *, input_data=None):
        """POST to create an operation execution and return the response JSON.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL``
            (e.g. ``'/apps/long_calibration'``).
        operation_id : str
            The operation ID to execute.
        input_data : dict or None
            Input parameters for the execution. Defaults to ``{}``.

        Returns
        -------
        tuple[requests.Response, dict]
            ``(response, data)`` tuple. Callers should check
            ``response.status_code`` (202 for async, 200 for sync).

        """
        if input_data is None:
            input_data = {}
        url = f'{self.BASE_URL}{entity_endpoint}/operations/{operation_id}/executions'
        response = requests.post(url, json={'parameters': input_data}, timeout=15)
        data = response.json()
        return response, data

    def wait_for_execution_status(
        self, exec_endpoint, target_statuses, *, max_wait=ACTION_TIMEOUT
    ):
        """Poll execution status until it reaches one of the target statuses.

        Parameters
        ----------
        exec_endpoint : str
            Full execution path relative to ``BASE_URL``
            (e.g. ``'/apps/long_calibration/operations/long_calibration/
            executions/<id>'``).
        target_statuses : list of str
            SOVD status strings to wait for (e.g. ``['completed', 'failed']``).
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The status response data when target status is reached.

        Raises
        ------
        AssertionError
            If no target status is reached within *max_wait*.

        """
        start_time = time.monotonic()
        last_status = None
        while time.monotonic() - start_time < max_wait:
            try:
                status_response = requests.get(
                    f'{self.BASE_URL}{exec_endpoint}', timeout=5
                )
                if status_response.status_code == 200:
                    data = status_response.json()
                    last_status = data.get('status')
                    if last_status in target_statuses:
                        return data
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'Execution did not reach status {target_statuses} within {max_wait}s. '
            f'Last status: {last_status}'
        )

    def wait_for_operation(self, entity_endpoint, operation_id, *, max_wait=15.0):
        """Wait for an operation to be discovered for an entity.

        Parameters
        ----------
        entity_endpoint : str
            Entity path relative to ``BASE_URL``
            (e.g. ``'/apps/calibration'``).
        operation_id : str
            The operation ID to wait for.
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        bool
            True if the operation was found, False on timeout.

        """
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{entity_endpoint}/operations', timeout=5
                )
                if response.status_code == 200:
                    ops = response.json().get('items', [])
                    if any(op.get('id') == operation_id for op in ops):
                        return True
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        return False

    # ------------------------------------------------------------------
    # Discovery assertions
    # ------------------------------------------------------------------

    def assert_entity_exists(self, entity_type, entity_id):
        """Assert that an entity of the given type and ID exists.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        entity_id : str
            The entity ID to look up.

        Returns
        -------
        dict
            The entity JSON data.

        """
        response = requests.get(
            f'{self.BASE_URL}/{entity_type}/{entity_id}', timeout=10
        )
        self.assertEqual(
            response.status_code,
            200,
            f'{entity_type}/{entity_id} not found: '
            f'{response.status_code} {response.text}',
        )
        return response.json()

    def assert_entity_not_found(self, entity_type, entity_id):
        """Assert that an entity returns 404.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        entity_id : str
            The entity ID to look up.

        """
        response = requests.get(
            f'{self.BASE_URL}/{entity_type}/{entity_id}', timeout=10
        )
        self.assertEqual(
            response.status_code,
            404,
            f'Expected 404 for {entity_type}/{entity_id}, '
            f'got {response.status_code}: {response.text}',
        )

    def assert_entity_has_capabilities(
        self, entity_type, entity_id, expected_capabilities
    ):
        """Assert that an entity advertises the given capability collections.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        entity_id : str
            The entity ID to inspect.
        expected_capabilities : list of str
            Collection names expected in the entity's ``capabilities`` field
            (e.g. ``['data', 'operations', 'configurations']``).

        """
        data = self.assert_entity_exists(entity_type, entity_id)
        capabilities = data.get('capabilities', [])
        for cap in expected_capabilities:
            self.assertIn(
                cap,
                capabilities,
                f'{entity_type}/{entity_id} missing capability {cap!r}. '
                f'Has: {capabilities}',
            )

    def assert_entity_list_contains(self, entity_type, expected_ids):
        """Assert that a collection listing contains the given IDs.

        Parameters
        ----------
        entity_type : str
            Plural entity type (``'apps'``, ``'components'``, ``'areas'``).
        expected_ids : set or list of str
            Entity IDs that must appear in the listing.

        """
        data = self.get_json(f'/{entity_type}')
        items = data.get('items', [])
        found_ids = {item.get('id', '') for item in items}
        missing = set(expected_ids) - found_ids
        self.assertFalse(
            missing,
            f'Missing {entity_type} IDs: {missing}. Found: {found_ids}',
        )

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def encode_topic_path(topic_path):
        """Encode a ROS topic path for use in URLs.

        Slashes are encoded as ``%2F`` for proper URL routing.

        Example::

            '/powertrain/engine/temperature' -> 'powertrain%2Fengine%2Ftemperature'

        Parameters
        ----------
        topic_path : str
            Full ROS topic path starting with ``'/'``.

        Returns
        -------
        str
            URL-encoded topic path without leading slash.

        Raises
        ------
        ValueError
            If *topic_path* doesn't start with ``'/'``.

        """
        if not topic_path.startswith('/'):
            raise ValueError(f"Topic path must start with '/': {topic_path}")
        # Remove leading slash and encode the rest
        topic_path = topic_path[1:]
        return quote(topic_path, safe='')
