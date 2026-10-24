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

"""Log lines keep their logger names, and per-name level control still works.

Three gateways run side by side:

- ``gw_debug`` is launched with ``--log-level rest_server:=debug`` only.
- ``gw_plain`` is launched without a flag and loads a MODULE plugin that logs
  one line when it is configured.
- ``gw_format`` prints the call site through ``RCUTILS_CONSOLE_OUTPUT_FORMAT``.

Presence is checked while the gateways run. Absence is checked after shutdown,
over each process's whole output.
"""

import os
import re
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

OUTPUT_TIMEOUT = 20.0


def _domain(offset):
    return {'ROS_DOMAIN_ID': str(get_test_domain_id(offset))}


def generate_test_description():
    plugin_path = os.path.join(
        get_package_prefix('ros2_medkit_gateway'),
        'lib', 'ros2_medkit_gateway', 'libtest_gateway_plugin.so',
    )
    gw_debug = create_gateway_node(
        extra_env=_domain(0),
        ros_arguments=['--log-level', 'rest_server:=debug'],
    )
    gw_plain = create_gateway_node(
        port=get_test_port(1),
        name='gateway_plain_log_level',
        extra_env=_domain(1),
        extra_params={
            'plugins': ['test_plugin'],
            'plugins.test_plugin.path': plugin_path,
        },
    )
    gw_format = create_gateway_node(
        port=get_test_port(2),
        name='gateway_location_format',
        extra_env={
            **_domain(2),
            'RCUTILS_CONSOLE_OUTPUT_FORMAT':
                '[{severity}] [{name}] {file_name}:{line_number} {message}',
        },
    )
    return (
        LaunchDescription([
            gw_debug,
            gw_plain,
            gw_format,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gw_debug': gw_debug, 'gw_plain': gw_plain, 'gw_format': gw_format},
    )


def _line_pattern(level, logger, text):
    return re.compile(
        r'^\[' + level + r'\] \[[0-9.]+\] \[' + logger + r'\]: ' + re.escape(text),
        re.MULTILINE,
    )


def _output(proc_output, process):
    return ''.join(
        chunk.text.decode(errors='replace') for chunk in proc_output[process]
    )


def _wait_for_line(proc_output, process, pattern):
    deadline = time.monotonic() + OUTPUT_TIMEOUT * get_time_scale()
    while time.monotonic() < deadline:
        if pattern.search(_output(proc_output, process)):
            return True
        time.sleep(0.2)
    return False


def _exercise(base_url):
    """Drive requests that log a rest_server DEBUG line, plus a health and a 404."""
    assert requests.get(f'{base_url}/health', timeout=5).status_code == 200
    components = requests.get(f'{base_url}/components', timeout=5).json()['items']
    component_id = components[0]['id']
    response = requests.get(f'{base_url}/components/{component_id}/operations', timeout=5)
    assert response.status_code == 200, response.text
    assert requests.get(f'{base_url}/no_such_route', timeout=5).status_code == 404


def _wait_healthy(base_url):
    deadline = time.monotonic() + 30.0 * get_time_scale()
    while time.monotonic() < deadline:
        try:
            if requests.get(f'{base_url}/health', timeout=2).status_code == 200:
                return
        except requests.exceptions.RequestException:
            pass
        time.sleep(0.2)
    raise AssertionError(f'{base_url} not healthy')


DEBUG_REST = re.compile(r'\[DEBUG\] \[[0-9.]+\] \[rest_server\]')
DEBUG_HTTP = re.compile(r'\[DEBUG\] \[[0-9.]+\] \[http_server\]')
LISTING_LINE = _line_pattern('DEBUG', 'rest_server', 'Listing operations for component')
STARTUP_LINES = (
    ('rest_server', 'HTTP request thread pool bounded'),
    ('rest_server', 'OpenAPI metadata check'),
    ('http_server', 'Starting HTTP server'),
)


class TestRestLoggerNames(GatewayTestCase):
    """Logger names, per-name levels, call sites and plugin lines on the console."""

    MIN_EXPECTED_APPS = 0

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.plain_url = f'http://localhost:{get_test_port(1)}{API_BASE_PATH}'
        _wait_healthy(cls.plain_url)

    def test_debug_line_appears_when_enabled_by_name(self, proc_output, gw_debug):
        """rest_server DEBUG lines print when raised by name."""
        _exercise(self.BASE_URL)
        self.assertTrue(
            _wait_for_line(proc_output, gw_debug, LISTING_LINE),
            _output(proc_output, gw_debug),
        )

    def test_info_lines_keep_their_logger_names(self, proc_output, gw_debug, gw_plain):
        """Startup INFO lines carry the same names with and without the flag."""
        for process in (gw_debug, gw_plain):
            for logger, text in STARTUP_LINES:
                self.assertTrue(
                    _wait_for_line(proc_output, process, _line_pattern('INFO', logger, text)),
                    f'{logger}: {text}\n' + _output(proc_output, process),
                )

    def test_plain_gateway_serves_requests(self, proc_output, gw_plain):
        """Requests reach a gateway launched without the flag."""
        _exercise(self.plain_url)
        self.assertTrue(
            _wait_for_line(
                proc_output, gw_plain,
                _line_pattern('INFO', 'rest_server', 'OpenAPI metadata check'),
            ),
        )

    def test_plugin_line_goes_through_the_process_sink(self, proc_output, gw_plain):
        """A line logged by a MODULE plugin is formatted by rcutils, with a ROS timestamp."""
        self.assertTrue(
            _wait_for_line(
                proc_output, gw_plain,
                _line_pattern('INFO', 'test_gateway_plugin', 'test_gateway_plugin configured'),
            ),
            _output(proc_output, gw_plain),
        )

    def test_call_site_reaches_the_console_format(self, proc_output, gw_format):
        """File name and line number of the call site reach the console format."""
        pattern = re.compile(
            r'^\[INFO\] \[rest_server\] \S*[^/\s]+\.cpp:([0-9]+) '
            r'HTTP request thread pool bounded',
            re.MULTILINE,
        )
        self.assertTrue(
            _wait_for_line(proc_output, gw_format, pattern), _output(proc_output, gw_format),
        )
        match = pattern.search(_output(proc_output, gw_format))
        self.assertGreater(int(match.group(1)), 0)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All gateways exit cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)

    def test_only_the_named_logger_prints_debug(self, proc_output, gw_debug):
        """Raising rest_server leaves http_server at INFO."""
        text = _output(proc_output, gw_debug)
        self.assertRegex(text, DEBUG_REST)
        self.assertNotRegex(text, DEBUG_HTTP)

    def test_no_debug_lines_without_the_flag(self, proc_output, gw_plain):
        """A gateway launched without the flag prints no DEBUG line for these names."""
        text = _output(proc_output, gw_plain)
        self.assertNotRegex(text, DEBUG_REST)
        self.assertNotRegex(text, DEBUG_HTTP)
