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

"""Plugin config reaches the plugin and is not served as ROS parameters.

Two gateways load ``topics_test_plugin`` with the same config: one gets it from a
params file, the other from ``-p`` arguments. The config holds a string, a
nested string, a bool, an integer, a double and a string array. On each gateway
the plugin must act on its config (it creates the Function named by
``function_id``), ``ros2 param`` must list no ``plugins.topics_test.*`` key but
``path``, and no canary may reach the gateway's output.

@verifies REQ_INTEROP_086
"""

import os
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node
from ros2_medkit_test_utils.param_client import ParameterClient

FILE_PORT = get_test_port(0)
ARGS_PORT = get_test_port(1)
FILE_NODE = 'gateway_cfg_file'
ARGS_NODE = 'gateway_cfg_args'

START_LINE = 'Initializing ROS 2 Medkit Gateway'

PREFIX = 'plugins.topics_test.'
SECRET = 'canary_plugin_password_zz_must_not_be_served'
NESTED_SECRET = 'canary_plugin_nested_token_zz_must_not_be_served'
FILE_FUNCTION_ID = 'cfg_from_file'
ARGS_FUNCTION_ID = 'cfg_from_args'

# One key per type the parameter parser handles, nesting included.
CONFIG_KEYS = (
    'password', 'nested.token', 'flag', 'count', 'ratio', 'labels',
    'flags', 'counts', 'ratios', 'function_id',
)


def _plugin_path():
    return os.path.join(
        get_package_prefix('ros2_medkit_integration_tests'),
        'lib', 'ros2_medkit_integration_tests', 'libtopics_test_plugin.so',
    )


def _file_params():
    return {
        'server.host': '127.0.0.1',
        'plugins': ['topics_test'],
        'plugins.topics_test.path': _plugin_path(),
        'plugins.topics_test.password': SECRET,
        'plugins.topics_test.nested.token': NESTED_SECRET,
        'plugins.topics_test.flag': True,
        'plugins.topics_test.count': 42,
        'plugins.topics_test.ratio': 2.5,
        'plugins.topics_test.labels': ['alpha', 'beta'],
        'plugins.topics_test.flags': [True, False],
        'plugins.topics_test.counts': [1, 2, 3],
        'plugins.topics_test.ratios': [0.5, 1.5],
        'plugins.topics_test.function_id': FILE_FUNCTION_ID,
    }


def _args():
    return [
        '-p', 'server.host:=127.0.0.1',
        '-p', 'plugins:=[topics_test]',
        '-p', f'plugins.topics_test.path:={_plugin_path()}',
        '-p', f'plugins.topics_test.password:={SECRET}',
        '-p', f'plugins.topics_test.nested.token:={NESTED_SECRET}',
        '-p', 'plugins.topics_test.flag:=true',
        '-p', 'plugins.topics_test.count:=42',
        '-p', 'plugins.topics_test.ratio:=2.5',
        '-p', 'plugins.topics_test.labels:=[alpha, beta]',
        '-p', 'plugins.topics_test.flags:=[true, false]',
        '-p', 'plugins.topics_test.counts:=[1, 2, 3]',
        '-p', 'plugins.topics_test.ratios:=[0.5, 1.5]',
        '-p', f'plugins.topics_test.function_id:={ARGS_FUNCTION_ID}',
    ]


@pytest.mark.launch_test
def generate_test_description():
    """Launch one gateway configured by a params file and one by `-p`."""
    from_file = create_gateway_node(
        port=FILE_PORT, name=FILE_NODE, extra_params=_file_params())
    from_args = create_gateway_node(
        port=ARGS_PORT, name=ARGS_NODE, ros_arguments=_args())
    return launch.LaunchDescription([
        from_file,
        from_args,
        launch_testing.actions.ReadyToTest(),
    ]), {'from_file': from_file, 'from_args': from_args}


class _PluginConfigNotServed:
    """Checks shared by both configuration sources."""

    NODE = ''
    FUNCTION_ID = ''
    MIN_EXPECTED_APPS = 0

    def test_01_the_plugin_received_its_config(self):
        """The plugin acts on `function_id`, so it got the config."""
        response = requests.get(
            f'{self.BASE_URL}/functions/{self.FUNCTION_ID}', timeout=30)
        self.assertEqual(response.status_code, 200, response.text)

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.params = ParameterClient(f'{cls.NODE}_probe')

    @classmethod
    def tearDownClass(cls):
        cls.params.close()
        super().tearDownClass()

    def test_02_no_plugin_key_but_path_is_a_parameter(self):
        names = set(self.params.list_names(f'/{self.NODE}'))
        # Positive checks: the node was read, and the one declared key is there.
        self.assertIn('server.port', names, sorted(names))
        self.assertIn(f'{PREFIX}path', names, sorted(names))
        served = sorted(n for n in names
                        if n.startswith(PREFIX) and n != f'{PREFIX}path')
        self.assertEqual(served, [], 'plugin config is served as parameters')

    def test_03_every_config_key_reads_as_not_set(self):
        """An undeclared key comes back with type PARAMETER_NOT_SET."""
        for key in CONFIG_KEYS:
            with self.subTest(key=key):
                self.assertTrue(self.params.is_not_set(f'/{self.NODE}', f'{PREFIX}{key}'))

    def test_04_the_declared_path_still_reads(self):
        """Positive control for test_03: a declared key is not NOT_SET."""
        self.assertFalse(self.params.is_not_set(f'/{self.NODE}', f'{PREFIX}path'))
        self.assertTrue(
            self.params.get(f'/{self.NODE}', f'{PREFIX}path').endswith('libtopics_test_plugin.so'))


class TestPluginConfigFromParamsFile(_PluginConfigNotServed, GatewayTestCase):
    """The config arrives in a params file."""

    BASE_URL = f'http://127.0.0.1:{FILE_PORT}{API_BASE_PATH}'
    NODE = FILE_NODE
    FUNCTION_ID = FILE_FUNCTION_ID


class TestPluginConfigFromCommandLine(_PluginConfigNotServed, GatewayTestCase):
    """The config arrives as `-p` arguments."""

    BASE_URL = f'http://127.0.0.1:{ARGS_PORT}{API_BASE_PATH}'
    NODE = ARGS_NODE
    FUNCTION_ID = ARGS_FUNCTION_ID


@launch_testing.post_shutdown_test()
class TestPluginConfigNotServedShutdown(unittest.TestCase):
    """Both gateways exit cleanly and neither printed a canary."""

    def test_exit_codes(self, proc_info, from_file, from_args):
        for proc in (from_file, from_args):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc)

    def test_no_canary_reached_the_log(self, proc_output, from_file, from_args):
        for proc in (from_file, from_args):
            text = ''.join(out.text.decode(errors='replace') for out in proc_output[proc])
            self.assertIn(START_LINE, text, 'the capture holds no gateway startup line')
            self.assertNotIn(SECRET, text)
            self.assertNotIn(NESTED_SECRET, text)
