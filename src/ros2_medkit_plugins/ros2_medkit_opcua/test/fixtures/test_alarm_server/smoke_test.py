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
Smoke test for test_alarm_server (issue #386).

Verifies, against a running test_alarm_server instance:
  * 3 condition nodes exist (Overpressure, Overheat, SensorLost).
  * Each is a subtype of AlarmConditionType (NodeId i=2915 in ns 0).
  * Each exposes Acknowledge and Confirm methods.
  * Each exposes the canonical AlarmConditionType select fields.

Usage:  python3 smoke_test.py opc.tcp://localhost:4842
"""

import asyncio
import sys

from asyncua import Client, ua

NS_URI = 'urn:test:alarms'
ALARM_TYPE_NS0 = ua.NodeId(2915, 0)  # AlarmConditionType
EXPECTED = ('Overpressure', 'Overheat', 'SensorLost')
REQUIRED_FIELDS = (
    'EventId',
    'EventType',
    'SourceNode',
    'Time',
    'Severity',
    'Message',
    'EnabledState',
    'ActiveState',
    'AckedState',
    'ConfirmedState',
    'Retain',
)
REQUIRED_METHODS = ('Acknowledge', 'Confirm')


async def find_condition(client, source_browse_name):
    """Locate the condition object as a HasComponent child of its source node."""
    objects = client.nodes.objects
    children = await objects.get_children()
    for ch in children:
        bn = await ch.read_browse_name()
        if bn.Name == source_browse_name:
            return ch
    return None


async def assert_condition(client, ns_idx, alarm_name):
    """Assert that a single AlarmConditionType node is present and well-formed."""
    source_name = alarm_name + 'Source'
    source = await find_condition(client, source_name)
    if source is None:
        raise AssertionError(f'Condition source {source_name} not found')
    children = await source.get_children()
    target = None
    for ch in children:
        bn = await ch.read_browse_name()
        if bn.NamespaceIndex == ns_idx and bn.Name == alarm_name:
            target = ch
            break
    if target is None:
        raise AssertionError(f'Condition node {alarm_name} not found under source')

    type_def = await target.read_type_definition()
    if type_def != ALARM_TYPE_NS0:
        raise AssertionError(
            f'{alarm_name}: type is {type_def}, expected {ALARM_TYPE_NS0}'
        )

    child_names = set()
    method_names = set()
    for child in await target.get_children():
        bn = await child.read_browse_name()
        nc = await child.read_node_class()
        if nc == ua.NodeClass.Method:
            method_names.add(bn.Name)
        else:
            child_names.add(bn.Name)

    missing_fields = [f for f in REQUIRED_FIELDS if f not in child_names]
    if missing_fields:
        raise AssertionError(f'{alarm_name}: missing fields {missing_fields}')

    missing_methods = [m for m in REQUIRED_METHODS if m not in method_names]
    if missing_methods:
        raise AssertionError(f'{alarm_name}: missing methods {missing_methods}')

    print(
        f'  OK {alarm_name}: type=AlarmConditionType, '
        f'fields={len(child_names)}, methods={sorted(method_names)}'
    )


async def run(url):
    """Run all condition assertions against a single server instance."""
    print(f'Connecting to {url}')
    async with Client(url=url, timeout=10) as client:
        ns_idx = await client.get_namespace_index(NS_URI)
        print(f'Found namespace {NS_URI} at index {ns_idx}')
        for name in EXPECTED:
            await assert_condition(client, ns_idx, name)
    print('PASS')


def main():
    """CLI entry point: smoke_test.py <opc.tcp://host:port>."""
    if len(sys.argv) < 2:
        print('usage: smoke_test.py opc.tcp://host:port', file=sys.stderr)
        sys.exit(2)
    try:
        asyncio.run(run(sys.argv[1]))
    except Exception as exc:  # noqa: BLE001 - diagnostics only
        print(f'FAIL: {exc}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
