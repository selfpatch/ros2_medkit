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

"""Read node identity from the ROS graph, as an operator's tools see it."""

import collections
import os
import subprocess
import time

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy


def join_fqn(namespace, name):
    """Return the fully qualified node name for *namespace* and *name*."""
    return namespace.rstrip('/') + '/' + name


def leaf_name(fqn):
    """Return the node name part of a fully qualified name."""
    return fqn.rsplit('/', 1)[-1]


def node_fqns(node):
    """Return the FQN of every node in the graph, hidden nodes included."""
    return [join_fqn(ns, name) for name, ns in node.get_node_names_and_namespaces()]


def duplicate_fqns(fqns):
    """Return the FQNs that appear more than once, sorted."""
    return sorted(fqn for fqn, count in collections.Counter(fqns).items() if count > 1)


def wait_for_graph(node, predicate, *, timeout=30.0, interval=0.2):
    """Poll the graph until ``predicate(fqns)`` is true; return the last FQN list."""
    deadline = time.monotonic() + timeout
    fqns = node_fqns(node)
    while time.monotonic() < deadline:
        fqns = node_fqns(node)
        if predicate(fqns):
            return fqns
        time.sleep(interval)
    return fqns


def _split_fqn(fqn):
    namespace, _, name = fqn.rpartition('/')
    return name, namespace or '/'


def subscribed_topics(node, fqn):
    """Return the topics the node at *fqn* subscribes to."""
    return {topic for topic, _ in node.get_subscriber_names_and_types_by_node(*_split_fqn(fqn))}


def advertised_services(node, fqn):
    """Return the services the node at *fqn* advertises."""
    return {service for service, _ in node.get_service_names_and_types_by_node(*_split_fqn(fqn))}


def get_bool_parameter(node, fqn, name, *, timeout=10.0):
    """Read a bool parameter from the node at *fqn*; None when unreadable."""
    client = node.create_client(GetParameters, fqn + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        result = future.result()
        if result is None or len(result.values) != 1:
            return None
        value = result.values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return None
        return value.bool_value
    finally:
        node.destroy_client(client)


def ros2_param_list(fqn, *, timeout=60.0):
    """Run ``ros2 param list <fqn>`` and return (parameter names, raw stdout).

    Uses ``--no-daemon`` so the answer comes from this process's own discovery
    on the test domain, not from a daemon started earlier on another one.
    """
    result = subprocess.run(
        ['ros2', 'param', 'list', fqn, '--no-daemon', '--spin-time', '5'],
        capture_output=True, text=True, timeout=timeout, env=os.environ.copy(),
        check=False,
    )
    names = [
        line.strip() for line in result.stdout.splitlines()
        if line.strip() and not line.rstrip().endswith(':')
    ]
    return names, result.stdout + result.stderr
