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

"""Drive ``ghost_node_injector`` (demo_nodes/ghost_node_injector.cpp) and read a ROS graph.

``--leftover`` leaves a real node's leftover in every graph on the domain; ``--ghost`` and
``--backed`` build the same empty-enclave shape for a node no graph saw running.
"""

import os
import queue
import re
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix

PACKAGE = 'ros2_medkit_integration_tests'


def fixture_path(name):
    """Absolute path of an executable this package installs next to its demo nodes."""
    path = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'test fixture not installed: {path}')
    return path


def split_fqn(fqn):
    """(name, namespace) of a fully qualified node name, as a ROS graph reports them."""
    namespace, _, name = fqn.rpartition('/')
    return name, namespace or '/'


def observed_enclaves(node, fqn):
    """Enclave of every entry `node`'s ROS graph lists for `fqn`, in graph order."""
    name, namespace = split_fqn(fqn)
    entries = node.get_node_names_and_namespaces_with_enclaves()
    return [
        enclave
        for node_name, node_namespace, enclave in entries
        if node_name == name and node_namespace == namespace
    ]


def wait_observed(node, fqn, predicate, timeout):
    """Poll `node`'s graph until `predicate(enclaves of fqn)` holds; return whether it did."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate(observed_enclaves(node, fqn)):
            return True
        time.sleep(0.05)
    return False


def stop_process(proc, timeout):
    """SIGTERM `proc` and wait for it, killing it if it outlives `timeout`."""
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=10)


class _FixtureProcess:
    """One ghost_node_injector process, its output collected line by line."""

    def __init__(self, args, env=None):
        self.proc = subprocess.Popen(
            [fixture_path('ghost_node_injector')] + args, env=env,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self._lines = queue.Queue()
        self._output = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self._output.append(line)
            self._lines.put(line)

    def wait_line(self, pattern, timeout):
        """Return the first match of `pattern` on a line printed from now on, or None."""
        regex = re.compile(pattern)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self._lines.get(timeout=0.1)
            except queue.Empty:
                if self.proc.poll() is not None and self._lines.empty():
                    return None
                continue
            match = regex.search(line)
            if match:
                return match
        return None

    def send(self, command):
        self.proc.stdin.write(command + '\n')
        self.proc.stdin.flush()

    def output(self):
        return ''.join(self._output)

    def stop(self, timeout=30.0):
        stop_process(self.proc, timeout)


class GhostInjection(_FixtureProcess):
    """``--ghost`` and ``--backed`` nodes for GIDs no participant owns.

    The backed nodes go out before the ghosts, so a ROS graph that lists a ghost of this
    injection also lists every backed node of it.
    """

    def __init__(self, ghosts=(), backed=(), env=None):
        args = []
        for fqn in backed:
            args += ['--backed', fqn]
        for fqn in ghosts:
            args += ['--ghost', fqn]
        super().__init__(args, env)
        self.entries = len(ghosts) + len(backed)

    def wait_status(self, timeout):
        """(matched_subscriptions, entries, acked) once the messages went out, or None."""
        match = self.wait_line(
            r'ghost_node_injector: matched_subscriptions=(\d+) entries=(\d+) acked=(true|false)',
            timeout)
        if match is None:
            return None
        return int(match.group(1)), int(match.group(2)), match.group(3) == 'true'


class LeftoverNode(_FixtureProcess):
    """A real node that, on `leave()`, leaves a leftover of itself in every graph.

    `announce_nodes()` adds `announce` nodes ``<name>_<NNNNN>`` to its participant; they are
    left over with it.
    """

    def __init__(self, fqn, delay_sec, announce=0, env=None):
        args = ['--leftover', fqn, '--delay', f'{delay_sec:.3f}']
        if announce:
            args += ['--announce', str(announce)]
        super().__init__(args, env)
        self.fqn = fqn
        self.announce = announce

    def announced_fqns(self):
        return [f'{self.fqn}_{index:05d}' for index in range(self.announce)]

    def wait_ready(self, timeout):
        """Whether the node runs and its participant's discovery message was captured."""
        match = self.wait_line(r'ghost_node_injector: leftover_ready participant_gid=', timeout)
        return match is not None

    def announce_nodes(self, timeout):
        """Publish the announced nodes; whether every matched reader acknowledged them."""
        self.send('announce')
        match = self.wait_line(
            r'ghost_node_injector: leftover_announced count=\d+ acked=(true|false)', timeout)
        return match is not None and match.group(1) == 'true'

    def leave(self):
        """Remove the node and its participant; the late sample follows after the delay."""
        self.send('leave')

    def publish(self):
        """Send the late sample now, after `leave()`, rather than when the delay runs out."""
        self.send('publish')
