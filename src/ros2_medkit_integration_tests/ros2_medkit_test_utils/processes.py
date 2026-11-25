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

"""Find the processes of ROS 2 nodes in /proc."""

import os


def _parent_pid(pid):
    with open(f'/proc/{pid}/stat', 'rb') as f:
        stat = f.read()
    # The fields after the command name in parentheses: state, ppid, ...
    return int(stat[stat.rindex(b')') + 2:].split()[1])


def node_pids(node_name, namespace, *, parent=None):
    """Return the PIDs of live processes started as this node, sorted.

    A node started by launch carries ``__node:=<name>`` and ``__ns:=<namespace>``
    on its command line. With *parent*, only children of that PID count.
    """
    wanted = {f'__node:={node_name}'.encode(), f'__ns:={namespace}'.encode()}
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                args = set(f.read().split(b'\0'))
            if not wanted <= args:
                continue
            if parent is not None and _parent_pid(entry) != parent:
                continue
        except (OSError, ValueError):
            continue
        found.append(int(entry))
    return sorted(found)
