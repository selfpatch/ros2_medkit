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

"""Merge ROS parameter files into one file for the gateway node.

Usage: merge_params.py <out> <file>...

Files are given from the lowest layer to the highest. Every node name that appears in
any file gets a section. Per file, `/**` applies first and the node's own section after
it. Across files a later one wins parameter by parameter; a list or scalar is replaced
as a whole.
"""

import sys

import yaml

NODE = '/ros2_medkit_gateway'
WILDCARD = '/**'


def flatten(tree, prefix=''):
    """Return {dotted.name: leaf}; a dict is a namespace, anything else is a value."""
    flat = {}
    for key, value in tree.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, f'{name}.'))
        else:
            flat[name] = value
    return flat


def sections(document):
    """Return {node name with a leading slash: flattened parameters} for one file."""
    found = {}
    for key, section in (document or {}).items():
        if isinstance(section, dict) and isinstance(section.get('ros__parameters'), dict):
            name = key if key == WILDCARD else '/' + key.lstrip('/')
            found.setdefault(name, {}).update(flatten(section['ros__parameters']))
    return found


def main(argv):
    out, files = argv[1], argv[2:]
    wildcard = {}
    nodes = {NODE: {}}
    for path in files:
        with open(path, encoding='utf-8') as handle:
            found = sections(yaml.safe_load(handle))
        from_wildcard = found.pop(WILDCARD, {})
        wildcard.update(from_wildcard)
        for name in found:
            nodes.setdefault(name, {})
        for name, merged in nodes.items():
            merged.update(from_wildcard)
            merged.update(found.get(name, {}))
    result = {name: {'ros__parameters': merged} for name, merged in nodes.items()}
    if wildcard:
        result = {WILDCARD: {'ros__parameters': wildcard}, **result}
    with open(out, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(result, handle, default_flow_style=False)


if __name__ == '__main__':
    main(sys.argv)
