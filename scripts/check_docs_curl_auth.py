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

"""Fail when a documented ``curl`` against the gateway sends no credential.

The gateway ships with ``auth.enabled: true`` and ``auth.require_auth_for:
"all"``, so a copy-pasted example without an ``Authorization`` header answers
401. Prose can claim every example carries a token; only a scan proves it.

What counts as a gateway request: the command names a URL whose path begins
with the API prefix ``/api/v1``, whatever the host. That is what makes the
request the gateway's rather than the web UI's or the docs server's, and it
holds for a ``${BASE}``-style placeholder as well as a literal
``http://localhost:8080``.

Two exemptions, both narrow:

* ``/api/v1/auth/`` routes. Authentication cannot bootstrap through a door that
  demands the credential it hands out, so these are exempt in the gateway too
  (``AllAuthRequirementPolicy::requires_authentication``).
* An example marked deliberately anonymous. Put the marker

      # docs-curl-auth: public - <reason>

  on its own line inside the same literal block, above the command it covers.
  It applies to every ``curl`` that follows it in that block. The reason is for
  the reader, who sees the marker in the rendered page; the scan only needs the
  prefix. Use it where the anonymous request IS the point - showing the 401, or
  showing what a route opened through ``auth.public_routes`` returns.

Continuation lines are folded before matching, so a header on the second line
of a multi-line invocation counts.

Usage: ``python3 scripts/check_docs_curl_auth.py [--root <dir>]``. Exits 0 when
every gateway example carries a credential, 1 listing ``file:line`` for each
that does not.
"""

import argparse
from pathlib import Path
import re
import sys

API_PREFIX = '/api/v1'

MARKER = '# docs-curl-auth: public'

# The path of a URL whose host is a literal, a placeholder, or a shell
# variable. Only the path is captured, so a page that moves off localhost stays
# covered. The host class excludes `/` on purpose: without that it matches the
# whole URL and backtracks until the capture group holds only the last segment,
# which exempts every example in the repository and makes this script report
# clean on a tree full of offenders.
URL_PATH = re.compile(
    r'(?:https?://[^\s\'"`/]+|\$\{?[A-Za-z_][A-Za-z0-9_]*\}?)(/[^\s\'"`\\]*)')

AUTH_HEADER = re.compile(r'[Aa]uthorization\s*:')


def documented_files(root):
    """Return the docs and README files a reader copies commands out of."""
    found = sorted(root.glob('docs/**/*.rst'))
    found += sorted(root.glob('README.md'))
    found += sorted(root.glob('src/**/README.md'))
    found += sorted(root.glob('postman/README.md'))
    return [f for f in found if '_build' not in f.parts]


def fold(lines):
    """Yield (line_number, folded_command, indent) per logical shell line.

    A trailing backslash continues the command, so the header on line two of a
    three-line invocation belongs to the command that started on line one. The
    line number reported is the one the command starts on, which is where a
    reader looks.
    """
    buffered = []
    start = None
    indent = 0
    for number, raw in enumerate(lines, start=1):
        text = raw.rstrip('\n')
        if start is None:
            start = number
            indent = len(text) - len(text.lstrip())
        buffered.append(text.strip())
        if text.rstrip().endswith('\\'):
            continue
        yield start, ' '.join(part.rstrip('\\').strip() for part in buffered), indent
        buffered = []
        start = None


def gateway_paths(command):
    """Return the API paths this command requests, in order."""
    return [path for path in URL_PATH.findall(command)
            if path == API_PREFIX or path.startswith(API_PREFIX + '/')]


def offenders_in(path, text):
    """Return (line, command) for every unauthenticated gateway request."""
    lines = text.splitlines()
    marked_until = -1
    found = []
    for number, command, indent in fold(lines):
        if not command:
            continue
        if command.startswith(MARKER):
            # Applies to the rest of this block: anything indented at least as
            # far as the marker, up to the first line that is not.
            marked_until = block_end(lines, number, indent)
            continue
        if 'curl' not in command:
            continue
        paths = gateway_paths(command)
        if not paths:
            continue
        if all(p.startswith(API_PREFIX + '/auth/') for p in paths):
            continue
        if AUTH_HEADER.search(command):
            continue
        if number <= marked_until:
            continue
        found.append((number, command))
    return found


def block_end(lines, marker_line, indent):
    """Return the last line number the marker at ``marker_line`` covers."""
    last = marker_line
    for number in range(marker_line + 1, len(lines) + 1):
        text = lines[number - 1]
        if not text.strip():
            last = number
            continue
        if len(text) - len(text.lstrip()) < indent:
            break
        last = number
    return last


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--root', default=str(Path(__file__).parent.parent.resolve()),
        help='repository root to scan (default: the parent of scripts/)')
    args = parser.parse_args()
    root = Path(args.root).resolve()

    total = 0
    for path in documented_files(root):
        for number, command in offenders_in(path, path.read_text(encoding='utf-8')):
            total += 1
            print(f'{path.relative_to(root)}:{number}: no Authorization header: '
                  f'{command[:120]}')

    if total:
        print(f'\n{total} documented gateway request(s) send no credential.')
        print('The gateway ships with require_auth_for "all", so each answers 401.')
        print('Add -H "Authorization: Bearer $TOKEN", or mark the example with')
        print(f'    {MARKER} - <reason>')
        return 1

    scanned = len(documented_files(root))
    print(f'{scanned} documented file(s) scanned: every gateway curl carries a credential.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
