#!/bin/bash
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

# Merges per-package compile_commands.json files into a single database.
# Run after colcon build to enable project-wide clang-tidy analysis.
#
# Usage: ./scripts/merge-compile-commands.sh
# Output: build/compile_commands.json

set -euo pipefail

MERGED="build/compile_commands.json"

# Find all per-package compile_commands.json files
DATABASES=(build/*/compile_commands.json)

if [ ${#DATABASES[@]} -eq 0 ] || [ ! -f "${DATABASES[0]}" ]; then
  echo "Error: No compile_commands.json found in build/*/."
  echo "Run 'colcon build' first."
  exit 1
fi

# Merge using jq if available, otherwise use Python
if command -v jq &>/dev/null; then
  jq -s 'add' "${DATABASES[@]}" > "$MERGED"
else
  python3 -c "
import json, sys
entries = []
for f in sys.argv[2:]:
    with open(f) as fh:
        entries.extend(json.load(fh))
with open(sys.argv[1], 'w') as fh:
    json.dump(entries, fh, indent=2)
" "$MERGED" "${DATABASES[@]}"
fi

if command -v jq &>/dev/null; then
  COUNT=$(jq length "$MERGED")
else
  COUNT=$(python3 -c "import json; print(len(json.load(open('$MERGED'))))" 2>/dev/null || echo '?')
fi
echo "Merged ${#DATABASES[@]} databases into $MERGED ($COUNT entries)"
