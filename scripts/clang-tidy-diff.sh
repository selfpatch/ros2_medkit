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

# Incremental clang-tidy: only analyses C++ source files passed as arguments.
# Designed for pre-commit which passes staged filenames.
# Requires: merged compile_commands.json (run ./scripts/merge-compile-commands.sh after build).
#
# Usage (standalone):
#   ./scripts/clang-tidy-diff.sh src/ros2_medkit_gateway/src/config.cpp
#
# Usage (via pre-commit): automatic - pre-commit passes changed files.

set -euo pipefail

COMPILE_DB="build/compile_commands.json"

if [ ! -f "$COMPILE_DB" ]; then
  echo "Warning: No merged compile_commands.json found."
  echo "Run: colcon build && ./scripts/merge-compile-commands.sh"
  exit 0
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLANG_TIDY_CONFIG=".clang-tidy"
ERRORS=0

for file in "$@"; do
  # Only process C++ source files (.cpp). Headers are checked transitively
  # through their including .cpp files. CI runs full clang-tidy on all files.
  case "$file" in
    *.cpp) ;;
    *) continue ;;
  esac

  if [[ "$file" == *"/vendored/"* ]]; then
    continue
  fi

  # Resolve to absolute path for matching against compile_commands.json
  ABS_FILE="$REPO_ROOT/$file"
  if [ ! -f "$ABS_FILE" ]; then
    ABS_FILE="$(realpath "$file" 2>/dev/null || echo "$file")"
  fi

  echo "clang-tidy: $file"
  if ! clang-tidy -p "$(dirname "$COMPILE_DB")" --config-file="$CLANG_TIDY_CONFIG" "$ABS_FILE"; then
    ERRORS=$((ERRORS + 1))
  fi
done

if [ "$ERRORS" -gt 0 ]; then
  echo "clang-tidy found issues in $ERRORS file(s)"
  exit 1
fi
