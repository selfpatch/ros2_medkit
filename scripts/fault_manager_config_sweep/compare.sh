#!/usr/bin/env bash
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

# Compare two sweep.sh output directories, case by case. A case is IDENTICAL when its capture is
# byte-equal, REORDERED when it has the same lines in another order, and SAME_SET when a repeated
# case (<case>.rN.txt) produced the same set of distinct outcomes on both sides, where an outcome is
# the sorted lines of one run. Anything else is DIFFERENT and prints the diff of the sorted lines.
# Usage: compare.sh <base_dir> <branch_dir>
set -u
base="$1"
branch="$2"
rc=0

cases() {
  find "$1" -name '*.txt' -printf '%f\n' | sed -E 's/\.r[0-9]+\.txt$//; s/\.txt$//' | sort -u
}

# outcomes <dir> <case>: the distinct sorted-line contents, one hash per line.
outcomes() {
  local f
  for f in "$1/$2.txt" "$1/$2".r*.txt; do
    [ -f "$f" ] && sort "$f" | md5sum
  done | sort -u
}

all="$( { cases "$base"; cases "$branch"; } | sort -u)"
for name in $all; do
  if [ -f "$base/$name.txt" ] && [ -f "$branch/$name.txt" ]; then
    if cmp -s "$base/$name.txt" "$branch/$name.txt"; then
      echo "IDENTICAL  $name"
      continue
    fi
  fi
  ob="$(outcomes "$base" "$name")"
  oh="$(outcomes "$branch" "$name")"
  if [ -z "$ob" ] || [ -z "$oh" ]; then
    echo "MISSING  $name"
    rc=1
  elif [ "$ob" = "$oh" ]; then
    if [ -f "$base/$name.txt" ]; then echo "REORDERED  $name"; else echo "SAME_SET  $name"; fi
  else
    echo "DIFFERENT  $name"
    rc=1
    diff <(cat "$base/$name.txt" "$base/$name".r*.txt 2>/dev/null | sort -u) \
      <(cat "$branch/$name.txt" "$branch/$name".r*.txt 2>/dev/null | sort -u)
  fi
done
exit "$rc"
