#!/bin/bash
# Copyright 2026 ahmedsleem109
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

# Show the faults ros2_medkit holds (healed ones included) and the recordings
# saved for failed attempts.
set -eu
GATEWAY="${GATEWAY_URL:-http://localhost:${GATEWAY_PORT:-8080}}/api/v1"
echo "Faults (status=all):"
curl -sf "$GATEWAY/faults?status=all" |
  jq -r '.items[] | "  \(.fault_code)  \(.status)  severity=\(.severity)"'
echo ""
echo "Recordings of failed attempts (episode_runner):"
curl -sf "$GATEWAY/apps/episode_runner/bulk-data/rosbags" |
  jq -r '.items[] | "  \(.id)  \(.size) bytes"' || echo "  (none yet)"
