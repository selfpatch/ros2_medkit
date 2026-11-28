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

# Kill the LeRobot policy process. The gateway's graph watchdog sees the node leave
# the graph and raises GRAPH_NODE_DISAPPEARED; the launch file starts the policy
# again after 5 s. An attempt running at that moment cannot finish (the lockstep
# simulation waits for a command that never comes): the episode runner aborts it
# after stall_timeout_sec (20 s) and does not report it as a policy failure.
set -eu
CONTAINER="${EXAMPLE_CONTAINER:-lerobot_so101_example}"
docker exec "$CONTAINER" pkill -KILL -f lerobot_policy_node
echo "Policy killed. It restarts in about 5 s and loads the checkpoint again."
