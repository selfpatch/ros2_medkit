# Copyright 2026 mfaferek93
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
#
# VDA 5050 v2.0.0 - Edge
# An edge connecting two nodes in the order graph.

# Required
string edge_id
uint32 sequence_id
bool released
string start_node_id
string end_node_id
Action[] actions

# Optional
string edge_description
float64 max_speed
string orientation_type
float64 max_height
float64 min_height
float64 orientation
string direction
bool rotation_allowed
float64 max_rotation_speed
float64 length
Trajectory trajectory
