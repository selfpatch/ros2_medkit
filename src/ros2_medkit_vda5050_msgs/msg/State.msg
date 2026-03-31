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
# VDA 5050 v2.0.0 - State (MQTT topic: state)
# Full AGV state published periodically and on state changes.

HeaderId header

# Required
string order_id
uint32 order_update_id
string last_node_id
uint32 last_node_sequence_id
bool driving
NodeState[] node_states
EdgeState[] edge_states
ActionState[] action_states
BatteryState battery_state
string operating_mode
ErrorEntry[] errors
SafetyState safety_state

# Optional
string zone_set_id
bool paused
bool new_base_request
float64 distance_since_last_node
AgvPosition agv_position
Velocity velocity
Load[] loads
InfoEntry[] information

# Operating mode constants
string MODE_AUTOMATIC=AUTOMATIC
string MODE_SEMIAUTOMATIC=SEMIAUTOMATIC
string MODE_MANUAL=MANUAL
string MODE_SERVICE=SERVICE
string MODE_TEACHIN=TEACHIN
