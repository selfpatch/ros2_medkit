# Copyright 2026 selfpatch GmbH
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
# MedkitDiscoveryHint - runtime hints from apps to the ros2_medkit gateway.
#
# Published on /ros2_medkit/discovery (topic beacon) or set as
# ros2_medkit.discovery.* node parameters (parameter beacon).
#
# All fields except entity_id are optional. Empty strings and
# empty arrays mean "not provided" - the plugin ignores them.

# === Required ===
string entity_id                          # Target entity (App or Component ID)

# === Identity hints ===
string stable_id                          # Stable ID alias (stored as x-medkit-beacon-stable-id metadata)
string display_name                       # Human-friendly name

# === Topology hints ===
string[] function_ids                     # Function membership (entity belongs to these Functions)
string[] depends_on                       # Entity dependencies (IDs of entities this one depends on)
string component_id                       # Parent Component binding

# === Transport hints ===
string transport_type                     # "nitros_zero_copy", "shared_memory", "intra_process", etc.
string negotiated_format                  # "nitros_image_bgr8", etc.

# === Process diagnostics ===
uint32 process_id                         # OS process ID (PID), 0 = not provided
string process_name                       # Process name (e.g. "component_container")
string hostname                           # Host identifier (for distributed systems)

# === Freeform ===
diagnostic_msgs/KeyValue[] metadata       # Arbitrary key-value pairs

# === Timing ===
builtin_interfaces/Time stamp             # Timestamp for TTL calculation
