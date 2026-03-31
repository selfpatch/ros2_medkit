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
# EntityInfo.msg - SOVD entity descriptor for service interface responses.
#
# Provides basic metadata about a SOVD entity (app, component, area, function)
# for consumers that need entity tree information via ROS 2 services.

# Unique entity identifier (e.g., "temp_sensor", "powertrain")
string id

# Human-readable entity name
string name

# Entity type: "app", "component", "area", "function"
string entity_type

# Parent entity ID ("" for top-level entities)
string parent_id

# Fully-qualified ROS 2 name (e.g., "/powertrain/engine/temp_sensor")
string fqn

# SOVD capabilities available for this entity
# (e.g., ["data", "faults", "operations", "configurations"])
string[] capabilities
