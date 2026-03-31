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
# VDA 5050 v2.0.0 - WheelDefinition (Factsheet sub-type)
# Definition of a single wheel on the AGV.

# Required
string type                   # DRIVE, CASTER, FIXED, MECANUM
bool is_active_driven
bool is_active_steered
WheelPosition position
float64 diameter              # meters
float64 width                 # meters

# Optional
float64 center_displacement   # meters
string constraints
