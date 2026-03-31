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
# VDA 5050 v2.0.0 - TypeSpecification (Factsheet sub-type)
# Basic type information about the AGV.

# Required
string series_name
string agv_kinematic          # DIFF, OMNI, THREEWHEEL
string agv_class              # FORKLIFT, CONVEYOR, TUGGER, CARRIER
float64 max_load_mass         # kg
string[] localization_types   # NATURAL, REFLECTOR, RFID, DMC, SPOT, GRID
string[] navigation_types     # PHYSICAL_LINE_GUIDED, VIRTUAL_LINE_GUIDED, AUTONOMOUS

# Optional
string series_description
