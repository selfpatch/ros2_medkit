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
# VDA 5050 v2.0.0 - PhysicalParameters (Factsheet sub-type)
# Physical dimensions and speed limits of the AGV.

# Required (all in SI units: m, m/s, m/s^2)
float64 speed_min
float64 speed_max
float64 acceleration_max
float64 deceleration_max
float64 height_max
float64 width
float64 length

# Required per v2.0.0 schema (despite being logically optional)
float64 height_min
