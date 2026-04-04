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
# VDA 5050 v2.0.0 - LoadSet (Factsheet sub-type)
# Description of a type of load the AGV can handle.

# Required
string set_name
string load_type
BoundingBoxReference bounding_box_reference
LoadDimensions load_dimensions

# Optional
string[] load_positions
float64 max_weight
bool rotation_allowed
string description
float64 min_loadhandling_height
float64 max_loadhandling_height
float64 min_loadhandling_depth
float64 max_loadhandling_depth
float64 min_loadhandling_tilt
float64 max_loadhandling_tilt
float64 agv_speed_limit
float64 agv_acceleration_limit
float64 agv_deceleration_limit
float64 pick_time
float64 drop_time
