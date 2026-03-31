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
# VDA 5050 v2.0.0 - Action
# Action to be executed at a node or on an edge, or as an instant action.
#
# Both order and instantActions schemas use "actionType" in v2.0.0.
# The JSON field is camelCase "actionType"; the ROS 2 field is snake_case "action_type".
# The agent's JSON serializer handles the case conversion.

# Required
string action_id
string action_type
string blocking_type

# Optional
string action_description
ActionParameter[] action_parameters

# Blocking type constants
string BLOCKING_NONE=NONE
string BLOCKING_SOFT=SOFT
string BLOCKING_HARD=HARD
