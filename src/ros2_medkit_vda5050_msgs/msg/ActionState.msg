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
# VDA 5050 v2.0.0 - ActionState
# Current state of an action reported in the AGV state message.

# Required
string action_id
string action_status

# Optional
string action_type
string action_description
string result_description

# Action status constants
string STATUS_WAITING=WAITING
string STATUS_INITIALIZING=INITIALIZING
string STATUS_RUNNING=RUNNING
# Note: PAUSED is not in v2.0.0 - forward-compatible extension from v2.1+
string STATUS_PAUSED=PAUSED
string STATUS_FINISHED=FINISHED
string STATUS_FAILED=FAILED
