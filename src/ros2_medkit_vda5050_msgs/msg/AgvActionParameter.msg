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
# VDA 5050 v2.0.0 - AgvActionParameter (Factsheet sub-type)
# Parameter description for a factsheet agvAction.
# Different from ActionParameter (used in orders) which has key+value.

# Required
string key
string value_data_type        # BOOL, NUMBER, INTEGER, FLOAT, STRING, OBJECT, ARRAY

# Optional
string description
bool is_optional
