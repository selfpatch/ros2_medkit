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
# VDA 5050 v2.0.0 - ErrorEntry
# An error reported in the AGV state message.

# Required
string error_type
string error_level

# Optional
string error_description
ErrorReference[] error_references

# Error level constants (v2.0.0)
string LEVEL_WARNING=WARNING
string LEVEL_FATAL=FATAL
