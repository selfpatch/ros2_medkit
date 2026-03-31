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
# VDA 5050 v2.0.0 - Timing (Factsheet sub-type)
# Timing constraints for communication intervals.

# Required
float64 min_order_interval      # Minimum interval for order messages (seconds)
float64 min_state_interval      # Minimum interval for state messages (seconds)

# Optional
float64 default_state_interval
float64 visualization_interval
