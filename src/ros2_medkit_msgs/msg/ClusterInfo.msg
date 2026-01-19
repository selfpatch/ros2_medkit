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
# ClusterInfo.msg - Information about an auto-detected fault cluster.
#
# When multiple related faults occur within a time window, they can be
# automatically grouped into a cluster. This message provides cluster
# information when queried with include_clusters=true.

# Unique cluster identifier (e.g., "comm_storm_20260119_103000")
string cluster_id

# The correlation rule ID that created this cluster
string rule_id

# Human-readable rule name
string rule_name

# Cluster label (e.g., "Communication Storm")
string label

# The representative fault code (shown to user as primary)
string representative_code

# Severity of the representative fault
string representative_severity

# All fault codes in this cluster
string[] fault_codes

# Number of faults in the cluster
uint32 count

# Timestamp of the first fault in the cluster
builtin_interfaces/Time first_at

# Timestamp of the last fault in the cluster
builtin_interfaces/Time last_at
