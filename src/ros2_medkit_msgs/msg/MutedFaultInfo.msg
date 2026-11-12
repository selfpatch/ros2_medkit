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
# MutedFaultInfo.msg - Information about a muted (correlated) fault record.
#
# When fault correlation is enabled, symptom records are muted (not published
# as events) but still stored. This message provides information about
# muted records when queried with include_muted=true.
#
# One entry per muted RECORD. A root cause mutes the symptoms of its own
# reporting source only, so two sources muted on one fault_code produce two
# entries carrying that code, told apart by source_id.

# The fault code of the muted symptom
string fault_code

# The fault code that caused this fault to be muted (root cause)
string root_cause_code

# The correlation rule ID that matched
string rule_id

# Time delay from root cause to this symptom [ms]
uint32 delay_ms

# Reporting source that owns the muted record (the source_id used in ReportFault).
# Together with fault_code it identifies the one record this entry describes, and
# it is the value the single-record services take to address it.
string source_id
