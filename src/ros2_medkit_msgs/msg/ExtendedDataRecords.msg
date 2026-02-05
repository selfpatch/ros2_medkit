# Copyright 2025 mfaferek93
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
# ExtendedDataRecords.msg - Extended data records for fault diagnostics.
#
# Contains timestamps of first and last occurrence for SOVD-compliant
# fault environment data responses.

# Timestamp of first occurrence (nanoseconds since epoch)
int64 first_occurrence_ns

# Timestamp of last occurrence (nanoseconds since epoch)
int64 last_occurrence_ns
