// Copyright 2026 selfpatch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_medkit_beacon_common/beacon_validator.hpp"

#include <algorithm>
#include <cctype>

namespace ros2_medkit_beacon {

namespace {

void truncate_if_needed(std::string & str, size_t max_length) {
  if (str.size() > max_length) {
    str.resize(max_length);
  }
}

bool is_valid_id(const std::string & id, size_t max_length) {
  if (id.empty() || id.size() > max_length) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
  });
}

bool is_valid_metadata_key(const std::string & key, size_t max_length) {
  if (key.empty() || key.size() > max_length) {
    return false;
  }
  return std::all_of(key.begin(), key.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
  });
}

template <typename Container>
void filter_invalid_ids(Container & ids, size_t max_entries, size_t max_length) {
  auto it = std::remove_if(ids.begin(), ids.end(), [max_length](const std::string & id) {
    return !is_valid_id(id, max_length);
  });
  ids.erase(it, ids.end());
  if (ids.size() > max_entries) {
    ids.resize(max_entries);
  }
}

}  // namespace

ValidationResult validate_beacon_hint(BeaconHint & hint, const ValidationLimits & limits) {
  if (!is_valid_id(hint.entity_id, limits.max_id_length)) {
    return {false, "invalid entity_id: '" + hint.entity_id + "'"};
  }

  // Truncate freeform string fields that lack character-set validation.
  truncate_if_needed(hint.display_name, limits.max_string_length);
  truncate_if_needed(hint.process_name, limits.max_string_length);
  truncate_if_needed(hint.hostname, limits.max_string_length);
  truncate_if_needed(hint.transport_type, limits.max_string_length);
  truncate_if_needed(hint.negotiated_format, limits.max_string_length);

  if (!hint.stable_id.empty() && !is_valid_id(hint.stable_id, limits.max_id_length)) {
    hint.stable_id.clear();
  }
  if (!hint.component_id.empty() && !is_valid_id(hint.component_id, limits.max_id_length)) {
    hint.component_id.clear();
  }

  filter_invalid_ids(hint.function_ids, limits.max_function_ids, limits.max_id_length);
  filter_invalid_ids(hint.depends_on, limits.max_depends_on, limits.max_id_length);

  auto & md = hint.metadata;
  if (md.size() > limits.max_metadata_entries) {
    auto it = md.begin();
    std::advance(it, static_cast<std::ptrdiff_t>(limits.max_metadata_entries));
    md.erase(it, md.end());
  }

  for (auto it = md.begin(); it != md.end();) {
    if (!is_valid_metadata_key(it->first, limits.max_metadata_key_length)) {
      it = md.erase(it);
    } else {
      if (it->second.size() > limits.max_metadata_value_length) {
        it->second.resize(limits.max_metadata_value_length);
      }
      ++it;
    }
  }

  return {true, ""};
}

}  // namespace ros2_medkit_beacon
