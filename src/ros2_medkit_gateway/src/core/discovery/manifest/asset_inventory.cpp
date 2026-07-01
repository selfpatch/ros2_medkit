// Copyright 2026 bburda
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

#include "ros2_medkit_gateway/core/discovery/manifest/asset_inventory.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

namespace {

std::string trim(const std::string & s) {
  const auto not_space = [](char c) {
    return std::isspace(static_cast<unsigned char>(c)) == 0;
  };
  auto begin = std::find_if(s.begin(), s.end(), not_space);
  auto end = std::find_if(s.rbegin(), s.rend(), not_space).base();
  if (begin >= end) {
    return std::string{};
  }
  return std::string(begin, end);
}

std::string to_lower(const std::string & s) {
  std::string out;
  out.reserve(s.size());
  for (const char c : s) {
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}

// A single parsed CSV field plus whether it was double-quoted (quoted fields
// are not whitespace-trimmed so leading/trailing spaces inside quotes survive).
struct CsvField {
  std::string value;
  bool quoted{false};
};

// Tokenize a CSV document into rows of fields, honoring RFC-4180 quoting
// (embedded commas / newlines / escaped `""`). Empty rows are dropped.
std::vector<std::vector<CsvField>> tokenize(const std::string & text) {
  std::vector<std::vector<CsvField>> rows;
  std::vector<CsvField> row;
  CsvField field;
  bool in_quotes = false;
  bool field_has_content = false;

  const auto end_field = [&]() {
    if (!field.quoted) {
      field.value = trim(field.value);
    }
    row.push_back(std::move(field));
    field = CsvField{};
    field_has_content = false;
  };

  const auto end_row = [&]() {
    end_field();
    // Drop rows that are entirely empty (e.g., a trailing newline or a blank
    // separator line): a single empty, unquoted field with no siblings.
    const bool blank = row.size() == 1u && !row[0].quoted && row[0].value.empty();
    if (!blank) {
      rows.push_back(std::move(row));
    }
    row.clear();
  };

  for (std::size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (in_quotes) {
      if (c == '"') {
        if (i + 1 < text.size() && text[i + 1] == '"') {
          field.value.push_back('"');
          ++i;
        } else {
          in_quotes = false;
        }
      } else {
        field.value.push_back(c);
      }
      continue;
    }

    if (c == '"' && !field_has_content) {
      in_quotes = true;
      field.quoted = true;
      field_has_content = true;
    } else if (c == ',') {
      end_field();
    } else if (c == '\r') {
      // Swallow CR; the following LF (if any) terminates the row.
    } else if (c == '\n') {
      end_row();
    } else {
      field.value.push_back(c);
      field_has_content = true;
    }
  }

  // Flush any trailing field/row not terminated by a newline.
  if (field_has_content || field.quoted || !row.empty()) {
    end_row();
  }

  return rows;
}

// Canonical destination for a header column.
enum class Column { kIgnore, kExtra, kId, kManufacturer, kModel, kSerial, kHardwareRev, kFirmware, kEndpoint, kRole };

Column canonical_column(const std::string & header_lower) {
  if (header_lower == "id") {
    return Column::kId;
  }
  if (header_lower == "manufacturer") {
    return Column::kManufacturer;
  }
  if (header_lower == "model") {
    return Column::kModel;
  }
  if (header_lower == "serial" || header_lower == "serial_number") {
    return Column::kSerial;
  }
  if (header_lower == "hardware_rev" || header_lower == "hardware_revision" || header_lower == "hw_rev") {
    return Column::kHardwareRev;
  }
  if (header_lower == "firmware" || header_lower == "firmware_version" || header_lower == "fw") {
    return Column::kFirmware;
  }
  if (header_lower == "endpoint") {
    return Column::kEndpoint;
  }
  if (header_lower == "role") {
    return Column::kRole;
  }
  return Column::kExtra;
}

void assign(AssetEntry & entry, Column column, const std::string & header, const std::string & value) {
  switch (column) {
    case Column::kId:
      entry.id = value;
      break;
    case Column::kManufacturer:
      entry.manufacturer = value;
      break;
    case Column::kModel:
      entry.model = value;
      break;
    case Column::kSerial:
      entry.serial = value;
      break;
    case Column::kHardwareRev:
      entry.hardware_rev = value;
      break;
    case Column::kFirmware:
      entry.firmware = value;
      break;
    case Column::kEndpoint:
      entry.endpoint = value;
      break;
    case Column::kRole:
      entry.role = value;
      break;
    case Column::kExtra:
      if (!value.empty()) {
        entry.extras.emplace_back(header, value);
      }
      break;
    case Column::kIgnore:
      break;
  }
}

}  // namespace

AssetCsvResult parse_asset_csv(const std::string & csv_text) {
  AssetCsvResult result;

  auto rows = tokenize(csv_text);
  if (rows.empty()) {
    throw std::runtime_error("asset CSV: no header row found");
  }

  // Build the column map from the header row.
  const auto & header_row = rows.front();
  std::vector<Column> columns;
  std::vector<std::string> header_names;
  columns.reserve(header_row.size());
  header_names.reserve(header_row.size());
  bool has_id = false;
  for (const auto & cell : header_row) {
    const std::string name = trim(cell.value);
    Column col = name.empty() ? Column::kIgnore : canonical_column(to_lower(name));
    if (col == Column::kId) {
      has_id = true;
    }
    columns.push_back(col);
    header_names.push_back(name);
  }
  if (!has_id) {
    throw std::runtime_error("asset CSV: missing required 'id' column");
  }

  for (std::size_t r = 1; r < rows.size(); ++r) {
    const auto & data_row = rows[r];
    AssetEntry entry;
    for (std::size_t c = 0; c < columns.size() && c < data_row.size(); ++c) {
      assign(entry, columns[c], header_names[c], data_row[c].value);
    }
    if (entry.id.empty()) {
      result.warnings.push_back("asset CSV: row " + std::to_string(r + 1) + " skipped (empty id)");
      continue;
    }
    result.entries.push_back(std::move(entry));
  }

  return result;
}

Component asset_entry_to_component(const AssetEntry & entry) {
  Component comp;
  comp.id = entry.id;
  comp.source = "inventory";
  comp.fqn = "/" + entry.id;

  // name <- "<manufacturer> <model>" (left empty when neither is set so the
  // consumer falls back to the id).
  std::string name = trim(entry.manufacturer + " " + entry.model);
  comp.name = name;

  // variant carries the hardware revision.
  comp.variant = entry.hardware_rev;

  // role becomes a tag for filtering.
  if (!entry.role.empty()) {
    comp.tags.push_back(entry.role);
  }

  // description: human-readable identity summary. base is the manufacturer/model
  // line; details append the remaining identity fields and any extras.
  std::vector<std::string> details;
  if (!entry.serial.empty()) {
    details.push_back("S/N " + entry.serial);
  }
  if (!entry.firmware.empty()) {
    details.push_back("FW " + entry.firmware);
  }
  if (!entry.endpoint.empty()) {
    details.push_back("endpoint " + entry.endpoint);
  }
  for (const auto & [key, value] : entry.extras) {
    details.push_back(key + " " + value);
  }

  std::string description = name;
  if (!details.empty()) {
    std::string joined;
    for (std::size_t i = 0; i < details.size(); ++i) {
      if (i > 0) {
        joined += ", ";
      }
      joined += details[i];
    }
    if (!description.empty()) {
      description += " ";
    }
    description += "(" + joined + ")";
  }
  comp.description = description;

  return comp;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
