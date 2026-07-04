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
#include <unordered_set>
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
  // True while the current unquoted field has only seen leading whitespace, so a
  // quote after it still opens a quoted field (e.g. `a, "x"` -> field == "x").
  bool field_only_leading_ws = true;

  const auto end_field = [&]() {
    if (!field.quoted) {
      field.value = trim(field.value);
    }
    row.push_back(std::move(field));
    field = CsvField{};
    field_has_content = false;
    field_only_leading_ws = true;
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

  // Skip a leading UTF-8 BOM (0xEF 0xBB 0xBF). Excel's "CSV UTF-8" export always
  // writes one; left in place it fuses onto the first header cell ("﻿id"),
  // which fails header detection and takes down the whole inventory import.
  std::size_t start = 0;
  if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF && static_cast<unsigned char>(text[1]) == 0xBB &&
      static_cast<unsigned char>(text[2]) == 0xBF) {
    start = 3;
  }

  for (std::size_t i = start; i < text.size(); ++i) {
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

    if (c == '"' && field_only_leading_ws && !field.quoted) {
      // Opening quote of a field, possibly after leading spaces: drop the
      // accumulated leading whitespace so the quoted value stays verbatim.
      field.value.clear();
      in_quotes = true;
      field.quoted = true;
      field_has_content = true;
      field_only_leading_ws = false;
    } else if (c == ',') {
      end_field();
    } else if (c == '\r') {
      // Swallow CR; the following LF (if any) terminates the row.
    } else if (c == '\n') {
      end_row();
    } else {
      field.value.push_back(c);
      field_has_content = true;
      if (std::isspace(static_cast<unsigned char>(c)) == 0) {
        field_only_leading_ws = false;
      }
    }
  }

  // Flush any trailing field/row not terminated by a newline.
  if (field_has_content || field.quoted || !row.empty()) {
    end_row();
  }

  return rows;
}

}  // namespace

AssetColumn asset_column(const std::string & name) {
  const std::string header_lower = to_lower(trim(name));
  if (header_lower.empty()) {
    return AssetColumn::kIgnore;
  }
  if (header_lower == "id") {
    return AssetColumn::kId;
  }
  if (header_lower == "manufacturer") {
    return AssetColumn::kManufacturer;
  }
  if (header_lower == "model") {
    return AssetColumn::kModel;
  }
  if (header_lower == "order_code" || header_lower == "order_number") {
    return AssetColumn::kOrderCode;
  }
  if (header_lower == "serial" || header_lower == "serial_number") {
    return AssetColumn::kSerial;
  }
  if (header_lower == "hardware_rev" || header_lower == "hardware_revision" || header_lower == "hw_rev") {
    return AssetColumn::kHardwareRev;
  }
  if (header_lower == "firmware" || header_lower == "firmware_version" || header_lower == "fw") {
    return AssetColumn::kFirmware;
  }
  if (header_lower == "endpoint") {
    return AssetColumn::kEndpoint;
  }
  if (header_lower == "role") {
    return AssetColumn::kRole;
  }
  if (header_lower == "area") {
    return AssetColumn::kArea;
  }
  return AssetColumn::kExtra;
}

void assign_asset_field(AssetEntry & entry, AssetColumn column, const std::string & header, const std::string & value) {
  switch (column) {
    case AssetColumn::kId:
      entry.id = value;
      break;
    case AssetColumn::kManufacturer:
      entry.manufacturer = value;
      break;
    case AssetColumn::kModel:
      entry.model = value;
      break;
    case AssetColumn::kOrderCode:
      entry.order_code = value;
      break;
    case AssetColumn::kSerial:
      entry.serial = value;
      break;
    case AssetColumn::kHardwareRev:
      entry.hardware_rev = value;
      break;
    case AssetColumn::kFirmware:
      entry.firmware = value;
      break;
    case AssetColumn::kEndpoint:
      entry.endpoint = value;
      break;
    case AssetColumn::kRole:
      entry.role = value;
      break;
    case AssetColumn::kArea:
      entry.area = value;
      break;
    case AssetColumn::kExtra:
      if (!value.empty()) {
        entry.extras.emplace_back(header, value);
      }
      break;
    case AssetColumn::kIgnore:
      break;
  }
}

AssetCsvResult parse_asset_csv(const std::string & csv_text) {
  AssetCsvResult result;

  auto rows = tokenize(csv_text);
  if (rows.empty()) {
    throw std::runtime_error("asset CSV: no header row found");
  }

  // Build the column map from the header row.
  const auto & header_row = rows.front();
  std::vector<AssetColumn> columns;
  std::vector<std::string> header_names;
  columns.reserve(header_row.size());
  header_names.reserve(header_row.size());
  bool has_id = false;
  for (const auto & cell : header_row) {
    const std::string name = trim(cell.value);
    AssetColumn col = asset_column(name);
    if (col == AssetColumn::kId) {
      has_id = true;
    }
    columns.push_back(col);
    header_names.push_back(name);
  }
  if (!has_id) {
    throw std::runtime_error("asset CSV: missing required 'id' column");
  }

  std::unordered_set<std::string> seen_ids;
  for (std::size_t r = 1; r < rows.size(); ++r) {
    const auto & data_row = rows[r];
    AssetEntry entry;
    for (std::size_t c = 0; c < columns.size() && c < data_row.size(); ++c) {
      assign_asset_field(entry, columns[c], header_names[c], data_row[c].value);
    }
    if (entry.id.empty()) {
      result.warnings.push_back("asset CSV: row " + std::to_string(r + 1) + " skipped (empty id)");
      continue;
    }
    // Dedup within the document: first row wins, later duplicates are dropped so
    // one bad export line cannot fail the whole manifest load downstream.
    if (!seen_ids.insert(entry.id).second) {
      result.warnings.push_back("asset CSV: row " + std::to_string(r + 1) + " skipped (duplicate id '" + entry.id +
                                "', first row wins)");
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
  // fqn/namespace_path are intentionally left empty: a bare inventory asset
  // carries no placement, so a synthetic "/id" must not override the real path
  // of a discovered node it merges with. Placement comes from the manifest
  // `namespace:` key (see parse_asset) when the operator declares it.
  // `area` is structural placement, not identity: no provenance entry.
  comp.area = entry.area;

  // name <- "<manufacturer> <model>" (left empty when neither is set so the
  // consumer falls back to the id).
  comp.name = trim(entry.manufacturer + " " + entry.model);

  // Structured nameplate: every canonical column lands on the typed identity
  // with per-field provenance "inventory", so the identity merge can rank a
  // hand-authored value against manifest / live protocol reads per field.
  auto set_field = [&comp](std::string AssetIdentity::*member, const std::string & value, const char * prov_key) {
    if (!value.empty()) {
      comp.identity.*member = value;
      comp.identity.provenance[prov_key] = "inventory";
    }
  };
  set_field(&AssetIdentity::manufacturer, entry.manufacturer, "manufacturer");
  set_field(&AssetIdentity::model, entry.model, "model");
  set_field(&AssetIdentity::order_code, entry.order_code, "order_code");
  set_field(&AssetIdentity::serial_number, entry.serial, "serial_number");
  set_field(&AssetIdentity::hardware_revision, entry.hardware_rev, "hardware_revision");
  set_field(&AssetIdentity::firmware_version, entry.firmware, "firmware_version");
  set_field(&AssetIdentity::network_endpoint, entry.endpoint, "network_endpoint");
  set_field(&AssetIdentity::role, entry.role, "role");
  for (const auto & [key, value] : entry.extras) {
    if (!value.empty()) {
      comp.identity.extra[key] = value;
      comp.identity.provenance["extra." + key] = "inventory";
    }
  }

  return comp;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
