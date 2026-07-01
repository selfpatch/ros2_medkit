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

#pragma once

#include "ros2_medkit_gateway/core/discovery/models/component.hpp"

#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief A manually declared inventory asset (hand-authored or CSV-imported).
 *
 * Represents one physical/logical asset the operator knows about but that a
 * protocol layer may not fully describe (or may not describe at all). The
 * canonical identity fields mirror the INV1 asset-identity model; anything not
 * covered by a canonical column is retained verbatim in @ref extras.
 *
 * @note INV1 dependency: once the structured `AssetIdentity` field lands on the
 *       Component entity, `asset_entry_to_component` should populate it directly
 *       (with per-field provenance) instead of folding identity into the
 *       existing description / variant / tag fields. This struct is the single
 *       place the two import paths (CSV and manifest `assets:` list) converge,
 *       so the migration is localized to `asset_entry_to_component`.
 */
struct AssetEntry {
  std::string id;            ///< Stable asset id (required); merge key into the tree
  std::string manufacturer;  ///< Vendor / OEM
  std::string model;         ///< Model / order code
  std::string serial;        ///< Serial number
  std::string hardware_rev;  ///< Hardware revision
  std::string firmware;      ///< Firmware / software version
  std::string endpoint;      ///< Network endpoint (URL / host:port)
  std::string role;          ///< Functional role (controller, sensor, ...)

  /// Non-canonical columns, kept in declared order (header -> value).
  std::vector<std::pair<std::string, std::string>> extras;
};

/**
 * @brief Result of parsing a CSV inventory document.
 */
struct AssetCsvResult {
  std::vector<AssetEntry> entries;    ///< One entry per non-empty data row with an id
  std::vector<std::string> warnings;  ///< Non-fatal issues (skipped rows, ...)
};

/**
 * @brief Parse a CSV inventory document into asset entries.
 *
 * The first non-empty line is the header. Column names are matched
 * case-insensitively after trimming; recognized canonical names are
 * `id, manufacturer, model, serial, hardware_rev, firmware, endpoint, role`
 * (a small set of common aliases is also accepted). Any other column is
 * preserved as an extra keyed by its original (trimmed) header name.
 *
 * RFC-4180-style quoting is honored: double-quoted fields may contain commas,
 * newlines, and escaped quotes (`""`). Unquoted fields are whitespace-trimmed.
 * Blank lines are skipped. Rows whose `id` is empty are skipped and reported in
 * @ref AssetCsvResult::warnings.
 *
 * @param csv_text Full CSV document.
 * @return Parsed entries plus any non-fatal warnings.
 * @throws std::runtime_error if the document has no header or no `id` column.
 */
AssetCsvResult parse_asset_csv(const std::string & csv_text);

/**
 * @brief Convert an inventory asset into a SOVD Component.
 *
 * The component is tagged `source = "inventory"` so its provenance stays
 * visible after the merge pipeline combines it (by id) with protocol-discovered
 * structure. Identity is projected onto the existing Component fields until the
 * INV1 structured identity model is available (see the AssetEntry note):
 *   - name        <- "<manufacturer> <model>" (falls back to id at display time)
 *   - variant     <- hardware_rev
 *   - tags        <- role (when present)
 *   - description <- human-readable identity summary (serial, firmware,
 *                    endpoint, extras)
 *
 * @param entry Asset entry (must have a non-empty id).
 * @return Component with `source = "inventory"` and identity populated.
 */
Component asset_entry_to_component(const AssetEntry & entry);

}  // namespace discovery
}  // namespace ros2_medkit_gateway
