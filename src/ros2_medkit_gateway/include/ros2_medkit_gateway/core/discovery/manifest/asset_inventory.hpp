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
 * @note `asset_entry_to_component` populates the Component's structured
 *       `AssetIdentity` directly, with per-field provenance "inventory", so
 *       identity merging ranks hand-authored values against other sources per
 *       field. This struct is the single place the two import paths (CSV and
 *       manifest `assets:` list) converge.
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
  std::string area;          ///< Optional Area id placing the asset in the tree

  /// Non-canonical columns, kept in declared order (header -> value).
  std::vector<std::pair<std::string, std::string>> extras;
};

/**
 * @brief Canonical destination of an inventory column / manifest asset key.
 */
enum class AssetColumn {
  kIgnore,  ///< Empty header cell: drop the value
  kExtra,   ///< Not a canonical name: keep as identity extra
  kId,
  kManufacturer,
  kModel,
  kSerial,
  kHardwareRev,
  kFirmware,
  kEndpoint,
  kRole,
  kArea,
};

/**
 * @brief Map a column / key name to its canonical destination.
 *
 * The single alias table shared by both import paths (CSV header cells and
 * manifest `assets:` keys): names are trimmed and matched case-insensitively;
 * `serial_number`, `hardware_revision` / `hw_rev` and `firmware_version` / `fw`
 * map to their typed fields. Empty names yield kIgnore; unknown names kExtra.
 */
AssetColumn asset_column(const std::string & name);

/**
 * @brief Assign `value` to the @ref AssetEntry field selected by `column`.
 *
 * kExtra appends (`header`, `value`) to `entry.extras` (empty values are
 * dropped); kIgnore is a no-op.
 */
void assign_asset_field(AssetEntry & entry, AssetColumn column, const std::string & header, const std::string & value);

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
 * The first non-empty line is the header. Column names are matched via
 * ::asset_column; recognized canonical names are
 * `id, manufacturer, model, serial, hardware_rev, firmware, endpoint, role,
 * area` (a small set of common aliases is also accepted). Any other column is
 * preserved as an extra keyed by its original (trimmed) header name.
 *
 * RFC-4180-style quoting is honored: double-quoted fields may contain commas,
 * newlines, and escaped quotes (`""`). Unquoted fields are whitespace-trimmed.
 * Blank lines are skipped. Rows whose `id` is empty are skipped and reported in
 * @ref AssetCsvResult::warnings; so are rows repeating an earlier row's id
 * (first row wins).
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
 * structure. Canonical fields land on the structured `Component::identity`
 * with per-field provenance `"inventory"` (see the AssetEntry note):
 *   - name <- "<manufacturer> <model>" (left empty when neither is set so
 *     consumers fall back to the id)
 *   - identity.manufacturer / model / serial_number / hardware_revision /
 *     firmware_version / network_endpoint / role <- the matching entry
 *     fields; empty fields are skipped and record no provenance
 *   - identity.extra[header] <- non-empty extras, provenance keyed
 *     `extra.<header>`
 *
 * `fqn` / `namespace_path` are left empty: a bare inventory asset carries no
 * placement, so a discovered node it merges with keeps its real path. An
 * `area` on the entry lands on `Component::area` (structural placement, no
 * provenance entry); without one the asset appears only in the flat component
 * list, never under an Area.
 *
 * @param entry Asset entry (must have a non-empty id).
 * @return Component with `source = "inventory"` and structured identity
 *         populated.
 */
Component asset_entry_to_component(const AssetEntry & entry);

}  // namespace discovery
}  // namespace ros2_medkit_gateway
