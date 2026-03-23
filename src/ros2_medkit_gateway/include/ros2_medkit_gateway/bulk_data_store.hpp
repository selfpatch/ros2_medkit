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

#include <filesystem>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <unordered_set>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief Filesystem-based storage for uploaded bulk data files.
 *
 * Stores files in a directory structure:
 *   {storage_dir}/{entity_id}/{category}/{item_id}/
 *     descriptor.json    — metadata sidecar
 *     {original_filename} — payload
 *
 * Thread-safe: uses mutex for rename + descriptor write operations.
 * Large payload writes happen outside the lock.
 *
 * Categories are configured at construction time. The "rosbags" category
 * is NOT managed here — it is handled by FaultManager.
 *
 * @todo #215 Per-category config (BulkDataCategoryConfig): access mode
 *   (read-only/write-only/read-write), per-category max_file_size, source_path/upload_path
 *   templates, human-readable name. Currently categories are plain string IDs with a single
 *   global upload limit. See https://github.com/selfpatch/ros2_medkit/issues/215
 */
class BulkDataStore {
 public:
  /**
   * @brief Metadata descriptor for an uploaded bulk data item.
   */
  struct ItemDescriptor {
    std::string id;
    std::string category;
    std::string name;  ///< Original filename
    std::string mime_type;
    size_t size{0};
    std::string created;  ///< ISO 8601 timestamp
    std::string description;
    nlohmann::json metadata;
  };

  /**
   * @brief Construct a BulkDataStore.
   * @param storage_dir Base directory for uploaded files
   * @param max_upload_bytes Global max upload size in bytes (0 = unlimited)
   * @param categories List of allowed category names (besides "rosbags")
   */
  BulkDataStore(const std::string & storage_dir, size_t max_upload_bytes, std::vector<std::string> categories = {});

  // === CRUD ===

  /**
   * @brief Upload a file.
   *
   * Uses atomic write: payload → tmp file → rename + write descriptor.json.
   *
   * @param entity_id Entity this file belongs to
   * @param category Bulk data category (must be in configured list)
   * @param filename Original filename
   * @param content_type MIME type
   * @param data File contents
   * @param description Optional text description
   * @param metadata Optional JSON metadata
   * @return ItemDescriptor on success, error message on failure
   */
  tl::expected<ItemDescriptor, std::string> store(const std::string & entity_id, const std::string & category,
                                                  const std::string & filename, const std::string & content_type,
                                                  const std::string & data, const std::string & description = "",
                                                  const nlohmann::json & metadata = {});

  /**
   * @brief Delete an uploaded item.
   * @param entity_id Entity ID
   * @param category Category name
   * @param item_id Item ID to delete
   * @return void on success, error message if not found
   */
  tl::expected<void, std::string> remove(const std::string & entity_id, const std::string & category,
                                         const std::string & item_id);

  // === Queries ===

  /// List configured categories (does NOT include "rosbags")
  std::vector<std::string> list_categories() const;

  /// List items in a category for an entity (scans filesystem)
  std::vector<ItemDescriptor> list_items(const std::string & entity_id, const std::string & category) const;

  /// Get single item descriptor
  std::optional<ItemDescriptor> get_item(const std::string & entity_id, const std::string & category,
                                         const std::string & item_id) const;

  /// Get filesystem path to uploaded file (for streaming download)
  std::optional<std::string> get_file_path(const std::string & entity_id, const std::string & category,
                                           const std::string & item_id) const;

  // === Validation ===

  /// Check if category is in configured list
  bool is_known_category(const std::string & category) const;

  /// Get max upload size in bytes
  size_t max_upload_bytes() const {
    return max_upload_bytes_;
  }

 private:
  std::string storage_dir_;
  size_t max_upload_bytes_;
  std::unordered_set<std::string> categories_;
  mutable std::mutex mutex_;

  /// Generate unique ID: {category}_{timestamp_ns}_{random_hex_8}
  static std::string generate_id(const std::string & category);

  /// Get item directory: {storage_dir}/{entity_id}/{category}/{item_id}/
  std::filesystem::path item_dir(const std::string & entity_id, const std::string & category,
                                 const std::string & item_id) const;

  /// Write descriptor.json sidecar. Returns false on I/O error.
  static bool write_descriptor(const std::filesystem::path & dir, const ItemDescriptor & desc);

  /// Read descriptor.json sidecar
  static std::optional<ItemDescriptor> read_descriptor(const std::filesystem::path & dir);

  /// Validate path component (reject traversal attacks)
  static tl::expected<void, std::string> validate_path_component(const std::string & value, const std::string & name);
};

}  // namespace ros2_medkit_gateway
