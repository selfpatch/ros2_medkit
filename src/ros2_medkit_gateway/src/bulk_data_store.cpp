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

#include "ros2_medkit_gateway/bulk_data_store.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>

#include "ros2_medkit_gateway/http/http_utils.hpp"

namespace ros2_medkit_gateway {

BulkDataStore::BulkDataStore(const std::string & storage_dir, size_t max_upload_bytes,
                             std::vector<std::string> categories)
  : storage_dir_(storage_dir), max_upload_bytes_(max_upload_bytes) {
  for (auto & cat : categories) {
    categories_.insert(std::move(cat));
  }
  // Ensure storage directory exists
  std::error_code ec;
  std::filesystem::create_directories(storage_dir_, ec);
}

// --- Validation ---

tl::expected<void, std::string> BulkDataStore::validate_path_component(const std::string & value,
                                                                       const std::string & name) {
  if (value.empty()) {
    return tl::unexpected(name + " cannot be empty");
  }
  // Reject path traversal and unsafe characters
  if (value.find("..") != std::string::npos) {
    return tl::unexpected(name + " contains invalid sequence '..'");
  }
  if (value.find('/') != std::string::npos || value.find('\\') != std::string::npos) {
    return tl::unexpected(name + " contains path separator");
  }
  if (value.find('\0') != std::string::npos) {
    return tl::unexpected(name + " contains null byte");
  }
  return {};
}

bool BulkDataStore::is_known_category(const std::string & category) const {
  return categories_.count(category) > 0;
}

std::vector<std::string> BulkDataStore::list_categories() const {
  return {categories_.begin(), categories_.end()};
}

// --- ID generation ---

std::string BulkDataStore::generate_id(const std::string & category) {
  // {category}_{timestamp_ns}_{random_hex_8}
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  // Generate 8 hex chars from random
  static thread_local std::mt19937 gen(std::random_device{}());
  std::uniform_int_distribution<uint32_t> dist(0, 0xFFFFFFFF);
  uint32_t rand_val = dist(gen);

  std::ostringstream oss;
  oss << category << "_" << ns << "_" << std::hex << std::setfill('0') << std::setw(8) << rand_val;
  return oss.str();
}

// --- Directory helpers ---

std::filesystem::path BulkDataStore::item_dir(const std::string & entity_id, const std::string & category,
                                              const std::string & item_id) const {
  return std::filesystem::path(storage_dir_) / entity_id / category / item_id;
}

// --- Descriptor I/O ---

bool BulkDataStore::write_descriptor(const std::filesystem::path & dir, const ItemDescriptor & desc) {
  nlohmann::json j;
  j["id"] = desc.id;
  j["category"] = desc.category;
  j["name"] = desc.name;
  j["mime_type"] = desc.mime_type;
  j["size"] = desc.size;
  j["created"] = desc.created;
  j["description"] = desc.description;
  if (!desc.metadata.empty()) {
    j["metadata"] = desc.metadata;
  }

  auto path = dir / "descriptor.json";
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    return false;
  }
  ofs << j.dump(2);
  ofs.flush();
  return ofs.good();
}

std::optional<BulkDataStore::ItemDescriptor> BulkDataStore::read_descriptor(const std::filesystem::path & dir) {
  auto path = dir / "descriptor.json";
  if (!std::filesystem::is_regular_file(path)) {
    return std::nullopt;
  }

  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return std::nullopt;
  }

  try {
    nlohmann::json j = nlohmann::json::parse(ifs);
    ItemDescriptor desc;
    desc.id = j.value("id", "");
    desc.category = j.value("category", "");
    desc.name = j.value("name", "");
    desc.mime_type = j.value("mime_type", "");
    desc.size = j.value("size", static_cast<size_t>(0));
    desc.created = j.value("created", "");
    desc.description = j.value("description", "");
    if (j.contains("metadata")) {
      desc.metadata = j["metadata"];
    }
    return desc;
  } catch (...) {
    return std::nullopt;
  }
}

// --- CRUD ---

tl::expected<BulkDataStore::ItemDescriptor, std::string>
BulkDataStore::store(const std::string & entity_id, const std::string & category, const std::string & filename,
                     const std::string & content_type, const std::string & data, const std::string & description,
                     const nlohmann::json & metadata) {
  // Validate inputs
  if (auto v = validate_path_component(entity_id, "entity_id"); !v) {
    return tl::unexpected(v.error());
  }
  if (auto v = validate_path_component(category, "category"); !v) {
    return tl::unexpected(v.error());
  }

  // Validate category is known
  if (!is_known_category(category)) {
    return tl::unexpected("Unknown category: " + category);
  }

  // Check upload size limit
  if (max_upload_bytes_ > 0 && data.size() > max_upload_bytes_) {
    return tl::unexpected("File size " + std::to_string(data.size()) + " exceeds maximum upload limit " +
                          std::to_string(max_upload_bytes_));
  }

  // Generate unique ID
  auto id = generate_id(category);

  // Create item directory
  auto dir = item_dir(entity_id, category, id);
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    return tl::unexpected("Failed to create directory: " + ec.message());
  }

  // Sanitize filename (use "upload" if empty, strip path components)
  std::string safe_filename = filename.empty() ? "upload" : std::filesystem::path(filename).filename().string();
  if (safe_filename.empty()) {
    safe_filename = "upload";
  }

  // Step 1: Write payload to temp file (outside lock)
  auto tmp_path = dir / ".data.tmp";
  {
    std::ofstream ofs(tmp_path, std::ios::binary);
    if (!ofs.is_open()) {
      return tl::unexpected("Failed to open temp file for writing");
    }
    ofs.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!ofs.good()) {
      return tl::unexpected("Failed to write payload data");
    }
  }

  // Build descriptor
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  ItemDescriptor desc;
  desc.id = id;
  desc.category = category;
  desc.name = safe_filename;
  desc.mime_type = content_type.empty() ? "application/octet-stream" : content_type;
  desc.size = data.size();
  desc.created = format_timestamp_ns(ns);
  desc.description = description;
  desc.metadata = metadata;

  // Step 2: Rename + write descriptor (under lock)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto final_path = dir / safe_filename;
    std::filesystem::rename(tmp_path, final_path, ec);
    if (ec) {
      // Fallback: try copy + remove (cross-device rename)
      std::filesystem::copy_file(tmp_path, final_path, ec);
      if (ec) {
        // Clean up temp file on failure
        std::error_code cleanup_ec;
        std::filesystem::remove(tmp_path, cleanup_ec);
        return tl::unexpected("Failed to finalize file: " + ec.message());
      }
      std::filesystem::remove(tmp_path, ec);
    }
    if (!write_descriptor(dir, desc)) {
      // Clean up the payload file we just moved — the item is incomplete
      std::error_code cleanup_ec;
      std::filesystem::remove_all(dir, cleanup_ec);
      return tl::unexpected("Failed to write descriptor");
    }
  }

  return desc;
}

tl::expected<void, std::string> BulkDataStore::remove(const std::string & entity_id, const std::string & category,
                                                      const std::string & item_id) {
  if (auto err = validate_path_component(entity_id, "entity_id"); !err) {
    return tl::unexpected(err.error());
  }
  if (auto err = validate_path_component(category, "category"); !err) {
    return tl::unexpected(err.error());
  }
  if (auto err = validate_path_component(item_id, "item_id"); !err) {
    return tl::unexpected(err.error());
  }

  auto dir = item_dir(entity_id, category, item_id);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::filesystem::is_directory(dir)) {
    return tl::unexpected("Bulk-data item not found: " + item_id);
  }

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  if (ec) {
    return tl::unexpected("Failed to delete item: " + ec.message());
  }

  return {};
}

// --- Queries ---

std::vector<BulkDataStore::ItemDescriptor> BulkDataStore::list_items(const std::string & entity_id,
                                                                     const std::string & category) const {
  std::vector<ItemDescriptor> items;

  auto cat_dir = std::filesystem::path(storage_dir_) / entity_id / category;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::filesystem::is_directory(cat_dir)) {
    return items;
  }

  for (const auto & entry : std::filesystem::directory_iterator(cat_dir)) {
    if (entry.is_directory()) {
      auto desc = read_descriptor(entry.path());
      if (desc) {
        items.push_back(std::move(*desc));
      }
    }
  }

  return items;
}

std::optional<BulkDataStore::ItemDescriptor> BulkDataStore::get_item(const std::string & entity_id,
                                                                     const std::string & category,
                                                                     const std::string & item_id) const {
  auto dir = item_dir(entity_id, category, item_id);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::filesystem::is_directory(dir)) {
    return std::nullopt;
  }

  return read_descriptor(dir);
}

std::optional<std::string> BulkDataStore::get_file_path(const std::string & entity_id, const std::string & category,
                                                        const std::string & item_id) const {
  auto dir = item_dir(entity_id, category, item_id);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::filesystem::is_directory(dir)) {
    return std::nullopt;
  }

  // Find the payload file (not descriptor.json and not .data.tmp)
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.is_regular_file()) {
      auto fname = entry.path().filename().string();
      if (fname != "descriptor.json" && fname != ".data.tmp") {
        return entry.path().string();
      }
    }
  }

  return std::nullopt;
}

}  // namespace ros2_medkit_gateway
