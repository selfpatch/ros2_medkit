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

#include "ros2_medkit_gateway/http/handlers/bulkdata_handlers.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <unordered_map>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

BulkDataHandlers::BulkDataHandlers(HandlerContext & ctx) : ctx_(ctx) {
}

void BulkDataHandlers::handle_list_categories(const httplib::Request & req, httplib::Response & res) {
  // Parse entity path from request URL
  auto entity_info = parse_entity_path(req.path);
  if (!entity_info) {
    HandlerContext::send_error(res, httplib::StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid entity path");
    return;
  }

  // Verify entity exists
  auto entity = ctx_.get_entity_info(entity_info->entity_id);
  if (entity.type == EntityType::UNKNOWN) {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                               {{"entity_id", entity_info->entity_id}});
    return;
  }

  // Currently only "rosbags" category is supported
  nlohmann::json response = {{"items", nlohmann::json::array({"rosbags"})}};

  HandlerContext::send_json(res, response);
}

void BulkDataHandlers::handle_list_descriptors(const httplib::Request & req, httplib::Response & res) {
  // Parse entity path from request URL
  auto entity_info = parse_entity_path(req.path);
  if (!entity_info) {
    HandlerContext::send_error(res, httplib::StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid entity path");
    return;
  }

  // Get entity for FQN lookup
  auto entity = ctx_.get_entity_info(entity_info->entity_id);
  if (entity.type == EntityType::UNKNOWN) {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                               {{"entity_id", entity_info->entity_id}});
    return;
  }

  // Extract and validate category from path
  auto category = extract_bulk_data_category(req.path);
  if (category.empty()) {
    HandlerContext::send_error(res, httplib::StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Missing category");
    return;
  }

  if (category != "rosbags") {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                               "Unknown category: " + category);
    return;
  }

  // Get FaultManager from node
  auto fault_mgr = ctx_.node()->get_fault_manager();

  // Get all faults for this entity (filter by FQN/namespace path)
  // Use the entity's FQN or namespace_path as the source_id filter
  std::string source_filter = entity.fqn.empty() ? entity.namespace_path : entity.fqn;
  auto faults_result = fault_mgr->get_faults(source_filter);

  // Build a map of fault_code -> fault_json for quick lookup
  std::unordered_map<std::string, nlohmann::json> fault_map;
  if (faults_result.success && faults_result.data.contains("faults")) {
    for (const auto & fault_json : faults_result.data["faults"]) {
      if (fault_json.contains("fault_code")) {
        std::string fc = fault_json["fault_code"].get<std::string>();
        fault_map[fc] = fault_json;
      }
    }
  }

  // Use batch rosbag retrieval (single service call) instead of N+1 individual calls
  auto rosbags_result = fault_mgr->get_rosbags(source_filter);

  nlohmann::json items = nlohmann::json::array();

  if (rosbags_result.success && rosbags_result.data.contains("rosbags")) {
    for (const auto & rosbag : rosbags_result.data["rosbags"]) {
      std::string fault_code = rosbag.value("fault_code", "");
      std::string format = rosbag.value("format", "mcap");
      uint64_t size_bytes = rosbag.value("size_bytes", 0);
      double duration_sec = rosbag.value("duration_sec", 0.0);

      // Use fault_code as bulk_data_id
      std::string bulk_data_id = fault_code;

      // Get timestamp from fault if available
      int64_t created_at_ns = 0;
      auto it = fault_map.find(fault_code);
      if (it != fault_map.end()) {
        double first_occurred = it->second.value("first_occurred", 0.0);
        created_at_ns = static_cast<int64_t>(first_occurred * 1'000'000'000);
      }

      nlohmann::json descriptor = {
          {"id", bulk_data_id},
          {"name", fault_code + " recording " + format_timestamp_ns(created_at_ns)},
          {"mimetype", get_rosbag_mimetype(format)},
          {"size", size_bytes},
          {"creation_date", format_timestamp_ns(created_at_ns)},
          {"x-medkit", {{"fault_code", fault_code}, {"duration_sec", duration_sec}, {"format", format}}}};
      items.push_back(descriptor);
    }
  }

  nlohmann::json response = {{"items", items}};
  HandlerContext::send_json(res, response);
}

void BulkDataHandlers::handle_download(const httplib::Request & req, httplib::Response & res) {
  // Parse entity path from request URL
  auto entity_info = parse_entity_path(req.path);
  if (!entity_info) {
    HandlerContext::send_error(res, httplib::StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid entity path");
    return;
  }

  // Get entity to verify it exists
  auto entity = ctx_.get_entity_info(entity_info->entity_id);
  if (entity.type == EntityType::UNKNOWN) {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                               {{"entity_id", entity_info->entity_id}});
    return;
  }

  // Extract category and bulk_data_id from path
  auto category = extract_bulk_data_category(req.path);
  auto bulk_data_id = extract_bulk_data_id(req.path);

  if (category != "rosbags") {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                               "Unknown category: " + category);
    return;
  }

  if (bulk_data_id.empty()) {
    HandlerContext::send_error(res, httplib::StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Missing bulk-data ID");
    return;
  }

  // Get FaultManager from node
  auto fault_mgr = ctx_.node()->get_fault_manager();

  // bulk_data_id is the fault_code
  std::string fault_code = bulk_data_id;

  // Get rosbag info
  auto rosbag_result = fault_mgr->get_rosbag(fault_code);
  if (!rosbag_result.success || !rosbag_result.data.contains("file_path")) {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found",
                               {{"bulk_data_id", bulk_data_id}});
    return;
  }

  // Security check: verify rosbag belongs to this entity
  // Get faults for entity and check if fault_code is in the list
  std::string source_filter = entity.fqn.empty() ? entity.namespace_path : entity.fqn;
  auto faults_result = fault_mgr->get_faults(source_filter);

  bool belongs_to_entity = false;
  if (faults_result.success && faults_result.data.contains("faults")) {
    for (const auto & fault_json : faults_result.data["faults"]) {
      if (fault_json.contains("fault_code") && fault_json["fault_code"].get<std::string>() == fault_code) {
        belongs_to_entity = true;
        break;
      }
    }
  }

  if (!belongs_to_entity) {
    HandlerContext::send_error(res, httplib::StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND,
                               "Bulk-data not found for this entity", {{"entity_id", entity_info->entity_id}});
    return;
  }

  // Get file path and stream the file
  std::string file_path = rosbag_result.data["file_path"].get<std::string>();
  std::string format = rosbag_result.data.value("format", "mcap");
  auto mimetype = get_rosbag_mimetype(format);
  std::string filename = fault_code + "." + format;

  // Set response headers for file download
  res.set_header("Content-Disposition", "attachment; filename=\"" + filename + "\"");

  if (!stream_file_to_response(res, file_path, mimetype)) {
    HandlerContext::send_error(res, httplib::StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to read rosbag file");
  }
}

bool BulkDataHandlers::stream_file_to_response(httplib::Response & res, const std::string & file_path,
                                               const std::string & content_type) {
  // Resolve the actual file path - rosbag2 creates a directory with the db3/mcap file inside
  std::string actual_path = resolve_rosbag_file_path(file_path);
  if (actual_path.empty()) {
    return false;
  }

  // Get file size without reading entire file into memory
  std::error_code ec;
  auto file_size = std::filesystem::file_size(actual_path, ec);
  if (ec) {
    return false;
  }

  // Use chunked streaming via content provider to avoid loading large rosbag files into memory.
  // Rosbag files can be hundreds of MB to multiple GB.
  static constexpr size_t kChunkSize = 64 * 1024;  // 64 KB chunks

  res.set_content_provider(
      static_cast<size_t>(file_size), content_type,
      [actual_path](size_t offset, size_t length, httplib::DataSink & sink) -> bool {
        std::ifstream file(actual_path, std::ios::binary);
        if (!file.is_open()) {
          return false;
        }

        file.seekg(static_cast<std::streamoff>(offset));
        if (!file.good()) {
          return false;
        }

        size_t remaining = length;
        std::vector<char> buf(std::min(remaining, kChunkSize));

        while (remaining > 0 && file.good()) {
          size_t to_read = std::min(remaining, kChunkSize);
          file.read(buf.data(), static_cast<std::streamsize>(to_read));
          auto bytes_read = static_cast<size_t>(file.gcount());
          if (bytes_read == 0) {
            break;
          }
          sink.write(buf.data(), bytes_read);
          remaining -= bytes_read;
        }

        return remaining == 0;
      });

  return true;
}

std::string BulkDataHandlers::resolve_rosbag_file_path(const std::string & path) {
  // If it's a regular file, return as-is
  if (std::filesystem::is_regular_file(path)) {
    return path;
  }

  // If it's a directory (rosbag2 directory structure), find the db3/mcap file inside
  if (std::filesystem::is_directory(path)) {
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
      if (entry.is_regular_file()) {
        auto ext = entry.path().extension().string();
        // Look for db3 (sqlite3 format) or mcap files
        if (ext == ".db3" || ext == ".mcap") {
          return entry.path().string();
        }
      }
    }
  }

  return "";  // File not found
}

std::string BulkDataHandlers::get_rosbag_mimetype(const std::string & format) {
  if (format == "mcap") {
    return "application/x-mcap";
  } else if (format == "sqlite3" || format == "db3") {
    return "application/x-sqlite3";
  }
  return "application/octet-stream";
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
