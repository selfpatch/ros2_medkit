// Copyright 2026 mfaferek93
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

// GET /{entity}/bulk-data/rosbags/{recording_id}/manifest
// GET /{entity}/bulk-data/rosbags/{recording_id}/files/{index}

#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/faults/rosbag_recording.hpp"
#include "ros2_medkit_gateway/core/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Last segment of the request path, ignoring a trailing slash.
std::string last_path_segment(std::string path) {
  while (!path.empty() && path.back() == '/') {
    path.pop_back();
  }
  const auto slash = path.rfind('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

/// Directory of the recording the request addresses, once the entity may read it.
tl::expected<std::string, ErrorInfo> resolve_recording(HandlerContext & ctx, const http::TypedRequest & req) {
  auto path_info = parse_entity_path(req.path());
  if (!path_info) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid entity path"));
  }

  auto entity_result = ctx.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  auto * fault_mgr = ctx.services().fault_manager();
  if (fault_mgr == nullptr || !fault_mgr->serves_rosbags()) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Unknown category: rosbags"));
  }

  const std::string recording_id = extract_bulk_data_id(req.path());
  if (recording_id.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing recording ID"));
  }

  const auto lookup = faults::find_recording(
      *fault_mgr, recording_id, detail::compute_bulkdata_source_filters(ctx.services().entity_cache(), entity));
  switch (lookup.status) {
    case faults::RecordingLookupStatus::kFound:
      return lookup.path;
    case faults::RecordingLookupStatus::kNotFound:
      return tl::unexpected(make_error(404, ERR_X_MEDKIT_RECORDING_NOT_FOUND, "No recording with this id",
                                       json{{"recording_id", recording_id}}));
    case faults::RecordingLookupStatus::kNotInEntity:
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Recording not found for this entity",
                                       json{{"entity_id", path_info->entity_id}}));
    case faults::RecordingLookupStatus::kUnavailable:
    default:
      return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Fault manager did not answer",
                                       json{{"recording_id", recording_id}}));
  }
}

}  // namespace

http::Result<std::string> BulkDataHandlers::recording_manifest(const http::TypedRequest & req) {
  const auto recording_path = resolve_recording(ctx_, req);
  if (!recording_path) {
    return tl::unexpected(recording_path.error());
  }

  auto manifest = faults::read_recording_manifest(*recording_path);
  if (!manifest) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Recording has no manifest"));
  }
  return std::move(*manifest);
}

nlohmann::json BulkDataHandlers::recording_manifest_schema() {
  const auto described = [](const char * type, const char * description) {
    return json{{"type", type}, {"description", description}};
  };
  json file = {{"type", "object"},
               {"required", {"name", "size", "sha256"}},
               {"properties",
                {{"name", described("string", "Storage file name inside the recording.")},
                 {"size", described("integer", "File size in bytes.")},
                 {"sha256", described("string", "Lowercase hex sha256 of the file.")}}}};
  json format = described("string", "Storage format of every file.");
  format["enum"] = {"mcap", "sqlite3"};
  json files = {{"type", "array"},
                {"description", "Storage files in recording order. A file is fetched by its index in this list."},
                {"items", std::move(file)}};
  return json{{"type", "object"},
              {"required", {"manifest_version", "recording_id", "format", "created_at_ns", "total_bytes", "files"}},
              {"properties",
               {{"manifest_version", described("integer", "Version of this layout. Currently 1.")},
                {"recording_id", described("string", "The recording this manifest belongs to.")},
                {"format", std::move(format)},
                {"created_at_ns", described("integer", "When the recording was closed, nanoseconds since the epoch.")},
                {"total_bytes", described("integer", "Sum of the file sizes.")},
                {"files", std::move(files)}}}};
}

http::Result<http::BinaryResponse> BulkDataHandlers::recording_file(const http::TypedRequest & req) {
  const auto recording_path = resolve_recording(ctx_, req);
  if (!recording_path) {
    return tl::unexpected(recording_path.error());
  }

  const auto manifest = faults::read_recording_manifest(*recording_path);
  if (!manifest) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Recording has no manifest"));
  }

  const std::string index_text = last_path_segment(req.path());
  const auto index = faults::parse_recording_file_index(index_text);
  const auto file_path = index ? faults::recording_file_path(*recording_path, *manifest, *index) : std::nullopt;
  if (!file_path) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "No such file in this recording", json{{"index", index_text}}));
  }

  std::error_code ec;
  const auto file_size = std::filesystem::file_size(*file_path, ec);
  if (ec) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "No such file in this recording", json{{"index", index_text}}));
  }
  http::BinaryResponse resp;
  resp.content_type = "application/octet-stream";
  resp.filename = std::filesystem::path(*file_path).filename().string();
  std::replace(resp.filename->begin(), resp.filename->end(), '"', '_');
  resp.supports_ranges = true;
  resp.total_size = static_cast<uint64_t>(file_size);
  static constexpr std::size_t kChunkSize = 64 * 1024;
  resp.provider = [path = *file_path](uint64_t offset, uint64_t length, httplib::DataSink & sink) -> bool {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
      return false;
    }
    file.seekg(static_cast<std::streamoff>(offset));
    if (!file.good()) {
      return false;
    }
    uint64_t remaining = length;
    std::vector<char> buf(static_cast<std::size_t>(std::min<uint64_t>(remaining, kChunkSize)));
    while (remaining > 0 && file.good()) {
      const auto to_read = static_cast<std::size_t>(std::min<uint64_t>(remaining, kChunkSize));
      file.read(buf.data(), static_cast<std::streamsize>(to_read));
      const auto bytes_read = static_cast<std::size_t>(file.gcount());
      if (bytes_read == 0) {
        break;
      }
      sink.write(buf.data(), bytes_read);
      remaining -= bytes_read;
    }
    return remaining == 0;
  };
  return resp;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
