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

#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/faults/rosbag_recording.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "ros2_medkit_gateway/core/http/entity_path_utils.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/bulk_data_store.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/dto/bulkdata.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Resolve the entity_id from the typed request. Bulk-data routes embed the
/// entity reference in the URL path; the registered route patterns capture
/// the entity id as group 1 (single-entity) or group 2 (nested subarea /
/// subcomponent). `parse_entity_path` walks the registered regex catalogue
/// and yields a normalised `EntityPathInfo` so both shapes resolve through
/// one helper. Returning an `ErrorInfo` keeps the failure surface aligned
/// with the legacy handler's "Invalid entity path" 400.
tl::expected<EntityPathInfo, ErrorInfo> parse_path(const http::TypedRequest & req) {
  auto info = parse_entity_path(req.path());
  if (!info) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Invalid entity path"));
  }
  return *info;
}

/// A regular file that is not a symlink. A link inside a bag could point at any file on the host.
bool is_plain_file(const std::filesystem::path & path) {
  std::error_code ec;
  return std::filesystem::symlink_status(path, ec).type() == std::filesystem::file_type::regular && !ec;
}

/// @p bag_path normalised, without a trailing slash. The fault manager normalises the same way.
std::filesystem::path normalized_bag_dir(const std::string & bag_path) {
  std::filesystem::path bag = std::filesystem::path(bag_path).lexically_normal();
  if (!bag.has_filename()) {
    bag = bag.parent_path();
  }
  return bag;
}

/// rosbag2_bagfile_information.relative_file_paths from <bag>/metadata.yaml. nullopt when the
/// file is missing, a symlink, unreadable, or not that shape. The fault manager reads the field
/// the same way (core/rosbag_served_size.cpp).
std::optional<std::vector<std::string>> rosbag_relative_file_paths(const std::string & bag_path) {
  const std::filesystem::path metadata_path = normalized_bag_dir(bag_path) / "metadata.yaml";
  if (!is_plain_file(metadata_path)) {
    return std::nullopt;
  }
  try {
    const YAML::Node root = YAML::LoadFile(metadata_path.string());
    if (!root.IsMap()) {
      return std::nullopt;
    }
    const YAML::Node info = root["rosbag2_bagfile_information"];
    if (!info || !info.IsMap()) {
      return std::nullopt;
    }
    const YAML::Node paths = info["relative_file_paths"];
    if (!paths || !paths.IsSequence()) {
      return std::nullopt;
    }
    std::vector<std::string> names;
    names.reserve(paths.size());
    for (const auto & entry : paths) {
      if (!entry.IsScalar()) {
        return std::nullopt;
      }
      names.push_back(entry.as<std::string>());
    }
    return names;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

/// @p relative_name inside @p bag_path when it is a direct child with a .db3 or .mcap extension.
/// An absolute or `..` name would otherwise pick any file on the host. Lexical only; callers
/// check the file with is_plain_file().
std::optional<std::filesystem::path> rosbag_named_storage_file(const std::string & bag_path,
                                                               const std::string & relative_name) {
  const std::filesystem::path bag = normalized_bag_dir(bag_path);
  std::filesystem::path named = (bag / relative_name).lexically_normal();
  if (named.parent_path() != bag) {
    return std::nullopt;
  }
  const std::string ext = named.extension().string();
  if (ext != ".db3" && ext != ".mcap") {
    return std::nullopt;
  }
  return named;
}

/// The first regular ``.db3`` or ``.mcap`` @p dir yields, in directory order.
/// Empty when there is none or the directory cannot be walked.
std::string first_storage_file_in(const std::string & dir) {
  std::error_code ec;
  std::filesystem::directory_iterator it(dir, ec);
  if (ec) {
    return "";
  }
  for (const std::filesystem::directory_iterator end; it != end; it.increment(ec)) {
    if (ec) {
      return "";
    }
    if (!is_plain_file(it->path())) {
      continue;
    }
    const auto ext = it->path().extension().string();
    if (ext == ".db3" || ext == ".mcap") {
      return it->path().string();
    }
  }
  return "";
}

}  // namespace

BulkDataHandlers::BulkDataHandlers(HandlerContext & ctx) : ctx_(ctx) {
}

std::string BulkDataHandlers::get_rosbag_mimetype(const std::string & format) {
  if (format == "mcap") {
    return "application/x-mcap";
  } else if (format == "sqlite3" || format == "db3") {
    return "application/x-sqlite3";
  }
  return "application/octet-stream";
}

std::vector<std::string> BulkDataHandlers::download_media_types() {
  // The first three are the complete range of get_rosbag_mimetype() directly
  // above - add a branch there and this list needs the type it returns. The
  // catch-all covers the store-backed categories, whose type is client-supplied
  // at upload and therefore not enumerable here; see the header for why the
  // route declares both halves rather than picking one.
  return {"application/x-mcap", "application/x-sqlite3", "application/octet-stream", "*/*"};
}

std::string BulkDataHandlers::resolve_rosbag_file_path(const std::string & path) {
  return detail::rosbag_storage(path).served_file;
}

std::vector<std::string> BulkDataHandlers::get_source_filters(const EntityInfo & entity) const {
  return detail::compute_bulkdata_source_filters(ctx_.services().entity_cache(), entity);
}

namespace detail {

std::string rosbag_recording_id(const std::string & file_path) {
  std::filesystem::path p(file_path);
  if (!p.has_filename()) {
    p = p.parent_path();  // tolerate a trailing slash
  }
  return p.filename().string();
}

RosbagStorage rosbag_storage(const std::string & bag_path) {
  // Only the std::error_code overloads: this runs once per listing row, and a
  // throw fails the whole listing for one unreadable bag.
  RosbagStorage storage;
  std::error_code ec;

  // A bare storage file is one storage file and has no metadata.yaml to ask.
  if (std::filesystem::is_regular_file(bag_path, ec) && !ec) {
    storage.served_file = bag_path;
    storage.file_count = 1;
    return storage;
  }
  if (!std::filesystem::is_directory(bag_path, ec) || ec) {
    return storage;
  }

  // Metadata that names files decides alone. The served file is the first name,
  // where the recording starts. When that file is missing or the name is not
  // followable, nothing is served: a later segment or a stray beside it is not
  // the start of the recording, and the response could not say which file it is.
  if (const auto names = rosbag_relative_file_paths(bag_path); names && !names->empty()) {
    storage.file_count = names->size();
    if (const auto named = rosbag_named_storage_file(bag_path, names->front()); named && is_plain_file(*named)) {
      storage.served_file = named->string();
    }
    return storage;
  }

  // No metadata, unreadable metadata or an empty list: the bag does not say what
  // it holds, so directory order picks the file and the count stays unknown.
  storage.served_file = first_storage_file_in(bag_path);
  return storage;
}

std::optional<uint64_t> rosbag_served_bytes(const RosbagStorage & storage) {
  // Measured here only for a recording held in one storage file. A split keeps
  // the row's figure, the fault manager's sum of the same named files.
  if (storage.file_count != std::size_t{1} || storage.served_file.empty()) {
    return std::nullopt;
  }
  std::error_code ec;
  const auto size = std::filesystem::file_size(storage.served_file, ec);
  if (ec) {
    return std::nullopt;
  }
  return static_cast<uint64_t>(size);
}

std::optional<uint64_t> rosbag_served_bytes(const std::string & bag_path) {
  return rosbag_served_bytes(rosbag_storage(bag_path));
}

bool rosbag_resolved_by_fault_code(const nlohmann::json & rosbag_data, const std::string & requested_id) {
  const std::string resolved = rosbag_data.value("recording_id", "");
  // An absent id means a peer that predates the field. It answers by fault code
  // only, and the download then authorizes on the requested code.
  return !resolved.empty() && resolved != requested_id;
}

std::string record_map_key(const std::string & owner, const std::string & fault_code) {
  // '\0' cannot appear in either id, so the two halves cannot run together.
  return owner + '\0' + fault_code;
}

std::vector<dto::BulkDataDescriptor>
fold_rosbag_rows_into_descriptors(const std::vector<nlohmann::json> & rows,
                                  const std::unordered_map<std::string, nlohmann::json> & faults_by_record) {
  struct RecordingEntry {
    std::string recording_id;
    std::string format;
    uint64_t size_bytes{0};
    std::optional<std::size_t> storage_files;
    double duration_sec{0.0};
    int64_t created_at_ns{0};
    bool manifest{false};
    std::vector<std::string> fault_codes;
  };
  std::vector<RecordingEntry> recordings;
  std::unordered_map<std::string, size_t> index_by_recording;

  for (const auto & row : rows) {
    // The fault manager supplies the recording id; falling back to the path
    // basename covers a peer or a replay predating that field.
    std::string recording_id = row.value("recording_id", "");
    if (recording_id.empty()) {
      recording_id = rosbag_recording_id(row.value("file_path", ""));
    }
    if (recording_id.empty()) {
      continue;  // nothing addressable - a row with neither id nor path
    }

    const std::string fault_code = row.value("fault_code", "");
    // The recording's own timestamp. Taking it from the fault gave every recording
    // of one fault the same date - and this change is what lets a fault hold more
    // than one, so the date is exactly what tells the occurrences apart. An
    // acknowledged fault is missing from the default fault listing entirely, which
    // dated its recordings 1970 while the rows were still served. Fall back to the
    // fault only for a row that predates the field.
    int64_t created_at_ns = row.value("created_at_ns", int64_t{0});
    if (created_at_ns == 0) {
      const auto key = record_map_key(row.value("owner", std::string{}), fault_code);
      if (auto it = faults_by_record.find(key); it != faults_by_record.end()) {
        const double first_occurred = it->second.value("first_occurred", 0.0);
        created_at_ns = static_cast<int64_t>(first_occurred * 1'000'000'000);
      }
    }

    if (auto found = index_by_recording.find(recording_id); found != index_by_recording.end()) {
      auto & entry = recordings[found->second];
      entry.fault_codes.push_back(fault_code);
      // Rows of one burst carry the same recording timestamp, so this only decides
      // anything on the fallback path, where each row is dated by its own fault.
      if (created_at_ns != 0 && (entry.created_at_ns == 0 || created_at_ns < entry.created_at_ns)) {
        entry.created_at_ns = created_at_ns;
      }
      continue;
    }

    RecordingEntry entry;
    entry.recording_id = recording_id;
    // Default to sqlite3 (the historical FaultManager default) when a bag predates
    // the persisted format field; the per-bag metadata normally carries the real one.
    entry.format = row.value("format", "sqlite3");
    // One metadata read per row gives the served size and the file count. Without a
    // single served file the row's own figure stands.
    const auto storage = rosbag_storage(row.value("file_path", ""));
    entry.size_bytes = rosbag_served_bytes(storage).value_or(row.value("size_bytes", uint64_t{0}));
    entry.storage_files = storage.file_count;
    entry.duration_sec = row.value("duration_sec", 0.0);
    entry.created_at_ns = created_at_ns;
    entry.manifest = row.value("manifest", false);
    entry.fault_codes.push_back(fault_code);
    index_by_recording.emplace(recording_id, recordings.size());
    recordings.push_back(std::move(entry));
  }

  std::vector<dto::BulkDataDescriptor> descriptors;
  descriptors.reserve(recordings.size());
  for (auto & entry : recordings) {
    std::sort(entry.fault_codes.begin(), entry.fault_codes.end());
    entry.fault_codes.erase(std::unique(entry.fault_codes.begin(), entry.fault_codes.end()), entry.fault_codes.end());

    dto::BulkDataDescriptor descriptor;
    descriptor.id = entry.recording_id;
    descriptor.name = entry.recording_id + " recording " + format_timestamp_ns(entry.created_at_ns);
    descriptor.mimetype = BulkDataHandlers::get_rosbag_mimetype(entry.format);
    descriptor.size = entry.size_bytes;
    descriptor.creation_date = format_timestamp_ns(entry.created_at_ns);
    descriptor.x_medkit = nlohmann::json{{"fault_codes", entry.fault_codes},
                                         {"duration_sec", entry.duration_sec},
                                         {"format", entry.format},
                                         // Redundant with the descriptor id, kept because
                                         // clients already group on it.
                                         {"recording_id", entry.recording_id}};
    if (entry.manifest) {
      // Present only when true: its absence is what a recording without a manifest looks like.
      (*descriptor.x_medkit)["manifest"] = true;
    }
    // Omitted when the bag does not say. A default of one would claim the
    // recording is whole, which is the one thing this field exists to establish.
    if (entry.storage_files) {
      (*descriptor.x_medkit)["storage_files"] = *entry.storage_files;
    }
    descriptors.push_back(std::move(descriptor));
  }
  return descriptors;
}

tl::expected<std::vector<nlohmann::json>, ErrorInfo> list_scoped_rosbag_rows(FaultManager & fault_mgr,
                                                                             const std::set<std::string> & scope) {
  // ListRosbags lists the rows of every record with a reporting source equal to the source asked
  // for or below it on path segments, the rule of faults::fault_in_source_scope. So each source of
  // the scope is asked once, and a row reached through two of them is kept once.
  std::vector<nlohmann::json> rows;
  std::set<std::string> seen;
  for (const auto & source : scope) {
    auto listed = fault_mgr.list_rosbags(source);
    if (!listed.success) {
      return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Recordings could not be listed",
                                       nlohmann::json{{"details", listed.error_message}}));
    }
    for (auto row : listed.data.value("rosbags", nlohmann::json::array())) {
      if (!row.is_object()) {
        continue;
      }
      row["owner"] = source;
      const std::string key = row.value("recording_id", std::string{}) + '\0' + row.value("file_path", std::string{}) +
                              '\0' + row.value("fault_code", std::string{});
      if (seen.insert(key).second) {
        rows.push_back(std::move(row));
      }
    }
  }
  return rows;
}

tl::expected<std::vector<dto::BulkDataDescriptor>, ErrorInfo>
list_scoped_rosbag_descriptors(FaultManager & fault_mgr, const std::set<std::string> & scope) {
  // An empty scope holds no record, so the fault manager is not asked.
  if (scope.empty()) {
    return std::vector<dto::BulkDataDescriptor>{};
  }
  // The records in scope, every status and muted ones, date a row that predates created_at_ns:
  // the earliest record of its code with a reporting source at or below the source it was listed under.
  auto faults_result = fault_mgr.list_faults("", /*include_prefailed=*/true, /*include_confirmed=*/true,
                                             /*include_cleared=*/true, /*include_healed=*/true,
                                             /*include_muted=*/true);
  if (!faults_result.success) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults",
                                     nlohmann::json{{"details", faults_result.error_message}}));
  }
  std::unordered_map<std::string, nlohmann::json> faults_by_record;
  for (const auto & fault_json : faults_result.data.value("faults", nlohmann::json::array())) {
    if (!fault_json.is_object() || !fault_json.contains("fault_code") || !fault_json["fault_code"].is_string()) {
      continue;
    }
    const auto code = fault_json["fault_code"].get<std::string>();
    const auto sources = fault_json.value("reporting_sources", nlohmann::json::array());
    for (const auto & source : scope) {
      const bool holds =
          sources.is_array() && std::any_of(sources.begin(), sources.end(), [&source](const nlohmann::json & s) {
            return s.is_string() && faults::source_in_scope(s.get<std::string>(), source);
          });
      if (!holds) {
        continue;
      }
      auto [it, inserted] = faults_by_record.emplace(record_map_key(source, code), fault_json);
      if (!inserted && fault_json.value("first_occurred", 0.0) < it->second.value("first_occurred", 0.0)) {
        it->second = fault_json;
      }
    }
  }

  auto rows = list_scoped_rosbag_rows(fault_mgr, scope);
  if (!rows) {
    return tl::unexpected(rows.error());
  }
  for (auto & row : *rows) {
    row["manifest"] = faults::recording_has_manifest(row.value("file_path", std::string{}));
  }
  return fold_rosbag_rows_into_descriptors(*rows, faults_by_record);
}

tl::expected<nlohmann::json, ErrorInfo> resolve_scoped_rosbag(FaultManager & fault_mgr,
                                                              const std::set<std::string> & scope,
                                                              const std::string & bulk_data_id,
                                                              const std::string & entity_id) {
  const auto not_for_entity = [&entity_id]() {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found for this entity", json{{"entity_id", entity_id}}));
  };

  // A recording id first. The fault manager reads an id it holds no recording under as a
  // fault code, which the entity's own records resolve below.
  auto rosbag_result = fault_mgr.get_rosbag(bulk_data_id);
  if (!rosbag_result.success && rosbag_result.failure == FaultFailure::Unavailable) {
    return tl::unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get rosbag",
                                     json{{"details", rosbag_result.error_message}}));
  }
  if (rosbag_result.success && rosbag_result.data.contains("file_path") &&
      !rosbag_resolved_by_fault_code(rosbag_result.data, bulk_data_id)) {
    // The recording is the entity's when one of the records it is attached to is in scope,
    // whatever the record's status and muted or not. A failed read is 503, not 404.
    auto held = fault_mgr.list_faults("", /*include_prefailed=*/true, /*include_confirmed=*/true,
                                      /*include_cleared=*/true, /*include_healed=*/true, /*include_muted=*/true);
    if (!held.success) {
      return tl::unexpected(
          make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get faults", json{{"details", held.error_message}}));
    }
    auto records = faults::attached_records(rosbag_result.data);
    if (records.empty()) {
      // An answer without fault_codes comes from a fault manager that was asked by fault code.
      records.push_back(faults::AttachedRecord{bulk_data_id, ""});
    }
    if (faults::attached_record_in_scope(records, held.data.value("faults", json::array()), scope)) {
      return tl::expected<json, ErrorInfo>{rosbag_result.data};
    }
    return not_for_entity();
  }

  // A fault code: the newest recording among the entity's records of that code.
  auto rows = list_scoped_rosbag_rows(fault_mgr, scope);
  if (!rows) {
    return tl::unexpected(rows.error());
  }
  std::optional<std::pair<int64_t, std::string>> newest;
  for (const auto & row : *rows) {
    if (row.value("fault_code", std::string{}) != bulk_data_id) {
      continue;
    }
    std::string recording_id = row.value("recording_id", std::string{});
    if (recording_id.empty()) {
      recording_id = rosbag_recording_id(row.value("file_path", std::string{}));
    }
    std::pair<int64_t, std::string> candidate{row.value("created_at_ns", int64_t{0}), std::move(recording_id)};
    if (!candidate.second.empty() && (!newest || candidate > *newest)) {
      newest = std::move(candidate);
    }
  }
  if (!newest) {
    return tl::unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found", json{{"bulk_data_id", bulk_data_id}}));
  }
  auto served = fault_mgr.get_rosbag(newest->second);
  if (!served.success && served.failure == FaultFailure::Unavailable) {
    return tl::unexpected(
        make_error(503, ERR_SERVICE_UNAVAILABLE, "Failed to get rosbag", json{{"details", served.error_message}}));
  }
  if (!served.success || !served.data.contains("file_path") ||
      rosbag_resolved_by_fault_code(served.data, newest->second)) {
    return not_for_entity();
  }
  return tl::expected<json, ErrorInfo>{served.data};
}

std::vector<std::string> compute_bulkdata_source_filters(const ThreadSafeEntityCache & cache,
                                                         const EntityInfo & entity) {
  // One rule for every entity type: the same fault-scope resolution that drives
  // GET /{entity}/faults (external app -> bare id, external component also owns
  // its own id, AREA recurses subareas, FUNCTION follows component hosts).
  // Rosbags are keyed by exact reporting source, so any private variant here
  // makes a fault the entity lists advertise a bag that 404s - e.g. an
  // area-scoped bulk_data_uri resolved through the area namespace never
  // matched a bag stored under a hosted app's bare id.
  auto sources = HandlerContext::resolve_entity_source_fqns(cache, entity);
  if (!sources.empty()) {
    return {sources.begin(), sources.end()};
  }

  if (entity.type == EntityType::FUNCTION) {
    // Functions are pure aggregated views over hosted apps - if no apps host
    // the function, there is nothing to query. No fall-through.
    return {};
  }

  // No resolvable hosted sources (manifest-only entities grouping topics rather
  // than nodes, or apps missing from the cache): fall back to FQN or namespace_path.
  std::string filter = entity.fqn.empty() ? entity.namespace_path : entity.fqn;
  if (filter.empty()) {
    return {};
  }
  return {filter};
}

}  // namespace detail

namespace {
/// Whether the fault manager serves the `rosbags` category. When it does not, the category is unknown.
bool rosbags_served(const HandlerContext & ctx) {
  auto * fault_mgr = ctx.services().fault_manager();
  return fault_mgr != nullptr && fault_mgr->serves_rosbags();
}
}  // namespace

// ---------------------------------------------------------------------------
// GET /{entity}/bulk-data - list categories
// ---------------------------------------------------------------------------

http::Result<dto::BulkDataCategoryList> BulkDataHandlers::list_categories(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  // Build categories list: "rosbags" when the fault manager serves them + BulkDataStore categories.
  dto::BulkDataCategoryList response;
  if (rosbags_served(ctx_)) {
    response.items.push_back("rosbags");
  }

  auto * store = ctx_.bulk_data_store();
  if (store) {
    for (const auto & cat : store->list_categories()) {
      response.items.push_back(cat);
    }
  }
  return response;
}

// ---------------------------------------------------------------------------
// GET /{entity}/bulk-data/{category_id} - list descriptors
// ---------------------------------------------------------------------------

http::Result<dto::Collection<dto::BulkDataDescriptor>>
BulkDataHandlers::list_descriptors(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  auto category = extract_bulk_data_category(req.path());
  if (category.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing category"));
  }

  if (category == "rosbags" && rosbags_served(ctx_)) {
    // === Rosbags: served via FaultManager ===
    auto source_filters = get_source_filters(entity);
    auto items = detail::list_scoped_rosbag_descriptors(
        *ctx_.services().fault_manager(), std::set<std::string>(source_filters.begin(), source_filters.end()));
    if (!items) {
      auto err = items.error();
      err.params[entity.id_field] = entity.id;
      return tl::unexpected(std::move(err));
    }
    dto::Collection<dto::BulkDataDescriptor> rosbag_response;
    rosbag_response.items = std::move(*items);
    return rosbag_response;
  }

  // === Non-rosbag categories: served via BulkDataStore ===
  auto * store = ctx_.bulk_data_store();
  if (!store || !store->is_known_category(category)) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Unknown category: " + category));
  }

  auto items_list = store->list_items(path_info->entity_id, category);
  dto::Collection<dto::BulkDataDescriptor> response;
  for (const auto & item : items_list) {
    dto::BulkDataDescriptor desc;
    desc.id = item.id;
    desc.name = item.name;
    desc.mimetype = item.mime_type;
    desc.size = item.size;
    desc.creation_date = item.created;
    if (!item.description.empty()) {
      desc.description = item.description;
    }
    if (!item.metadata.empty()) {
      desc.x_medkit = item.metadata;
    }
    response.items.push_back(std::move(desc));
  }
  return response;
}

// ---------------------------------------------------------------------------
// GET /{entity}/bulk-data/{category_id}/{file_id} - binary download
//
// Uses `reg.binary_download`: the framework wires the provider callback into
// cpp-httplib's range-aware content provider, sets `Content-Disposition`
// from `filename`, and propagates the typed `ErrorInfo` for failures. The
// handler stays free of `httplib::Response`.
// ---------------------------------------------------------------------------

http::Result<http::BinaryResponse> BulkDataHandlers::download(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  auto category = extract_bulk_data_category(req.path());
  auto bulk_data_id = extract_bulk_data_id(req.path());

  if (bulk_data_id.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing bulk-data ID"));
  }

  // Resolve the actual on-disk file path and the wire metadata (mimetype +
  // filename) once, branching on rosbag vs user-uploaded categories. The
  // downstream BinaryResponse assembly is identical for both branches.
  std::string actual_path;
  std::string mimetype;
  std::string filename;

  if (category == "rosbags" && rosbags_served(ctx_)) {
    // === Rosbags: served via FaultManager ===
    //
    // The id is a recording id, or a fault code on the address that predates recording
    // ids, which serves the newest recording among the entity's records of that code.
    auto source_filters = get_source_filters(entity);
    auto served = detail::resolve_scoped_rosbag(*ctx_.services().fault_manager(),
                                                std::set<std::string>(source_filters.begin(), source_filters.end()),
                                                bulk_data_id, path_info->entity_id);
    if (!served) {
      return tl::unexpected(served.error());
    }
    const json & rosbag_data = *served;

    std::string file_path = rosbag_data.at("file_path").get<std::string>();
    // Default to sqlite3 (the historical FaultManager default) for bags predating the
    // format field; metadata normally carries the real one persisted at capture time.
    std::string format = rosbag_data.value("format", "sqlite3");
    mimetype = get_rosbag_mimetype(format);
    // Named after the recording that was actually served, which for a compatibility
    // URL is not the segment the client sent.
    filename = rosbag_data.value("recording_id", bulk_data_id) + "." + format;

    // Only the storage file is served, not metadata.yaml. When the listing size differs from
    // Content-Length, see "One recording, one size" in docs/api/rest.rst.
    actual_path = resolve_rosbag_file_path(file_path);
  } else {
    // === Non-rosbag categories: served via BulkDataStore ===
    auto * store = ctx_.bulk_data_store();
    if (!store || !store->is_known_category(category)) {
      return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Unknown category: " + category));
    }

    auto stored_path = store->get_file_path(path_info->entity_id, category, bulk_data_id);
    if (!stored_path) {
      return tl::unexpected(
          make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data not found", json{{"bulk_data_id", bulk_data_id}}));
    }
    actual_path = *stored_path;

    auto item = store->get_item(path_info->entity_id, category, bulk_data_id);
    filename = item ? item->name : bulk_data_id;
    mimetype = item ? item->mime_type : "application/octet-stream";
  }

  if (actual_path.empty()) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to read bulk-data file"));
  }

  // One descriptor serves every range, so the bytes cannot change between the check and the read.
  // O_NOFOLLOW refuses a file swapped for a symlink after resolving; O_NONBLOCK keeps a FIFO from
  // blocking the open, and fstat then rejects it.
  const int fd = ::open(actual_path.c_str(), O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to read bulk-data file"));
  }
  const std::shared_ptr<int> file(new int(fd), [](const int * owned) {
    ::close(*owned);
    delete owned;
  });
  struct stat file_stat {};
  if (::fstat(*file, &file_stat) != 0 || !S_ISREG(file_stat.st_mode)) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Failed to read bulk-data file"));
  }
  const auto file_size = static_cast<uint64_t>(file_stat.st_size);

  // Sanitise the filename for the Content-Disposition header. The framework
  // emits `attachment; filename="<sanitised>"`; embedded quotes are mapped to
  // underscores to preserve the legacy header-safety contract.
  std::string safe_name = filename;
  std::replace(safe_name.begin(), safe_name.end(), '"', '_');

  http::BinaryResponse resp;
  resp.content_type = mimetype;
  resp.filename = safe_name;
  resp.supports_ranges = true;
  resp.total_size = file_size;
  // 64 KB chunks, matching the legacy `stream_file_to_response` block size.
  static constexpr std::size_t kChunkSize = 64 * 1024;
  // cpp-httplib calls the provider on worker threads, once per range. pread keeps no file offset,
  // so concurrent calls on the shared descriptor are safe.
  resp.provider = [file](uint64_t offset, uint64_t length, httplib::DataSink & sink) -> bool {
    uint64_t remaining = length;
    uint64_t position = offset;
    std::vector<char> buf(std::min<uint64_t>(remaining, kChunkSize));
    while (remaining > 0) {
      const auto to_read = static_cast<std::size_t>(std::min<uint64_t>(remaining, kChunkSize));
      const ssize_t bytes_read = ::pread(*file, buf.data(), to_read, static_cast<off_t>(position));
      if (bytes_read < 0 && errno == EINTR) {
        continue;
      }
      if (bytes_read <= 0) {
        break;
      }
      sink.write(buf.data(), static_cast<std::size_t>(bytes_read));
      remaining -= static_cast<uint64_t>(bytes_read);
      position += static_cast<uint64_t>(bytes_read);
    }
    return remaining == 0;
  };
  return resp;
}

// ---------------------------------------------------------------------------
// POST /{entity}/bulk-data/{category_id} - multipart upload (201 + Location)
// ---------------------------------------------------------------------------

http::Result<std::pair<http::Created<dto::BulkDataDescriptor>, http::ResponseAttachments>>
BulkDataHandlers::upload(const http::TypedRequest & req, const http::MultipartBody & body) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  // Check lock access for bulk-data (typed validator returns ErrorInfo directly).
  if (auto lock_err = ctx_.validate_lock_access(req, entity, "bulk-data"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  auto category = extract_bulk_data_category(req.path());
  if (category.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing category"));
  }

  // Rosbags are managed by the fault system, not user-uploadable.
  if (category == "rosbags") {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Category 'rosbags' does not support upload. "
                                     "Rosbags are managed by the fault system."));
  }

  auto * store = ctx_.bulk_data_store();
  if (store == nullptr) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Bulk data storage not configured"));
  }

  if (!store->is_known_category(category)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Unknown bulk-data category: " + category));
  }

  // Locate the `file`, `description`, and `metadata` parts. cpp-httplib parses
  // every named part into MultipartBody.parts; walk the vector instead of
  // relying on `req.files` map ordering so the typed surface does not leak
  // through to the cpp-httplib shape.
  // FIXME(#409): httplib::MultipartFormData is unavailable in cpp-httplib
  // >= 0.20; migrate to the req.form API (tracked with the vendored-header pin).
  const httplib::MultipartFormData * file_part = nullptr;
  const httplib::MultipartFormData * description_part = nullptr;
  const httplib::MultipartFormData * metadata_part = nullptr;
  for (const auto & part : body.parts) {
    if (part.name == "file" && !file_part) {
      file_part = &part;
    } else if (part.name == "description" && !description_part) {
      description_part = &part;
    } else if (part.name == "metadata" && !metadata_part) {
      metadata_part = &part;
    }
  }

  if (!file_part) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing 'file' field in multipart/form-data request"));
  }

  std::string filename = file_part->filename.empty() ? "upload" : file_part->filename;
  std::string content_type = file_part->content_type.empty() ? "application/octet-stream" : file_part->content_type;

  // Enforce the configured maximum upload size (0 = unbounded).
  if (store->max_upload_bytes() > 0 && file_part->content.size() > store->max_upload_bytes()) {
    return tl::unexpected(make_error(413, ERR_PAYLOAD_TOO_LARGE, "File size exceeds maximum upload limit"));
  }

  std::string description;
  if (description_part) {
    description = description_part->content;
  }

  json metadata = json::object();
  if (metadata_part && !metadata_part->content.empty()) {
    auto parsed = json::parse(metadata_part->content, nullptr, false);
    if (parsed.is_discarded()) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Invalid JSON in 'metadata' field"));
    }
    if (!parsed.is_object()) {
      return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "metadata must be a JSON object"));
    }
    metadata = std::move(parsed);
  }

  auto result =
      store->store(path_info->entity_id, category, filename, content_type, file_part->content, description, metadata);
  if (!result) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, result.error()));
  }

  const auto & stored = *result;
  dto::BulkDataDescriptor descriptor;
  descriptor.id = stored.id;
  descriptor.name = stored.name;
  descriptor.mimetype = stored.mime_type;
  descriptor.size = stored.size;
  descriptor.creation_date = stored.created;
  if (!stored.description.empty()) {
    descriptor.description = stored.description;
  }
  if (!stored.metadata.empty()) {
    descriptor.x_medkit = stored.metadata;
  }

  http::ResponseAttachments att;
  att.with_location(child_resource_path(req.path(), stored.id));
  return std::make_pair(http::Created<dto::BulkDataDescriptor>{std::move(descriptor)}, std::move(att));
}

// ---------------------------------------------------------------------------
// DELETE /{entity}/bulk-data/{category_id}/{file_id} - 204 No Content
// ---------------------------------------------------------------------------

http::Result<http::NoContent> BulkDataHandlers::remove(const http::TypedRequest & req) {
  auto path_info = parse_path(req);
  if (!path_info) {
    return tl::unexpected(path_info.error());
  }

  auto entity_result = ctx_.validate_entity_for_route(req, path_info->entity_id);
  if (!entity_result) {
    return tl::unexpected(flatten_validator_error(entity_result.error()));
  }
  const auto & entity = *entity_result;

  if (auto access = HandlerContext::validate_collection_access_typed(entity, ResourceCollection::BULK_DATA); !access) {
    return tl::unexpected(make_error(400, ERR_COLLECTION_NOT_SUPPORTED, access.error().message));
  }

  if (auto lock_err = ctx_.validate_lock_access(req, entity, "bulk-data"); !lock_err) {
    return tl::unexpected(lock_err.error());
  }

  auto category = extract_bulk_data_category(req.path());
  if (category.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing category"));
  }

  // Rosbags are managed by the fault system, not user-deletable.
  if (category == "rosbags") {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER,
                                     "Category 'rosbags' does not support deletion. "
                                     "Rosbags are managed by the fault system."));
  }

  auto item_id = extract_bulk_data_id(req.path());
  if (item_id.empty()) {
    return tl::unexpected(make_error(400, ERR_INVALID_REQUEST, "Missing bulk-data ID"));
  }

  auto * store = ctx_.bulk_data_store();
  if (store == nullptr) {
    return tl::unexpected(make_error(500, ERR_INTERNAL_ERROR, "Bulk data storage not configured"));
  }

  if (!store->is_known_category(category)) {
    return tl::unexpected(make_error(400, ERR_INVALID_PARAMETER, "Unknown bulk-data category: " + category));
  }

  auto result = store->remove(path_info->entity_id, category, item_id);
  if (!result) {
    return tl::unexpected(make_error(404, ERR_RESOURCE_NOT_FOUND, "Bulk-data item not found"));
  }

  return http::NoContent{};
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
