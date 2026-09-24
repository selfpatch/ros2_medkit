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

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/dto/bulkdata.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief HTTP handlers for SOVD bulk-data endpoints.
 *
 * PR-403 commit 25: 11 bulk-data routes migrated to the typed RouteRegistry
 * API. Every handler returns `http::Result<T>` (or a `pair<T,
 * ResponseAttachments>` for the upload route that needs to emit 201 +
 * Location). The download route uses `reg.binary_download` so it can emit
 * `Content-Disposition`, set `supports_ranges`, and supply a chunked content
 * provider without touching `httplib::Response`. The upload route uses
 * `reg.multipart_upload<BulkDataDescriptor>` so multipart parsing remains
 * inside the framework while the handler stays typed. Wire format is
 * unchanged byte-for-byte, including the rosbag MIME-type-by-format mapping
 * and the Content-Disposition filename sanitisation.
 *
 * Supports SOVD entity paths:
 * - /apps/{id}/bulk-data[/{category}[/{id}]]
 * - /components/{id}/bulk-data[/{category}[/{id}]]
 * - /areas/{id}/bulk-data[/{category}[/{id}]]
 * - /functions/{id}/bulk-data[/{category}[/{id}]]
 * - Nested entities (subareas, subcomponents)
 */
class BulkDataHandlers {
 public:
  /**
   * @brief Construct BulkDataHandlers.
   * @param ctx Handler context for sending responses and accessing FaultManager
   */
  explicit BulkDataHandlers(HandlerContext & ctx);

  /// GET /{entity}/bulk-data - list bulk-data categories.
  http::Result<dto::BulkDataCategoryList> list_categories(const http::TypedRequest & req);

  /// GET /{entity}/bulk-data/{category_id} - list bulk-data descriptors.
  http::Result<dto::Collection<dto::BulkDataDescriptor>> list_descriptors(const http::TypedRequest & req);

  /// GET /{entity}/bulk-data/{category_id}/{file_id} - binary download.
  http::Result<http::BinaryResponse> download(const http::TypedRequest & req);

  /// POST /{entity}/bulk-data/{category_id} - multipart upload, 201 + Location.
  http::Result<std::pair<http::Created<dto::BulkDataDescriptor>, http::ResponseAttachments>>
  upload(const http::TypedRequest & req, const http::MultipartBody & body);

  /// DELETE /{entity}/bulk-data/{category_id}/{file_id} - 204 No Content.
  http::Result<http::NoContent> remove(const http::TypedRequest & req);

  /**
   * @brief Get MIME type for rosbag format.
   * @param format Storage format ("mcap", "sqlite3", "db3")
   * @return MIME type string
   */
  static std::string get_rosbag_mimetype(const std::string & format);

  /**
   * @brief Media types `download()` can put on the wire, for the OpenAPI
   *        document to declare on the six binary-download routes.
   *
   * Lives here rather than at the registration because this is the file that
   * decides the value: the concrete types are exactly the range of
   * get_rosbag_mimetype(), and a registration cannot see through the handler
   * to find them.
   *
   * The list ends with `*&#47;*` and that entry is load-bearing, not filler. A
   * non-rosbag category serves BulkDataStore::ItemDescriptor::mime_type, which
   * is whatever the uploading client put on its multipart part
   * (bulk_data_store.cpp, `mime_type = content_type.empty() ? ... :
   * content_type`). Uploading a `text/csv` makes the download serve
   * `text/csv`, so the served set is open and no finite list is truthful.
   * Declaring only the three concrete types would under-declare the route -
   * the defect this document's derivation exists to remove - and declaring
   * only the catch-all would throw away the part that IS derivable.
   *
   * @return Concrete rosbag media types followed by the `*&#47;*` catch-all.
   */
  static std::vector<std::string> download_media_types();

 private:
  HandlerContext & ctx_;

  /**
   * @brief Get source filters for rosbag queries based on entity type.
   *
   * Thin instance wrapper that fetches the cache from ctx_ and delegates to
   * detail::compute_bulkdata_source_filters. The pure logic (entity-type
   * branching) is unit-tested via the free function instead of the member
   * to keep the handler's public surface unchanged.
   */
  std::vector<std::string> get_source_filters(const EntityInfo & entity) const;

  /**
   * @brief Resolve rosbag file path from storage path.
   *
   * Rosbag2 creates a directory containing the actual db3/mcap file.
   * This function resolves the directory to the actual file path.
   *
   * @param path Path to rosbag (can be file or directory)
   * @return Resolved file path, or empty string if not found
   */
  static std::string resolve_rosbag_file_path(const std::string & path);
};

namespace detail {

/**
 * @brief Compute rosbag source filters for an entity based on its type.
 *
 * Pure helper that drives ``BulkDataHandlers::get_source_filters``. Lives in
 * a ``detail`` namespace to signal "not part of the public API" while still
 * being directly unit-testable without spinning up a ``GatewayNode``.
 *
 * Every entity type resolves through ``faults::resolve_entity_source_fqns``,
 * the same rule that scopes ``GET /{entity}/faults``: an external app owns
 * its bare entity id, every other app its ``effective_fqn()``; an external
 * component also owns its own id; AREA recurses subareas; FUNCTION follows
 * app and component hosts. When resolution yields nothing, APP / AREA /
 * COMPONENT fall back to the entity's FQN or namespace path (manifest-only
 * deployments grouping topics rather than nodes); FUNCTION never falls back
 * (pure aggregated view).
 *
 * @param cache Entity cache to resolve hosted apps in (used for FUNCTION /
 *              COMPONENT only)
 * @param entity Entity information
 * @return Vector of source filter strings (empty if no valid filters)
 */
std::vector<std::string> compute_bulkdata_source_filters(const ThreadSafeEntityCache & cache,
                                                         const EntityInfo & entity);

/**
 * @brief Identity of a rosbag recording, derived from its path.
 *
 * The bag directory basename (e.g. ``fault_MOTOR_OVERHEAT_1738662600000``) is
 * the recording's public name: it addresses the bag under
 * ``/bulk-data/rosbags/{id}`` and groups the link rows that serve the same
 * bytes. The fault manager stores the same value; this derivation is the
 * fallback for a peer or a replay that predates the stored field. Empty when
 * the path is empty or has no usable basename.
 *
 * @param file_path Bag path as stored by the fault manager (directory)
 * @return Basename of the bag directory, or empty string
 */
std::string rosbag_recording_id(const std::string & file_path);

/**
 * @brief Fault codes of the records a rosbag recording is attached to.
 *
 * A recording is shared by every fault of a burst, so it belongs to several
 * records, each a (fault code, owner) pair. These are the codes of those
 * records. The download authorizes on the pairs, not on the codes: the codes
 * only say which of the entity's records to ask about, and
 * ``rosbag_rows_hold_recording`` says whether one of those records' owners
 * really holds the recording. A code alone would let an entity owning any
 * record of the code download another owner's recording of it.
 *
 * When the wire carries no ``fault_codes`` the response came from a peer that
 * predates the field, where the addressed id *was* the fault code, so the
 * requested id stands in for it.
 *
 * @param rosbag_data Rosbag response from the fault manager
 * @param requested_id The ``{file_id}`` path segment the client asked for
 * @return Non-empty list of fault codes whose in-scope records to check
 */
std::vector<std::string> rosbag_attached_fault_codes(const nlohmann::json & rosbag_data,
                                                     const std::string & requested_id);

/**
 * @brief Does one owner's rosbag listing hold the recording that was served?
 *
 * ListRosbags answers the rows whose record is owned by exactly the source it
 * was asked for, one row per (fault code, recording) link. A row naming the
 * served recording therefore proves that a record of that owner is attached to
 * it. A row names it by the same recording id, or, when either side predates
 * recording ids, by the same bag path.
 *
 * @param rows The ``rosbags`` array of one owner's ListRosbags answer
 * @param served Rosbag response from the fault manager for the download
 * @return True when a row names the served recording
 */
bool rosbag_rows_hold_recording(const nlohmann::json & rows, const nlohmann::json & served);

/**
 * @brief Did the fault manager read the URL segment as a FAULT CODE rather than a
 *        recording id?
 *
 * True on the pre-#620 compatibility path, where the segment named a fault and the
 * answer is that fault's newest recording. It matters for authorization: a burst
 * shares one bag, so the union over attached faults would answer 200 for a fault
 * code the entity does not own. The bytes are ones it could already fetch under its
 * own code, but the 200 itself discloses that another fault shares its recording.
 * On this path the requested code has to be in scope as well.
 *
 * @param rosbag_data Rosbag response from the fault manager
 * @param requested_id The ``{file_id}`` path segment the client asked for
 * @return True when the resolved recording is not the id that was asked for
 */
bool rosbag_resolved_by_fault_code(const nlohmann::json & rosbag_data, const std::string & requested_id);

/**
 * @brief Fold rosbag link rows into one descriptor per recording.
 *
 * The fault manager returns one row per ``(fault, recording)`` link, so a burst
 * of correlated faults arrives as several rows naming one bag, and one fault
 * can name several bags. Emitting one descriptor per row would repeat an id and
 * report the full bag size once per attached fault, which reads as several bags
 * worth of storage. Rows are therefore grouped by recording, the attached codes
 * collected into ``x-medkit.fault_codes`` (sorted, so the output is stable), and
 * the recording dated by the earliest fault of its burst. Rows with neither a
 * recording id nor a usable path are dropped - nothing could address them.
 *
 * Order follows first appearance, which is the order the fault manager listed
 * the rows in.
 *
 * @param rows Rosbag rows as returned by the fault manager, each stamped with
 *        the ``owner`` it was listed under
 * @param faults_by_record Fault records keyed by ``record_map_key``, for
 *        timestamp enrichment
 * @return One descriptor per distinct recording
 */
std::vector<dto::BulkDataDescriptor>
fold_rosbag_rows_into_descriptors(const std::vector<nlohmann::json> & rows,
                                  const std::unordered_map<std::string, nlohmann::json> & faults_by_record);

/**
 * @brief Key one fault record for the descriptor date lookup.
 *
 * A record is the pair (reporting source, fault code), and the two owners of
 * one code have their own ``first_occurred``. Keying the lookup on the code
 * alone let whichever record was listed last date the other's recordings.
 */
std::string record_map_key(const std::string & owner, const std::string & fault_code);

}  // namespace detail

}  // namespace handlers
}  // namespace ros2_medkit_gateway
