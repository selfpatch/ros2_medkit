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

#include <cstddef>
#include <cstdint>
#include <optional>
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

  /**
   * @brief Resolve rosbag file path from storage path.
   *
   * Rosbag2 creates a directory containing the actual db3/mcap file.
   * This function resolves the directory to the actual file path. A path that is
   * already a regular file is returned unchanged.
   *
   * The single place that decides which bytes a recording IS, which is why it is
   * reachable from outside the class rather than being a private helper of the
   * download path. `download()` streams the file this returns and reports its
   * length, and `detail::rosbag_served_bytes` sizes the listing through it, so a
   * change to which file a recording resolves to moves both at once.
   *
   * That does not make the two numbers equal in every case, and since the split
   * fix it deliberately does not. For a recording held in one storage file the
   * listing resolves through here and reports exactly what the download sends.
   * For a recording split across several files the listing does not come through
   * here at all: it carries the recording's total from the fault manager while
   * this route still hands over one file, and the gap is what tells a client the
   * transfer is partial, alongside ``x-medkit.storage_files`` in the descriptor.
   * See the size rule in ``docs/api/rest.rst``.
   *
   * The bag's own ``metadata.yaml`` decides, and when it has decided nothing
   * else gets a vote. The answer is the first file named in
   * ``relative_file_paths`` that is on disk. That order is the capture order, so
   * for a split recording the first name is where the recording begins and a
   * client that fetches one gets its start rather than an arbitrary slice. A
   * name that is not on disk is skipped, because a half-copied bag leaves
   * metadata naming a file that is gone and failing the request there would cost
   * a recording whose other segments are readable. A name that is absolute or
   * climbs through ``..`` is skipped as well: it would resolve outside the bag
   * directory and put a file that is not part of the recording on the wire.
   *
   * When the bag named files and none of them is on disk the answer is the empty
   * string. Falling through to the directory there served whatever ``.db3`` or
   * ``.mcap`` sat beside the recording, under this recording's id and against a
   * ``storage_files`` count the served file is not a member of.
   *
   * Only when the bag will not say what it holds - no ``metadata.yaml``, one
   * this process cannot read or parse, or a ``relative_file_paths`` naming
   * nothing - is the directory scanned for the first ``.db3`` or ``.mcap`` in
   * whatever order it yields. That is the same set of shapes
   * ``detail::rosbag_storage_file_count`` declines, which is the invariant: the
   * count and the served file are both read from the metadata, or both from the
   * directory, never one from each.
   *
   * @param path Path to rosbag (can be file or directory)
   * @return Resolved file path, or empty string if not found
   */
  static std::string resolve_rosbag_file_path(const std::string & path);

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
 * @brief Fault codes a rosbag download is authorized against.
 *
 * A recording is shared by every fault of a burst, so ownership is the union
 * over those faults rather than a single code: the entity that owns any one of
 * them may download the bag. That grants nothing new - before recordings had
 * their own identity, each of those faults already addressed its own copy of
 * the same bytes - it only renames the door.
 *
 * When the wire carries no ``fault_codes`` the response came from a peer that
 * predates the field, where the addressed id *was* the fault code; authorizing
 * against the requested id then reproduces the previous check exactly.
 *
 * @param rosbag_data Rosbag response from the fault manager
 * @param requested_id The ``{file_id}`` path segment the client asked for
 * @return Non-empty list of fault codes to test against the entity's scope
 */
std::vector<std::string> rosbag_attached_fault_codes(const nlohmann::json & rosbag_data,
                                                     const std::string & requested_id);

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
 * @brief How many storage files one recording is held in.
 *
 * Read from the bag's own ``metadata.yaml``, the same field
 * ``BulkDataHandlers::resolve_rosbag_file_path`` picks the served file out of,
 * so the count and the choice of segment cannot describe different recordings.
 * A @p bag_path that is itself a storage file is one by definition and carries
 * no metadata beside it under that name to consult.
 *
 * It reaches the client as ``x-medkit.storage_files`` on the rosbag descriptor,
 * and what it is for is the split case. There the descriptor ``size`` is the
 * whole recording while the download hands over one segment, so ``size`` and
 * ``Content-Length`` differ. Without this field that difference has no stated
 * reason, and a client holding one segment has no way to learn that the rest of
 * the recording exists. A ``1`` says the transfer was the whole recording.
 *
 * nullopt when the bag will not say - no metadata, unreadable metadata, not the
 * shape rosbag2 writes, or a ``relative_file_paths`` naming nothing - and the
 * field is then omitted from the descriptor rather than defaulted. Counting the
 * directory's ``.db3`` / ``.mcap`` files instead would count a stray beside the
 * recording, and defaulting to one would claim a recording is whole on the
 * evidence of nothing. A list naming nothing is declined rather than reported as
 * zero for the same reason: the recording is not empty, the bag did not answer.
 *
 * The set of shapes declined here is exactly the set on which
 * ``BulkDataHandlers::resolve_rosbag_file_path`` falls back to the directory, so
 * the count and the served file are read from the metadata together or from the
 * directory together. A count taken from one source describing a file chosen by
 * the other is the state this pairing exists to make unreachable.
 *
 * Never throws, for the same reason as the two helpers around it: it runs once
 * per row of a listing, and one unreadable recording must not cost the entity's
 * other recordings.
 *
 * @param bag_path Bag path as stored by the fault manager. A bag directory, or
 *                 a bare storage file, which both answer
 * @return The number of storage files the recording names, or nullopt when the
 *         bag's metadata cannot be read
 */
std::optional<std::size_t> rosbag_storage_file_count(const std::string & bag_path);

/**
 * @brief Bytes a rosbag download puts on the wire for one recording.
 *
 * Answers only for a recording held in a single storage file, which is the only
 * shape where one number describes the transfer. That is a bag directory whose
 * ``metadata.yaml`` names one file, or a @p bag_path that is itself a storage
 * file, which is one by definition and carries no metadata to consult.
 * ``BulkDataHandlers::resolve_rosbag_file_path`` picks that file and the
 * download streams it alone, so the length a client is told to expect is that
 * file's length and nothing else. Reporting the bag directory's total instead
 * overstated every download by ``metadata.yaml`` - on a short recording, by
 * around a tenth of the transfer - and a client sizing a buffer or a progress
 * bar from the listing never reached the end.
 *
 * Returns nullopt for anything else, and the caller then keeps the row's own
 * figure. That covers a bag this process cannot see or read at all, and it
 * covers a recording split across several storage files past the configured
 * maximum bag size: the download hands over one segment, so no single file is
 * the transfer, and answering with the segment it hands over advertised a split
 * recording at the size of one part of it. The row's figure
 * is the fault manager's answer to the same question, decided from the same
 * ``metadata.yaml``, so deferring to it keeps the two API surfaces agreeing on
 * one recording.
 *
 * Never throws and never reports a filesystem error upwards. It runs once per
 * row of a listing, and an error here is one unreadable recording, not a failed
 * request for the entity's other recordings.
 *
 * @param bag_path Bag path as stored by the fault manager. A bag directory, or
 *                 a bare storage file, which both answer
 * @return The single storage file's size, or nullopt when there is not exactly
 *         one, or when this process cannot see it
 */
std::optional<uint64_t> rosbag_served_bytes(const std::string & bag_path);

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
 * The descriptor size is measured on the file the download resolves (see
 * ``rosbag_served_bytes``), not taken from the row. A row whose bag this
 * process cannot see keeps the row's own figure: it is the only number left,
 * and a recording listed with a zero size reads as an empty one.
 *
 * ``x-medkit.storage_files`` carries how many files the recording is held in
 * (see ``rosbag_storage_file_count``), and is omitted for a bag whose metadata
 * this process cannot read.
 *
 * @param rows Rosbag rows as returned by the fault manager
 * @param faults_by_code Faults keyed by code, for timestamp enrichment
 * @return One descriptor per distinct recording
 */
std::vector<dto::BulkDataDescriptor>
fold_rosbag_rows_into_descriptors(const std::vector<nlohmann::json> & rows,
                                  const std::unordered_map<std::string, nlohmann::json> & faults_by_code);

}  // namespace detail

}  // namespace handlers
}  // namespace ros2_medkit_gateway
