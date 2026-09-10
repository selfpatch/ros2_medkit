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

#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/core/entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Zero-config freeze-frames for plugin-backed entities.
 *
 * Plugin entities (PLC apps bridged by protocol plugins) report faults under
 * their bare SOVD entity id, and their live values are not ROS topics - the
 * fault_manager's snapshot capture can never freeze-frame them. This class
 * closes that gap on the gateway side: it subscribes to fault_manager events
 * and, when a fault confirms with a plugin-owned reporting source, snapshots
 * that entity's current data values (DataProvider::list_data, which serves
 * from the plugin's latest polled values) at fault time.
 *
 * The frames are merged into the fault detail's environment_data.snapshots
 * only when the fault_manager captured no freeze-frame itself - a configured
 * freeze-frame always wins, while a rosbag-only capture does not suppress the
 * entity frames (see FaultHandlers::merge_entity_freeze_frames).
 *
 * Retention mirrors the fault_manager's freeze-frame semantics: frames are
 * kept across EVENT_CLEARED (the confirmed-state record stays attached to
 * the cleared fault's detail) and overwritten on every EVENT_CONFIRMED, so
 * a re-occurrence re-samples the plugin at its own confirm time.
 *
 * With an EntityFreezeFrameStore the frames also survive the process. Without
 * one a restart loses them, and the startup catch-up then re-reads the plant
 * as it is *now* and stamps that re-read with the restart - the values at
 * fault time are gone. With one, the stored frame is reloaded before the
 * catch-up and served exactly as it was captured, and the catch-up re-reads
 * only the faults that have no frame.
 */
class EntityFreezeFrameCapture {
 public:
  /// Capture-path identifiers stored in Frame::source and served as
  /// ``x-medkit.source``. The plugin's own DataProvider, and the in-process
  /// dispatch of the plugin's `x-plc-data` route for plugins that export no
  /// DataProvider.
  static constexpr const char * kSourceDataProvider = "plugin_data_provider";
  static constexpr const char * kSourceXPlcDataRoute = "plugin_x_plc_data_route";

  /// Persisted marker for a startup catch-up frame, served as
  /// ``x-medkit.capture_origin``. Stored so a reloaded frame keeps saying
  /// which clock its captured_at came from: the property is intrinsic to the
  /// frame, so a restart must not launder a catch-up frame into a confirm-edge
  /// one. A confirm-edge frame stores the empty string and stays unmarked.
  static constexpr const char * kCaptureOriginStartup = "startup";

  /// One captured frame: the entity's data values at fault-confirm time.
  /// captured_at_ns dates the capture, not the values - a disconnected entity
  /// serves its last known values, whose age is bounded only by the outage.
  struct Frame {
    std::string entity_id;
    nlohmann::json values;  ///< compact {resource_id: value} dict
    int64_t captured_at_ns{0};
    /// True when the frame comes from the startup catch-up: captured_at_ns is
    /// then the gateway's start, possibly long after the fault confirmed.
    bool startup_catchup{false};
    std::optional<bool> connected;    ///< payload's top-level link flag, when reported
    nlohmann::json source_timestamp;  ///< payload's own "timestamp" field verbatim (null when absent)
    /// Which capture path read the values (kSourceDataProvider /
    /// kSourceXPlcDataRoute), served as ``x-medkit.source``. These values are
    /// entity data, not a ROS message, so ``topic`` and ``message_type`` are
    /// empty on the wire and would otherwise leave a consumer with nothing at
    /// all saying where the numbers came from. Empty when the caller named no
    /// path.
    std::string source;
  };

  /// Resolves an entity id to its owning plugin's DataProvider (nullptr when
  /// the entity is not plugin-owned). Called from the internal capture thread.
  using DataProviderResolver = std::function<DataProvider *(const std::string & entity_id)>;

  /// Fallback for plugins without a DataProvider: fetches the entity's current
  /// values by dispatching the owning plugin's own `x-plc-data` route
  /// in-process (typically wraps PluginManager::fetch_entity_data_via_route).
  /// Returns the parsed route response, nullopt when unavailable. Called from
  /// the internal capture thread, concurrently with HTTP threads serving the
  /// same route - dispatched handlers must be thread-safe.
  using RouteDataFetcher = std::function<std::optional<nlohmann::json>(const std::string & entity_id)>;

  /// One standing fault: its code and the entities that reported it.
  struct StandingFault {
    std::string fault_code;
    std::vector<std::string> reporting_sources;
    /// When THIS occurrence of the fault started, from the list reply's
    /// `first_occurred` (seconds on the wire, nanoseconds here). The
    /// fault_manager resets it only when a CLEARED fault reactivates, so it is
    /// what tells a stored frame from a previous occurrence apart from one
    /// belonging to the occurrence being served now. `last_occurred` cannot do
    /// this: it moves on every report, so a fault that keeps failing would look
    /// re-occurred at every restart. 0 when the reply did not report it, which
    /// reads as "cannot tell" and keeps the stored frame.
    int64_t first_occurred_ns{0};
  };

  /// Lists the faults that are already confirmed when this object starts, so
  /// their missed confirm edge can still be framed. Called once, on the capture
  /// thread (it may block on a service becoming available), never after the
  /// destructor has joined that thread - so it must not outlive its owner. Any
  /// blocking wait inside must poll should_abort and bail when it turns true:
  /// that is what lets the destructor's join interrupt the wait.
  using StandingFaultLister = std::function<std::vector<StandingFault>(const std::function<bool()> & should_abort)>;

  /// Reports every fault code the fault_manager still holds, whatever its
  /// status, so a reloaded frame for a fault that is gone can be dropped.
  /// Returns nullopt when the fault_manager could not be asked - the caller
  /// then drops nothing, because "could not tell" must never read as "gone".
  /// Called once, on the capture thread, right after the standing-fault
  /// lister has already waited for the services.
  using KnownFaultCodeLister =
      std::function<std::optional<std::unordered_set<std::string>>(const std::function<bool()> & should_abort)>;

  /**
   * @param node ROS 2 node used to resolve the fault-events topic name and logger
   * @param exec shared subscription executor; the fault-events subscription is
   *        created and torn down on its serial worker (issue #375 invariant).
   *        The subscription callback only enqueues - plugin calls run on a
   *        dedicated capture thread so a slow read never stalls that worker.
   * @param resolver entity-to-DataProvider resolver (typically wraps PluginManager)
   * @param route_fetcher x-plc-data route fallback for entities whose plugin
   *        has no DataProvider (the commercial PLC bridges); may be null
   * @param max_faults retained-frame bound; oldest fault's frames evicted past it.
   *        Counts reloaded and freshly captured frames together.
   * @param standing_lister lists the faults already confirmed at startup
   * @param store frame persistence; null keeps the frames in process memory only
   * @param known_code_lister reports the fault codes the fault_manager still
   *        holds, so reloaded frames for faults that are gone can be dropped;
   *        null keeps every reloaded frame
   */
  EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                           DataProviderResolver resolver, RouteDataFetcher route_fetcher = nullptr,
                           size_t max_faults = 256, StandingFaultLister standing_lister = nullptr,
                           std::shared_ptr<EntityFreezeFrameStore> store = nullptr,
                           KnownFaultCodeLister known_code_lister = nullptr);

  ~EntityFreezeFrameCapture();

  // Non-copyable, non-movable
  EntityFreezeFrameCapture(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture & operator=(const EntityFreezeFrameCapture &) = delete;
  EntityFreezeFrameCapture(EntityFreezeFrameCapture &&) = delete;
  EntityFreezeFrameCapture & operator=(EntityFreezeFrameCapture &&) = delete;

  /// Frames captured for a fault code (empty when none). Thread-safe.
  std::vector<Frame> frames_for(const std::string & fault_code) const;

  /// Build the compact {resource_id: value} dict from a DataProvider::list_data
  /// response. Items without a "value" field map to null; a response without an
  /// "items" array is kept verbatim (plugin-defined shape). Total: malformed
  /// items (non-string id/name) are skipped, never thrown on.
  static nlohmann::json values_from_list_content(const nlohmann::json & content);

  /// True when list-data-shaped content (DataProvider::list_data or x-plc-data
  /// route) carries values at all: a non-empty items array. The entity's
  /// `connected` flag is deliberately not consulted - a bridge serving its last
  /// known values while the link is down is the case a loss-of-comms fault most
  /// needs frozen. The row of nulls a cold cache yields is rejected by
  /// values_have_data() instead.
  static bool content_has_live_data(const nlohmann::json & content);

  /// True when the payload's top-level `connected` flag reports the link down.
  /// Consulted only by the gateway's fault-trigger value fetcher: a down link
  /// serves frozen last-known values, and a threshold rule must hold state on
  /// them (fetcher yields nullopt) instead of firing on a stale number for the
  /// whole outage. The freeze-frame paths deliberately ignore this flag.
  static bool content_reports_disconnected(const nlohmann::json & content);

  /// True when a compact values dict holds at least one non-null value.
  /// Rejects the {} / all-null rows a cold poll cache or dead link yields.
  static bool values_have_data(const nlohmann::json & values);

  /// Parse a fault-transport ListFaults reply body into standing faults.
  /// Returns nullopt when the body is not shaped like a ListFaults answer (no
  /// "faults" array). Total over untrusted content: items that are not objects
  /// or lack a string "fault_code" / array "reporting_sources" are skipped,
  /// non-string source entries are dropped, nothing throws.
  static std::optional<std::vector<StandingFault>> standing_faults_from_list_reply(const nlohmann::json & data);

 private:
  /// Subscription-worker side: filter for EVENT_CONFIRMED and enqueue only.
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  /// capture_thread_ main loop: drains queued confirm events.
  void capture_worker();

  /// Frame the faults that were already confirmed when this object came up:
  /// plugins start reporting while the gateway is still wiring the capture, so
  /// a device that is in fault at boot confirms to nobody. Frames are
  /// process-local, so this restores what the missed edge would have produced
  /// rather than duplicating anything. Runs on capture_thread_. Capped at
  /// max_faults_ stored frames so the catch-up cannot FIFO-evict its own
  /// earlier frames; fault codes with a confirm already queued are left to the
  /// drain loop (one plugin read per confirm).
  void capture_standing_faults();

  /// Bounded, abortable wait until the events subscription has a matched
  /// publisher. Orders the standing-fault snapshot after matching: a confirm
  /// published after the snapshot but before the reader matches would
  /// otherwise be missed by both paths.
  void wait_for_events_publisher(const std::function<bool()> & should_abort) const;

  /// Per-event capture (all plugin calls happen here, on capture_thread_).
  /// Returns true when at least one frame was stored; startup_catchup marks
  /// the stored frames as catch-up captures.
  bool capture_for_event(const ros2_medkit_msgs::msg::FaultEvent & event, bool startup_catchup = false);

  /// Build a frame from list-data-shaped content, enforcing the shared
  /// no-row-of-nulls invariant on both capture paths. @p source names the path
  /// that read the content and is stored verbatim in Frame::source.
  std::optional<Frame> frame_from_content(const std::string & entity_id, const std::string & fault_code,
                                          const nlohmann::json & content, const std::string & source);

  /// Capture via the plugin's own x-plc-data route (no DataProvider exported).
  /// Returns nullopt when the route yields nothing usable.
  std::optional<Frame> capture_via_route(const std::string & entity_id, const std::string & fault_code);

  /// Log a fallback failure once per fault code (faults re-confirm on every
  /// clear/re-report cycle; one line per code is enough for an operator).
  void log_fallback_failure_once(const std::string & fault_code, const std::string & message);

  /// Fill frames_ from the store. Runs in the constructor, so a frame captured
  /// before the last shutdown is already being served when the first request
  /// arrives, and the catch-up (which starts later, on capture_thread_)
  /// already sees it. Honours max_faults_ by keeping the newest codes: a store
  /// larger than the bound must not evict what it just loaded.
  void load_persisted_frames();

  /// Serialize a frame for the store. The columns carry entity, timestamp,
  /// source and origin; the blob carries the values and the payload
  /// provenance, so a reload reproduces the frame exactly.
  static StoredEntityFreezeFrame to_stored(const std::string & fault_code, const Frame & frame);

  /// Inverse of to_stored. Returns nullopt for a row whose blob is not shaped
  /// like a frame (a hand-edited or half-written file).
  static std::optional<Frame> from_stored(const StoredEntityFreezeFrame & row);

  /// Write the code's frames through to the store, replacing what was there.
  /// Caller holds mutex_, so the file and the map cannot disagree.
  void persist_frames_locked(const std::string & fault_code, const std::vector<Frame> & frames);

  /// Drop the code's rows from the store. Caller holds mutex_.
  void erase_persisted_locked(const std::string & fault_code);

  /// Drop reloaded frames whose fault the fault_manager no longer holds (its
  /// store was replaced or wiped under ours). Codes reported in any status,
  /// cleared included, are kept: a cleared fault keeps its frame. Does nothing
  /// without a known-code lister, or when the lister cannot answer.
  void prune_frames_for_unknown_faults(const std::function<bool()> & should_abort);

  /// Reloaded codes whose stored frame belongs to an EARLIER occurrence of
  /// their fault: it cleared and confirmed again while the gateway was down, so
  /// the frame holds the previous incident's values and serving it unmarked
  /// would present them as this occurrence's.
  ///
  /// Read-only on purpose. The row stays until a re-read has actually been
  /// tried, so a fault whose entity answers gets its frame replaced rather than
  /// deleted and then not re-taken. It is dropped only by drop_stale_frame(),
  /// after the attempt failed. Only reloaded codes are eligible: a frame this
  /// process captured is by definition this occurrence's.
  std::unordered_set<std::string> stale_reloaded_codes(const std::vector<StandingFault> & standing) const;

  /// Last resort for a stale-occurrence code the catch-up could not re-read:
  /// erase it from memory and from the store, and say so. Keeping it would
  /// serve the previous incident's values unmarked, which is the defect this
  /// path exists to fix, so the frame goes. Never silently, though: the
  /// operator is losing evidence and only the log can tell them.
  /// @p entities names the reporting sources that were tried (empty when the
  /// fault named none), @p reason why no frame could be taken.
  void drop_stale_frame(const std::string & fault_code, const std::string & entities, const char * reason);

  std::unique_ptr<ros2_common::Ros2SubscriptionSlot> subscription_slot_;
  DataProviderResolver resolver_;
  RouteDataFetcher route_fetcher_;
  rclcpp::Logger logger_;
  const size_t max_faults_;

  /// Guards frames_, insertion_order_ and fallback_logged_: capture thread
  /// writes, HTTP handler threads read.
  mutable std::mutex mutex_;
  /// Keyed by fault_code only: cross-entity isolation relies on the
  /// fault_manager keeping reporting_sources append-only for a code and on
  /// get_fault gating by source scope. A per-source clear upstream would need
  /// per-entity eviction here too.
  std::unordered_map<std::string, std::vector<Frame>> frames_;
  std::deque<std::string> insertion_order_;          ///< eviction order (FIFO)
  std::unordered_set<std::string> fallback_logged_;  ///< fault codes already warned about (bounded)
  /// Codes whose frames came from the store and have not been re-captured
  /// since. Only these are eligible for the unknown-fault prune: a frame this
  /// process captured is by definition for a fault the fault_manager just
  /// confirmed, whatever a stale list reply says.
  std::unordered_set<std::string> reloaded_codes_;
  /// Store-write failures already warned about, so a read-only or full volume
  /// costs one line, not one per capture.
  bool store_write_warned_{false};

  /// Confirm events pending capture: fed by the subscription worker, drained
  /// by capture_thread_. Bounded - oldest event dropped when full.
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr> queue_;
  bool stop_{false};
  std::thread capture_thread_;

  /// Lists faults already confirmed at construction; run once on that thread.
  StandingFaultLister standing_lister_;

  /// Frame persistence; null keeps the frames in process memory only.
  std::shared_ptr<EntityFreezeFrameStore> store_;
  /// Reports the codes the fault_manager still holds; run once, on the
  /// capture thread, after the standing-fault lister.
  KnownFaultCodeLister known_code_lister_;
};

}  // namespace ros2_medkit_gateway
