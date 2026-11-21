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

#include <chrono>
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

#include "ros2_medkit_fault_detection/fault_types.hpp"
#include "ros2_medkit_gateway/core/data/data_item_reader.hpp"
#include "ros2_medkit_gateway/core/faults/entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_gateway/core/faults/freeze_frame_source.hpp"
#include "ros2_medkit_gateway/core/logging.hpp"

namespace ros2_medkit_gateway {

class PluginManager;

/// Settings of the freeze-frame capture. Names and defaults follow the node parameters.
struct FreezeFrameCaptureConfig {
  /// `entity_freeze_frame.enabled`: capture at all.
  bool enabled{true};
  /// Retained-frame bound: past it, the frames of the fault captured longest ago are
  /// evicted. Reloaded and captured frames count together. Not a node parameter.
  size_t max_faults{256};
  /// Wait between two startup fault-list attempts while none has succeeded. Not a node
  /// parameter.
  std::chrono::milliseconds list_retry_interval{std::chrono::seconds(5)};
};

/// Freezes the data values of an entity when a fault of that entity confirms.
///
/// A plugin-owned entity reports faults under its bare entity id, and its values
/// are not ROS topics, so the fault manager's own snapshot capture never frames
/// it. This class subscribes to a `FaultEventSource` and, on `fault_confirmed`,
/// reads the data of each reporting source through the same path as
/// `GET /{entity}/data` (the plugin's `DataProvider`, else its `x-plc-data`
/// route). The frames are served through `FreezeFrameSource` and merged into the
/// fault detail's `environment_data`. A frame set stays after `fault_cleared`. The next
/// confirmation replaces it, or drops it with a warning when the read gets no values and
/// the occurrence is another one.
///
/// With an `EntityFreezeFrameStore`, each capture is written through with the fault's
/// occurrence, and the stored frames load on construction. With a lister, no reloaded
/// frame is served before a list has succeeded. A failed list is retried every
/// `list_retry_interval`, and live confirms are captured in between; a capture replaces
/// the reloaded frame of its code. The first list that succeeds drops a frame of another
/// occurrence, or of a fault the list does not hold. An unknown occurrence or an empty
/// list keeps the frame. With no lister, every reloaded frame is kept and served at once.
/// Then each confirmed, unmuted fault with no frame is read and marked startup, while the
/// bound has room.
///
/// The reads run on a thread of this object, never on the thread that delivers
/// the event.
class FreezeFrameCapture : public FreezeFrameSource {
 public:
  /// Stored `capture_origin` of a catch-up frame. A reloaded frame keeps it.
  static constexpr const char * kCaptureOriginStartup = "startup";

  /// Resolves an entity id to the `DataProvider` of its plugin (null when there is none).
  using DataProviderResolver = PluginDataReaders::ProviderResolver;

  /// Fetches an entity's current values through its plugin's `x-plc-data` route.
  using RouteDataFetcher = PluginDataReaders::RouteFetcher;

  /// The fault manager's identity of one occurrence of a fault: the record's
  /// `occurrence_count` and `first_occurred`. Both change only when a CLEARED fault
  /// reactivates. `first_occurred_ns` is rounded through the ListFaults encoding (seconds
  /// as a double), so an event and a reply of one occurrence give equal values.
  struct Occurrence {
    uint32_t count{0};
    int64_t first_occurred_ns{0};
    /// False when either value is missing: then nothing can be told from it.
    bool known() const {
      return count != 0 && first_occurred_ns != 0;
    }
    bool operator==(const Occurrence & other) const {
      return count == other.count && first_occurred_ns == other.first_occurred_ns;
    }
    bool operator!=(const Occurrence & other) const {
      return !(*this == other);
    }
  };

  /// One fault of a ListFaults reply: its code, the entities that reported it, its
  /// status, its occurrence and whether correlation mutes it.
  struct StandingFault {
    std::string fault_code;
    std::vector<std::string> reporting_sources;
    /// CONFIRMED unless set: the status of a standing fault.
    std::string status{ros2_medkit::fault_detection::fault_status::kConfirmed};
    Occurrence occurrence{};
    /// A muted fault publishes no confirm event, so it is not framed at start either.
    bool muted{false};
  };

  /// Every fault the fault manager holds when the capture starts, in any status and
  /// muted ones included. Nullopt when the list could not be read; a throw counts the
  /// same. Called on the capture thread until one call succeeds, never after the
  /// destructor has joined it. A blocking wait inside must poll `should_abort` and
  /// return when it turns true.
  using StandingFaultLister =
      std::function<std::optional<std::vector<StandingFault>>(const std::function<bool()> & should_abort)>;

  /// Blocks until the event feed delivers, or `should_abort` turns true. Runs on the
  /// capture thread before the lister, so a confirmation that falls between the
  /// lister's snapshot and the feed attaching is not lost.
  using EventsReadyWaiter = std::function<void(const std::function<bool()> & should_abort)>;

  struct Hooks {
    /// Both reads of an entity's data. Without them nothing is captured.
    PluginDataReaders readers;
    /// Optional. Judges the reloaded frames and frames the confirmed faults at start.
    StandingFaultLister standing_lister;
    /// Optional wait for the feed.
    EventsReadyWaiter events_ready;
    /// Optional persistence. Null keeps the frames in memory only.
    std::shared_ptr<EntityFreezeFrameStore> store;
  };

  /// @param events Feed of fault events. Subscribed on construction; the subscription
  ///        ends on destruction. Other subscribers may share it. Must outlive this object.
  FreezeFrameCapture(FaultEventSource & events, const FreezeFrameCaptureConfig & config, Hooks hooks);

  ~FreezeFrameCapture() override;
  FreezeFrameCapture(const FreezeFrameCapture &) = delete;
  FreezeFrameCapture & operator=(const FreezeFrameCapture &) = delete;
  FreezeFrameCapture(FreezeFrameCapture &&) = delete;
  FreezeFrameCapture & operator=(FreezeFrameCapture &&) = delete;

  std::vector<Frame> frames_for(const std::string & fault_code) const override;

  /// Parses a ListFaults reply body into standing faults. Nullopt when the body has no
  /// "faults" array. Items that lack a string "fault_code" or an array "reporting_sources"
  /// are skipped, and non-string source entries are dropped. A fault named in
  /// "muted_faults" is marked muted. A "first_occurred" that is
  /// not a positive number of seconds within int64 nanoseconds, or an "occurrence_count"
  /// that is not a positive uint32, leaves that part of `occurrence` at 0. Never throws.
  static std::optional<std::vector<StandingFault>> standing_faults_from_list_reply(const nlohmann::json & data);

 private:
  void on_fault_event(const ros2_medkit::fault_detection::FaultEventRecord & event);
  void capture_worker();
  /// Lists the standing faults until a list succeeds. Captures live confirms between two
  /// attempts. Nullopt when the capture stops first.
  std::optional<std::vector<StandingFault>> list_standing_faults();
  /// Captures the confirms that arrive until @p deadline. False when the capture stops.
  bool capture_confirms_until(std::chrono::steady_clock::time_point deadline);
  void capture_standing_faults(const std::vector<StandingFault> & listed);
  bool capture_for_event(const ros2_medkit::fault_detection::FaultEventRecord & event, const Occurrence & occurrence,
                         bool startup_catchup = false);
  /// The occurrence of @p record, through the ListFaults reply's encoding.
  static Occurrence occurrence_of(const ros2_medkit::fault_detection::FaultRecord & record);
  std::optional<Frame> frame_from_content(const std::string & entity_id, const std::string & fault_code,
                                          const nlohmann::json & content, const std::string & source);
  std::optional<Frame> capture_via_route(const std::string & entity_id, const std::string & fault_code);
  void log_fallback_failure_once(const std::string & fault_code, const std::string & message);

  /// Fills frames_ from the store, keeping the newest codes under max_faults_.
  void load_persisted_frames();
  static StoredEntityFreezeFrame to_stored(const std::string & fault_code, const Frame & frame,
                                           const Occurrence & occurrence);
  /// Nullopt for a row whose blob is not shaped like a frame.
  static std::optional<Frame> from_stored(const StoredEntityFreezeFrame & row);
  /// Replaces the code's rows in the store. The caller does not hold mutex_.
  void write_persisted(const std::string & fault_code, const std::vector<StoredEntityFreezeFrame> & rows);
  /// Deletes the rows of each code from the store. The caller does not hold mutex_.
  void erase_persisted(const std::vector<std::string> & fault_codes);
  /// Keeps or discards each reloaded frame against @p listed, then lets frames_for serve
  /// the ones kept.
  void judge_reloaded_frames(const std::vector<StandingFault> & listed);
  /// Erases a code's frames from memory. The caller holds mutex_.
  void forget_locked(const std::string & fault_code);

  PluginDataReaders readers_;
  StandingFaultLister standing_lister_;
  EventsReadyWaiter events_ready_;
  std::shared_ptr<EntityFreezeFrameStore> store_;
  const log::Logger logger_{"entity_freeze_frame"};
  const size_t max_faults_;
  const std::chrono::milliseconds list_retry_interval_;

  /// Guards frames_, insertion_order_, fallback_logged_, reloaded_codes_, occurrences_,
  /// reloaded_judged_ and store_write_warned_.
  mutable std::mutex mutex_;
  /// Keyed by fault code only: cross-entity isolation rests on the fault manager keeping
  /// reporting_sources append-only for a code and on `get_fault` gating by source scope.
  std::unordered_map<std::string, std::vector<Frame>> frames_;
  std::deque<std::string> insertion_order_;          ///< eviction order (FIFO)
  std::unordered_set<std::string> fallback_logged_;  ///< fault codes already warned about (bounded)
  /// Codes whose frames came from the store and were not captured again since. Only
  /// these are judged at start.
  std::unordered_set<std::string> reloaded_codes_;
  /// The occurrence of every code in frames_, reloaded or captured.
  std::unordered_map<std::string, Occurrence> occurrences_;
  /// False until a startup fault list has judged the reloaded frames: until then
  /// frames_for serves none of them.
  bool reloaded_judged_{false};
  /// One warning per process for a failed store write or delete. Only the capture
  /// thread writes the store, so the writes keep the capture order.
  bool store_write_warned_{false};

  /// Confirm events waiting for capture. Bounded: the oldest event is dropped when full.
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ros2_medkit::fault_detection::FaultEventRecord> queue_;
  bool stop_{false};
  std::thread capture_thread_;
  /// Declared last, so destroyed first: if the constructor throws, delivery ends before the queue goes.
  FaultEventSubscription subscription_;
};

/// Builds the capture over the data of the plugins in `plugins`.
///
/// A host attaches the result with `GatewayServices::freeze_frames()`, and passes:
/// - `events`: its fault event source, which may also feed the fault SSE stream;
/// - `plugins`: the manager that owns the entities;
/// - `standing_lister`, `events_ready` and `store`: optional, see `Hooks`.
///
/// Returns null when the configuration is disabled or `plugins` has no plugin loaded.
std::unique_ptr<FreezeFrameCapture>
make_freeze_frame_capture(const FreezeFrameCaptureConfig & config, FaultEventSource & events, PluginManager * plugins,
                          FreezeFrameCapture::StandingFaultLister standing_lister = nullptr,
                          FreezeFrameCapture::EventsReadyWaiter events_ready = nullptr,
                          std::shared_ptr<EntityFreezeFrameStore> store = nullptr);

}  // namespace ros2_medkit_gateway
