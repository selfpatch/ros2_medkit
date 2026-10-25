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

#include <condition_variable>
#include <cstddef>
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
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_gateway/core/faults/freeze_frame_source.hpp"
#include "ros2_medkit_gateway/core/logging.hpp"

namespace ros2_medkit_gateway {

class PluginManager;

/// Settings of the freeze-frame capture. Names and defaults follow the node parameters.
struct FreezeFrameCaptureConfig {
  /// `entity_freeze_frame.enabled`: capture at all.
  bool enabled{true};
  /// Retained-frame bound: the oldest fault's frames are evicted past it. Not a node parameter.
  size_t max_faults{256};
};

/// Freezes the data values of an entity when a fault of that entity confirms.
///
/// A plugin-owned entity reports faults under its bare entity id, and its values
/// are not ROS topics, so the fault manager's own snapshot capture never frames
/// it. This class subscribes to a `FaultEventSource` and, on `fault_confirmed`,
/// reads the data of each reporting source through the same path as
/// `GET /{entity}/data` (the plugin's `DataProvider`, else its `x-plc-data`
/// route). The frames are served through `FreezeFrameSource` and merged into the
/// fault detail's `environment_data`. A frame set stays after `fault_cleared` and
/// is replaced on the next confirmation.
///
/// The reads run on a thread of this object, never on the thread that delivers
/// the event.
class FreezeFrameCapture : public FreezeFrameSource {
 public:
  /// Resolves an entity id to the `DataProvider` of its plugin (null when there is none).
  using DataProviderResolver = PluginDataReaders::ProviderResolver;

  /// Fetches an entity's current values through its plugin's `x-plc-data` route.
  using RouteDataFetcher = PluginDataReaders::RouteFetcher;

  /// One standing fault: its code and the entities that reported it.
  struct StandingFault {
    std::string fault_code;
    std::vector<std::string> reporting_sources;
  };

  /// Lists the faults that are confirmed when the capture starts, so their missed
  /// confirm edge can still be framed. Called once, on the capture thread, never
  /// after the destructor has joined it. A blocking wait inside must poll
  /// `should_abort` and return when it turns true.
  using StandingFaultLister = std::function<std::vector<StandingFault>(const std::function<bool()> & should_abort)>;

  /// Blocks until the event feed delivers, or `should_abort` turns true. Runs on the
  /// capture thread before the lister, so a confirmation that falls between the
  /// lister's snapshot and the feed attaching is not lost.
  using EventsReadyWaiter = std::function<void(const std::function<bool()> & should_abort)>;

  struct Hooks {
    /// Both reads of an entity's data. Without them nothing is captured.
    PluginDataReaders readers;
    /// Optional startup catch-up.
    StandingFaultLister standing_lister;
    /// Optional wait for the feed.
    EventsReadyWaiter events_ready;
  };

  /// @param events Feed of fault events. Dedicated to this object: it subscribes on
  ///        construction and unsubscribes on destruction. Must outlive this object.
  FreezeFrameCapture(FaultEventSource & events, const FreezeFrameCaptureConfig & config, Hooks hooks);

  ~FreezeFrameCapture() override;
  FreezeFrameCapture(const FreezeFrameCapture &) = delete;
  FreezeFrameCapture & operator=(const FreezeFrameCapture &) = delete;
  FreezeFrameCapture(FreezeFrameCapture &&) = delete;
  FreezeFrameCapture & operator=(FreezeFrameCapture &&) = delete;

  std::vector<Frame> frames_for(const std::string & fault_code) const override;

  /// Parses a ListFaults reply body into standing faults. Nullopt when the body has no
  /// "faults" array. Items that lack a string "fault_code" or an array "reporting_sources"
  /// are skipped, and non-string source entries are dropped. Never throws.
  static std::optional<std::vector<StandingFault>> standing_faults_from_list_reply(const nlohmann::json & data);

 private:
  void on_fault_event(const ros2_medkit::fault_detection::FaultEventRecord & event);
  void capture_worker();
  void capture_standing_faults();
  bool capture_for_event(const ros2_medkit::fault_detection::FaultEventRecord & event, bool startup_catchup = false);
  std::optional<Frame> frame_from_content(const std::string & entity_id, const std::string & fault_code,
                                          const nlohmann::json & content, const std::string & source);
  std::optional<Frame> capture_via_route(const std::string & entity_id, const std::string & fault_code);
  void log_fallback_failure_once(const std::string & fault_code, const std::string & message);

  FaultEventSource & events_;
  PluginDataReaders readers_;
  StandingFaultLister standing_lister_;
  EventsReadyWaiter events_ready_;
  const log::Logger logger_{"entity_freeze_frame"};
  const size_t max_faults_;

  /// Guards frames_, insertion_order_ and fallback_logged_.
  mutable std::mutex mutex_;
  /// Keyed by fault code only: cross-entity isolation rests on the fault manager keeping
  /// reporting_sources append-only for a code and on `get_fault` gating by source scope.
  std::unordered_map<std::string, std::vector<Frame>> frames_;
  std::deque<std::string> insertion_order_;          ///< eviction order (FIFO)
  std::unordered_set<std::string> fallback_logged_;  ///< fault codes already warned about (bounded)

  /// Confirm events waiting for capture. Bounded: the oldest event is dropped when full.
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ros2_medkit::fault_detection::FaultEventRecord> queue_;
  bool stop_{false};
  std::thread capture_thread_;
};

/// Builds the capture over the data of the plugins in `plugins`.
///
/// A host attaches the result with `GatewayServices::freeze_frames()`, and passes:
/// - `events`: a feed of its own for this capture alone (see the constructor);
/// - `plugins`: the manager that owns the entities;
/// - `standing_lister` and `events_ready`: optional, see `Hooks`.
///
/// Returns null when the configuration is disabled or `plugins` has no plugin loaded.
std::unique_ptr<FreezeFrameCapture>
make_freeze_frame_capture(const FreezeFrameCaptureConfig & config, FaultEventSource & events, PluginManager * plugins,
                          FreezeFrameCapture::StandingFaultLister standing_lister = nullptr,
                          FreezeFrameCapture::EventsReadyWaiter events_ready = nullptr);

}  // namespace ros2_medkit_gateway
