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

#include "ros2_medkit_gateway/core/faults/freeze_frame_capture.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <utility>

#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"

namespace ros2_medkit_gateway {

namespace {

namespace fd = ros2_medkit::fault_detection;

/// Capture backlog bound: a confirm burst beyond this drops the oldest event.
constexpr size_t kMaxQueuedEvents = 64;

/// fallback_logged_ bound: past this the set resets (re-arming one warn per
/// code) instead of growing forever on churny/synthetic fault codes.
constexpr size_t kMaxLoggedFaultCodes = 1024;

}  // namespace

FreezeFrameCapture::FreezeFrameCapture(FaultEventSource & events, const FreezeFrameCaptureConfig & config, Hooks hooks)
  : events_(events)
  , readers_(std::move(hooks.readers))
  , standing_lister_(std::move(hooks.standing_lister))
  , events_ready_(std::move(hooks.events_ready))
  , max_faults_(config.max_faults > 0 ? config.max_faults : 1) {
  events_.subscribe([this](const fd::FaultEventRecord & event) {
    on_fault_event(event);
  });
  capture_thread_ = std::thread([this] {
    capture_worker();
  });
}

FreezeFrameCapture::~FreezeFrameCapture() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  queue_cv_.notify_all();
  // Joins through any in-flight plugin call: a hung read delays shutdown
  // rather than leaving a capture racing plugin unload.
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  events_.unsubscribe();
}

std::vector<FreezeFrameCapture::Frame> FreezeFrameCapture::frames_for(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = frames_.find(fault_code);
  return it != frames_.end() ? it->second : std::vector<Frame>{};
}

std::optional<std::vector<FreezeFrameCapture::StandingFault>>
FreezeFrameCapture::standing_faults_from_list_reply(const nlohmann::json & data) {
  if (!data.is_object()) {
    return std::nullopt;
  }
  // ListFaults answers under "faults" (see Ros2FaultServiceTransport).
  const auto faults = data.find("faults");
  if (faults == data.end() || !faults->is_array()) {
    return std::nullopt;
  }
  std::vector<StandingFault> standing;
  for (const auto & item : *faults) {
    if (!item.is_object()) {
      continue;
    }
    const auto code = item.find("fault_code");
    const auto sources = item.find("reporting_sources");
    if (code == item.end() || !code->is_string() || sources == item.end() || !sources->is_array()) {
      continue;
    }
    StandingFault fault;
    fault.fault_code = code->get<std::string>();
    for (const auto & src : *sources) {
      if (src.is_string()) {
        fault.reporting_sources.push_back(src.get<std::string>());
      }
    }
    standing.push_back(std::move(fault));
  }
  return standing;
}

std::optional<FreezeFrameCapture::Frame> FreezeFrameCapture::frame_from_content(const std::string & entity_id,
                                                                                const std::string & fault_code,
                                                                                const nlohmann::json & content,
                                                                                const std::string & source) {
  if (!content_has_live_data(content)) {
    log_fallback_failure_once(fault_code, "entity '" + entity_id + "' returned no data items");
    return std::nullopt;
  }
  Frame frame;
  frame.entity_id = entity_id;
  frame.source = source;
  frame.values = values_from_list_content(content);
  if (!values_have_data(frame.values)) {
    // Items present but nothing usable in them (all-null values, or no usable
    // ids): still a dead/cold row - no frame, same invariant as above.
    log_fallback_failure_once(fault_code, "entity '" + entity_id + "' values are all null");
    return std::nullopt;
  }
  // Provenance for the snapshot's x-medkit block: the payload's link flag and
  // its own timestamp, when the plugin reports them. captured_at_ns dates the
  // capture; a disconnected entity's values may be much older.
  if (content.contains("connected") && content["connected"].is_boolean()) {
    frame.connected = content["connected"].get<bool>();
  }
  if (content.contains("timestamp")) {
    frame.source_timestamp = content["timestamp"];
  }
  frame.captured_at_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  return frame;
}

std::optional<FreezeFrameCapture::Frame> FreezeFrameCapture::capture_via_route(const std::string & entity_id,
                                                                               const std::string & fault_code) {
  std::optional<nlohmann::json> content;
  try {
    content = readers_.route(entity_id);
  } catch (const std::exception & e) {
    log_fallback_failure_once(fault_code, std::string("x-plc-data dispatch threw: ") + e.what());
    return std::nullopt;
  }
  if (!content) {
    return std::nullopt;  // not plugin-owned, no x-plc-data route, or handler error
  }
  return frame_from_content(entity_id, fault_code, *content, kSourceXPlcDataRoute);
}

void FreezeFrameCapture::log_fallback_failure_once(const std::string & fault_code, const std::string & message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fallback_logged_.size() >= kMaxLoggedFaultCodes) {
      fallback_logged_.clear();
    }
    if (!fallback_logged_.insert(fault_code).second) {
      return;
    }
  }
  MEDKIT_LOG_WARN(logger_, "Entity freeze-frame for fault '%s': %s", fault_code.c_str(), message.c_str());
}

void FreezeFrameCapture::on_fault_event(const fd::FaultEventRecord & event) {
  // Freeze-frames are taken when a fault confirms, not on every update.
  if (event.event_type != fd::fault_event_type::kConfirmed) {
    return;
  }

  // The event arrives on the feed's thread, which may serve other work, so only
  // enqueue here. The plugin reads run on capture_thread_, where a slow read
  // cannot stall the feed and a handler that re-enters it cannot deadlock.
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      return;
    }
    if (queue_.size() >= kMaxQueuedEvents) {
      MEDKIT_LOG_WARN(logger_, "Entity freeze-frame capture backlog full; dropping oldest confirm event");
      queue_.pop_front();
    }
    queue_.push_back(event);
  }
  queue_cv_.notify_one();
}

void FreezeFrameCapture::capture_standing_faults() {
  if (!standing_lister_) {
    return;
  }
  const auto should_abort = [this] {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return stop_;
  };
  // Snapshot only after the events subscription has matched: a confirm
  // published after the lister's snapshot but before the reader matched would
  // be missed by both paths.
  if (events_ready_) {
    events_ready_(should_abort);
  }
  if (should_abort()) {
    return;
  }
  // The lister blocks until the fault services answer, so it runs here rather
  // than on a thread of its own: this thread is joined by the destructor, and
  // should_abort is what lets that join interrupt the wait.
  std::vector<StandingFault> standing;
  try {
    standing = standing_lister_(should_abort);
  } catch (const std::exception & e) {
    MEDKIT_LOG_WARN(logger_, "Entity freeze-frame startup catch-up failed: standing-fault lister threw: %s", e.what());
    return;
  } catch (...) {
    MEDKIT_LOG_WARN(logger_, "Entity freeze-frame startup catch-up failed: standing-fault lister threw");
    return;
  }
  // Codes with a confirm already queued belong to the drain loop: capturing
  // them here too would read the plugin twice for one confirm.
  std::unordered_set<std::string> queued_codes;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      return;
    }
    for (const auto & queued : queue_) {
      queued_codes.insert(queued.fault.fault_code);
    }
  }
  size_t framed = 0;
  size_t over_cap = 0;
  for (const auto & fault : standing) {
    if (should_abort()) {
      return;
    }
    if (fault.fault_code.empty() || fault.reporting_sources.empty()) {
      continue;
    }
    if (queued_codes.count(fault.fault_code) != 0) {
      continue;
    }
    if (framed >= max_faults_) {
      ++over_cap;  // storing more would FIFO-evict this catch-up's own frames
      continue;
    }
    fd::FaultEventRecord event;
    event.event_type = fd::fault_event_type::kConfirmed;
    event.fault.fault_code = fault.fault_code;
    event.fault.reporting_sources = fault.reporting_sources;
    if (capture_for_event(event, /*startup_catchup=*/true)) {
      ++framed;
    }
  }
  if (over_cap > 0) {
    MEDKIT_LOG_WARN(logger_,
                    "Entity freeze-frame startup catch-up truncated: %zu standing fault(s) beyond the retained-frame "
                    "bound of %zu",
                    over_cap, max_faults_);
  }
  if (framed > 0) {
    MEDKIT_LOG_INFO(logger_, "Entity freeze-frame: captured %zu fault(s) that were already confirmed at startup",
                    framed);
  }
}

void FreezeFrameCapture::capture_worker() {
  capture_standing_faults();
  for (;;) {
    fd::FaultEventRecord event;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return stop_ || !queue_.empty();
      });
      if (stop_) {
        // Drop the backlog: the owner destroys this right before plugin
        // shutdown, and a late capture would call into an unloading plugin.
        return;
      }
      event = std::move(queue_.front());
      queue_.pop_front();
    }
    capture_for_event(event);
  }
}

bool FreezeFrameCapture::capture_for_event(const fd::FaultEventRecord & event, bool startup_catchup) {
  const std::string & fault_code = event.fault.fault_code;

  std::vector<Frame> frames;
  for (const auto & source : event.fault.reporting_sources) {
    DataProvider * provider = readers_.provider ? readers_.provider(source) : nullptr;
    if (provider == nullptr) {
      // No DataProvider: the owning plugin may still serve live values through
      // its own x-plc-data route (the commercial PLC bridges). For non-plugin
      // (ROS) sources the fetcher resolves no owner and returns nullopt.
      if (readers_.route) {
        if (auto frame = capture_via_route(source, fault_code)) {
          frames.push_back(std::move(*frame));
        }
      }
      continue;
    }

    // list_data is expected to serve from the plugin's latest polled values
    // (cheap); plugin code still gets exception-guarded like every other
    // provider call site.
    try {
      auto result = provider->list_data(source);
      if (!result) {
        log_fallback_failure_once(fault_code, "list_data('" + source + "') failed: " + result.error().message);
        continue;
      }
      if (auto frame = frame_from_content(source, fault_code, result->content, kSourceDataProvider)) {
        frames.push_back(std::move(*frame));
      }
    } catch (const std::exception & e) {
      log_fallback_failure_once(fault_code, "plugin threw for entity '" + source + "': " + e.what());
    }
  }

  if (frames.empty()) {
    return false;
  }
  if (startup_catchup) {
    for (auto & frame : frames) {
      frame.startup_catchup = true;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (frames_.find(fault_code) == frames_.end()) {
    insertion_order_.push_back(fault_code);
    while (frames_.size() >= max_faults_ && !insertion_order_.empty()) {
      frames_.erase(insertion_order_.front());
      insertion_order_.pop_front();
    }
  }
  frames_[fault_code] = std::move(frames);

  MEDKIT_LOG_DEBUG(logger_, "Captured entity freeze-frame(s) for fault '%s'", fault_code.c_str());
  return true;
}

std::unique_ptr<FreezeFrameCapture> make_freeze_frame_capture(const FreezeFrameCaptureConfig & config,
                                                              FaultEventSource & events, PluginManager * plugins,
                                                              FreezeFrameCapture::StandingFaultLister standing_lister,
                                                              FreezeFrameCapture::EventsReadyWaiter events_ready) {
  // Only plugins own entities whose values the fault manager cannot capture.
  if (!config.enabled || plugins == nullptr || !plugins->has_plugins()) {
    return nullptr;
  }
  FreezeFrameCapture::Hooks hooks;
  hooks.readers = plugin_data_readers(plugins);
  hooks.standing_lister = std::move(standing_lister);
  hooks.events_ready = std::move(events_ready);
  return std::make_unique<FreezeFrameCapture>(events, config, std::move(hooks));
}

}  // namespace ros2_medkit_gateway
