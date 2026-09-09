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

#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Capture backlog bound: a confirm burst beyond this drops the oldest event.
constexpr size_t kMaxQueuedEvents = 64;

/// fallback_logged_ bound: past this the set resets (re-arming one warn per
/// code) instead of growing forever on churny/synthetic fault codes.
constexpr size_t kMaxLoggedFaultCodes = 1024;

/// Bound on waiting for the events subscription to match a publisher before
/// the standing-fault snapshot; past it the catch-up proceeds best-effort.
constexpr std::chrono::seconds kEventsMatchTimeout{10};

/// How many over-the-bound fault codes the truncation warning names before it
/// stops. Enough for an operator to act on, short of a 256-code log line.
constexpr size_t kMaxNamedOverCapCodes = 10;

/// Read a string field totally: json::value() throws type_error.302 when the
/// key is present but not a string, and plugin content is untrusted.
std::string string_field(const nlohmann::json & item, const char * field) {
  const auto it = item.find(field);
  return it != item.end() && it->is_string() ? it->get<std::string>() : std::string();
}

/// Reporting sources as one comma-separated string, for a log line that has to
/// name the entities an operator would go and look at.
std::string join_sources(const std::vector<std::string> & sources) {
  std::string joined;
  for (const auto & source : sources) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += source;
  }
  return joined;
}

}  // namespace

EntityFreezeFrameCapture::EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                                                   DataProviderResolver resolver, RouteDataFetcher route_fetcher,
                                                   size_t max_faults, StandingFaultLister standing_lister,
                                                   std::shared_ptr<EntityFreezeFrameStore> store,
                                                   KnownFaultCodeLister known_code_lister)
  : resolver_(std::move(resolver))
  , route_fetcher_(std::move(route_fetcher))
  , logger_(node->get_logger())
  , max_faults_(max_faults > 0 ? max_faults : 1)
  , standing_lister_(std::move(standing_lister))
  , store_(std::move(store))
  , known_code_lister_(std::move(known_code_lister)) {
  // Before anything can serve or capture: a frame taken before the last
  // shutdown is the one the operator is owed, and the catch-up must see it so
  // it re-reads only the faults that have none.
  load_persisted_frames();
  // Resolve the topic from the gateway node (it owns fault_manager.namespace);
  // the subscription itself is created on the executor's dedicated _sub node so
  // it never races rcl's hash-map on the main node (issue #375).
  const auto fault_events_topic = build_fault_manager_events_topic(node);
  // Volatile on purpose: a transient_local reader does not match the fault
  // manager's volatile publisher, so upgrading the QoS here would break
  // matching against older fault managers. The startup catch-up covers the
  // pre-match window instead.
  auto slot = ros2_common::Ros2SubscriptionSlot::create_typed<ros2_medkit_msgs::msg::FaultEvent>(
      exec, fault_events_topic, rclcpp::QoS(100).reliable(),
      [this](const std::shared_ptr<const ros2_medkit_msgs::msg::FaultEvent> & msg) {
        on_fault_event(msg);
      });
  if (!slot) {
    RCLCPP_ERROR(logger_, "EntityFreezeFrameCapture: failed to subscribe to %s: %s", fault_events_topic.c_str(),
                 slot.error().c_str());
    return;
  }
  subscription_slot_ = std::move(*slot);
  capture_thread_ = std::thread([this] {
    capture_worker();
  });

  RCLCPP_INFO(logger_, "EntityFreezeFrameCapture initialized, subscribed to %s", fault_events_topic.c_str());
}

EntityFreezeFrameCapture::~EntityFreezeFrameCapture() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  queue_cv_.notify_all();
  // Joins through any in-flight plugin call: a hung read delays shutdown
  // rather than leaving a capture racing plugin unload. Must precede the slot
  // reset - the capture thread reads the slot's publisher count.
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  // Not a synchronous barrier: the slot posts an async destroy with a bounded
  // deadline, so use-after-free safety also relies on the owner tearing down
  // the subscription executor (joining its worker) before this object's node
  // dies - keep that ordering in main.cpp / gateway_node shutdown. Late
  // callbacks before that teardown see stop_ and drop their event.
  subscription_slot_.reset();
}

std::vector<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::frames_for(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = frames_.find(fault_code);
  return it != frames_.end() ? it->second : std::vector<Frame>{};
}

StoredEntityFreezeFrame EntityFreezeFrameCapture::to_stored(const std::string & fault_code, const Frame & frame) {
  StoredEntityFreezeFrame row;
  row.fault_code = fault_code;
  row.entity_id = frame.entity_id;
  row.frame = nlohmann::json::object();
  row.frame["values"] = frame.values;
  // Written only when the capture had them, so the reload reproduces the
  // frame's own "reported nothing" as absence rather than as a null.
  if (frame.connected.has_value()) {
    row.frame["connected"] = *frame.connected;
  }
  if (!frame.source_timestamp.is_null()) {
    row.frame["source_timestamp"] = frame.source_timestamp;
  }
  row.captured_at_ns = frame.captured_at_ns;
  row.source = frame.source;
  row.capture_origin = frame.startup_catchup ? kCaptureOriginStartup : "";
  return row;
}

std::optional<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::from_stored(const StoredEntityFreezeFrame & row) {
  if (row.entity_id.empty() || !row.frame.is_object()) {
    return std::nullopt;
  }
  const auto values = row.frame.find("values");
  if (values == row.frame.end()) {
    return std::nullopt;
  }
  Frame frame;
  frame.entity_id = row.entity_id;
  frame.values = *values;
  frame.captured_at_ns = row.captured_at_ns;
  frame.source = row.source;
  // Reloading must not launder a catch-up frame into a confirm-edge one: its
  // captured_at is still a gateway start, so the marker still belongs on it.
  frame.startup_catchup = row.capture_origin == kCaptureOriginStartup;
  const auto connected = row.frame.find("connected");
  if (connected != row.frame.end() && connected->is_boolean()) {
    frame.connected = connected->get<bool>();
  }
  const auto source_timestamp = row.frame.find("source_timestamp");
  if (source_timestamp != row.frame.end()) {
    frame.source_timestamp = *source_timestamp;
  }
  return frame;
}

void EntityFreezeFrameCapture::persist_frames_locked(const std::string & fault_code,
                                                     const std::vector<Frame> & frames) {
  if (!store_) {
    return;
  }
  std::vector<StoredEntityFreezeFrame> rows;
  rows.reserve(frames.size());
  for (const auto & frame : frames) {
    rows.push_back(to_stored(fault_code, frame));
  }
  auto written = store_->replace_frames(fault_code, rows);
  if (!written && !store_write_warned_) {
    // One line for the life of the process: a read-only or full volume would
    // otherwise log once per confirm, and the frames still work in memory.
    store_write_warned_ = true;
    RCLCPP_WARN(logger_, "Entity freeze-frame store write failed, frames are process-local until restart: %s",
                written.error().c_str());
  }
}

void EntityFreezeFrameCapture::erase_persisted_locked(const std::string & fault_code) {
  if (!store_) {
    return;
  }
  auto erased = store_->erase_frames(fault_code);
  if (!erased && !store_write_warned_) {
    store_write_warned_ = true;
    RCLCPP_WARN(logger_, "Entity freeze-frame store delete failed: %s", erased.error().c_str());
  }
}

void EntityFreezeFrameCapture::load_persisted_frames() {
  if (!store_) {
    return;
  }
  auto rows = store_->load_all();
  if (!rows) {
    RCLCPP_WARN(logger_, "Entity freeze-frame store unreadable, starting with no reloaded frames: %s",
                rows.error().c_str());
    return;
  }

  std::unordered_map<std::string, std::vector<Frame>> loaded;
  std::unordered_map<std::string, int64_t> newest;
  size_t unreadable = 0;
  for (const auto & row : *rows) {
    auto frame = from_stored(row);
    if (!frame) {
      ++unreadable;
      continue;
    }
    auto it = newest.find(row.fault_code);
    if (it == newest.end()) {
      newest.emplace(row.fault_code, row.captured_at_ns);
    } else {
      it->second = std::max(it->second, row.captured_at_ns);
    }
    loaded[row.fault_code].push_back(std::move(*frame));
  }

  // Oldest code first, so the retained-frame bound drops what a restart can
  // least afford to keep rather than what it just read.
  std::vector<std::string> codes;
  codes.reserve(loaded.size());
  for (const auto & entry : loaded) {
    codes.push_back(entry.first);
  }
  std::sort(codes.begin(), codes.end(), [&newest](const std::string & a, const std::string & b) {
    if (newest.at(a) != newest.at(b)) {
      return newest.at(a) < newest.at(b);
    }
    return a < b;
  });
  const size_t over_cap = codes.size() > max_faults_ ? codes.size() - max_faults_ : 0;

  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < codes.size(); ++i) {
    const auto & code = codes[i];
    if (i < over_cap) {
      // Past the bound: drop the rows too, or every start re-reads frames it
      // can never serve and the file grows without one.
      erase_persisted_locked(code);
      continue;
    }
    insertion_order_.push_back(code);
    frames_[code] = std::move(loaded[code]);
    reloaded_codes_.insert(code);
  }
  if (!frames_.empty()) {
    RCLCPP_INFO(logger_, "Entity freeze-frame: reloaded frames for %zu fault(s) from the store", frames_.size());
  }
  if (over_cap > 0) {
    RCLCPP_WARN(logger_,
                "Entity freeze-frame store held %zu fault(s) beyond the retained-frame bound of %zu; "
                "the oldest were dropped",
                over_cap, max_faults_);
  }
  if (unreadable > 0) {
    RCLCPP_WARN(logger_, "Entity freeze-frame store: %zu unreadable row(s) skipped", unreadable);
  }
}

void EntityFreezeFrameCapture::prune_frames_for_unknown_faults(const std::function<bool()> & should_abort) {
  if (!known_code_lister_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reloaded_codes_.empty()) {
      return;
    }
  }
  std::optional<std::unordered_set<std::string>> known;
  try {
    known = known_code_lister_(should_abort);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Entity freeze-frame prune skipped: known-fault lister threw: %s", e.what());
    return;
  } catch (...) {
    RCLCPP_WARN(logger_, "Entity freeze-frame prune skipped: known-fault lister threw");
    return;
  }
  if (!known) {
    return;  // could not ask: "cannot tell" must never read as "the fault is gone"
  }

  size_t dropped = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = reloaded_codes_.begin(); it != reloaded_codes_.end();) {
      if (known->count(*it) != 0) {
        ++it;  // reported in any status, cleared included: the frame stays
        continue;
      }
      const std::string code = *it;
      it = reloaded_codes_.erase(it);
      frames_.erase(code);
      insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), code),
                             insertion_order_.end());
      erase_persisted_locked(code);
      ++dropped;
    }
  }
  if (dropped > 0) {
    RCLCPP_INFO(logger_,
                "Entity freeze-frame: dropped %zu reloaded frame(s) for fault(s) the fault manager no longer holds",
                dropped);
  }
}

std::unordered_set<std::string>
EntityFreezeFrameCapture::stale_reloaded_codes(const std::vector<StandingFault> & standing) const {
  std::unordered_set<std::string> stale;
  std::lock_guard<std::mutex> lock(mutex_);
  if (reloaded_codes_.empty()) {
    return stale;
  }
  for (const auto & fault : standing) {
    if (fault.first_occurred_ns <= 0 || reloaded_codes_.count(fault.fault_code) == 0) {
      continue;
    }
    const auto entry = frames_.find(fault.fault_code);
    if (entry == frames_.end()) {
      continue;
    }
    // The newest of the code's frames dates the stored capture: they are all
    // written by one capture, so if even that one predates the occurrence the
    // whole set belongs to an incident that has since been cleared.
    int64_t newest = 0;
    for (const auto & frame : entry->second) {
      newest = std::max(newest, frame.captured_at_ns);
    }
    if (fault.first_occurred_ns <= newest) {
      continue;  // same occurrence: the stored frame is the one to serve
    }
    stale.insert(fault.fault_code);
  }
  return stale;
}

void EntityFreezeFrameCapture::drop_stale_frame(const std::string & fault_code, const std::string & entities,
                                                const char * reason) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.erase(fault_code);
    insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), fault_code),
                           insertion_order_.end());
    reloaded_codes_.erase(fault_code);
    erase_persisted_locked(fault_code);
  }
  // The operator is losing evidence here. Keeping the frame would serve the
  // previous incident's values as this one's, so it goes, but never silently.
  RCLCPP_WARN(logger_,
              "Entity freeze-frame for fault '%s': the stored frame is from an earlier occurrence and entity '%s' "
              "could not be re-read (%s). The stored frame was discarded, so this occurrence has no freeze-frame.",
              fault_code.c_str(), entities.c_str(), reason);
}

nlohmann::json EntityFreezeFrameCapture::values_from_list_content(const nlohmann::json & content) {
  if (!content.contains("items") || !content["items"].is_array()) {
    return content;
  }
  nlohmann::json values = nlohmann::json::object();
  for (const auto & item : content["items"]) {
    if (!item.is_object()) {
      continue;
    }
    std::string key = string_field(item, "id");
    if (key.empty()) {
      key = string_field(item, "name");
    }
    if (key.empty()) {
      continue;
    }
    values[key] = item.contains("value") ? item["value"] : nlohmann::json{};
  }
  return values;
}

bool EntityFreezeFrameCapture::content_has_live_data(const nlohmann::json & content) {
  if (!content.is_object()) {
    return false;
  }
  // Judged on the values, not on the link flag. A bridge that serves its last
  // known values while disconnected is exactly what a loss-of-comms fault wants
  // frozen - refusing on `connected: false` denied a frame to the one fault
  // where the values before the link died are the whole story. The all-null row
  // a genuinely cold cache yields is still rejected, by values_have_data().
  return content.contains("items") && content["items"].is_array() && !content["items"].empty();
}

bool EntityFreezeFrameCapture::content_reports_disconnected(const nlohmann::json & content) {
  return content.is_object() && content.contains("connected") && content["connected"].is_boolean() &&
         !content["connected"].get<bool>();
}

bool EntityFreezeFrameCapture::values_have_data(const nlohmann::json & values) {
  if (values.is_object()) {
    for (const auto & entry : values.items()) {
      if (!entry.value().is_null()) {
        return true;
      }
    }
    return false;
  }
  return !values.is_null();
}

std::optional<std::vector<EntityFreezeFrameCapture::StandingFault>>
EntityFreezeFrameCapture::standing_faults_from_list_reply(const nlohmann::json & data) {
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
    // Seconds on the wire (fault_msg_conversions), nanoseconds here so it can
    // be compared with a frame's captured_at_ns without converting per row.
    // Anything that is not a positive number leaves it at 0, which reads as
    // "the reply did not say" and never costs a stored frame.
    const auto first_occurred = item.find("first_occurred");
    if (first_occurred != item.end() && first_occurred->is_number()) {
      const double seconds = first_occurred->get<double>();
      // The range is checked on the nanosecond product, before the cast: a
      // double outside int64's range makes the conversion undefined, and a
      // NaN fails every comparison so it lands here too. Untrusted input on
      // this path is a malformed or hostile reply, not just a stale clock.
      const double nanoseconds = seconds * 1e9;
      constexpr double kMaxRepresentableNs = 9.2e18;  // below int64 max, with room for the ulp
      if (nanoseconds > 0.0 && nanoseconds < kMaxRepresentableNs) {
        fault.first_occurred_ns = static_cast<int64_t>(nanoseconds);
      }
    }
    standing.push_back(std::move(fault));
  }
  return standing;
}

std::optional<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::frame_from_content(const std::string & entity_id, const std::string & fault_code,
                                             const nlohmann::json & content, const std::string & source) {
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

std::optional<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::capture_via_route(const std::string & entity_id, const std::string & fault_code) {
  std::optional<nlohmann::json> content;
  try {
    content = route_fetcher_(entity_id);
  } catch (const std::exception & e) {
    log_fallback_failure_once(fault_code, std::string("x-plc-data dispatch threw: ") + e.what());
    return std::nullopt;
  }
  if (!content) {
    return std::nullopt;  // not plugin-owned, no x-plc-data route, or handler error
  }
  return frame_from_content(entity_id, fault_code, *content, kSourceXPlcDataRoute);
}

void EntityFreezeFrameCapture::log_fallback_failure_once(const std::string & fault_code, const std::string & message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fallback_logged_.size() >= kMaxLoggedFaultCodes) {
      fallback_logged_.clear();
    }
    if (!fallback_logged_.insert(fault_code).second) {
      return;
    }
  }
  RCLCPP_WARN(logger_, "Entity freeze-frame for fault '%s': %s", fault_code.c_str(), message.c_str());
}

void EntityFreezeFrameCapture::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  // Mirror the fault_manager's own capture trigger: freeze-frames are taken
  // when a fault confirms, not on every update.
  if (msg->event_type != ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED) {
    return;
  }

  // This runs on the shared subscription worker (which also serves all /data
  // sampling and subscribe/unsubscribe), so only enqueue here: the plugin
  // calls run on capture_thread_, where a slow or live read cannot stall the
  // worker and a handler that re-enters the executor cannot self-deadlock.
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      return;
    }
    if (queue_.size() >= kMaxQueuedEvents) {
      RCLCPP_WARN(logger_, "Entity freeze-frame capture backlog full; dropping oldest confirm event");
      queue_.pop_front();
    }
    queue_.push_back(msg);
  }
  queue_cv_.notify_one();
}

void EntityFreezeFrameCapture::wait_for_events_publisher(const std::function<bool()> & should_abort) const {
  if (!subscription_slot_) {
    return;
  }
  const auto deadline = std::chrono::steady_clock::now() + kEventsMatchTimeout;
  while (std::chrono::steady_clock::now() < deadline && !should_abort()) {
    try {
      if (subscription_slot_->publisher_count() > 0) {
        return;
      }
    } catch (const std::exception &) {
      return;  // graph query failed; proceed best-effort
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void EntityFreezeFrameCapture::capture_standing_faults() {
  if (!standing_lister_ && !known_code_lister_) {
    return;
  }
  const auto should_abort = [this] {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return stop_;
  };
  // Snapshot only after the events subscription has matched: a confirm
  // published after the lister's snapshot but before the reader matched would
  // be missed by both paths.
  wait_for_events_publisher(should_abort);
  if (should_abort()) {
    return;
  }
  // The lister blocks until the fault services answer, so it runs here rather
  // than on a thread of its own: this thread is joined by the destructor, and
  // should_abort is what lets that join interrupt the wait.
  std::vector<StandingFault> standing;
  if (standing_lister_) {
    try {
      standing = standing_lister_(should_abort);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Entity freeze-frame startup catch-up failed: standing-fault lister threw: %s", e.what());
      return;
    } catch (...) {
      RCLCPP_WARN(logger_, "Entity freeze-frame startup catch-up failed: standing-fault lister threw");
      return;
    }
  }
  // Runs after the lister has already waited the fault services out, so the
  // extra query costs a round trip rather than a second startup stall.
  prune_frames_for_unknown_faults(should_abort);
  if (!standing_lister_ || should_abort()) {
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
      queued_codes.insert(queued->fault.fault_code);
    }
  }
  // Reloaded frames from an occurrence that has since been cleared and
  // re-confirmed. Identified here, still on disk: whether the row goes is
  // decided below, by whether the re-read could replace it.
  const auto stale = stale_reloaded_codes(standing);

  size_t captured = 0;
  size_t re_read = 0;
  size_t discarded = 0;
  size_t over_cap = 0;

  // ---- Pass 1: the stale codes, in place ----------------------------------
  // Each already owns a slot in frames_, and capture_for_event evicts only when
  // the code is new to the map, so a re-read can neither exceed the
  // retained-frame bound nor push anyone else out. That makes the bound check
  // wrong here in both directions. It would refuse a read that costs nothing,
  // and while these codes are counted as absent a bare code admitted against
  // that under-count would FIFO-evict a live frame that is still wanted. So the
  // stale codes are settled first, and only then is the real occupancy known.
  for (const auto & fault : standing) {
    if (should_abort()) {
      return;
    }
    if (fault.fault_code.empty() || stale.count(fault.fault_code) == 0) {
      continue;
    }
    // A stale code also jumps the queued-confirm skip. That skip exists so one
    // confirm costs one plugin read, but here the alternative is leaving a
    // frame from a dead occurrence in place on the chance the drain loop
    // succeeds. One extra read is the cheaper mistake.
    if (fault.reporting_sources.empty()) {
      // The staleness test and the re-read test must agree on this, or a fault
      // with no entity is stale to one and invisible to the other, and its row
      // survives to be served.
      drop_stale_frame(fault.fault_code, "", "the fault reports no entity to read");
      ++discarded;
      continue;
    }
    ros2_medkit_msgs::msg::FaultEvent event;
    event.event_type = ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED;
    event.fault.fault_code = fault.fault_code;
    event.fault.reporting_sources = fault.reporting_sources;
    // capture_for_event replaces the code's frames and its rows as one
    // delete-then-insert, so a successful re-read swaps the stale frame out
    // without a window in which the fault has none.
    if (capture_for_event(event, /*startup_catchup=*/true)) {
      ++captured;
      ++re_read;
    } else {
      drop_stale_frame(fault.fault_code, join_sources(fault.reporting_sources), "the entity served no usable values");
      ++discarded;  // the slot it held is now free for the pass below
    }
  }

  // ---- Pass 2: the faults that have no frame at all ------------------------
  // Occupancy is read after the stale pass, so it is what frames_ really holds:
  // every stale code has by now been replaced in place or dropped. A bare code
  // is therefore admitted only against a slot that is genuinely free, and the
  // FIFO inside capture_for_event can only reach codes that are neither stale
  // nor mid-re-read.
  std::vector<std::string> over_cap_codes;
  std::unordered_set<std::string> already_framed;
  size_t framed = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    already_framed.reserve(frames_.size());
    for (const auto & entry : frames_) {
      already_framed.insert(entry.first);
    }
    framed = frames_.size();
  }
  for (const auto & fault : standing) {
    if (should_abort()) {
      return;
    }
    if (fault.fault_code.empty() || fault.reporting_sources.empty()) {
      continue;
    }
    // Codes with a confirm already queued belong to the drain loop: capturing
    // them here too would read the plugin twice for one confirm.
    if (queued_codes.count(fault.fault_code) != 0) {
      continue;
    }
    // The stored frame is the one from this fault's own confirm edge. Re-reading
    // the plant now would replace it with today's values under a "startup"
    // marker, which is exactly what persisting the frame is here to stop. This
    // also covers a stale code the pass above just replaced.
    if (already_framed.count(fault.fault_code) != 0) {
      continue;
    }
    if (framed >= max_faults_) {
      ++over_cap;  // storing more would FIFO-evict a frame that is still wanted
      // Named, not just counted: "3 faults went unframed" leaves an operator
      // with no way to tell which fault details are missing their context.
      if (over_cap_codes.size() < kMaxNamedOverCapCodes) {
        over_cap_codes.push_back(fault.fault_code);
      }
      continue;
    }
    ros2_medkit_msgs::msg::FaultEvent event;
    event.event_type = ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED;
    event.fault.fault_code = fault.fault_code;
    event.fault.reporting_sources = fault.reporting_sources;
    if (capture_for_event(event, /*startup_catchup=*/true)) {
      ++framed;
      ++captured;
    }
  }
  if (over_cap > 0) {
    RCLCPP_WARN(logger_,
                "Entity freeze-frame startup catch-up truncated: %zu standing fault(s) beyond the retained-frame "
                "bound of %zu, so they have no freeze-frame: %s%s",
                over_cap, max_faults_, join_sources(over_cap_codes).c_str(),
                over_cap > over_cap_codes.size() ? ", ..." : "");
  }
  if (captured > 0) {
    RCLCPP_INFO(logger_, "Entity freeze-frame: captured %zu fault(s) that were already confirmed at startup", captured);
  }
  if (re_read > 0 || discarded > 0) {
    RCLCPP_INFO(logger_,
                "Entity freeze-frame: %zu reloaded frame(s) from an earlier occurrence re-read, %zu discarded with no "
                "replacement",
                re_read, discarded);
  }
}

void EntityFreezeFrameCapture::capture_worker() {
  capture_standing_faults();
  for (;;) {
    ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr event;
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
      event = queue_.front();
      queue_.pop_front();
    }
    capture_for_event(*event);
  }
}

bool EntityFreezeFrameCapture::capture_for_event(const ros2_medkit_msgs::msg::FaultEvent & event,
                                                 bool startup_catchup) {
  const std::string & fault_code = event.fault.fault_code;

  std::vector<Frame> frames;
  for (const auto & source : event.fault.reporting_sources) {
    DataProvider * provider = resolver_ ? resolver_(source) : nullptr;
    if (provider == nullptr) {
      // No DataProvider: the owning plugin may still serve live values through
      // its own x-plc-data route (the commercial PLC bridges). For non-plugin
      // (ROS) sources the fetcher resolves no owner and returns nullopt.
      if (route_fetcher_) {
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
      const std::string evicted = insertion_order_.front();
      frames_.erase(evicted);
      insertion_order_.pop_front();
      // The store follows the map out: an evicted frame that stayed on disk
      // would come back on the next start and the bound would mean nothing
      // across restarts.
      reloaded_codes_.erase(evicted);
      erase_persisted_locked(evicted);
    }
  }
  // A capture on this fault's own edge supersedes whatever was reloaded for it,
  // so the code is no longer a candidate for the reloaded-frame prune.
  reloaded_codes_.erase(fault_code);
  frames_[fault_code] = std::move(frames);
  // Under the same lock as the map, so the file and what is being served
  // cannot disagree about what was frozen.
  persist_frames_locked(fault_code, frames_[fault_code]);

  RCLCPP_DEBUG(logger_, "Captured entity freeze-frame(s) for fault '%s'", fault_code.c_str());
  return true;
}

}  // namespace ros2_medkit_gateway
