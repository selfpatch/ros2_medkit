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

#include <algorithm>
#include <chrono>
#include <exception>
#include <limits>
#include <memory>
#include <set>
#include <utility>

#include "ros2_medkit_gateway/core/faults/fault_json.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"

namespace ros2_medkit_gateway {

namespace {

namespace fd = ros2_medkit::fault_detection;

/// Capture backlog bound: a confirm burst beyond this drops the oldest event.
constexpr size_t kMaxQueuedEvents = 64;

/// fallback_logged_ bound: past this the set resets (re-arming one warn per
/// code) instead of growing forever on churny/synthetic fault codes.
constexpr size_t kMaxLoggedFaultCodes = 1024;

/// How many fault codes a warning names: the catch-up truncation and the dropped reloaded frames.
constexpr size_t kMaxNamedOverCapCodes = 10;

/// 2^63, exact as a double. A value below it fits in int64.
constexpr double kMaxRepresentableNs = 9223372036854775808.0;

/// The occurrence of one ListFaults reply item.
FreezeFrameCapture::Occurrence occurrence_from_item(const nlohmann::json & item) {
  FreezeFrameCapture::Occurrence occurrence;
  const auto count = item.find("occurrence_count");
  if (count != item.end() && count->is_number_integer()) {
    const int64_t value = count->get<int64_t>();
    if (value > 0 && value <= std::numeric_limits<uint32_t>::max()) {
      occurrence.count = static_cast<uint32_t>(value);
    }
  }
  const auto first_occurred = item.find("first_occurred");
  if (first_occurred != item.end() && first_occurred->is_number()) {
    // Range-checked before the cast, which is undefined outside int64. NaN fails the check.
    const double nanoseconds = first_occurred->get<double>() * 1e9;
    if (nanoseconds > 0.0 && nanoseconds < kMaxRepresentableNs) {
      occurrence.first_occurred_ns = static_cast<int64_t>(nanoseconds);
    }
  }
  return occurrence;
}

/// The key of one record. '\0' cannot appear in a fault code or in a reporting source,
/// so no pair of ids can collide with another pair.
std::string record_key(const std::string & fault_code, const std::string & owner) {
  return fault_code + '\0' + owner;
}

/// The owner of a shared record, and of a record that names none.
constexpr const char * kSharedOwner = "*";

std::string owner_of(const fd::FaultRecord & record) {
  return record.owner.empty() ? kSharedOwner : record.owner;
}

/// The fault code and the owner of a `record_key`.
std::pair<std::string, std::string> split_record_key(const std::string & key) {
  const auto at = key.find('\0');
  return {key.substr(0, at), at == std::string::npos ? std::string{} : key.substr(at + 1)};
}

std::string join(const std::vector<std::string> & items) {
  std::string joined;
  for (const auto & item : items) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += item;
  }
  return joined;
}

}  // namespace

FreezeFrameCapture::FreezeFrameCapture(FaultEventSource & events, const FreezeFrameCaptureConfig & config, Hooks hooks)
  : readers_(std::move(hooks.readers))
  , standing_lister_(std::move(hooks.standing_lister))
  , events_ready_(std::move(hooks.events_ready))
  , store_(std::move(hooks.store))
  , hosted_(std::move(hooks.hosted))
  , max_faults_(config.max_faults > 0 ? config.max_faults : 1)
  , list_retry_interval_(config.list_retry_interval) {
  // Before the first event and the first request: the catch-up must see the reloaded frames.
  load_persisted_frames();
  // With no lister nothing judges the reloaded frames, so they are served as loaded.
  reloaded_judged_ = !standing_lister_;
  subscription_ = events.subscribe([this](const fd::FaultEventRecord & event) {
    on_fault_event(event);
  });
  capture_thread_ = std::thread([this] {
    capture_worker();
  });
}

FreezeFrameCapture::~FreezeFrameCapture() {
  // Returns once no callback runs, so none sees the teardown below.
  subscription_.reset();
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
}

std::vector<FreezeFrameCapture::Frame> FreezeFrameCapture::frames_for(const std::string & fault_code,
                                                                      const std::string & owner) const {
  const auto key = record_key(fault_code, owner);
  std::lock_guard<std::mutex> lock(mutex_);
  // Until judged, a reloaded frame may belong to an earlier occurrence of the record.
  if (!reloaded_judged_ && reloaded_records_.count(key) != 0) {
    return {};
  }
  auto it = frames_.find(key);
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
  // Muting is per record: a muted entry names its record by fault_code and source_id.
  std::unordered_set<std::string> muted;
  const auto muted_faults = data.find("muted_faults");
  if (muted_faults != data.end() && muted_faults->is_array()) {
    for (const auto & item : *muted_faults) {
      if (item.is_object() && item.contains("fault_code") && item["fault_code"].is_string()) {
        const auto source = item.find("source_id");
        const std::string owner = source != item.end() && source->is_string() && !source->get<std::string>().empty()
                                      ? source->get<std::string>()
                                      : kSharedOwner;
        muted.insert(record_key(item["fault_code"].get<std::string>(), owner));
      }
    }
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
    const auto status = item.find("status");
    if (status != item.end() && status->is_string()) {
      fault.status = status->get<std::string>();
    }
    const auto owner = item.find("owner");
    if (owner != item.end() && owner->is_string() && !owner->get<std::string>().empty()) {
      fault.owner = owner->get<std::string>();
    }
    fault.occurrence = occurrence_from_item(item);
    fault.muted = muted.count(record_key(fault.fault_code, fault.owner)) != 0;
    standing.push_back(std::move(fault));
  }
  return standing;
}

FreezeFrameCapture::Occurrence FreezeFrameCapture::occurrence_of(const fd::FaultRecord & record) {
  // The reply's own encoding, so both sources round first_occurred the same way.
  return occurrence_from_item(fault_record_to_json(record));
}

StoredEntityFreezeFrame FreezeFrameCapture::to_stored(const std::string & fault_code, const std::string & owner,
                                                      const Frame & frame, const Occurrence & occurrence) {
  StoredEntityFreezeFrame row;
  row.fault_code = fault_code;
  row.owner = owner;
  row.entity_id = frame.entity_id;
  row.frame = nlohmann::json::object();
  row.frame["values"] = frame.values;
  // An absent field stays absent on reload.
  if (frame.connected.has_value()) {
    row.frame["connected"] = *frame.connected;
  }
  if (!frame.source_timestamp.is_null()) {
    row.frame["source_timestamp"] = frame.source_timestamp;
  }
  row.captured_at_ns = frame.captured_at_ns;
  row.source = frame.source;
  row.capture_origin = frame.startup_catchup ? kCaptureOriginStartup : "";
  row.occurrence_count = occurrence.count;
  row.first_occurred_ns = occurrence.first_occurred_ns;
  return row;
}

std::optional<FreezeFrameCapture::Frame> FreezeFrameCapture::from_stored(const StoredEntityFreezeFrame & row) {
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

void FreezeFrameCapture::write_persisted(const std::string & fault_code, const std::string & owner,
                                         const std::vector<StoredEntityFreezeFrame> & rows) {
  if (!store_) {
    return;
  }
  auto written = store_->replace_frames(fault_code, owner, rows);
  if (!written) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!store_write_warned_) {
      // A read-only or full volume costs one line, and the frames still work in memory.
      store_write_warned_ = true;
      MEDKIT_LOG_WARN(logger_,
                      "Entity freeze-frame store write failed; frames stay in memory and a restart may lose them "
                      "(logged once): %s",
                      written.error().c_str());
    }
  }
}

void FreezeFrameCapture::erase_persisted(const std::vector<std::string> & record_keys) {
  if (!store_) {
    return;
  }
  for (const auto & key : record_keys) {
    const auto [fault_code, owner] = split_record_key(key);
    auto erased = store_->erase_frames(fault_code, owner);
    if (!erased) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!store_write_warned_) {
        store_write_warned_ = true;
        MEDKIT_LOG_WARN(logger_, "Entity freeze-frame store delete failed: %s", erased.error().c_str());
      }
    }
  }
}

void FreezeFrameCapture::load_persisted_frames() {
  if (!store_) {
    return;
  }
  auto rows = store_->load_all();
  if (!rows) {
    MEDKIT_LOG_WARN(logger_, "Entity freeze-frame store unreadable, starting with no reloaded frames: %s",
                    rows.error().c_str());
    return;
  }

  // All keyed by record.
  std::unordered_map<std::string, std::vector<Frame>> loaded;
  std::unordered_map<std::string, int64_t> newest;
  // One capture writes all rows of a record, so they carry one occurrence.
  std::unordered_map<std::string, Occurrence> occurrences;
  size_t unreadable = 0;
  for (const auto & row : *rows) {
    auto frame = from_stored(row);
    if (!frame) {
      ++unreadable;
      continue;
    }
    const auto key = record_key(row.fault_code, row.owner);
    auto it = newest.find(key);
    if (it == newest.end()) {
      newest.emplace(key, row.captured_at_ns);
    } else {
      it->second = std::max(it->second, row.captured_at_ns);
    }
    occurrences[key] = Occurrence{row.occurrence_count, row.first_occurred_ns};
    loaded[key].push_back(std::move(*frame));
  }

  // Oldest record first: past the bound the oldest go, and FIFO eviction keeps that order.
  std::vector<std::string> keys;
  keys.reserve(loaded.size());
  for (const auto & entry : loaded) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end(), [&newest](const std::string & a, const std::string & b) {
    const int64_t at_a = newest.at(a);
    const int64_t at_b = newest.at(b);
    return at_a != at_b ? at_a < at_b : a < b;
  });
  const size_t over_cap = keys.size() > max_faults_ ? keys.size() - max_faults_ : 0;

  size_t reloaded = 0;
  std::vector<std::string> past_the_bound;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < keys.size(); ++i) {
      const auto & key = keys[i];
      if (i < over_cap) {
        // A row past the bound would be read and dropped again on every start.
        past_the_bound.push_back(key);
        continue;
      }
      insertion_order_.push_back(key);
      frames_[key] = std::move(loaded[key]);
      reloaded_records_.insert(key);
      occurrences_[key] = occurrences[key];
    }
    reloaded = frames_.size();
  }
  erase_persisted(past_the_bound);
  if (reloaded > 0) {
    MEDKIT_LOG_INFO(logger_, "Entity freeze-frame: reloaded frames for %zu fault(s) from the store", reloaded);
  }
  if (over_cap > 0) {
    MEDKIT_LOG_WARN(logger_,
                    "Entity freeze-frame store held %zu fault(s) beyond the retained-frame bound of %zu; "
                    "the oldest were dropped",
                    over_cap, max_faults_);
  }
  if (unreadable > 0) {
    MEDKIT_LOG_WARN(logger_, "Entity freeze-frame store: %zu unreadable row(s) skipped", unreadable);
  }
}

void FreezeFrameCapture::judge_reloaded_frames(const std::vector<StandingFault> & listed) {
  std::unordered_map<std::string, const StandingFault *> by_record;
  for (const auto & fault : listed) {
    by_record.emplace(record_key(fault.fault_code, fault.owner), &fault);
  }
  struct Superseded {
    std::string key;
    std::string entities;
    Occurrence stored;
    const StandingFault * fault;
  };
  std::vector<Superseded> superseded;
  std::vector<std::string> orphans;
  std::vector<std::string> dropped;
  bool kept_for_empty_list = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A fault manager whose storage is unreachable answers with an empty list, so empty is "cannot tell".
    kept_for_empty_list = listed.empty() && !reloaded_records_.empty();
    for (const auto & key : reloaded_records_) {
      const auto held = occurrences_.find(key);
      const Occurrence stored = held != occurrences_.end() ? held->second : Occurrence{};
      const auto entry = by_record.find(key);
      if (entry == by_record.end()) {
        if (!listed.empty()) {
          orphans.push_back(key);
        }
        continue;
      }
      const Occurrence & now = entry->second->occurrence;
      // An occurrence that is not known on either side cannot tell: the frame stays.
      if (!stored.known() || !now.known() || now == stored) {
        continue;
      }
      std::vector<std::string> entities;
      for (const auto & frame : frames_[key]) {
        entities.push_back(frame.entity_id);
      }
      superseded.push_back({key, join(entities), stored, entry->second});
    }
    for (const auto & key : orphans) {
      forget_locked(key);
      dropped.push_back(key);
    }
    for (const auto & entry : superseded) {
      forget_locked(entry.key);
      dropped.push_back(entry.key);
    }
    reloaded_judged_ = true;
  }
  // Outside mutex_, so a fault-detail request does not wait for the deletes. The records are
  // gone from memory, and the next capture runs on this thread after the deletes.
  erase_persisted(dropped);
  if (kept_for_empty_list) {
    MEDKIT_LOG_INFO(logger_,
                    "Entity freeze-frame: the fault manager lists no fault in any status, so no reloaded frame is "
                    "dropped");
  }
  if (!orphans.empty()) {
    // Named by code: the records of one code are one fault the fault manager no longer holds.
    std::set<std::string> codes;
    for (const auto & key : orphans) {
      codes.insert(split_record_key(key).first);
    }
    std::vector<std::string> named(codes.begin(), codes.end());
    const size_t total = named.size();
    if (named.size() > kMaxNamedOverCapCodes) {
      named.resize(kMaxNamedOverCapCodes);
    }
    MEDKIT_LOG_WARN(logger_,
                    "Entity freeze-frame: dropped the reloaded frames of %zu fault(s) the fault manager no longer "
                    "holds: %s%s",
                    total, join(named).c_str(), total > named.size() ? ", ..." : "");
  }
  std::sort(superseded.begin(), superseded.end(), [](const Superseded & a, const Superseded & b) {
    return a.key < b.key;
  });
  for (const auto & entry : superseded) {
    // Serving it would present the previous occurrence's values as this one's.
    const auto fault_code = split_record_key(entry.key).first;
    MEDKIT_LOG_WARN(logger_,
                    "Entity freeze-frame for fault '%s': the stored frame of entity '%s' is from an earlier occurrence "
                    "and was discarded (stored occurrence %u, the fault manager lists occurrence %u, %s).",
                    fault_code.c_str(), entry.entities.c_str(), entry.stored.count, entry.fault->occurrence.count,
                    entry.fault->status.c_str());
  }
}

void FreezeFrameCapture::forget_locked(const std::string & key) {
  frames_.erase(key);
  insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), key), insertion_order_.end());
  reloaded_records_.erase(key);
  occurrences_.erase(key);
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
  // A route payload carries them at the top level, a provider payload in its `x-medkit` block.
  const nlohmann::json * link = &content;
  if (content.contains("x-medkit") && content["x-medkit"].is_object() && !content.contains("connected")) {
    link = &content["x-medkit"];
  }
  if (link->contains("connected") && (*link)["connected"].is_boolean()) {
    frame.connected = (*link)["connected"].get<bool>();
  }
  if (link->contains("timestamp")) {
    frame.source_timestamp = (*link)["timestamp"];
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

std::optional<FreezeFrameCapture::Frame> FreezeFrameCapture::capture_from_entity(const std::string & entity_id,
                                                                                 const std::string & fault_code) {
  DataProvider * provider = readers_.provider ? readers_.provider(entity_id) : nullptr;
  if (provider == nullptr) {
    // No DataProvider: the owning plugin may still serve live values through
    // its own x-plc-data route (the commercial PLC bridges). For non-plugin
    // (ROS) sources the fetcher resolves no owner and returns nullopt.
    if (!readers_.route) {
      return std::nullopt;
    }
    return capture_via_route(entity_id, fault_code);
  }

  // list_data is expected to serve from the plugin's latest polled values
  // (cheap); plugin code still gets exception-guarded like every other
  // provider call site.
  try {
    auto result = provider->list_data(entity_id);
    if (!result) {
      log_fallback_failure_once(fault_code, "list_data('" + entity_id + "') failed: " + result.error().message);
      return std::nullopt;
    }
    return frame_from_content(entity_id, fault_code, result->content, kSourceDataProvider);
  } catch (const std::exception & e) {
    log_fallback_failure_once(fault_code, "plugin threw for entity '" + entity_id + "': " + e.what());
  }
  return std::nullopt;
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

std::optional<std::vector<FreezeFrameCapture::StandingFault>> FreezeFrameCapture::list_standing_faults() {
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
  size_t failures = 0;
  for (;;) {
    if (should_abort()) {
      return std::nullopt;
    }
    // The lister blocks until the fault services answer, so it runs here rather
    // than on a thread of its own: this thread is joined by the destructor, and
    // should_abort is what lets that join interrupt the wait.
    std::optional<std::vector<StandingFault>> listed;
    std::string reason;
    try {
      listed = standing_lister_(should_abort);
    } catch (const std::exception & e) {
      reason = std::string(" (the lister threw: ") + e.what() + ")";
    } catch (...) {
      reason = " (the lister threw)";
    }
    if (listed) {
      if (failures > 0) {
        MEDKIT_LOG_INFO(logger_,
                        "Entity freeze-frame: the startup fault list was read after %zu failed attempt(s); the "
                        "reloaded frames are judged now",
                        failures);
      }
      return listed;
    }
    if (should_abort()) {
      return std::nullopt;
    }
    if (failures++ == 0) {
      // A fault cleared and confirmed again while the gateway was down would serve the
      // previous occurrence's frame, so the reloaded frames wait for a list.
      MEDKIT_LOG_WARN(logger_,
                      "Entity freeze-frame: the startup fault list could not be read%s, so the reloaded frames stay "
                      "hidden and the list is retried every %g s",
                      reason.c_str(), static_cast<double>(list_retry_interval_.count()) / 1000.0);
    }
    if (!capture_confirms_until(std::chrono::steady_clock::now() + list_retry_interval_)) {
      return std::nullopt;
    }
  }
}

bool FreezeFrameCapture::capture_confirms_until(std::chrono::steady_clock::time_point deadline) {
  for (;;) {
    fd::FaultEventRecord event;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      const bool woken = queue_cv_.wait_until(lock, deadline, [this] {
        return stop_ || !queue_.empty();
      });
      if (stop_) {
        return false;
      }
      if (!woken) {
        return true;
      }
      event = std::move(queue_.front());
      queue_.pop_front();
    }
    capture_for_event(event, occurrence_of(event.fault));
  }
}

void FreezeFrameCapture::capture_standing_faults(const std::vector<StandingFault> & listed) {
  const auto should_abort = [this] {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return stop_;
  };
  // Records with a confirm already queued belong to the drain loop: capturing
  // them here too would read the plugin twice for one confirm. The key is the
  // record and not the code, because another source's record of the same code
  // has its own confirm and its own frame.
  std::unordered_set<std::string> queued_records;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      return;
    }
    for (const auto & queued : queue_) {
      queued_records.insert(record_key(queued.fault.fault_code, owner_of(queued.fault)));
    }
  }

  // Occupancy is read once, so a record is admitted only against a free slot and the
  // FIFO never evicts a frame still wanted.
  std::unordered_set<std::string> already_framed;
  size_t framed = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : frames_) {
      already_framed.insert(entry.first);
    }
    framed = frames_.size();
  }
  size_t captured = 0;
  size_t over_cap = 0;
  std::vector<std::string> over_cap_codes;
  for (const auto & fault : listed) {
    if (should_abort()) {
      return;
    }
    // What a live confirm event would frame: a confirmed fault that is not muted.
    if (fault.status != fd::fault_status::kConfirmed || fault.muted) {
      continue;
    }
    if (fault.fault_code.empty() || fault.reporting_sources.empty()) {
      continue;
    }
    const auto key = record_key(fault.fault_code, fault.owner);
    // A queued confirm frames the record, and a reloaded frame from this occurrence is the one to serve.
    if (queued_records.count(key) != 0 || already_framed.count(key) != 0) {
      continue;
    }
    if (framed >= max_faults_) {
      ++over_cap;  // storing more would FIFO-evict a frame that is still wanted
      if (over_cap_codes.size() < kMaxNamedOverCapCodes) {
        over_cap_codes.push_back(fault.fault_code);
      }
      continue;
    }
    fd::FaultEventRecord event;
    event.event_type = fd::fault_event_type::kConfirmed;
    event.fault.fault_code = fault.fault_code;
    event.fault.owner = fault.owner;
    event.fault.reporting_sources = fault.reporting_sources;
    if (capture_for_event(event, fault.occurrence, /*startup_catchup=*/true)) {
      ++framed;
      ++captured;
    }
  }
  if (over_cap > 0) {
    MEDKIT_LOG_WARN(logger_,
                    "Entity freeze-frame startup catch-up truncated: %zu standing fault(s) beyond the retained-frame "
                    "bound of %zu, so they have no freeze-frame: %s%s",
                    over_cap, max_faults_, join(over_cap_codes).c_str(),
                    over_cap > over_cap_codes.size() ? ", ..." : "");
  }
  if (captured > 0) {
    MEDKIT_LOG_INFO(logger_, "Entity freeze-frame: captured %zu fault(s) that were already confirmed at startup",
                    captured);
  }
}

void FreezeFrameCapture::capture_worker() {
  if (standing_lister_) {
    const auto listed = list_standing_faults();
    if (!listed) {
      return;
    }
    // One answer decides both the reloaded frames and what is read now.
    judge_reloaded_frames(*listed);
    capture_standing_faults(*listed);
  }
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
    capture_for_event(event, occurrence_of(event.fault));
  }
}

bool FreezeFrameCapture::capture_for_event(const fd::FaultEventRecord & event, const Occurrence & occurrence,
                                           bool startup_catchup) {
  const std::string & fault_code = event.fault.fault_code;
  const std::string owner = owner_of(event.fault);

  std::vector<Frame> frames;
  // At most one frame per entity: the store keys a record's rows by entity id.
  std::unordered_set<std::string> framed;
  for (const auto & source : event.fault.reporting_sources) {
    if (framed.count(source) != 0) {
      continue;
    }
    if (auto frame = capture_from_entity(source, fault_code)) {
      framed.insert(source);
      frames.push_back(std::move(*frame));
      continue;
    }
    if (!hosted_) {
      continue;
    }
    // The source reads nothing of its own: frame the entities it hosts instead.
    std::vector<std::string> hosted;
    try {
      hosted = hosted_(source);
    } catch (const std::exception & e) {
      log_fallback_failure_once(fault_code, "hosted-entity lookup threw for '" + source + "': " + e.what());
      continue;
    }
    for (const auto & hosted_id : hosted) {
      if (hosted_id == source || framed.count(hosted_id) != 0) {
        continue;
      }
      if (auto frame = capture_from_entity(hosted_id, fault_code)) {
        framed.insert(hosted_id);
        frames.push_back(std::move(*frame));
      }
    }
  }

  const auto key = record_key(fault_code, owner);
  if (frames.empty()) {
    uint32_t dropped_count = 0;
    std::vector<std::string> dropped_entities;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto held = occurrences_.find(key);
      // The same occurrence, or one that cannot be told, keeps the frame from its confirmation.
      if (held == occurrences_.end() || !held->second.known() || !occurrence.known() || held->second == occurrence) {
        return false;
      }
      dropped_count = held->second.count;
      const auto frames_it = frames_.find(key);
      if (frames_it != frames_.end()) {
        for (const auto & frame : frames_it->second) {
          dropped_entities.push_back(frame.entity_id);
        }
      }
      forget_locked(key);
    }
    erase_persisted({key});
    // Serving the held frame would present the previous occurrence's values as this one's.
    MEDKIT_LOG_WARN(logger_,
                    "Entity freeze-frame for fault '%s': occurrence %u confirmed and no entity read for it served "
                    "values, so this occurrence has no freeze-frame. The frame of occurrence %u (entity '%s') was "
                    "dropped.",
                    fault_code.c_str(), occurrence.count, dropped_count, join(dropped_entities).c_str());
    return false;
  }
  if (startup_catchup) {
    for (auto & frame : frames) {
      frame.startup_catchup = true;
    }
  }

  std::vector<std::string> evicted;
  std::vector<StoredEntityFreezeFrame> rows;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames_.find(key) != frames_.end()) {
      // A recapture is the newest frame, as its captured_at says after a reload.
      insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), key),
                             insertion_order_.end());
      insertion_order_.push_back(key);
    } else {
      insertion_order_.push_back(key);
      while (frames_.size() >= max_faults_ && !insertion_order_.empty()) {
        const std::string oldest = insertion_order_.front();
        insertion_order_.pop_front();
        forget_locked(oldest);
        // The rows go too, or the evicted frame comes back on the next start.
        evicted.push_back(oldest);
      }
    }
    // A capture on this record's own edge supersedes what was reloaded for it.
    reloaded_records_.erase(key);
    occurrences_[key] = occurrence;
    auto & stored = frames_[key];
    stored = std::move(frames);
    if (store_) {
      rows.reserve(stored.size());
      for (const auto & frame : stored) {
        rows.push_back(to_stored(fault_code, owner, frame, occurrence));
      }
    }
  }
  // Outside mutex_, so a fault-detail request does not wait for the store.
  erase_persisted(evicted);
  write_persisted(fault_code, owner, rows);

  MEDKIT_LOG_DEBUG(logger_, "Captured entity freeze-frame(s) for fault '%s' of '%s'", fault_code.c_str(),
                   owner.c_str());
  return true;
}

std::unique_ptr<FreezeFrameCapture> make_freeze_frame_capture(const FreezeFrameCaptureConfig & config,
                                                              FaultEventSource & events, PluginManager * plugins,
                                                              FreezeFrameCapture::StandingFaultLister standing_lister,
                                                              FreezeFrameCapture::EventsReadyWaiter events_ready,
                                                              std::shared_ptr<EntityFreezeFrameStore> store,
                                                              FreezeFrameCapture::HostedEntitiesResolver hosted) {
  // Only plugins own entities whose values the fault manager cannot capture.
  if (!config.enabled || plugins == nullptr || !plugins->has_plugins()) {
    return nullptr;
  }
  FreezeFrameCapture::Hooks hooks;
  hooks.readers = plugin_data_readers(plugins);
  hooks.standing_lister = std::move(standing_lister);
  hooks.events_ready = std::move(events_ready);
  hooks.store = std::move(store);
  hooks.hosted = std::move(hosted);
  return std::make_unique<FreezeFrameCapture>(events, config, std::move(hooks));
}

}  // namespace ros2_medkit_gateway
