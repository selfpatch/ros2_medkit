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

// The core capture with a store, in process. Four tests need a retained-frame bound
// below the gateway node's literal 256. Two order a request inside the start: one holds
// the lister, one holds a store delete.

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/faults/entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_gateway/core/faults/fault_json.hpp"
#include "ros2_medkit_gateway/core/faults/freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/core/faults/sqlite_entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/core/logging.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::DataProvider;
using ros2_medkit_gateway::DataProviderError;
using ros2_medkit_gateway::DataProviderErrorInfo;
using ros2_medkit_gateway::EntityFreezeFrameStore;
using ros2_medkit_gateway::FaultEventSource;
using ros2_medkit_gateway::FaultEventSubscription;
using ros2_medkit_gateway::FreezeFrameCapture;
using ros2_medkit_gateway::FreezeFrameCaptureConfig;
using ros2_medkit_gateway::InMemoryEntityFreezeFrameStore;
using ros2_medkit_gateway::PluginDataReaders;
using ros2_medkit_gateway::SqliteEntityFreezeFrameStore;
using ros2_medkit_gateway::StoredEntityFreezeFrame;
using ros2_medkit_gateway::handlers::FaultHandlers;
namespace fd = ros2_medkit::fault_detection;
namespace mlog = ros2_medkit_gateway::log;

namespace {

/// Log lines of the process, so a test can wait for one and assert on its text.
class RecordingSink : public mlog::Sink {
 public:
  bool enabled(const std::string & /*logger*/, mlog::Level level) const override {
    return level >= mlog::Level::kInfo;
  }

  void write(const std::string & /*logger*/, mlog::Level level, std::string_view message,
             const mlog::Location & /*location*/) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      text_ += level >= mlog::Level::kWarn ? "WARN " : "INFO ";
      text_.append(message.data(), message.size());
      text_ += '\n';
    }
    cv_.notify_all();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    text_.clear();
  }

  std::string text() {
    std::lock_guard<std::mutex> lock(mutex_);
    return text_;
  }

  bool wait_for(const std::string & needle, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      return text_.find(needle) != std::string::npos;
    });
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::string text_;
};

/// Installed once, before the first capture thread logs.
RecordingSink & log_sink() {
  static const std::shared_ptr<RecordingSink> sink = [] {
    auto created = std::make_shared<RecordingSink>();
    mlog::set_sink(created);
    return created;
  }();
  return *sink;
}

/// A fault event source the test drives by hand.
class FakeFaultEvents : public FaultEventSource {
 public:
  FaultEventSubscription subscribe(Callback callback) override {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(callback);
    return FaultEventSubscription([this] {
      std::lock_guard<std::mutex> cancel_lock(mutex_);
      callback_ = nullptr;
    });
  }

  void deliver(const fd::FaultEventRecord & event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (callback_) {
      callback_(event);
    }
  }

 private:
  std::mutex mutex_;
  Callback callback_;
};

fd::FaultEventRecord confirmed(const std::string & fault_code, const std::vector<std::string> & sources) {
  fd::FaultEventRecord event;
  event.event_type = fd::fault_event_type::kConfirmed;
  event.fault.fault_code = fault_code;
  event.fault.reporting_sources = sources;
  return event;
}

/// Plugin DataProvider of one entity, with a settable temperature.
class FakePlcDataProvider : public DataProvider {
 public:
  explicit FakePlcDataProvider(std::string entity_id) : entity_id_(std::move(entity_id)) {
  }

  void set_temperature(double value) {
    temperature_.store(value);
  }

  tl::expected<ros2_medkit_gateway::dto::DataListResult, DataProviderErrorInfo>
  list_data(const std::string & entity_id) override {
    if (entity_id != entity_id_) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::EntityNotFound, "not found", 404});
    }
    json items = json::array();
    items.push_back({{"id", "temperature"}, {"value", temperature_.load()}});
    items.push_back({{"id", "pressure"}, {"value", 3.2}});
    return ros2_medkit_gateway::dto::DataListResult{json{{"items", std::move(items)}}};
  }

  tl::expected<ros2_medkit_gateway::dto::DataValue, DataProviderErrorInfo>
  read_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "unused", 500});
  }

  tl::expected<ros2_medkit_gateway::dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/,
             const json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only", 405});
  }

 private:
  std::string entity_id_;
  std::atomic<double> temperature_{42.5};
};

/// Entity of the catch-up probe. It serves nothing, so its read stores no frame and evicts nothing.
constexpr const char * kProbeEntity = "probe_entity";

/// The x-plc-data route of the plant. Serves `level` for every entity, counts reads per
/// entity, and can make an entity unreachable.
class Plant {
 public:
  explicit Plant(double level) : level_(level) {
  }

  void make_unreachable(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    unreachable_.insert(entity_id);
  }

  std::optional<json> read(const std::string & entity_id) {
    std::optional<json> answer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reads_[entity_id] += 1;
      if (entity_id == kProbeEntity || unreachable_.count(entity_id) != 0) {
        answer = std::nullopt;
      } else {
        answer = json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", level_}}})}};
      }
    }
    cv_.notify_all();
    return answer;
  }

  int reads(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = reads_.find(entity_id);
    return it == reads_.end() ? 0 : it->second;
  }

  /// Waits until @p entity_id has been read at least @p count times.
  bool wait_for_reads(const std::string & entity_id, int count, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      const auto it = reads_.find(entity_id);
      return it != reads_.end() && it->second >= count;
    });
  }

 private:
  double level_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::map<std::string, int> reads_;
  std::set<std::string> unreachable_;
};

constexpr int64_t kStoredAtNs = 1'000'000'000'000'000'000;

using Occurrence = FreezeFrameCapture::Occurrence;

/// Occurrences as the fault manager numbers them. Their times sit on the fault manager's
/// clock, so they say nothing about a frame's captured_at.
constexpr Occurrence kFirstOccurrence{1, kStoredAtNs - 120'000'000'000};
constexpr Occurrence kSecondOccurrence{2, kStoredAtNs - 60'000'000'000};

/// One stored row, shaped as the capture writes it.
StoredEntityFreezeFrame make_stored_row(const std::string & fault_code, const std::string & entity_id,
                                        int64_t captured_at_ns, double level = 7.0, Occurrence occurrence = {}) {
  StoredEntityFreezeFrame row;
  row.fault_code = fault_code;
  row.entity_id = entity_id;
  row.frame = json{{"values", {{"level", level}}}, {"connected", false}, {"source_timestamp", "2026-09-08T17:51:40Z"}};
  row.captured_at_ns = captured_at_ns;
  row.source = FreezeFrameCapture::kSourceXPlcDataRoute;
  row.occurrence_count = occurrence.count;
  row.first_occurred_ns = occurrence.first_occurred_ns;
  return row;
}

FreezeFrameCapture::StandingFault standing_fault(const std::string & code, const std::vector<std::string> & sources,
                                                 Occurrence occurrence = {}, const std::string & status = "CONFIRMED",
                                                 bool muted = false) {
  FreezeFrameCapture::StandingFault fault;
  fault.fault_code = code;
  fault.reporting_sources = sources;
  fault.status = status;
  fault.occurrence = occurrence;
  fault.muted = muted;
  return fault;
}

FreezeFrameCapture::StandingFaultLister listed(std::vector<FreezeFrameCapture::StandingFault> faults) {
  return [faults = std::move(faults)](const std::function<bool()> &) {
    return faults;
  };
}

std::vector<std::string> stored_codes(EntityFreezeFrameStore & store) {
  std::vector<std::string> codes;
  auto rows = store.load_all();
  if (rows) {
    for (const auto & row : *rows) {
      codes.push_back(row.fault_code);
    }
  }
  std::sort(codes.begin(), codes.end());
  return codes;
}

/// An in-memory store whose erase_frames, or replace_frames, waits until the test opens
/// the latch.
class LatchStore : public EntityFreezeFrameStore {
 public:
  enum class Held { kErase, kReplace };

  explicit LatchStore(Held held) : held_(held) {
  }

  tl::expected<void, std::string> replace_frames(const std::string & fault_code,
                                                 const std::vector<StoredEntityFreezeFrame> & frames) override {
    if (held_ == Held::kReplace) {
      hold();
    }
    return rows_.replace_frames(fault_code, frames);
  }

  tl::expected<void, std::string> erase_frames(const std::string & fault_code) override {
    if (held_ == Held::kErase) {
      hold();
    }
    return rows_.erase_frames(fault_code);
  }

  tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() override {
    return rows_.load_all();
  }

  /// Seeds a row without passing the latch.
  void seed(const std::string & fault_code, const std::vector<StoredEntityFreezeFrame> & frames) {
    ASSERT_TRUE(rows_.replace_frames(fault_code, frames));
  }

  bool wait_until_held(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this] {
      return holding_;
    });
  }

  void open() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      open_ = true;
    }
    cv_.notify_all();
  }

 private:
  void hold() {
    std::unique_lock<std::mutex> lock(mutex_);
    holding_ = true;
    cv_.notify_all();
    cv_.wait(lock, [this] {
      return open_;
    });
  }

  const Held held_;
  InMemoryEntityFreezeFrameStore rows_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool holding_{false};
  bool open_{false};
};

// The bound probe: two slots, one live reloaded frame (PLC_KEEP, the older, so the FIFO
// front), one stale reloaded frame (PLC_STALE) and one standing fault with no frame (PLC_NEW).
constexpr int64_t kKeepAtNs = kStoredAtNs;
constexpr int64_t kStaleAtNs = kStoredAtNs + 10'000'000'000;
constexpr Occurrence kKeepOccurrence{1, kKeepAtNs - 60'000'000'000};
constexpr Occurrence kStaleStoredOccurrence{1, kStaleAtNs - 60'000'000'000};
constexpr Occurrence kStaleOccurrence{2, kStaleAtNs + 60'000'000'000};

std::vector<FreezeFrameCapture::StandingFault> bound_probe_standing() {
  return {standing_fault("PLC_NEW", {"route_new"}, {1, kStaleAtNs + 30'000'000'000}),
          standing_fault("PLC_STALE", {"route_stale"}, kStaleOccurrence),  // a later occurrence: stale
          standing_fault("PLC_KEEP", {"route_keep"}, kKeepOccurrence)};    // the stored occurrence: live
}

void seed_bound_probe(EntityFreezeFrameStore & store) {
  ASSERT_TRUE(
      store.replace_frames("PLC_KEEP", {make_stored_row("PLC_KEEP", "route_keep", kKeepAtNs, 10.0, kKeepOccurrence)}));
  ASSERT_TRUE(store.replace_frames(
      "PLC_STALE", {make_stored_row("PLC_STALE", "route_stale", kStaleAtNs, 11.0, kStaleStoredOccurrence)}));
}

class FreezeFrameCapturePersistenceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    log_sink().clear();
  }

  std::unique_ptr<FreezeFrameCapture> make_capture(size_t max_faults, Plant & plant,
                                                   std::shared_ptr<EntityFreezeFrameStore> store,
                                                   FreezeFrameCapture::StandingFaultLister standing = nullptr,
                                                   DataProvider * provider = nullptr,
                                                   std::chrono::milliseconds list_retry = 5s) {
    FreezeFrameCapture::Hooks hooks;
    hooks.readers = PluginDataReaders{[provider](const std::string & entity_id) -> DataProvider * {
                                        return entity_id == "plc_app" ? provider : nullptr;
                                      },
                                      [&plant](const std::string & entity_id) {
                                        return plant.read(entity_id);
                                      }};
    hooks.standing_lister = std::move(standing);
    hooks.store = std::move(store);
    return std::make_unique<FreezeFrameCapture>(events_, FreezeFrameCaptureConfig{true, max_faults, list_retry},
                                                std::move(hooks));
  }

  /// Queues a probe confirm and waits for its read. The drain loop starts after the
  /// catch-up and handles confirms in order, so the read shows both are done.
  bool wait_for_catch_up(Plant & plant) {
    const int probes = plant.reads(kProbeEntity);
    events_.deliver(confirmed("PROBE_AFTER_CATCH_UP", {kProbeEntity}));
    return plant.wait_for_reads(kProbeEntity, probes + 1, 15s);
  }

  /// Polls until the capture holds frames for @p fault_code.
  static bool wait_for_frames(const FreezeFrameCapture & capture, const std::string & fault_code) {
    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!capture.frames_for(fault_code).empty()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  FakeFaultEvents events_;
};

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, CaptureWritesTheFrameThroughToTheStore) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  FakePlcDataProvider provider("plc_app");
  Plant plant(0.0);
  auto capture = make_capture(256, plant, store, nullptr, &provider);

  auto event = confirmed("PLC_PERSIST", {"plc_app"});
  event.fault.occurrence_count = 3;
  event.fault.first_occurred_ns = 1'788'948'705'123'456'789;
  events_.deliver(event);
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_PERSIST"));
  const auto served = capture->frames_for("PLC_PERSIST");
  ASSERT_EQ(served.size(), 1u);

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_EQ((*rows)[0].fault_code, "PLC_PERSIST");
  EXPECT_EQ((*rows)[0].entity_id, "plc_app");
  EXPECT_EQ((*rows)[0].captured_at_ns, served[0].captured_at_ns);
  EXPECT_EQ((*rows)[0].frame["values"], served[0].values);
  EXPECT_EQ((*rows)[0].source, FreezeFrameCapture::kSourceDataProvider);
  EXPECT_EQ((*rows)[0].capture_origin, "");  // captured on the confirm edge
  // The occurrence as a ListFaults reply of the same record carries it, so a restart can match it.
  const auto replied = FreezeFrameCapture::standing_faults_from_list_reply(
      json{{"faults", json::array({ros2_medkit_gateway::fault_record_to_json(event.fault)})}});
  ASSERT_TRUE(replied.has_value());
  ASSERT_EQ(replied->size(), 1u);
  EXPECT_EQ((*rows)[0].occurrence_count, 3u);
  EXPECT_NE((*rows)[0].first_occurred_ns, 0);
  EXPECT_EQ((*rows)[0].first_occurred_ns, (*replied)[0].occurrence.first_occurred_ns);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ReConfirmOverwritesTheStoredRow) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  FakePlcDataProvider provider("plc_app");
  Plant plant(0.0);
  auto capture = make_capture(256, plant, store, nullptr, &provider);

  events_.deliver(confirmed("PLC_RECONFIRM", {"plc_app"}));
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_RECONFIRM"));

  // One frame per fault: a stale row would bring the previous occurrence back on restart.
  provider.set_temperature(99.0);
  events_.deliver(confirmed("PLC_RECONFIRM", {"plc_app"}));
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  bool overwritten = false;
  while (std::chrono::steady_clock::now() < deadline && !overwritten) {
    auto rows = store->load_all();
    overwritten = rows.has_value() && rows->size() == 1u &&
                  std::abs((*rows)[0].frame["values"].value("temperature", 0.0) - 99.0) < 1e-9;
    if (!overwritten) {
      std::this_thread::sleep_for(5ms);
    }
  }
  EXPECT_TRUE(overwritten);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AReloadedFrameIsServedExactlyAsItWasCaptured) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  json first_wire;
  int64_t captured_at_ns = 0;
  {
    Plant plant(41.0);
    auto capture = make_capture(256, plant, store);
    events_.deliver(confirmed("PLC_RESTART", {"route_plc_app"}));
    ASSERT_TRUE(wait_for_frames(*capture, "PLC_RESTART"));
    const auto frames = capture->frames_for("PLC_RESTART");
    ASSERT_EQ(frames.size(), 1u);
    captured_at_ns = frames[0].captured_at_ns;
    first_wire = FaultHandlers::merge_entity_freeze_frames(json{{"snapshots", json::array()}}, frames);
  }

  // A second life on the same store, with the plant reading something else.
  Plant moved_on(999.0);
  auto restarted = make_capture(256, moved_on, store);
  const auto reloaded = restarted->frames_for("PLC_RESTART");
  ASSERT_EQ(reloaded.size(), 1u);
  EXPECT_EQ(reloaded[0].captured_at_ns, captured_at_ns);
  EXPECT_FALSE(reloaded[0].startup_catchup);
  const auto second_wire = FaultHandlers::merge_entity_freeze_frames(json{{"snapshots", json::array()}}, reloaded);
  EXPECT_EQ(second_wire, first_wire);
  ASSERT_EQ(second_wire["snapshots"].size(), 1u);
  EXPECT_FALSE(second_wire["snapshots"][0].contains("capture_origin"));
  EXPECT_EQ(moved_on.reads("route_plc_app"), 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, CatchUpSkipsAReloadedCodeAndFramesOneWithoutARow) {
  // The stored row has no occurrence, so the judgement cannot tell: the frame is kept.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_HAS_FRAME", {make_stored_row("PLC_HAS_FRAME", "route_stored_app", 4242)}));

  Plant plant(7.0);
  auto capture = make_capture(256, plant, store,
                              listed({standing_fault("PLC_HAS_FRAME", {"route_stored_app"}, kSecondOccurrence),
                                      standing_fault("PLC_NO_FRAME", {"route_fresh_app"}, kFirstOccurrence)}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  // Positive control: a standing fault with no row still gets its startup frame.
  const auto fresh = capture->frames_for("PLC_NO_FRAME");
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_TRUE(fresh[0].startup_catchup);

  const auto stored = capture->frames_for("PLC_HAS_FRAME");
  ASSERT_EQ(stored.size(), 1u);
  EXPECT_EQ(stored[0].captured_at_ns, 4242);  // the stored captured_at
  EXPECT_FALSE(stored[0].startup_catchup);
  EXPECT_EQ(plant.reads("route_stored_app"), 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ReloadDropsAFaultTheManagerNoLongerHoldsAndKeepsAClearedOne) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_GONE", {make_stored_row("PLC_GONE", "route_a", 100)}));
  ASSERT_TRUE(store->replace_frames("PLC_CLEARED", {make_stored_row("PLC_CLEARED", "route_b", 200)}));
  // Past the 10 codes the warning names.
  for (int i = 1; i <= 10; ++i) {
    const std::string code = "PLC_GONE_" + std::string(i < 10 ? "0" : "") + std::to_string(i);
    ASSERT_TRUE(store->replace_frames(code, {make_stored_row(code, "route_a", 100 + i)}));
  }

  Plant plant(7.0);
  // The fault manager reports the cleared fault and knows nothing of the other.
  auto capture = make_capture(256, plant, store, listed({standing_fault("PLC_CLEARED", {"route_b"}, {}, "CLEARED")}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_TRUE(capture->frames_for("PLC_GONE").empty());
  EXPECT_FALSE(capture->frames_for("PLC_CLEARED").empty());  // a cleared fault keeps its frame
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_CLEARED"}));
  const auto logs = log_sink().text();
  EXPECT_NE(logs.find("WARN Entity freeze-frame: dropped the reloaded frames of 11 fault(s) the fault manager no "
                      "longer holds: PLC_GONE, PLC_GONE_01, PLC_GONE_02, PLC_GONE_03, PLC_GONE_04, PLC_GONE_05, "
                      "PLC_GONE_06, PLC_GONE_07, PLC_GONE_08, PLC_GONE_09, ...\n"),
            std::string::npos)
      << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ALiveConfirmIsCapturedWhileTheFaultListIsRetried) {
  // In process: the lister fails on every call, which a fault manager that never answers
  // cannot show. Same seeding as the test above, whose lister can answer and drops PLC_GONE.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_GONE", {make_stored_row("PLC_GONE", "route_a", 100)}));
  ASSERT_TRUE(store->replace_frames("PLC_CLEARED", {make_stored_row("PLC_CLEARED", "route_b", 200)}));

  Plant plant(7.0);
  std::atomic<int> calls{0};
  auto capture = make_capture(
      256, plant, store,
      [&calls](const std::function<bool()> &) -> std::optional<std::vector<FreezeFrameCapture::StandingFault>> {
        // The first call throws, the later ones cannot list: both count as a failure.
        if (calls.fetch_add(1) == 0) {
          throw std::runtime_error("fault manager unreachable");
        }
        return std::nullopt;
      },
      nullptr, 20ms);
  ASSERT_TRUE(log_sink().wait_for("the list is retried every 0.02 s", 15s));

  // A confirm of PLC_CLEARED while no list has succeeded is read at once.
  events_.deliver(confirmed("PLC_CLEARED", {"route_b"}));
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_CLEARED"));
  const auto live = capture->frames_for("PLC_CLEARED");
  ASSERT_EQ(live.size(), 1u);
  EXPECT_NE(live[0].captured_at_ns, 200);  // the live capture replaced the reloaded frame
  EXPECT_DOUBLE_EQ(live[0].values.value("level", 0.0), 7.0);

  // Later attempts fail with nullopt: still nothing is served or dropped.
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (calls.load() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GE(calls.load(), 3);
  EXPECT_TRUE(capture->frames_for("PLC_GONE").empty()) << "a reloaded frame was served before a list succeeded";
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_CLEARED", "PLC_GONE"}));
  const auto logs = log_sink().text();
  const std::string warned = "the startup fault list could not be read";
  EXPECT_EQ(logs.find(warned), logs.rfind(warned)) << "one warning for many failed attempts\n" << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, StoppingDuringAFaultListRetryReturnsPromptly) {
  // In process: the stop must land inside the retry wait, which only a held lister can order.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_HELD", {make_stored_row("PLC_HELD", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);
  std::atomic<int> calls{0};
  auto capture = make_capture(
      256, plant, store,
      [&calls](const std::function<bool()> &) -> std::optional<std::vector<FreezeFrameCapture::StandingFault>> {
        calls.fetch_add(1);
        return std::nullopt;
      },
      nullptr, std::chrono::hours(1));
  // Logged right before the wait for the next attempt starts.
  ASSERT_TRUE(log_sink().wait_for("the list is retried every 3600 s", 15s));
  EXPECT_TRUE(capture->frames_for("PLC_HELD").empty());

  const auto started = std::chrono::steady_clock::now();
  capture.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - started, 5s) << "the destructor waited for the retry interval";
  EXPECT_EQ(calls.load(), 1);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AnEmptyFaultListDropsNoReloadedFrame) {
  // A fault manager whose storage is unreachable answers with an empty list. That must
  // not cost frames that cannot be read again, so an empty answer is "cannot tell".
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_A", {make_stored_row("PLC_A", "route_a", 100)}));
  ASSERT_TRUE(store->replace_frames("PLC_B", {make_stored_row("PLC_B", "route_b", 200)}));

  Plant plant(7.0);
  auto capture = make_capture(256, plant, store, listed({}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_FALSE(capture->frames_for("PLC_A").empty());
  EXPECT_FALSE(capture->frames_for("PLC_B").empty());
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_A", "PLC_B"}));
  const auto logs = log_sink().text();
  EXPECT_NE(logs.find("Entity freeze-frame: the fault manager lists no fault in any status, so no reloaded frame is "
                      "dropped"),
            std::string::npos)
      << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, TheRetainedFrameBoundCountsReloadedFrames) {
  const auto path =
      std::filesystem::temp_directory_path() / ("test_freeze_frame_bound_" + std::to_string(::getpid()) + ".db");
  std::filesystem::remove(path);
  auto store = std::make_shared<SqliteEntityFreezeFrameStore>(path.string());
  ASSERT_TRUE(store->replace_frames("PLC_LOADED_OLDEST", {make_stored_row("PLC_LOADED_OLDEST", "route_o", 50)}));
  ASSERT_TRUE(store->replace_frames("PLC_LOADED_A", {make_stored_row("PLC_LOADED_A", "route_a", 100)}));
  ASSERT_TRUE(store->replace_frames("PLC_LOADED_B", {make_stored_row("PLC_LOADED_B", "route_b", 200)}));
  // Two unreadable rows, one of each kind, both skipped and counted: JSON that is not a
  // frame, and a column that is not JSON at all.
  StoredEntityFreezeFrame unreadable = make_stored_row("PLC_UNREADABLE", "route_u", 300);
  unreadable.frame = json{{"not_values", 1}};
  ASSERT_TRUE(store->replace_frames("PLC_UNREADABLE", {unreadable}));
  {
    sqlite3 * db = nullptr;
    ASSERT_EQ(sqlite3_open(path.c_str(), &db), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db,
                           "INSERT INTO entity_freeze_frames "
                           "(fault_code, entity_id, frame, captured_at_ns, source, capture_origin) "
                           "VALUES ('PLC_NOT_JSON','route_n','{not json',250,'','')",
                           nullptr, nullptr, nullptr),
              SQLITE_OK);
    sqlite3_close(db);
  }

  // The fault manager still holds the reloaded faults, so the list keeps their frames.
  const auto standing =
      listed({standing_fault("PLC_FRESH", {"route_fresh"}), standing_fault("PLC_LOADED_A", {"route_a"}, {}, "CLEARED"),
              standing_fault("PLC_LOADED_B", {"route_b"}, {}, "CLEARED")});

  {
    // Bound of 2. The store holds 3 readable codes, so the oldest is dropped at load, and
    // the 2 reloaded frames fill the bound: catching PLC_FRESH up would evict one of them.
    Plant plant(7.0);
    auto capped = make_capture(/*max_faults=*/2, plant, store, standing);
    ASSERT_TRUE(wait_for_catch_up(plant));
    EXPECT_TRUE(capped->frames_for("PLC_FRESH").empty());
    EXPECT_EQ(plant.reads("route_fresh"), 0);
    EXPECT_TRUE(capped->frames_for("PLC_LOADED_OLDEST").empty());
    EXPECT_FALSE(capped->frames_for("PLC_LOADED_A").empty());
    EXPECT_FALSE(capped->frames_for("PLC_LOADED_B").empty());
    const auto logs = log_sink().text();
    EXPECT_NE(logs.find("reloaded frames for 2 fault(s) from the store"), std::string::npos) << logs;
    EXPECT_NE(logs.find("held 1 fault(s) beyond the retained-frame bound of 2"), std::string::npos) << logs;
    EXPECT_NE(logs.find("2 unreadable row(s) skipped"), std::string::npos) << logs;
    EXPECT_NE(logs.find("so they have no freeze-frame: PLC_FRESH"), std::string::npos) << logs;
  }
  // The rows past the bound are gone with the frames; unreadable rows stay as they are.
  EXPECT_EQ(stored_codes(*store),
            (std::vector<std::string>{"PLC_LOADED_A", "PLC_LOADED_B", "PLC_NOT_JSON", "PLC_UNREADABLE"}));

  // Positive control: one more slot and the same catch-up frames PLC_FRESH.
  {
    Plant plant(7.0);
    auto roomy = make_capture(/*max_faults=*/3, plant, store, standing);
    ASSERT_TRUE(wait_for_catch_up(plant));
    EXPECT_FALSE(roomy->frames_for("PLC_FRESH").empty());
  }
  store.reset();
  std::filesystem::remove(path);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AnEvictedFaultLosesItsStoredRowToo) {
  // Otherwise an evicted frame comes back on the next start and the file grows without bound.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  FakePlcDataProvider provider("plc_app");
  Plant plant(0.0);
  auto capture = make_capture(/*max_faults=*/1, plant, store, nullptr, &provider);

  events_.deliver(confirmed("PLC_EVICT_FIRST", {"plc_app"}));
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_EVICT_FIRST"));
  events_.deliver(confirmed("PLC_EVICT_SECOND", {"plc_app"}));
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_EVICT_SECOND"));

  EXPECT_TRUE(capture->frames_for("PLC_EVICT_FIRST").empty());
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_EVICT_SECOND"}));
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ARecaptureRefreshesTheEvictionAge) {
  // At the bound, a fault confirmed again is the newest frame, so the next new fault
  // evicts the next-oldest one, in this life and after a reload.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  {
    Plant plant(7.0);
    auto capture = make_capture(/*max_faults=*/3, plant, store);
    for (const char * code : {"PLC_FIRST", "PLC_SECOND", "PLC_THIRD"}) {
      events_.deliver(confirmed(code, {"route_app"}));
      ASSERT_TRUE(wait_for_frames(*capture, code));
    }
    const int64_t first_capture = capture->frames_for("PLC_FIRST")[0].captured_at_ns;
    events_.deliver(confirmed("PLC_FIRST", {"route_app"}));
    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline &&
           capture->frames_for("PLC_FIRST")[0].captured_at_ns == first_capture) {
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_GT(capture->frames_for("PLC_FIRST")[0].captured_at_ns, first_capture);
    events_.deliver(confirmed("PLC_FOURTH", {"route_app"}));
    ASSERT_TRUE(wait_for_frames(*capture, "PLC_FOURTH"));

    EXPECT_FALSE(capture->frames_for("PLC_FIRST").empty()) << "the fault just confirmed again was evicted";
    EXPECT_TRUE(capture->frames_for("PLC_SECOND").empty());
    EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_FIRST", "PLC_FOURTH", "PLC_THIRD"}));
  }

  Plant plant(7.0);
  auto reloaded = make_capture(/*max_faults=*/3, plant, store);
  events_.deliver(confirmed("PLC_FIFTH", {"route_app"}));
  ASSERT_TRUE(wait_for_frames(*reloaded, "PLC_FIFTH"));
  EXPECT_FALSE(reloaded->frames_for("PLC_FIRST").empty()) << "after a reload the recaptured fault was evicted";
  EXPECT_TRUE(reloaded->frames_for("PLC_THIRD").empty());
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_FIFTH", "PLC_FIRST", "PLC_FOURTH"}));
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AReloadedFrameFromAnEarlierOccurrenceIsReReadAndMarkedStartup) {
  // The fault cleared and confirmed again while the gateway was down: the stored frame
  // holds the previous occurrence's values. The fault manager's clock is behind the
  // gateway's, so the new occurrence's first_occurred predates the stored captured_at.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_REOCCUR", {make_stored_row("PLC_REOCCUR", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));

  Plant plant(99.0);
  auto capture =
      make_capture(256, plant, store, listed({standing_fault("PLC_REOCCUR", {"route_stored_app"}, kSecondOccurrence)}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  const auto served = capture->frames_for("PLC_REOCCUR");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_DOUBLE_EQ(served[0].values.value("level", 0.0), 99.0);  // read at this start
  EXPECT_TRUE(served[0].startup_catchup);
  EXPECT_GT(served[0].captured_at_ns, kStoredAtNs);

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_DOUBLE_EQ((*rows)[0].frame["values"].value("level", 0.0), 99.0);
  EXPECT_EQ((*rows)[0].capture_origin, "startup");
  EXPECT_EQ(plant.reads("route_stored_app"), 1);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ANewOccurrenceWithNoValuesDropsTheFrameOfThePreviousOne) {
  // PLC_RELOADED holds a reloaded frame judged valid at start, PLC_LIVE one captured in
  // this life. Both then confirm as their next occurrence while the entity is down.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_RELOADED", {make_stored_row("PLC_RELOADED", "route_reloaded", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);
  auto capture =
      make_capture(256, plant, store, listed({standing_fault("PLC_RELOADED", {"route_reloaded"}, kFirstOccurrence)}));
  ASSERT_TRUE(wait_for_catch_up(plant));
  ASSERT_EQ(capture->frames_for("PLC_RELOADED").size(), 1u);

  auto live_first = confirmed("PLC_LIVE", {"route_live"});
  live_first.fault.occurrence_count = kFirstOccurrence.count;
  live_first.fault.first_occurred_ns = kFirstOccurrence.first_occurred_ns;
  events_.deliver(live_first);
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_LIVE"));

  plant.make_unreachable("route_reloaded");
  plant.make_unreachable("route_live");
  for (const char * code : {"PLC_RELOADED", "PLC_LIVE"}) {
    auto second = confirmed(code, {std::string(code) == "PLC_LIVE" ? "route_live" : "route_reloaded"});
    second.fault.occurrence_count = kSecondOccurrence.count;
    second.fault.first_occurred_ns = kSecondOccurrence.first_occurred_ns;
    events_.deliver(second);
  }
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_EQ(plant.reads("route_reloaded"), 1);
  EXPECT_EQ(plant.reads("route_live"), 2);
  EXPECT_TRUE(capture->frames_for("PLC_RELOADED").empty()) << "occurrence 2 serves the frame of occurrence 1";
  EXPECT_TRUE(capture->frames_for("PLC_LIVE").empty()) << "occurrence 2 serves the frame of occurrence 1";
  EXPECT_TRUE(stored_codes(*store).empty());
  const auto logs = log_sink().text();
  for (const char * code : {"PLC_RELOADED", "PLC_LIVE"}) {
    EXPECT_NE(logs.find(std::string("WARN Entity freeze-frame for fault '") + code + "': occurrence 2 confirmed"),
              std::string::npos)
        << logs;
  }
  EXPECT_NE(logs.find("so this occurrence has no freeze-frame"), std::string::npos) << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, TheSameOccurrenceWithNoValuesKeepsItsFrame) {
  // A confirm of the occurrence the frame belongs to, or one that carries no occurrence,
  // with nothing to read: the frame from the confirmation stays.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_SAME",
                                    {make_stored_row("PLC_SAME", "route_same", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);
  auto capture =
      make_capture(256, plant, store, listed({standing_fault("PLC_SAME", {"route_same"}, kFirstOccurrence)}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  plant.make_unreachable("route_same");
  auto same = confirmed("PLC_SAME", {"route_same"});
  same.fault.occurrence_count = kFirstOccurrence.count;
  same.fault.first_occurred_ns = kFirstOccurrence.first_occurred_ns;
  events_.deliver(same);
  events_.deliver(confirmed("PLC_SAME", {"route_same"}));  // no occurrence
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_EQ(plant.reads("route_same"), 2);
  const auto served = capture->frames_for("PLC_SAME");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_EQ(served[0].captured_at_ns, kStoredAtNs);
  EXPECT_DOUBLE_EQ(served[0].values.value("level", 0.0), 10.0);
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_SAME"}));
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AStaleFrameWhoseEntityCannotBeReReadIsDroppedWithAWarning) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_LINK_DOWN", {make_stored_row("PLC_LINK_DOWN", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);
  plant.make_unreachable("route_stored_app");

  auto capture =
      make_capture(256, plant, store,
                   listed({standing_fault("PLC_LINK_DOWN", {"route_stored_app"}, {2, kStoredAtNs + 60'000'000'000})}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_EQ(plant.reads("route_stored_app"), 1);  // the re-read was tried
  EXPECT_TRUE(capture->frames_for("PLC_LINK_DOWN").empty());
  EXPECT_TRUE(stored_codes(*store).empty());
  const auto logs = log_sink().text();
  EXPECT_NE(logs.find("WARN Entity freeze-frame for fault 'PLC_LINK_DOWN': the stored frame of entity "
                      "'route_stored_app' is from an earlier occurrence and was discarded (stored occurrence 1, the "
                      "fault manager lists occurrence 2, CONFIRMED)."),
            std::string::npos)
      << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AReloadedFrameFromTheSameOccurrenceIsKeptUnmarked) {
  // Control for the earlier-occurrence test: the fault never cleared, so the fault
  // manager lists the stored occurrence.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_REOCCUR", {make_stored_row("PLC_REOCCUR", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);

  auto capture =
      make_capture(256, plant, store, listed({standing_fault("PLC_REOCCUR", {"route_stored_app"}, kFirstOccurrence)}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  const auto served = capture->frames_for("PLC_REOCCUR");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_DOUBLE_EQ(served[0].values.value("level", 0.0), 10.0);
  EXPECT_EQ(served[0].captured_at_ns, kStoredAtNs);
  EXPECT_FALSE(served[0].startup_catchup);
  EXPECT_EQ(plant.reads("route_stored_app"), 0);

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_DOUBLE_EQ((*rows)[0].frame["values"].value("level", 0.0), 10.0);
  EXPECT_EQ((*rows)[0].captured_at_ns, kStoredAtNs);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AMutedFaultWithAnotherOccurrenceLosesItsStaleFrame) {
  // A muted fault has no confirm event, so nothing frames its new occurrence. The stored
  // frame is from the previous one and must not be served for it.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_MUTED", {make_stored_row("PLC_MUTED", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);

  auto capture = make_capture(
      256, plant, store,
      listed({standing_fault("PLC_MUTED", {"route_stored_app"}, kSecondOccurrence, "CONFIRMED", /*muted=*/true)}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_TRUE(capture->frames_for("PLC_MUTED").empty());
  EXPECT_TRUE(stored_codes(*store).empty());
  EXPECT_EQ(plant.reads("route_stored_app"), 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AFaultListedAsClearedWithAnotherOccurrenceIsNotReadAtStart) {
  // It confirmed and cleared again while the gateway was down: a read now would not
  // show the values at that confirmation.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_FLAP", {make_stored_row("PLC_FLAP", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);

  auto capture = make_capture(256, plant, store,
                              listed({standing_fault("PLC_FLAP", {"route_stored_app"}, kSecondOccurrence, "CLEARED")}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_TRUE(capture->frames_for("PLC_FLAP").empty());
  EXPECT_EQ(plant.reads("route_stored_app"), 0);
  EXPECT_TRUE(stored_codes(*store).empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AReloadedFrameIsNotServedBeforeItIsJudged) {
  // The REST server answers before the capture has asked the fault manager.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames(
      "PLC_HELD", {make_stored_row("PLC_HELD", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)}));
  Plant plant(99.0);

  std::mutex gate_mutex;
  std::condition_variable gate_cv;
  bool entered = false;
  bool released = false;
  auto capture =
      make_capture(256, plant, store,
                   [&](const std::function<bool()> & should_abort) -> std::vector<FreezeFrameCapture::StandingFault> {
                     std::unique_lock<std::mutex> lock(gate_mutex);
                     entered = true;
                     gate_cv.notify_all();
                     while (!released && !should_abort()) {
                       gate_cv.wait_for(lock, 10ms);
                     }
                     return {standing_fault("PLC_HELD", {"route_stored_app"}, kFirstOccurrence)};
                   });
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    ASSERT_TRUE(gate_cv.wait_for(lock, 15s, [&] {
      return entered;
    }));
  }
  EXPECT_TRUE(capture->frames_for("PLC_HELD").empty()) << "a reloaded frame was served before it was judged";

  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    released = true;
  }
  gate_cv.notify_all();
  ASSERT_TRUE(wait_for_catch_up(plant));
  const auto served = capture->frames_for("PLC_HELD");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_EQ(served[0].captured_at_ns, kStoredAtNs);
  EXPECT_EQ(plant.reads("route_stored_app"), 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AFaultDetailReadDoesNotWaitForTheStoreDeletesOfTheJudgement) {
  // In process: the store delete is held open to order a fault-detail read inside it.
  auto store = std::make_shared<LatchStore>(LatchStore::Held::kErase);
  store->seed("PLC_ORPHAN", {make_stored_row("PLC_ORPHAN", "route_stored_app", kStoredAtNs, 10.0, kFirstOccurrence)});
  Plant plant(99.0);
  auto capture = make_capture(256, plant, store, listed({standing_fault("PLC_OTHER", {}, kFirstOccurrence)}));
  // Opens the latch before the capture joins its thread, also when an assertion returns early.
  const std::shared_ptr<void> open_on_exit(nullptr, [&store](void *) {
    store->open();
  });
  ASSERT_TRUE(store->wait_until_held(15s));

  auto read = std::async(std::launch::async, [&capture] {
    return capture->frames_for("ANY").size();
  });
  const bool answered = read.wait_for(5s) == std::future_status::ready;
  store->open();
  EXPECT_TRUE(answered) << "frames_for waited for the store delete of a dropped frame";
  EXPECT_EQ(read.get(), 0u);
  ASSERT_TRUE(wait_for_catch_up(plant));
  EXPECT_TRUE(capture->frames_for("PLC_ORPHAN").empty());
  EXPECT_TRUE(stored_codes(*store).empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, AFaultDetailReadDoesNotWaitForTheStoreWriteOfACapture) {
  // In process: the store write is held open to order a fault-detail read inside it.
  auto store = std::make_shared<LatchStore>(LatchStore::Held::kReplace);
  FakePlcDataProvider provider("plc_app");
  Plant plant(0.0);
  auto capture = make_capture(256, plant, store, nullptr, &provider);
  // Opens the latch before the capture joins its thread, also when an assertion returns early.
  const std::shared_ptr<void> open_on_exit(nullptr, [&store](void *) {
    store->open();
  });
  events_.deliver(confirmed("PLC_WRITTEN", {"plc_app"}));
  ASSERT_TRUE(store->wait_until_held(15s));

  auto read = std::async(std::launch::async, [&capture] {
    return capture->frames_for("ANY").size();
  });
  const bool answered = read.wait_for(5s) == std::future_status::ready;
  store->open();
  EXPECT_TRUE(answered) << "frames_for waited for the store write of a capture";
  EXPECT_EQ(read.get(), 0u);
  ASSERT_TRUE(wait_for_frames(*capture, "PLC_WRITTEN"));
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (stored_codes(*store).empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_WRITTEN"}));
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ASameOccurrenceFrameIsKeptWhenTheGatewayClockIsBehind) {
  // The gateway's clock is behind the fault manager's, so the frame's captured_at
  // predates the occurrence's first_occurred. The occurrence is the stored one.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  const Occurrence occurrence{1, kStoredAtNs + 30'000'000'000};
  ASSERT_TRUE(store->replace_frames(
      "PLC_SKEWED", {make_stored_row("PLC_SKEWED", "route_stored_app", kStoredAtNs, 10.0, occurrence)}));
  Plant plant(99.0);

  auto capture =
      make_capture(256, plant, store, listed({standing_fault("PLC_SKEWED", {"route_stored_app"}, occurrence)}));
  ASSERT_TRUE(wait_for_catch_up(plant));

  EXPECT_EQ(plant.reads("route_stored_app"), 0);
  const auto served = capture->frames_for("PLC_SKEWED");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_DOUBLE_EQ(served[0].values.value("level", 0.0), 10.0);
  EXPECT_EQ(served[0].captured_at_ns, kStoredAtNs);
  EXPECT_FALSE(served[0].startup_catchup);
}

/// @verifies REQ_INTEROP_088
TEST_F(FreezeFrameCapturePersistenceTest, ADiscardedStaleFrameAtTheBoundNeverCostsALiveFrame) {
  // Both slots hold reloaded frames. The stale one is discarded, which frees one slot for
  // the two confirmed faults with no frame: the first listed gets it, the other is left out.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  seed_bound_probe(*store);
  Plant plant(99.0);

  auto capture = make_capture(/*max_faults=*/2, plant, store, listed(bound_probe_standing()));
  ASSERT_TRUE(wait_for_catch_up(plant));

  const auto keep = capture->frames_for("PLC_KEEP");
  ASSERT_EQ(keep.size(), 1u) << "the live reloaded frame was evicted";
  EXPECT_DOUBLE_EQ(keep[0].values.value("level", 0.0), 10.0);
  EXPECT_EQ(keep[0].captured_at_ns, kKeepAtNs);
  EXPECT_FALSE(keep[0].startup_catchup);

  const auto fresh = capture->frames_for("PLC_NEW");
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_TRUE(fresh[0].startup_catchup);
  EXPECT_TRUE(capture->frames_for("PLC_STALE").empty());
  EXPECT_EQ(plant.reads("route_keep"), 0);
  EXPECT_EQ(plant.reads("route_new"), 1);
  EXPECT_EQ(plant.reads("route_stale"), 0);

  EXPECT_EQ(stored_codes(*store), (std::vector<std::string>{"PLC_KEEP", "PLC_NEW"}));
  const auto logs = log_sink().text();
  EXPECT_NE(logs.find("so they have no freeze-frame: PLC_STALE"), std::string::npos) << logs;
}
