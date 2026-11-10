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

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/faults/rosbag_recording.hpp"
#include "ros2_medkit_gateway/core/gateway_services.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"

using namespace ros2_medkit_gateway;
using faults::RecordingLookupStatus;

namespace {

/// A fault manager as the lookup sees it: recordings, the faults they cover, and a
/// switch for each way of not answering.
class FakeFaultTransport : public FaultServiceTransport {
 public:
  struct Recording {
    std::string path;
    std::vector<std::string> fault_codes;
  };

  std::map<std::string, Recording> recordings;
  /// fault code -> reporting sources
  std::map<std::string, std::vector<std::string>> faults;
  bool get_rosbag_unavailable{false};
  /// Stands for a store that could not be read: an answer, but not about the recording.
  bool get_rosbag_declines_everything{false};
  bool get_fault_unavailable{false};
  bool list_rosbags_fails{false};
  /// Filters whose listing fails while the others answer.
  std::set<std::string> list_rosbags_fails_for;
  int list_rosbags_calls{0};

  FaultResult get_rosbag(const std::string & id) override {
    FaultResult r;
    r.success = false;
    if (get_rosbag_unavailable) {
      r.failure = FaultFailure::Unavailable;
      return r;
    }
    if (get_rosbag_declines_everything) {
      r.error_message = "Fault storage unavailable";
      return r;
    }
    auto found = recordings.find(id);
    if (found == recordings.end()) {
      // The fault manager's fallback: read the id as a fault code and serve that
      // fault's newest recording.
      for (auto it = recordings.rbegin(); it != recordings.rend(); ++it) {
        const auto & codes = it->second.fault_codes;
        if (std::find(codes.begin(), codes.end(), id) != codes.end()) {
          r.success = true;
          r.data = {{"file_path", it->second.path}, {"recording_id", it->first}, {"fault_codes", codes}};
          return r;
        }
      }
      r.error_message = "Fault not found: " + id;
      return r;
    }
    r.success = true;
    r.data = {{"file_path", found->second.path}, {"recording_id", id}, {"fault_codes", found->second.fault_codes}};
    return r;
  }

  FaultResult list_rosbags(const std::string & entity_fqn) override {
    ++list_rosbags_calls;
    FaultResult r;
    r.success = !list_rosbags_fails && list_rosbags_fails_for.count(entity_fqn) == 0;
    if (!r.success) {
      return r;
    }
    json rows = json::array();
    for (const auto & [id, recording] : recordings) {
      for (const auto & code : recording.fault_codes) {
        const auto sources = faults.find(code);
        if (sources == faults.end()) {
          continue;
        }
        if (std::find(sources->second.begin(), sources->second.end(), entity_fqn) != sources->second.end()) {
          rows.push_back({{"fault_code", code}, {"recording_id", id}, {"file_path", recording.path}});
        }
      }
    }
    r.data = {{"rosbags", rows}};
    return r;
  }

  FaultResult get_fault(const std::string & fault_code, const std::string & /*source_id*/) override {
    FaultResult r;
    r.success = false;
    if (get_fault_unavailable) {
      r.failure = FaultFailure::Unavailable;
      return r;
    }
    const auto found = faults.find(fault_code);
    if (found == faults.end()) {
      return r;
    }
    r.success = true;
    r.data = {{"fault_code", fault_code}, {"reporting_sources", found->second}};
    return r;
  }

  FaultResult report_fault(const std::string & /*fault_code*/, uint8_t /*severity*/,
                           const std::string & /*description*/, const std::string & /*source_id*/) override {
    return {};
  }
  FaultResult list_faults(const std::string & /*source_id*/, bool /*include_prefailed*/, bool /*include_confirmed*/,
                          bool /*include_cleared*/, bool /*include_healed*/, bool /*include_muted*/,
                          bool /*include_clusters*/) override {
    return {};
  }
  FaultWithEnvJsonResult get_fault_with_env(const std::string & /*fault_code*/,
                                            const std::string & /*source_id*/) override {
    return {};
  }
  FaultResult clear_fault(const std::string & /*fault_code*/, bool /*skip_correlation_auto_clear*/) override {
    return {};
  }
  FaultResult get_snapshots(const std::string & /*fault_code*/, const std::string & /*topic*/) override {
    return {};
  }
  bool wait_for_services(std::chrono::duration<double> /*timeout*/) override {
    return true;
  }
  bool is_available() const override {
    return true;
  }
};

class FindRecordingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transport_ = std::make_shared<FakeFaultTransport>();
    manager_ = std::make_unique<FaultManager>(transport_);
    transport_->recordings["fault_LIDAR_1000"] = {"/var/bags/fault_LIDAR_1000", {"LIDAR"}};
    transport_->recordings["fault_MOTOR_2000"] = {"/var/bags/fault_MOTOR_2000", {"MOTOR"}};
    transport_->faults["LIDAR"] = {"/sensors/lidar"};
    transport_->faults["MOTOR"] = {"/powertrain/motor"};
  }

  faults::RecordingLookup find(const std::string & id, const std::vector<std::string> & filters = {"/sensors/lidar"}) {
    return faults::find_recording(*manager_, id, filters);
  }

  std::shared_ptr<FakeFaultTransport> transport_;
  std::unique_ptr<FaultManager> manager_;
};

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, ARecordingOfTheEntityIsFound) {
  const auto lookup = find("fault_LIDAR_1000");
  EXPECT_EQ(lookup.status, RecordingLookupStatus::kFound);
  EXPECT_EQ(lookup.path, "/var/bags/fault_LIDAR_1000");
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, ARecordingOfAnotherEntityIsNotReportedAsMissing) {
  // It exists, so the answer must not be the one that proves absence.
  EXPECT_EQ(find("fault_MOTOR_2000").status, RecordingLookupStatus::kNotInEntity);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, APrefixSiblingSourceDoesNotOwnTheRecording) {
  transport_->faults["LIDAR"] = {"/sensors/lidar_rear"};
  EXPECT_EQ(find("fault_LIDAR_1000").status, RecordingLookupStatus::kNotInEntity);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, AFaultCodeIsNotARecordingId) {
  // The fault manager resolves the code to the fault's newest recording. That is an
  // answer about the id too: no recording carries it.
  EXPECT_EQ(find("LIDAR").status, RecordingLookupStatus::kNotFound);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, AnUnknownIdIsMissingOnceTheListingConfirmsIt) {
  const auto lookup = find("fault_LIDAR_9999");
  EXPECT_EQ(lookup.status, RecordingLookupStatus::kNotFound);
  EXPECT_EQ(transport_->list_rosbags_calls, 1);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, AnUnansweredLookupIsNotAnAbsentRecording) {
  transport_->get_rosbag_unavailable = true;
  EXPECT_EQ(find("fault_LIDAR_1000").status, RecordingLookupStatus::kUnavailable);
  EXPECT_EQ(find("fault_LIDAR_9999").status, RecordingLookupStatus::kUnavailable);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, ADeclinedLookupWithNoListingIsNotAnAbsentRecording) {
  transport_->list_rosbags_fails = true;
  EXPECT_EQ(find("fault_LIDAR_9999").status, RecordingLookupStatus::kUnavailable);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, ADeclinedLookupOfAListedRecordingIsNotAnAbsentRecording) {
  // The store could not be read for the lookup, and the listing still carries the id.
  transport_->get_rosbag_declines_everything = true;
  EXPECT_EQ(find("fault_LIDAR_1000").status, RecordingLookupStatus::kUnavailable);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, AnUnansweredFaultReadIsNotARefusal) {
  transport_->get_fault_unavailable = true;
  EXPECT_EQ(find("fault_LIDAR_1000").status, RecordingLookupStatus::kUnavailable);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, EveryFilterOfTheEntityIsListedBeforeAbsenceIsClaimed) {
  const auto lookup = find("fault_LIDAR_9999", {"/sensors/lidar", "/sensors/camera", "/sensors/imu"});
  EXPECT_EQ(lookup.status, RecordingLookupStatus::kNotFound);
  EXPECT_EQ(transport_->list_rosbags_calls, 3);
}

// @verifies REQ_INTEROP_073
TEST_F(FindRecordingTest, ABurstRecordingIsFoundThroughAnyOfItsFaults) {
  transport_->recordings["fault_LIDAR_1000"].fault_codes = {"MOTOR", "LIDAR"};
  EXPECT_EQ(find("fault_LIDAR_1000").status, RecordingLookupStatus::kFound);
  EXPECT_EQ(find("fault_LIDAR_1000", {"/powertrain/motor"}).status, RecordingLookupStatus::kFound);
  EXPECT_EQ(find("fault_LIDAR_1000", {"/chassis"}).status, RecordingLookupStatus::kNotInEntity);
}

// === Range header ===

// @verifies REQ_INTEROP_073
TEST(RecordingRangeTest, RangesInsideTheFileAreAccepted) {
  EXPECT_TRUE(faults::range_within_file("bytes=0-", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=99-", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=0-99", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=10-10", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=-1", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=-500", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=0-9, 20-29", 100));
  EXPECT_TRUE(faults::range_within_file("bytes=-", 100));
}

// @verifies REQ_INTEROP_073
TEST(RecordingRangeTest, AStartAtOrPastTheEndIsRefused) {
  EXPECT_FALSE(faults::range_within_file("bytes=100-", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=200000-", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=100-150", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=0-", 0));
}

// @verifies REQ_INTEROP_073
TEST(RecordingRangeTest, AnEndPastTheFileIsRefused) {
  EXPECT_FALSE(faults::range_within_file("bytes=0-100", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=50-99999999999999999999", 100));
}

// @verifies REQ_INTEROP_073
TEST(RecordingRangeTest, OneBadRangeRefusesTheWholeHeader) {
  EXPECT_FALSE(faults::range_within_file("bytes=0-9,100-", 100));
}

// @verifies REQ_INTEROP_073
TEST(RecordingRangeTest, WhatIsNotAByteRangeIsRefused) {
  EXPECT_FALSE(faults::range_within_file("", 100));
  EXPECT_FALSE(faults::range_within_file("items=0-5", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=abc-", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=5-2", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=-0", 100));
  EXPECT_FALSE(faults::range_within_file("bytes=7", 100));
}

// === Manifest and file addressing ===

class RecordingFilesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() / ("rosbag_recording_test_" + std::to_string(getpid()));
    std::filesystem::remove_all(root_);
    bag_ = root_ / "fault_LIDAR_1000";
    std::filesystem::create_directories(bag_);
    std::ofstream(bag_ / "fault_LIDAR_1000_0.mcap") << "first";
    std::ofstream(bag_ / "fault_LIDAR_1000_1.mcap") << "second";
    std::ofstream(root_ / "outside.mcap") << "not part of the recording";
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  static std::string manifest_naming(const std::vector<json> & names) {
    json files = json::array();
    for (const auto & name : names) {
      files.push_back({{"name", name}, {"size", 1}, {"sha256", ""}});
    }
    return json{{"manifest_version", 1}, {"files", files}}.dump();
  }

  std::filesystem::path root_;
  std::filesystem::path bag_;
};

// @verifies REQ_INTEROP_073
TEST_F(RecordingFilesTest, TheManifestIsReadByteForByte) {
  EXPECT_FALSE(faults::recording_has_manifest(bag_.string()));
  EXPECT_FALSE(faults::read_recording_manifest(bag_.string()).has_value());

  const std::string bytes = "{\n  \"manifest_version\": 1,\r\n \"files\": []  }\n\n";
  std::ofstream(bag_ / "manifest.json", std::ios::binary) << bytes;
  EXPECT_TRUE(faults::recording_has_manifest(bag_.string()));
  EXPECT_EQ(faults::read_recording_manifest(bag_.string()), bytes);
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingFilesTest, AMissingRecordingDirectoryHasNoManifest) {
  EXPECT_FALSE(faults::recording_has_manifest((root_ / "absent").string()));
  EXPECT_FALSE(faults::recording_has_manifest(""));
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingFilesTest, AFileIsAddressedByItsPositionInTheManifest) {
  const auto manifest = manifest_naming({"fault_LIDAR_1000_0.mcap", "fault_LIDAR_1000_1.mcap"});
  EXPECT_EQ(faults::recording_file_path(bag_.string(), manifest, 0), (bag_ / "fault_LIDAR_1000_0.mcap").string());
  EXPECT_EQ(faults::recording_file_path(bag_.string(), manifest, 1), (bag_ / "fault_LIDAR_1000_1.mcap").string());
  EXPECT_FALSE(faults::recording_file_path(bag_.string(), manifest, 2).has_value());
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingFilesTest, AManifestCannotNameAFileOutsideItsDirectory) {
  for (const json & name : {json("../outside.mcap"), json((root_ / "outside.mcap").string()), json(".."), json("."),
                            json(""), json("sub/../../outside.mcap"), json(7), json(nullptr)}) {
    EXPECT_FALSE(faults::recording_file_path(bag_.string(), manifest_naming({name}), 0).has_value()) << name.dump();
  }
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingFilesTest, AFileMissingOnDiskIsNotServed) {
  EXPECT_FALSE(faults::recording_file_path(bag_.string(), manifest_naming({"fault_LIDAR_1000_7.mcap"}), 0).has_value());
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingFilesTest, AManifestThatIsNotOneYieldsNoFile) {
  EXPECT_FALSE(faults::recording_file_path(bag_.string(), "not json", 0).has_value());
  EXPECT_FALSE(faults::recording_file_path(bag_.string(), "[]", 0).has_value());
  EXPECT_FALSE(faults::recording_file_path(bag_.string(), R"({"files": {"0": "x"}})", 0).has_value());
  EXPECT_FALSE(faults::recording_file_path(bag_.string(), R"({"files": ["fault_LIDAR_1000_0.mcap"]})", 0).has_value());
}

// @verifies REQ_INTEROP_073
TEST(RecordingFileIndexTest, OnlyDecimalDigitsAreAnIndex) {
  EXPECT_EQ(faults::parse_recording_file_index("0"), 0u);
  EXPECT_EQ(faults::parse_recording_file_index("11"), 11u);
  EXPECT_EQ(faults::parse_recording_file_index("007"), 7u);
  for (const char * text : {"", "-1", "+1", "1.0", "0x1", "one", " 1", "1 ", "fault_LIDAR_1000_0.mcap", "1234567890"}) {
    EXPECT_FALSE(faults::parse_recording_file_index(text).has_value()) << text;
  }
}

// === Listing descriptor ===

// @verifies REQ_INTEROP_072
TEST(RecordingDescriptorTest, TheManifestKeyIsPresentOnlyForARecordingThatHasOne) {
  const std::vector<json> rows = {
      {{"fault_code", "LIDAR"}, {"recording_id", "fault_LIDAR_1000"}, {"format", "mcap"}, {"manifest", true}},
      {{"fault_code", "MOTOR"}, {"recording_id", "fault_MOTOR_2000"}, {"format", "mcap"}, {"manifest", false}},
      {{"fault_code", "OLD"}, {"recording_id", "fault_OLD_3000"}, {"format", "mcap"}}};
  const auto descriptors = handlers::detail::fold_rosbag_rows_into_descriptors(rows, {});
  ASSERT_EQ(descriptors.size(), 3u);
  ASSERT_TRUE(descriptors[0].x_medkit.has_value());
  EXPECT_EQ((*descriptors[0].x_medkit)["manifest"], true);
  EXPECT_FALSE(descriptors[1].x_medkit->contains("manifest"));
  EXPECT_FALSE(descriptors[2].x_medkit->contains("manifest"));
}

// === Handlers ===

class RecordingServices final : public GatewayServices {
 public:
  ThreadSafeEntityCache cache;
  FaultManager * faults{nullptr};

  const ThreadSafeEntityCache & entity_cache() const override {
    return cache;
  }
  FaultManager * fault_manager() override {
    return faults;
  }
  EntityDiscovery * discovery() override {
    return nullptr;
  }
  ConfigurationManager * configuration_manager() override {
    return nullptr;
  }
  DataAccessManager * data_access_manager() override {
    return nullptr;
  }
  OperationManager * operation_manager() override {
    return nullptr;
  }
  SubscriptionManager * subscription_manager() override {
    return nullptr;
  }
  ResourceSamplerRegistry * sampler_registry() override {
    return nullptr;
  }
  TransportRegistry * transport_registry() override {
    return nullptr;
  }
  TypeSchemaProvider * type_schemas() override {
    return nullptr;
  }
  std::shared_ptr<SSEClientTracker> sse_client_tracker() override {
    return nullptr;
  }
  GatewayNode * ros_node() override {
    return nullptr;
  }
  PluginManager * plugin_manager() override {
    return nullptr;
  }
  ScriptManager * script_manager() override {
    return nullptr;
  }
  LockManager * lock_manager() override {
    return nullptr;
  }
  UpdateManager * update_manager() override {
    return nullptr;
  }
  LogManager * log_manager() override {
    return nullptr;
  }
  BulkDataStore * bulk_data_store() override {
    return nullptr;
  }
  FaultTriggerEngine * fault_trigger_engine() override {
    return nullptr;
  }
  FreezeFrameSource * freeze_frames() override {
    return nullptr;
  }
  FaultEventSource * fault_events() override {
    return nullptr;
  }
  TopicDataProvider * topic_data_provider() override {
    return nullptr;
  }
  TriggerManager * trigger_manager() override {
    return nullptr;
  }
};

/// Two apps reporting under their own ids, and a component hosting both. The lidar
/// has a split recording with a manifest on disk, the motor a recording without one.
class RecordingHandlersTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = std::filesystem::temp_directory_path() / ("rosbag_recording_handlers_" + std::to_string(getpid()));
    std::filesystem::remove_all(root_);
    const auto lidar_bag = root_ / "fault_LIDAR_1000";
    const auto motor_bag = root_ / "fault_MOTOR_2000";
    std::filesystem::create_directories(lidar_bag);
    std::filesystem::create_directories(motor_bag);
    std::ofstream(lidar_bag / "fault_LIDAR_1000_0.mcap", std::ios::binary) << first_;
    std::ofstream(lidar_bag / "fault_LIDAR_1000_1.mcap", std::ios::binary) << second_;
    manifest_ = json{{"manifest_version", 1},
                     {"recording_id", "fault_LIDAR_1000"},
                     {"files",
                      {{{"name", "fault_LIDAR_1000_0.mcap"}, {"size", first_.size()}},
                       {{"name", "fault_LIDAR_1000_1.mcap"}, {"size", second_.size()}}}}}
                    .dump(2) +
                "\n";
    std::ofstream(lidar_bag / "manifest.json", std::ios::binary) << manifest_;
    std::ofstream(motor_bag / "fault_MOTOR_2000_0.mcap", std::ios::binary) << "motor";

    App lidar;
    lidar.id = "lidar";
    lidar.external = true;
    lidar.component_id = "machine";
    App motor;
    motor.id = "motor";
    motor.external = true;
    motor.component_id = "machine";
    Component machine;
    machine.id = "machine";
    services_.cache.update_all({}, {machine}, {lidar, motor}, {});

    transport_ = std::make_shared<FakeFaultTransport>();
    transport_->recordings["fault_LIDAR_1000"] = {lidar_bag.string(), {"LIDAR"}};
    transport_->recordings["fault_MOTOR_2000"] = {motor_bag.string(), {"MOTOR"}};
    transport_->faults["LIDAR"] = {"lidar"};
    transport_->faults["MOTOR"] = {"motor"};
    manager_ = std::make_unique<FaultManager>(transport_);
    services_.faults = manager_.get();
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(root_, ec);
  }

  static httplib::Request request(const std::string & path, const std::string & range = "") {
    httplib::Request req;
    req.path = "/api/v1" + path;
    if (!range.empty()) {
      req.set_header("Range", range);
    }
    return req;
  }

  /// What the framework would put on the wire for the whole response.
  static std::string body_of(const http::BinaryResponse & response) {
    std::string body;
    httplib::DataSink sink;
    sink.write = [&body](const char * data, size_t size) {
      body.append(data, size);
      return true;
    };
    EXPECT_TRUE(response.provider(0, response.total_size, sink));
    return body;
  }

  /// The error of the manifest or file route, whichever the path addresses.
  ErrorInfo error_of(const httplib::Request & req) {
    ErrorInfo none;
    none.http_status = 0;
    if (req.path.find("/files/") == std::string::npos) {
      const auto response = handlers_.recording_manifest(http::TypedRequest(req));
      return response.has_value() ? none : response.error();
    }
    const auto response = handlers_.recording_file(http::TypedRequest(req));
    return response.has_value() ? none : response.error();
  }

  const std::string first_ = std::string(1000, 'a');
  const std::string second_ = std::string(300, 'b');
  std::string manifest_;
  std::filesystem::path root_;
  RecordingServices services_;
  std::shared_ptr<FakeFaultTransport> transport_;
  std::unique_ptr<FaultManager> manager_;
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  handlers::HandlerContext ctx_{services_, cors_, auth_, tls_, nullptr};
  handlers::BulkDataHandlers handlers_{ctx_};
};

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, TheManifestIsServedByteForByte) {
  const auto req = request("/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/manifest");
  const auto response = handlers_.recording_manifest(http::TypedRequest(req));
  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(*response, manifest_);
}

// @verifies REQ_INTEROP_073
TEST(RecordingManifestSchemaTest, TheSchemaNamesEveryFieldOfTheManifest) {
  const auto schema = handlers::BulkDataHandlers::recording_manifest_schema();
  for (const char * key : {"manifest_version", "recording_id", "format", "created_at_ns", "total_bytes", "files"}) {
    EXPECT_TRUE(schema["properties"].contains(key)) << key;
  }
  for (const char * key : {"name", "size", "sha256"}) {
    EXPECT_TRUE(schema["properties"]["files"]["items"]["properties"].contains(key)) << key;
  }
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, AFileIsServedByItsIndex) {
  const auto req = request("/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/1");
  const auto response = handlers_.recording_file(http::TypedRequest(req));
  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(response->content_type, "application/octet-stream");
  EXPECT_TRUE(response->supports_ranges);
  EXPECT_EQ(response->total_size, second_.size());
  EXPECT_EQ(response->filename, "fault_LIDAR_1000_1.mcap");
  EXPECT_EQ(body_of(*response), second_);
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, ARangeInsideTheFileIsLeftToTheFramework) {
  const auto req = request("/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/0", "bytes=999-");
  EXPECT_TRUE(handlers_.recording_file(http::TypedRequest(req)).has_value());
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, ARangeStartingAtOrPastTheEndIs416) {
  for (const char * range : {"bytes=1000-", "bytes=200000-", "bytes=0-1000"}) {
    const auto req = request("/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/0", range);
    const auto response = handlers_.recording_file(http::TypedRequest(req));
    ASSERT_FALSE(response.has_value()) << range;
    EXPECT_EQ(response.error().http_status, 416) << range;
    EXPECT_EQ(response.error().params["size"], first_.size());
  }
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, AnUnknownRecordingIsTheOnly404ThatSaysSo) {
  for (const char * path : {"/apps/lidar/bulk-data/rosbags/fault_LIDAR_9999/manifest",
                            "/apps/lidar/bulk-data/rosbags/fault_LIDAR_9999/files/0",
                            // A fault code is not a recording id on these routes.
                            "/apps/lidar/bulk-data/rosbags/LIDAR/manifest"}) {
    const auto req = request(path);
    const auto error = error_of(req);
    EXPECT_EQ(error.http_status, 404) << path;
    EXPECT_EQ(error.code, ERR_X_MEDKIT_RECORDING_NOT_FOUND) << path;
  }
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, Every404ThatIsNotAboutTheRecordingCarriesNoVendorCode) {
  struct Case {
    const char * path;
    const char * why;
  };
  for (const auto & c :
       {Case{"/apps/motor/bulk-data/rosbags/fault_LIDAR_1000/manifest", "another entity's recording"},
        Case{"/apps/absent/bulk-data/rosbags/fault_LIDAR_1000/manifest", "unknown entity"},
        Case{"/apps/motor/bulk-data/rosbags/fault_MOTOR_2000/manifest", "recording without a manifest"},
        Case{"/apps/motor/bulk-data/rosbags/fault_MOTOR_2000/files/0", "recording without a manifest"},
        Case{"/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/2", "index past the list"},
        Case{"/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/first", "index that is no number"},
        Case{"/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/fault_LIDAR_1000_0.mcap",
             "file name in place of an index"}}) {
    const auto req = request(c.path);
    const auto error = error_of(req);
    EXPECT_EQ(error.http_status, 404) << c.why;
    EXPECT_NE(error.code, ERR_X_MEDKIT_RECORDING_NOT_FOUND) << c.why;
  }
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, AFileGoneFromDiskIsAPlain404) {
  std::filesystem::remove(root_ / "fault_LIDAR_1000" / "fault_LIDAR_1000_1.mcap");
  const auto req = request("/apps/lidar/bulk-data/rosbags/fault_LIDAR_1000/files/1");
  const auto response = handlers_.recording_file(http::TypedRequest(req));
  ASSERT_FALSE(response.has_value());
  EXPECT_EQ(response.error().http_status, 404);
  EXPECT_EQ(response.error().code, ERR_RESOURCE_NOT_FOUND);
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, AFaultManagerThatDoesNotAnswerIs503) {
  transport_->get_rosbag_unavailable = true;
  for (const char * id : {"fault_LIDAR_1000", "fault_LIDAR_9999"}) {
    const auto req = request(std::string("/apps/lidar/bulk-data/rosbags/") + id + "/manifest");
    const auto response = handlers_.recording_manifest(http::TypedRequest(req));
    ASSERT_FALSE(response.has_value());
    EXPECT_EQ(response.error().http_status, 503) << id;
  }
}

// @verifies REQ_INTEROP_073
TEST_F(RecordingHandlersTest, ARecordingIsReachableThroughTheComponentHostingItsApp) {
  const auto req = request("/components/machine/bulk-data/rosbags/fault_LIDAR_1000/files/0");
  const auto response = handlers_.recording_file(http::TypedRequest(req));
  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(body_of(*response), first_);
}

// @verifies REQ_INTEROP_072
TEST_F(RecordingHandlersTest, TheListingSaysWhichRecordingsHaveAManifest) {
  const auto req = request("/components/machine/bulk-data/rosbags");
  const auto listing = handlers_.list_descriptors(http::TypedRequest(req));
  ASSERT_TRUE(listing.has_value()) << listing.error().message;
  ASSERT_EQ(listing->items.size(), 2u);
  for (const auto & item : listing->items) {
    ASSERT_TRUE(item.x_medkit.has_value());
    if (item.id == "fault_LIDAR_1000") {
      EXPECT_EQ((*item.x_medkit)["manifest"], true);
    } else {
      EXPECT_FALSE(item.x_medkit->contains("manifest")) << item.id;
    }
  }
}

// @verifies REQ_INTEROP_072
TEST_F(RecordingHandlersTest, AListingThatCouldNotReadEverySourceIs503) {
  // The component lists under two sources. One of them failing must not come back
  // as a 200 that simply lacks that source's recordings.
  transport_->list_rosbags_fails_for = {"motor"};
  const auto partial = request("/components/machine/bulk-data/rosbags");
  const auto refused = handlers_.list_descriptors(http::TypedRequest(partial));
  ASSERT_FALSE(refused.has_value());
  EXPECT_EQ(refused.error().http_status, 503);

  // The source that answers is still listed on its own.
  const auto single = request("/apps/lidar/bulk-data/rosbags");
  const auto listed = handlers_.list_descriptors(http::TypedRequest(single));
  ASSERT_TRUE(listed.has_value());
  EXPECT_EQ(listed->items.size(), 1u);

  transport_->list_rosbags_fails_for.clear();
  transport_->list_rosbags_fails = true;
  const auto down = handlers_.list_descriptors(http::TypedRequest(single));
  ASSERT_FALSE(down.has_value());
  EXPECT_EQ(down.error().http_status, 503);
}

}  // namespace
