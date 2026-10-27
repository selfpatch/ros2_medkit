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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#ifndef CPPHTTPLIB_OPENSSL_SUPPORT
#error "gateway_core must export CPPHTTPLIB_OPENSSL_SUPPORT"
#endif
#include <httplib.h>
#include <nlohmann/json.hpp>

#include "ros2_medkit_fault_manager/core/fault_manager_config.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"
#include "ros2_medkit_fault_manager/core/log_sink.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_fault_reporter/core/reporter_core.hpp"
#include "ros2_medkit_gateway/core/aggregation/aggregation_host.hpp"
#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/builtin_samplers.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/discovery/entity_discovery.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_fan_out.hpp"
#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_gateway/core/faults/freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/core/gateway_services.hpp"
#include "ros2_medkit_gateway/core/http/handlers/sse_transport_provider.hpp"
#include "ros2_medkit_gateway/core/http/rate_limiter.hpp"
#include "ros2_medkit_gateway/core/http/rest_server.hpp"
#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/core/managers/bulk_data_store.hpp"
#include "ros2_medkit_gateway/core/managers/configuration_manager.hpp"
#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/managers/lock_reaper.hpp"
#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/core/managers/script_manager.hpp"
#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/core/subscription_transport.hpp"
#include "ros2_medkit_gateway/core/transports/action_transport.hpp"
#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"
#include "ros2_medkit_gateway/core/transports/service_transport.hpp"
#include "ros2_medkit_gateway/core/transports/topic_transport.hpp"
#include "ros2_medkit_gateway/core/trigger_host.hpp"
#include "ros2_medkit_gateway/core/types/type_schema_provider.hpp"

namespace core = ros2_medkit_fault_reporter::core;
using ros2_medkit_fault_reporter::FilterConfig;

namespace {
class StderrSink : public ros2_medkit_fault_manager::core::LogSink {
 public:
  void log(ros2_medkit_fault_manager::core::LogLevel /*level*/, const std::string & logger,
           const std::string & message) override {
    std::fprintf(stderr, "[%s] %s\n", logger.c_str(), message.c_str());
  }
};

class RecordingEventSink : public ros2_medkit_fault_manager::core::EventSink {
 public:
  void publish(const ros2_medkit::fault_detection::FaultEventRecord & event) override {
    types.push_back(event.event_type);
  }
  std::vector<std::string> types;
};

class IgnoreCaptureHooks : public ros2_medkit_fault_manager::core::CaptureHooks {
 public:
  ros2_medkit_fault_manager::core::CaptureResult capture_now(const std::string & code) override {
    captured.push_back(code);
    return {ros2_medkit_fault_manager::core::CaptureEnqueue::kAccepted, {}};
  }
  void on_prefailed(const std::string & /*code*/) override {
  }
  void on_cleared(const std::string & /*code*/) override {
  }
  std::vector<std::string> captured;
};

class SettableWallClock : public ros2_medkit_fault_manager::core::WallClock {
 public:
  int64_t now_ns() const override {
    return now;
  }
  int64_t now{0};
};

class FixedWallClock : public ros2_medkit_fault_manager::core::WallClock {
 public:
  int64_t now_ns() const override {
    return 2000000000LL;
  }
};

namespace gw = ros2_medkit_gateway;

// Minimal transports: every call answers "not supported".
class NoFaultTransport : public gw::FaultServiceTransport {
 public:
  gw::FaultResult report_fault(const std::string & /*fault_code*/, uint8_t /*severity*/,
                               const std::string & /*description*/, const std::string & /*source_id*/) override {
    return none();
  }
  // Once set, every list answers one fault whose occurrence_count grows with each call.
  gw::FaultResult list_faults(const std::string & /*source_id*/, bool /*include_prefailed*/, bool /*include_confirmed*/,
                              bool /*include_cleared*/, bool /*include_healed*/, bool /*include_muted*/,
                              bool /*include_clusters*/) override {
    if (!reports_faults.load()) {
      return none();
    }
    gw::json fault = {{"fault_code", "CHECK_FAULT"},
                      {"occurrence_count", ++list_calls},
                      {"reporting_sources", gw::json::array({"hosted_app"})}};
    return {true, {{"faults", gw::json::array({fault})}}, {}, gw::FaultFailure::Declined};
  }
  gw::FaultWithEnvJsonResult get_fault_with_env(const std::string & fault_code,
                                                const std::string & /*source_id*/) override {
    std::lock_guard<std::mutex> lock(detail_mutex_);
    const auto it = detail_sources_.find(fault_code);
    if (it == detail_sources_.end()) {
      return {false, "not supported", {}, gw::FaultFailure::Unavailable};
    }
    const gw::json fault = {{"fault_code", fault_code},
                            {"severity", 2},
                            {"status", "CONFIRMED"},
                            {"occurrence_count", 1},
                            {"reporting_sources", gw::json::array({it->second})}};
    return {true, {}, {{"fault", fault}, {"environment_data", gw::json::object()}}, gw::FaultFailure::Declined};
  }
  // Makes the detail of `fault_code` answer, reported by `source`.
  void serve_detail(const std::string & fault_code, const std::string & source) {
    std::lock_guard<std::mutex> lock(detail_mutex_);
    detail_sources_[fault_code] = source;
  }
  gw::FaultResult get_fault(const std::string & /*fault_code*/, const std::string & /*source_id*/) override {
    return none();
  }
  gw::FaultResult clear_fault(const std::string & /*fault_code*/, bool /*skip_correlation_auto_clear*/) override {
    return none();
  }
  gw::FaultResult get_snapshots(const std::string & /*fault_code*/, const std::string & /*topic*/) override {
    return none();
  }
  gw::FaultResult get_rosbag(const std::string & /*fault_code*/) override {
    return none();
  }
  gw::FaultResult list_rosbags(const std::string & /*entity_fqn*/) override {
    return none();
  }
  bool wait_for_services(std::chrono::duration<double> /*timeout*/) override {
    return false;
  }
  bool is_available() const override {
    return false;
  }
  bool serves_rosbags() const override {
    return rosbags.load();
  }
  std::atomic<bool> reports_faults{false};
  std::atomic<bool> rosbags{false};
  std::atomic<int> list_calls{0};

 private:
  std::mutex detail_mutex_;
  std::map<std::string, std::string> detail_sources_;
  static gw::FaultResult none() {
    return {false, {}, "not supported", gw::FaultFailure::Unavailable};
  }
};

// Answers every sample with a value that grows by one per call.
class CountingTopicProvider : public gw::TopicDataProvider {
 public:
  tl::expected<gw::TopicSampleResult, gw::ErrorInfo> sample(const std::string & topic,
                                                            std::chrono::milliseconds /*timeout*/) override {
    gw::TopicSampleResult out;
    out.topic_name = topic;
    out.message_type = "check/msg/Counter";
    out.data = gw::json{{"value", ++samples}};
    out.has_data = true;
    return out;
  }
  tl::expected<std::vector<gw::TopicSampleResult>, gw::ErrorInfo>
  sample_parallel(const std::vector<std::string> & topics, std::chrono::milliseconds timeout) override {
    std::vector<gw::TopicSampleResult> out;
    out.reserve(topics.size());
    for (const auto & topic : topics) {
      out.push_back(*sample(topic, timeout));
    }
    return out;
  }
  std::optional<gw::TopicInfo> get_topic_info(const std::string & /*topic*/) override {
    return std::nullopt;
  }
  bool has_publishers(const std::string & /*topic*/) override {
    return true;
  }
  std::vector<gw::TopicInfo> discover(const std::string & /*namespace_prefix*/) override {
    return {};
  }
  std::vector<gw::TopicInfo> discover_all() override {
    return {};
  }
  std::map<std::string, gw::ComponentTopics> build_component_topic_map() override {
    return {};
  }
  gw::ComponentTopics get_component_topics(const std::string & /*component_fqn*/) override {
    return {};
  }
  gw::TopicDiscoveryResult discover_topics_by_namespace() override {
    return {};
  }
  std::set<std::string> discover_topic_namespaces() override {
    return {};
  }
  gw::ComponentTopics get_topics_for_namespace(const std::string & /*ns_prefix*/) override {
    return {};
  }
  std::vector<gw::TopicEndpoint> get_topic_publishers(const std::string & /*topic*/) override {
    return {};
  }
  std::vector<gw::TopicEndpoint> get_topic_subscribers(const std::string & /*topic*/) override {
    return {};
  }
  gw::TopicConnection get_topic_connection(const std::string & /*topic*/) override {
    return {};
  }
  std::atomic<int> samples{0};
};

// A plugin that serves the data of the entities it owns; every read counts up.
class CheckDataPlugin : public gw::GatewayPlugin, public gw::DataProvider {
 public:
  std::string name() const override {
    return "check_plugin";
  }
  void configure(const gw::json & /*config*/) override {
  }
  tl::expected<gw::dto::DataListResult, gw::DataProviderErrorInfo>
  list_data(const std::string & /*entity_id*/) override {
    return gw::dto::DataListResult{
        {{"items", gw::json::array({{{"id", "speed"}, {"name", "speed"}, {"value", speed.load()}}})}}};
  }
  tl::expected<gw::dto::DataValue, gw::DataProviderErrorInfo> read_data(const std::string & /*entity_id*/,
                                                                        const std::string & resource_name) override {
    ++read_calls;
    const int pinned_value = pinned.load();
    const int value = pinned_value >= 0 ? pinned_value : ++reads;
    if (wrapped) {
      return gw::dto::DataValue{
          {{"id", resource_name}, {"data", {{"value", value}}}, {"x-medkit", {{"source", "plugin"}}}}};
    }
    return gw::dto::DataValue{{{"source", "plugin"}, {"name", resource_name}, {"value", value}}};
  }
  tl::expected<gw::dto::DataWriteResult, gw::DataProviderErrorInfo> write_data(const std::string & /*entity_id*/,
                                                                               const std::string & /*resource_name*/,
                                                                               const gw::json & /*value*/) override {
    return tl::make_unexpected(gw::DataProviderErrorInfo{gw::DataProviderError::ReadOnly, "read only", 405});
  }
  std::atomic<int> reads{0};
  std::atomic<int> speed{10};
  // At or above zero every read answers this value instead of the counter.
  std::atomic<int> pinned{-1};
  std::atomic<int> read_calls{0};
  // True: reads answer the body of a topic item, {id, data, x-medkit}, with the value inside data.
  std::atomic<bool> wrapped{false};
};

// A fault event feed the check drives by hand.
class ControlledFaultEvents : public gw::FaultEventSource {
 public:
  void subscribe(Callback callback) override {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(callback);
  }
  void unsubscribe() override {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = nullptr;
  }
  void confirm(const std::string & fault_code, const std::string & source) {
    ros2_medkit::fault_detection::FaultEventRecord event;
    event.event_type = ros2_medkit::fault_detection::fault_event_type::kConfirmed;
    event.fault.fault_code = fault_code;
    event.fault.reporting_sources = {source};
    std::lock_guard<std::mutex> lock(mutex_);
    if (callback_) {
      callback_(event);
    }
  }

 private:
  std::mutex mutex_;
  Callback callback_;
};

class NoParameterTransport : public gw::ParameterTransport {
 public:
  bool is_self_node(const std::string & /*node_name*/) const override {
    return false;
  }
  gw::ParameterResult list_parameters(const std::string & /*node_name*/) override {
    return none();
  }
  gw::ParameterResult get_parameter(const std::string & /*node_name*/, const std::string & /*param_name*/) override {
    return none();
  }
  gw::ParameterResult set_parameter(const std::string & /*node_name*/, const std::string & /*param_name*/,
                                    const gw::json & /*value*/) override {
    return none();
  }
  gw::ParameterResult list_own_parameters() override {
    return none();
  }
  gw::ParameterResult get_own_parameter(const std::string & /*param_name*/) override {
    return none();
  }
  gw::ParameterResult get_default(const std::string & /*node_name*/, const std::string & /*param_name*/) override {
    return none();
  }
  gw::ParameterResult list_defaults(const std::string & /*node_name*/) override {
    return none();
  }
  bool is_node_available(const std::string & /*node_name*/) const override {
    return false;
  }
  void shutdown() override {
  }

 private:
  static gw::ParameterResult none() {
    return {false, {}, "not supported", gw::ParameterErrorCode::SERVICE_UNAVAILABLE};
  }
};

class NoTopicTransport : public gw::TopicTransport {
 public:
  gw::json publish(const std::string & /*topic_path*/, const std::string & /*msg_type*/, const gw::json & /*data*/,
                   std::chrono::duration<double> /*timeout*/) override {
    return {{"error", "not supported"}};
  }
  gw::TopicSample sample(const std::string & topic, std::chrono::duration<double> /*timeout*/) override {
    gw::TopicSample out{};
    out.success = false;
    out.status = "error";
    out.topic = topic;
    out.error_message = "not supported";
    return out;
  }
  std::pair<uint64_t, uint64_t> count_publishers_subscribers(const std::string & /*topic_name*/) const override {
    return {0, 0};
  }
};

class NoServiceTransport : public gw::ServiceTransport {
 public:
  gw::ServiceCallResult call(const std::string & /*service_path*/, const std::string & /*service_type*/,
                             const gw::json & /*request*/, std::chrono::duration<double> /*timeout*/) override {
    gw::ServiceCallResult out{};
    out.success = false;
    out.error_message = "not supported";
    return out;
  }
};

class NoActionTransport : public gw::ActionTransport {
 public:
  gw::ActionSendGoalResult send_goal(const std::string & /*action_path*/, const std::string & /*action_type*/,
                                     const gw::json & /*goal*/, std::chrono::duration<double> /*timeout*/) override {
    gw::ActionSendGoalResult out{};
    out.success = false;
    out.goal_accepted = false;
    out.error_message = "not supported";
    return out;
  }
  gw::ActionCancelResult cancel_goal(const std::string & /*action_path*/, const std::string & /*goal_id*/,
                                     std::chrono::duration<double> /*timeout*/) override {
    gw::ActionCancelResult out;
    out.error_message = "not supported";
    return out;
  }
  gw::ActionGetResultResult get_result(const std::string & /*action_path*/, const std::string & /*action_type*/,
                                       const std::string & /*goal_id*/,
                                       std::chrono::duration<double> /*timeout*/) override {
    return {false, gw::ActionGoalStatus::UNKNOWN, {}, "not supported"};
  }
  void subscribe_status(const std::string & /*action_path*/, StatusCallback /*callback*/) override {
  }
  void unsubscribe_status(const std::string & /*action_path*/) override {
  }
};

class EmptyDiscovery : public gw::EntityDiscovery {
 public:
  std::optional<gw::Area> get_area(const std::string & /*id*/) override {
    return std::nullopt;
  }
  std::optional<gw::Component> get_component(const std::string & /*id*/) override {
    return std::nullopt;
  }
  std::optional<gw::App> get_app(const std::string & /*id*/) override {
    return std::nullopt;
  }
  std::optional<gw::Function> get_function(const std::string & /*id*/) override {
    return std::nullopt;
  }
  gw::DiscoveryMode get_mode() const override {
    return gw::DiscoveryMode::RUNTIME_ONLY;
  }
  std::string get_strategy_name() const override {
    return "none";
  }
  std::optional<gw::discovery::MergeReport> get_merge_report() const override {
    return std::nullopt;
  }
  std::optional<gw::discovery::LinkingResult> get_linking_result() const override {
    return std::nullopt;
  }
  gw::discovery::ManifestConfig get_manifest_config() const override {
    return {};
  }
};

class NoTypeSchemas : public gw::TypeSchemaProvider {
 public:
  std::optional<gw::json> message_schema(const std::string & /*type_name*/) override {
    return std::nullopt;
  }
  std::optional<gw::json> type_info(const std::string & /*type_name*/) override {
    return std::nullopt;
  }
  std::shared_ptr<const gw::json> service_type_info(const std::string & /*type_name*/) override {
    return nullptr;
  }
  std::shared_ptr<const gw::json> action_type_info(const std::string & /*type_name*/) override {
    return nullptr;
  }
};

// Every never-null getter is backed by a real core object; the optional ones stay null.
class MinimalServices : public gw::GatewayServices {
 public:
  MinimalServices()
    : fault_transport_(std::make_shared<NoFaultTransport>())
    , fault_(fault_transport_)
    , config_(std::make_shared<NoParameterTransport>())
    , data_(std::make_shared<NoTopicTransport>())
    , operations_(std::make_shared<NoServiceTransport>(), std::make_shared<NoActionTransport>(), nullptr)
    , sse_(std::make_shared<gw::SSEClientTracker>(4)) {
  }
  const gw::ThreadSafeEntityCache & entity_cache() const override {
    return cache_;
  }
  gw::EntityDiscovery * discovery() override {
    return &discovery_;
  }
  gw::FaultManager * fault_manager() override {
    return &fault_;
  }
  gw::ConfigurationManager * configuration_manager() override {
    return &config_;
  }
  gw::DataAccessManager * data_access_manager() override {
    return &data_;
  }
  gw::OperationManager * operation_manager() override {
    return &operations_;
  }
  gw::SubscriptionManager * subscription_manager() override {
    return &subscriptions_;
  }
  gw::ResourceSamplerRegistry * sampler_registry() override {
    return &samplers_;
  }
  gw::TransportRegistry * transport_registry() override {
    return &transports_;
  }
  gw::TypeSchemaProvider * type_schemas() override {
    return &schemas_;
  }
  std::shared_ptr<gw::SSEClientTracker> sse_client_tracker() override {
    return sse_;
  }
  gw::GatewayNode * ros_node() override {
    return nullptr;
  }
  gw::PluginManager * plugin_manager() override {
    return plugins_.get();
  }
  gw::ScriptManager * script_manager() override {
    return scripts_;
  }
  gw::LockManager * lock_manager() override {
    return locks_;
  }
  gw::UpdateManager * update_manager() override {
    return nullptr;
  }
  gw::LogManager * log_manager() override {
    return nullptr;
  }
  gw::BulkDataStore * bulk_data_store() override {
    return bulk_store_.get();
  }
  gw::FaultTriggerEngine * fault_trigger_engine() override {
    return nullptr;
  }
  gw::FreezeFrameSource * freeze_frames() override {
    return freeze_frames_;
  }
  gw::FaultEventSource * fault_events() override {
    return fault_events_;
  }
  gw::TopicDataProvider * topic_data_provider() override {
    return nullptr;
  }
  gw::TriggerManager * trigger_manager() override {
    return triggers_;
  }

  gw::ThreadSafeEntityCache & mutable_cache() {
    return cache_;
  }
  NoFaultTransport & fault_transport() {
    return *fault_transport_;
  }
  // Plugins exist only after this call; the plugin owns the entity ids it is given.
  CheckDataPlugin & add_data_plugin(const std::vector<std::string> & owned_entities) {
    plugins_ = std::make_unique<gw::PluginManager>();
    auto plugin = std::make_unique<CheckDataPlugin>();
    auto & ref = *plugin;
    plugins_->add_plugin(std::move(plugin));
    plugins_->register_entity_ownership("check_plugin", owned_entities);
    return ref;
  }
  void set_freeze_frames(gw::FreezeFrameSource * source) {
    freeze_frames_ = source;
  }
  void set_fault_events(gw::FaultEventSource * source) {
    fault_events_ = source;
  }
  void set_bulk_store(std::unique_ptr<gw::BulkDataStore> store) {
    bulk_store_ = std::move(store);
  }
  void set_lock_manager(gw::LockManager * locks) {
    locks_ = locks;
  }
  void set_script_manager(gw::ScriptManager * scripts) {
    scripts_ = scripts;
  }
  void set_trigger_manager(gw::TriggerManager * triggers) {
    triggers_ = triggers;
  }

 private:
  gw::ThreadSafeEntityCache cache_;
  EmptyDiscovery discovery_;
  NoTypeSchemas schemas_;
  std::shared_ptr<NoFaultTransport> fault_transport_;
  std::unique_ptr<gw::BulkDataStore> bulk_store_;
  std::unique_ptr<gw::PluginManager> plugins_;
  gw::FreezeFrameSource * freeze_frames_{nullptr};
  gw::FaultEventSource * fault_events_{nullptr};
  gw::LockManager * locks_{nullptr};
  gw::ScriptManager * scripts_{nullptr};
  gw::TriggerManager * triggers_{nullptr};
  gw::FaultManager fault_;
  gw::ConfigurationManager config_;
  gw::DataAccessManager data_;
  gw::OperationManager operations_;
  gw::SubscriptionManager subscriptions_;
  gw::ResourceSamplerRegistry samplers_;
  gw::TransportRegistry transports_;
  std::shared_ptr<gw::SSEClientTracker> sse_;
};

int free_port() {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (fd < 0 || ::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    return 0;
  }
  socklen_t len = sizeof(addr);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
  ::close(fd);
  return ntohs(addr.sin_port);
}

bool check(bool ok, const char * what) {
  std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
  return ok;
}

// A REST server on an ephemeral port, stopped and joined on scope exit.
class RunningServer {
 public:
  explicit RunningServer(MinimalServices & services, gw::AggregationManager * aggregation = nullptr)
    : port_(free_port()) {
    gw::RestConfig rest;
    rest.http_thread_pool_size = 6;
    server_ = std::make_unique<gw::RESTServer>(services, nullptr, "127.0.0.1", port_, gw::CorsConfig{},
                                               gw::AuthConfig{}, gw::RateLimitConfig{}, rest);
    if (aggregation != nullptr) {
      server_->set_aggregation_manager(aggregation);
    }
    if (auto * triggers = services.trigger_manager()) {
      server_->set_trigger_handlers(*triggers);
    }
    thread_ = std::thread([this] {
      server_->start();
    });
    for (int i = 0; i < 100 && !server_->is_running(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  ~RunningServer() {
    server_->stop();
    thread_.join();
  }
  RunningServer(const RunningServer &) = delete;
  RunningServer & operator=(const RunningServer &) = delete;
  RunningServer(RunningServer &&) = delete;
  RunningServer & operator=(RunningServer &&) = delete;
  int port() const {
    return port_;
  }

 private:
  int port_;
  std::unique_ptr<gw::RESTServer> server_;
  std::thread thread_;
};

// Reads SSE frames from `path` until `want` of them arrived, the stream ended or the read timed out.
std::vector<nlohmann::json> read_events(httplib::Client & client, const std::string & path, size_t want) {
  std::vector<nlohmann::json> events;
  std::string buffer;
  client.Get(
      path, httplib::Headers{},
      [](const httplib::Response &) {
        return true;
      },
      [&](const char * data, size_t len) {
        buffer.append(data, len);
        for (size_t end = buffer.find("\n\n"); end != std::string::npos; end = buffer.find("\n\n")) {
          const std::string frame = buffer.substr(0, end);
          buffer.erase(0, end + 2);
          if (frame.rfind("data: ", 0) == 0) {
            events.push_back(nlohmann::json::parse(frame.substr(6), nullptr, false));
          }
        }
        return events.size() < want;
      });
  return events;
}

httplib::Client make_client(int port) {
  httplib::Client client("127.0.0.1", port);
  client.set_connection_timeout(5);
  client.set_read_timeout(5);
  return client;
}

// Creates a fast cyclic subscription on `resource`; returns {subscription id, events path}.
std::pair<std::string, std::string> subscribe(httplib::Client & client, const std::string & resource,
                                              const std::string & app = "hosted_app") {
  const nlohmann::json body = {{"resource", resource}, {"interval", "fast"}, {"duration", 60}};
  auto res = client.Post("/api/v1/apps/" + app + "/cyclic-subscriptions", body.dump(), "application/json");
  if (!res || res->status != 201) {
    std::printf("subscribe %s -> %d %s\n", resource.c_str(), res ? res->status : 0, res ? res->body.c_str() : "");
    return {};
  }
  const auto json = nlohmann::json::parse(res->body, nullptr, false);
  return {json.value("id", ""), json.value("event_source", "")};
}

// True when the first two sampled payloads exist and `field` differs between them.
bool values_change(const std::vector<nlohmann::json> & events,
                   const std::function<nlohmann::json(const nlohmann::json &)> & pick) {
  if (events.size() < 2 || !events[0].contains("payload") || !events[1].contains("payload")) {
    return false;
  }
  const auto first = pick(events[0]["payload"]);
  const auto second = pick(events[1]["payload"]);
  return !first.is_null() && !second.is_null() && first != second;
}

// `GET /` capabilities of a server over `services`.
nlohmann::json root_capabilities(MinimalServices & services) {
  RunningServer running(services);
  auto client = make_client(running.port());
  auto res = client.Get("/api/v1/");
  if (!res || res->status != 200) {
    return nlohmann::json::object();
  }
  return nlohmann::json::parse(res->body, nullptr, false).value("capabilities", nlohmann::json::object());
}

// True once `pred` holds, polled for up to five seconds.
bool wait_until(const std::function<bool()> & pred) {
  for (int i = 0; i < 100; ++i) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return pred();
}

// The `freeze_frame` snapshot named `entity` in a fault detail body, or null.
nlohmann::json freeze_frame_of(const std::string & body, const std::string & entity) {
  const auto detail = nlohmann::json::parse(body, nullptr, false);
  if (!detail.is_object() || !detail.contains("environment_data") ||
      !detail["environment_data"].contains("snapshots")) {
    return nullptr;
  }
  for (const auto & snap : detail["environment_data"]["snapshots"]) {
    if (snap.value("type", "") == "freeze_frame" && snap.value("name", "") == entity) {
      return snap;
    }
  }
  return nullptr;
}

gw::App hosted_app() {
  gw::App app;
  app.id = "hosted_app";
  app.name = "Hosted app";
  app.external = true;
  return app;
}

// Two consumers on ONE upstream fault event source: the fault SSE stream (read over HTTP) and the freeze-frame
// capture. Returns {the SSE stream carried the confirmation, the capture holds a frame for it}.
std::pair<bool, bool> share_one_event_source(bool through_fan_out) {
  MinimalServices services;
  auto plugin_app = hosted_app();
  plugin_app.id = "plugin_app";
  services.mutable_cache().update_apps({plugin_app});
  services.add_data_plugin({"plugin_app"});
  ControlledFaultEvents upstream;
  gw::FaultEventFanOut fan_out(upstream);
  std::unique_ptr<gw::FaultEventSource> sse_branch;
  std::unique_ptr<gw::FaultEventSource> capture_branch;
  gw::FaultEventSource * sse_source = &upstream;
  gw::FaultEventSource * capture_source = &upstream;
  if (through_fan_out) {
    sse_branch = fan_out.make_branch();
    capture_branch = fan_out.make_branch();
    sse_source = sse_branch.get();
    capture_source = capture_branch.get();
  }
  auto capture =
      gw::make_freeze_frame_capture(gw::FreezeFrameCaptureConfig{}, *capture_source, services.plugin_manager());
  if (!capture) {
    return {false, false};
  }
  services.set_freeze_frames(capture.get());
  services.set_fault_events(sse_source);
  RunningServer running(services);
  std::atomic<bool> sse_seen{false};
  std::atomic<bool> stream_done{false};
  std::thread reader([&] {
    auto client = make_client(running.port());
    std::string buffer;
    client.Get(
        "/api/v1/faults/stream", httplib::Headers{},
        [](const httplib::Response &) {
          return true;
        },
        [&](const char * data, size_t len) {
          buffer.append(data, len);
          if (buffer.find("fault_confirmed") != std::string::npos) {
            sse_seen = true;
          }
          return !sse_seen && !stream_done;
        });
  });
  // The stream attaches asynchronously, so the confirmation is repeated until the stream has seen one.
  for (int i = 1; i <= 60 && !sse_seen; ++i) {
    upstream.confirm("SHARED_" + std::to_string(i), "plugin_app");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stream_done = true;
  reader.join();
  const bool framed = wait_until([&] {
    return !capture->frames_for("SHARED_1").empty();
  });
  return {sse_seen.load(), framed};
}

void add_sse_transport(MinimalServices & services) {
  services.transport_registry()->register_transport(
      std::make_unique<gw::SseTransportProvider>(*services.subscription_manager(), services.sse_client_tracker()));
  services.subscription_manager()->set_on_removed([&services](const gw::CyclicSubscriptionInfo & info) {
    if (auto * transport = services.transport_registry()->get_transport(info.protocol)) {
      transport->stop(info.id);
    }
  });
}
gw::App online_app(const std::string & id, const std::string & name) {
  gw::App app = hosted_app();
  app.id = id;
  app.name = name;
  app.is_online = true;
  return app;
}

// Items of a collection body, or an empty array.
nlohmann::json items_of(const httplib::Result & res) {
  return res ? nlohmann::json::parse(res->body, nullptr, false).value("items", nlohmann::json::array())
             : nlohmann::json::array();
}

// A lock taken over HTTP with a short expiry is reaped by the core: the lock is gone and the cyclic subscriptions in
// its scope are removed, with no host code calling cleanup_expired.
bool check_lock_reaping() {
  bool ok = true;
  MinimalServices services;
  services.fault_transport().reports_faults = true;
  services.mutable_cache().update_apps({online_app("lock_app", "Lock app")});
  gw::register_builtin_samplers(services);
  add_sse_transport(services);
  gw::LockConfig lock_config;
  lock_config.cleanup_interval = 1;
  gw::LockManager locks(services.entity_cache(), lock_config);
  services.set_lock_manager(&locks);
  auto reaper = std::make_unique<gw::LockReaper>(locks, services.subscription_manager(), lock_config.cleanup_interval,
                                                 gw::log::Logger("check_cores_without_ros"));
  {
    RunningServer running(services);
    auto client = make_client(running.port());
    const auto sub = subscribe(client, "/api/v1/apps/lock_app/faults", "lock_app");
    ok &= check(!sub.first.empty(), "a cyclic subscription is created before the lock");
    const nlohmann::json lock_body = {{"lock_expiration", 1}, {"scopes", {"cyclic-subscriptions"}}};
    auto acquired = client.Post("/api/v1/apps/lock_app/locks", httplib::Headers{{"X-Client-Id", "check_client"}},
                                lock_body.dump(), "application/json");
    ok &= check(acquired && acquired->status == 201, "a lock with a one second expiry is acquired over HTTP");
    ok &= check(locks.get_lock("lock_app").has_value(), "the lock is held before it expires");
    const bool lock_gone = wait_until([&] {
      return !locks.get_lock("lock_app").has_value();
    });
    ok &= check(lock_gone, "the expired lock is gone");
    const bool sub_gone = wait_until([&] {
      return items_of(client.Get("/api/v1/apps/lock_app/cyclic-subscriptions")).empty();
    });
    ok &= check(sub_gone, "the expiry removed the cyclic subscriptions in the lock scope");
  }
  reaper.reset();
  services.transport_registry()->shutdown_all(*services.subscription_manager());
  return ok;
}

// A script configured through the core script config runs over HTTP on the default provider, and its output is
// readable.
bool check_script_host() {
  bool ok = true;
  const auto dir = std::filesystem::temp_directory_path() / "check_cores_without_ros_scripts";
  std::filesystem::remove_all(dir);
  gw::ScriptsConfig config;
  config.scripts_dir = dir.string();
  config.default_timeout_sec = 10;
  auto host = gw::make_script_host(config, nullptr);
  ok &= check(host.manager && host.manager->has_backend(), "a scripts directory gives the script manager a backend");
  {
    const gw::ScriptsConfig none;
    auto without = gw::make_script_host(none, nullptr);
    ok &= check(without.manager && !without.manager->has_backend(),
                "no scripts directory leaves the manager without a backend");
  }
  MinimalServices services;
  services.mutable_cache().update_apps({online_app("script_app", "Script app")});
  services.set_script_manager(host.manager.get());
  {
    RunningServer running(services);
    auto client = make_client(running.port());
    auto uploaded =
        client.Post("/api/v1/apps/script_app/scripts",
                    httplib::MultipartFormDataItems{{"file", "#!/bin/sh\necho '{\"greeting\": \"from the core\"}'\n",
                                                     "greet.sh", "application/octet-stream"}});
    const std::string script_id = uploaded && uploaded->status == 201
                                      ? nlohmann::json::parse(uploaded->body, nullptr, false).value("id", "")
                                      : "";
    ok &= check(!script_id.empty(), "a script is uploaded over HTTP");
    const nlohmann::json start_body = {{"execution_type", "now"}};
    auto started = client.Post("/api/v1/apps/script_app/scripts/" + script_id + "/executions", start_body.dump(),
                               "application/json");
    ok &= check(started && started->status == 202, "an execution starts and answers 202");
    const std::string exec_id =
        started ? nlohmann::json::parse(started->body, nullptr, false).value("id", "") : std::string{};
    nlohmann::json execution;
    const bool finished = wait_until([&] {
      auto res = client.Get("/api/v1/apps/script_app/scripts/" + script_id + "/executions/" + exec_id);
      execution = res ? nlohmann::json::parse(res->body, nullptr, false) : nlohmann::json();
      return execution.is_object() && execution.contains("status") && execution.value("status", "") != "running";
    });
    ok &= check(finished && execution.value("status", "") == "completed", "the execution completes");
    ok &= check(execution.contains("parameters") && execution["parameters"].value("greeting", "") == "from the core",
                "the script output is readable");
  }
  std::filesystem::remove_all(dir);
  return ok;
}

// Two core servers in one process: the second is a peer of the first through the core aggregation config.
bool check_aggregation_host() {
  bool ok = true;
  MinimalServices peer_services;
  peer_services.mutable_cache().update_apps({online_app("peer_app", "Peer app")});
  RunningServer peer(peer_services);

  gw::AggregationConfig config;
  config.enabled = true;
  config.peers.push_back({"http://127.0.0.1:" + std::to_string(peer.port()), "peer_b"});
  gw::AggregationHost host(config, 0, "", gw::log::Logger("check_cores_without_ros"));

  MinimalServices front_services;
  std::vector<gw::Area> areas;
  std::vector<gw::Component> components;
  std::vector<gw::App> apps{online_app("front_app", "Front app")};
  std::vector<gw::Function> functions;
  const auto routing = host.merge_peers(areas, components, apps, functions);
  ok &= check(routing.has_value(), "the host merges the configured peer");
  front_services.mutable_cache().update_all(areas, components, apps, functions);

  RunningServer front(front_services, &host.manager());
  auto client = make_client(front.port());
  auto local = client.Get("/api/v1/apps/front_app");
  ok &= check(local && local->status == 200, "the first server answers its own entity");
  auto remote = client.Get("/api/v1/apps/peer_app");
  const auto remote_json = remote ? nlohmann::json::parse(remote->body, nullptr, false) : nlohmann::json();
  ok &= check(remote && remote->status == 200 && remote_json.value("name", "") == "Peer app",
              "the first server reads the peer's entity from the peer");
  host.shutdown();
  return ok;
}

// A host without a lifecycle reader: readiness comes from graph presence, transitions are not implemented.
bool check_lifecycle_without_reader() {
  bool ok = true;
  MinimalServices services;
  auto managed = online_app("managed_app", "Managed app");
  gw::ServiceInfo get_state;
  get_state.name = "get_state";
  get_state.full_path = "/managed_app/get_state";
  get_state.type = "lifecycle_msgs/srv/GetState";
  gw::ServiceInfo change_state;
  change_state.name = "change_state";
  change_state.full_path = "/managed_app/change_state";
  change_state.type = "lifecycle_msgs/srv/ChangeState";
  managed.services = {get_state, change_state};
  auto offline = online_app("offline_app", "Offline app");
  offline.is_online = false;
  services.mutable_cache().update_apps({managed, offline});
  RunningServer running(services);
  auto client = make_client(running.port());
  auto managed_status = client.Get("/api/v1/apps/managed_app/status");
  ok &= check(managed_status && managed_status->status == 200 &&
                  nlohmann::json::parse(managed_status->body, nullptr, false).value("status", "") == "ready",
              "without a reader a lifecycle node reports readiness from presence");
  auto offline_status = client.Get("/api/v1/apps/offline_app/status");
  ok &= check(offline_status && offline_status->status == 200 &&
                  nlohmann::json::parse(offline_status->body, nullptr, false).value("status", "") == "notReady",
              "without a reader an offline node is notReady");
  auto restart = client.Put("/api/v1/apps/managed_app/status/restart", "", "application/json");
  ok &= check(restart && restart->status == 501, "without a lifecycle provider a transition answers 501");
  return ok;
}

// Collects the events of one trigger's SSE stream until the stream ends.
class TriggerEventReader {
 public:
  TriggerEventReader(int port, const std::string & path) {
    thread_ = std::thread([this, port, path] {
      httplib::Client client("127.0.0.1", port);
      client.set_connection_timeout(5);
      client.set_read_timeout(60);
      std::string buffer;
      client.Get(
          path, httplib::Headers{},
          [](const httplib::Response &) {
            return true;
          },
          [&](const char * data, size_t len) {
            buffer.append(data, len);
            for (size_t end = buffer.find("\n\n"); end != std::string::npos; end = buffer.find("\n\n")) {
              const std::string frame = buffer.substr(0, end);
              buffer.erase(0, end + 2);
              const auto at = frame.find("data: ");
              if (at != std::string::npos) {
                std::lock_guard<std::mutex> lock(mutex_);
                events_.push_back(nlohmann::json::parse(frame.substr(at + 6), nullptr, false));
              }
            }
            return true;
          });
    });
  }
  // The stream ends when the trigger is deleted.
  void join() {
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  ~TriggerEventReader() {
    join();
  }
  TriggerEventReader(const TriggerEventReader &) = delete;
  TriggerEventReader & operator=(const TriggerEventReader &) = delete;
  TriggerEventReader(TriggerEventReader &&) = delete;
  TriggerEventReader & operator=(TriggerEventReader &&) = delete;
  std::vector<nlohmann::json> events() {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

 private:
  std::mutex mutex_;
  std::vector<nlohmann::json> events_;
  std::thread thread_;
};

httplib::Result post_trigger(httplib::Client & client, const std::string & resource, const nlohmann::json & condition,
                             const std::string & path = "") {
  nlohmann::json body = {{"resource", resource}, {"trigger_condition", condition}, {"multishot", true}};
  if (!path.empty()) {
    body["path"] = path;
  }
  return client.Post("/api/v1/apps/plugin_app/triggers", body.dump(), "application/json");
}

// Triggers on a host with no ROS node: a fault confirmation from the host's single event source (shared with the fault
// SSE stream through the fan-out) fires a fault trigger, a plugin-served data item fires a data trigger when it crosses
// a threshold, and the trigger count is bounded.
bool check_triggers() {
  bool ok = true;
  MinimalServices services;
  auto plugin_app = hosted_app();
  plugin_app.id = "plugin_app";
  services.mutable_cache().update_apps({plugin_app});
  auto & plugin = services.add_data_plugin({"plugin_app"});
  plugin.pinned = 10;
  plugin.wrapped = true;
  ControlledFaultEvents upstream;
  gw::FaultEventFanOut fan_out(upstream);
  auto sse_branch = fan_out.make_branch();
  services.set_fault_events(sse_branch.get());
  gw::ResourceChangeNotifier notifier;
  gw::ConditionRegistry conditions;
  gw::register_builtin_conditions(conditions);
  gw::TriggerHostConfig config;
  config.max_triggers = 3;
  config.data_poll_interval_ms = 100;
  gw::TriggerHostWiring wiring;
  wiring.cache = &services.entity_cache();
  wiring.polled_plugins = services.plugin_manager();
  auto trigger_branch = fan_out.make_branch();
  wiring.fault_events = trigger_branch.get();
  auto host = gw::make_trigger_host(config, notifier, conditions, wiring);
  ok &= check(host.max_triggers == 3 && host.data_poll_interval_ms == 100,
              "the host builds with the configured trigger limit and polling interval");
  {
    // Settings outside their documented range fall back with a notice; a disabled host builds nothing.
    std::vector<std::string> notices;
    auto wiring_with_notices = wiring;
    wiring_with_notices.fault_events = nullptr;
    wiring_with_notices.warn = [&notices](const std::string & message) {
      notices.push_back(message);
    };
    gw::ResourceChangeNotifier other_notifier;
    gw::ConditionRegistry other_conditions;
    gw::TriggerHostConfig out_of_range;
    out_of_range.max_triggers = 0;
    out_of_range.data_poll_interval_ms = 5;
    auto low = gw::make_trigger_host(out_of_range, other_notifier, other_conditions, wiring_with_notices);
    ok &= check(low.max_triggers == 1000 && low.data_poll_interval_ms == 100 && notices.size() == 2,
                "a limit below 1 uses 1000 and an interval below 100 ms is raised to 100 ms, each with a notice");
    out_of_range.max_triggers = 5000000000LL;
    out_of_range.data_poll_interval_ms = 1000000000LL;
    auto high = gw::make_trigger_host(out_of_range, other_notifier, other_conditions, wiring_with_notices);
    ok &= check(high.max_triggers == 1000 && high.data_poll_interval_ms == 60000 && notices.size() == 4,
                "a limit past what an int holds uses 1000 and an interval past 60 s is lowered to 60 s");
    out_of_range.enabled = false;
    ok &= check(!gw::make_trigger_host(out_of_range, other_notifier, other_conditions, wiring_with_notices).manager,
                "a disabled configuration builds no trigger manager");
  }
  services.set_trigger_manager(host.manager.get());
  RunningServer running(services);
  auto client = make_client(running.port());

  // Fault trigger.
  auto fault_trigger = post_trigger(client, "/api/v1/apps/plugin_app/faults", {{"condition_type", "OnChange"}});
  ok &= check(fault_trigger && fault_trigger->status == 201, "a fault trigger is created over HTTP");
  const auto fault_json = fault_trigger ? nlohmann::json::parse(fault_trigger->body, nullptr, false) : nlohmann::json();
  const std::string fault_id = fault_json.is_object() ? fault_json.value("id", "") : "";
  const std::string fault_events_path = fault_json.is_object() ? fault_json.value("event_source", "") : "";
  TriggerEventReader fault_reader(running.port(), fault_events_path);
  std::atomic<bool> sse_seen{false};
  std::atomic<bool> stream_done{false};
  std::thread fault_stream([&] {
    auto stream_client = make_client(running.port());
    std::string buffer;
    stream_client.Get(
        "/api/v1/faults/stream", httplib::Headers{},
        [](const httplib::Response &) {
          return true;
        },
        [&](const char * data, size_t len) {
          buffer.append(data, len);
          if (buffer.find("TRIGGER_FAULT") != std::string::npos) {
            sse_seen = true;
          }
          return !sse_seen && !stream_done;
        });
  });
  for (int i = 0; i < 60 && !sse_seen; ++i) {
    upstream.confirm("TRIGGER_FAULT", "plugin_app");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  stream_done = true;
  fault_stream.join();
  ok &= check(sse_seen.load(), "the fault SSE stream still gets the confirmation");
  const bool fault_fired = wait_until([&] {
    return !fault_reader.events().empty();
  });
  ok &= check(fault_fired, "a confirmation from the host's event source fires the fault trigger");
  if (fault_fired) {
    const auto event = fault_reader.events().front();
    ok &= check(event.contains("payload") && event["payload"].value("fault_code", "") == "TRIGGER_FAULT" &&
                    event["payload"].value("event_type", "") == "fault_confirmed",
                "the trigger event carries the confirmed fault");
  }
  if (!fault_id.empty()) {
    client.Delete("/api/v1/apps/plugin_app/triggers/" + fault_id);
  }
  fault_reader.join();

  // Data trigger on a plugin-served item.
  auto data_trigger =
      post_trigger(client, "/api/v1/apps/plugin_app/data/speed",
                   {{"condition_type", "EnterRange"}, {"lower_bound", 50}, {"upper_bound", 100}}, "/value");
  ok &= check(data_trigger && data_trigger->status == 201, "a data trigger on a plugin-served item is created");
  const auto data_json = data_trigger ? nlohmann::json::parse(data_trigger->body, nullptr, false) : nlohmann::json();
  const std::string data_id = data_json.is_object() ? data_json.value("id", "") : "";
  TriggerEventReader data_reader(running.port(), data_json.is_object() ? data_json.value("event_source", "") : "");
  const bool polled = wait_until([&] {
    return plugin.read_calls.load() >= 5;
  });
  ok &= check(polled, "the host reads the data item again and again");
  ok &= check(data_reader.events().empty(), "the data trigger does not fire while the value is below the range");
  plugin.pinned = 60;
  const bool data_fired = wait_until([&] {
    return !data_reader.events().empty();
  });
  ok &= check(data_fired, "the data trigger fires when the value enters the range");
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  ok &= check(data_reader.events().size() == 1, "the data trigger fires once, not on every sample");
  if (!data_id.empty()) {
    client.Delete("/api/v1/apps/plugin_app/triggers/" + data_id);
  }
  data_reader.join();

  // The count limit.
  std::vector<std::string> ids;
  for (int i = 0; i < 3; ++i) {
    auto created = post_trigger(client, "/api/v1/apps/plugin_app/faults", {{"condition_type", "OnChange"}});
    ok &= check(created && created->status == 201, "a trigger below the limit is created");
  }
  auto refused = post_trigger(client, "/api/v1/apps/plugin_app/faults", {{"condition_type", "OnChange"}});
  ok &= check(refused && refused->status == 503, "a trigger past triggers.max_triggers is refused with 503");
  return ok;
}

}  // namespace

static int run() {
  int sunk = 0;
  FilterConfig cfg;
  cfg.default_threshold = 2;
  core::ReporterCore reporter("no_ros", cfg, [&sunk](const ros2_medkit::fault_detection::FaultReport &) {
    ++sunk;
  });

  bool ok = true;
  ok &= check(!reporter.report("X", 1, "d"), "first report below threshold is filtered");
  ok &= check(reporter.report("X", 1, "d"), "second report reaches the sink");
  ok &= check(sunk == 1, "exactly one report reached the sink");

  int before = sunk;
  ok &= check(!reporter.report_passed("X"), "one PASSED below threshold is filtered");
  ok &= check(sunk == before, "no PASSED reached the sink");
  ok &= check(!reporter.report("", 1, "d"), "empty fault code is not forwarded");

  // Fault manager core: a FAILED report with threshold -1 confirms and lists as a neutral record.
  namespace fm = ros2_medkit_fault_manager;
  namespace fd = ros2_medkit::fault_detection;
  const auto db_path = std::filesystem::temp_directory_path() / "check_cores_without_ros_faults.db";
  std::filesystem::remove(db_path);
  {
    StderrSink sink;
    fm::SqliteFaultStorage storage(db_path.string(), sink);
    fm::DebounceConfig debounce;
    debounce.confirmation_threshold = -1;
    ok &= check(storage.report_fault_event("NO_ROS_FAULT", fd::ReportKind::kFailed, fd::severity::kError, "d", "/src",
                                           1000000000LL, debounce),
                "a FAILED report creates the fault");
    const auto faults = storage.list_faults(false, 0, {});
    ok &= check(faults.size() == 1, "list_faults returns one record");
    ok &= check(!faults.empty() && faults[0].status == fd::fault_status::kConfirmed, "the record is CONFIRMED");
    ok &= check(!faults.empty() && faults[0].first_occurred_ns == 1000000000LL, "the record keeps its nanosecond time");
  }
  std::filesystem::remove(db_path);

  // Fault manager config rules and startup parts.
  {
    fm::core::FaultManagerConfig config;
    config.confirmation_threshold = 5;
    config.auto_confirm_after_sec = std::nan("");
    const auto notices = config.validate();
    ok &= check(config.confirmation_threshold == -5, "confirmation_threshold 5 becomes -5");
    ok &= check(config.auto_confirm_after_sec == 0.0, "NaN auto_confirm_after_sec falls back to 0");
    ok &= check(notices.size() == 2, "two notices: the sign flip and the NaN fallback");

    fm::core::FaultManagerConfig wide;
    wide.confirmation_threshold = -4294967297LL;
    wide.healing_threshold = 4294967296LL;
    const auto wide_notices = wide.validate();
    ok &= check(wide.confirmation_threshold == -2147483647LL && wide.healing_threshold == 2147483647LL,
                "int64 thresholds outside the int32 range clamp to its ends");
    ok &= check(wide_notices.size() == 2 &&
                    wide_notices[0].text.find("confirmation_threshold must be in [-2147483647, 2147483647]") == 0,
                "the clamp notice names the range");

    const auto parts_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_parts.db";
    std::filesystem::remove(parts_db);
    StderrSink sink;
    fm::core::FaultManagerConfig sqlite_config;
    sqlite_config.storage_type = "sqlite";
    sqlite_config.database_path = parts_db.string();
    {
      auto parts = fm::core::build_core_parts(sqlite_config, sink);
      ok &= check(parts.storage != nullptr, "build_core_parts opens a sqlite storage");
      ok &= check(parts.audit == nullptr && parts.correlation == nullptr, "audit and correlation stay off by default");
    }
    std::filesystem::remove(parts_db);
  }

  // Fault manager core: report, list and clear through the orchestration, SQLite storage.
  {
    namespace fmc = ros2_medkit_fault_manager::core;
    const auto core_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_core.db";
    std::filesystem::remove(core_db);
    StderrSink sink;
    fmc::FaultManagerConfig core_config;
    core_config.storage_type = "sqlite";
    core_config.database_path = core_db.string();
    core_config.confirmation_threshold = -1;
    RecordingEventSink events;
    IgnoreCaptureHooks hooks;
    FixedWallClock clock;
    {
      fmc::FaultManagerCore fault_core(core_config, fmc::build_core_parts(core_config, sink), events, hooks, clock,
                                       sink);
      fd::FaultReport report;
      report.fault_code = "CORE_FAULT";
      report.kind = fd::ReportKind::kFailed;
      report.severity = fd::severity::kError;
      report.description = "d";
      report.source_id = "/src";
      ok &= check(fault_core.report(report).accepted, "core accepts a FAILED report");
      const auto listed = fault_core.list(fmc::ListQuery{});
      ok &= check(listed.faults.size() == 1 && listed.faults[0].status == fd::fault_status::kConfirmed,
                  "core list returns the fault CONFIRMED");
      ok &= check(events.types.size() == 1 && events.types[0] == fd::fault_event_type::kConfirmed,
                  "the sink saw exactly one fault_confirmed");
      const auto cleared = fault_core.clear("CORE_FAULT", false);
      ok &= check(cleared.success, "core clear succeeds");
      ok &= check(events.types.size() == 2 && events.types[1] == fd::fault_event_type::kCleared,
                  "the sink saw fault_cleared after the clear");
    }
    std::filesystem::remove(core_db);
  }

  // Fault manager core: tick drives auto-confirmation and correlation cleanup, each on its own period.
  {
    namespace fmc = ros2_medkit_fault_manager::core;
    constexpr int64_t kMs = 1000000LL;
    const int64_t t0 = 10000000000LL;
    const auto tick_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_tick.db";
    const auto corr_yaml = std::filesystem::temp_directory_path() / "check_cores_without_ros_corr.yaml";
    std::filesystem::remove(tick_db);
    {
      std::ofstream out(corr_yaml.string());
      out << "correlation:\n  enabled: true\n  default_window_ms: 500\n  patterns:\n    sensor_errors:\n"
             "      codes: [\"SENSOR_*\"]\n  rules:\n    - id: r1\n      name: R1\n      mode: hierarchical\n"
             "      root_cause:\n        codes: [\"ESTOP_001\"]\n      symptoms:\n"
             "        - pattern: sensor_errors\n      window_ms: 2000\n      mute_symptoms: true\n";
    }
    StderrSink sink;
    fmc::FaultManagerConfig tick_config;
    tick_config.storage_type = "sqlite";
    tick_config.database_path = tick_db.string();
    tick_config.confirmation_threshold = -3;
    tick_config.auto_confirm_after_sec = 1.0;
    tick_config.correlation_config_file = corr_yaml.string();
    tick_config.correlation_cleanup_interval_sec = 5.0;
    RecordingEventSink events;
    IgnoreCaptureHooks hooks;
    SettableWallClock clock;
    clock.now = t0;
    {
      fmc::FaultManagerCore fault_core(tick_config, fmc::build_core_parts(tick_config, sink), events, hooks, clock,
                                       sink);
      fd::FaultReport report;
      report.fault_code = "TICK_FAULT";
      report.kind = fd::ReportKind::kFailed;
      report.severity = fd::severity::kError;
      report.description = "d";
      report.source_id = "/src";
      ok &= check(fault_core.report(report).accepted, "tick: core accepts a FAILED report");
      fmc::ListQuery all_statuses;
      all_statuses.statuses = {fd::fault_status::kPrefailed};
      const auto before = fault_core.list(all_statuses);
      ok &= check(before.faults.size() == 1 && before.faults[0].status == fd::fault_status::kPrefailed,
                  "tick: the fault is PREFAILED");

      fault_core.tick(t0 + 499 * kMs);
      ok &= check(events.types.empty() && hooks.captured.empty(), "tick at 499 ms: no event, no capture");
      ok &= check(fault_core.auto_confirm_runs() == 0, "tick at 499 ms: auto-confirm job has not run");
      fault_core.tick(t0 + 1001 * kMs);
      ok &= check(events.types.size() == 1 && events.types[0] == fd::fault_event_type::kConfirmed,
                  "tick at 1001 ms: one fault_confirmed");
      ok &= check(hooks.captured.size() == 1 && hooks.captured[0] == "TICK_FAULT",
                  "tick at 1001 ms: capture requested once");
      ok &= check(fault_core.auto_confirm_runs() == 1, "tick at 1001 ms: auto-confirm job ran once");

      ok &= check(fault_core.correlation_cleanup_runs() == 0, "cleanup has not run before its interval");
      fault_core.tick(t0 + 2499 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 0, "tick at 2499 ms: cleanup not due");
      fault_core.tick(t0 + 5001 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 1, "tick at 5001 ms: cleanup ran once");
      fault_core.tick(t0 + 5002 * kMs);
      fault_core.tick(t0 + 7000 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 1, "ticks inside the interval: cleanup does not repeat");
      fault_core.tick(t0 + 10001 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 2, "tick at 10001 ms: cleanup ran a second time");
      ok &= check(events.types.size() == 1 && hooks.captured.size() == 1, "later ticks emit no second confirmation");

      // A tick earlier than the last one runs nothing and pulls the due times back to now + interval.
      const uint64_t confirm_runs = fault_core.auto_confirm_runs();
      fault_core.tick(t0 + 2000 * kMs);
      ok &= check(fault_core.auto_confirm_runs() == confirm_runs && fault_core.correlation_cleanup_runs() == 2,
                  "backwards tick: no job runs");
      ok &= check(events.types.size() == 1 && hooks.captured.size() == 1, "backwards tick: no event, no capture");
      fault_core.tick(t0 + 4499 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 2, "after a backwards tick: cleanup waits for its due time");
      fault_core.tick(t0 + 7001 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 3, "after a backwards tick: cleanup resumes");
      ok &= check(events.types.size() == 1, "after a backwards tick: still one fault_confirmed");
    }
    std::filesystem::remove(tick_db);
    std::filesystem::remove(corr_yaml);

    // A callback that runs more than one period late keeps the job's phase.
    const auto late_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_late.db";
    std::filesystem::remove(late_db);
    {
      fmc::FaultManagerConfig late_config = tick_config;
      late_config.database_path = late_db.string();
      late_config.correlation_config_file.clear();
      clock.now = t0;
      fmc::FaultManagerCore late_core(late_config, fmc::build_core_parts(late_config, sink), events, hooks, clock,
                                      sink);
      late_core.tick(t0 + 2200 * kMs);
      ok &= check(late_core.auto_confirm_runs() == 1, "late tick at 2200 ms (due 1000 ms): job ran once");
      late_core.tick(t0 + 2300 * kMs);
      ok &= check(late_core.auto_confirm_runs() == 1, "tick at 2300 ms: no second run inside one period");
      late_core.tick(t0 + 3000 * kMs);
      ok &= check(late_core.auto_confirm_runs() == 2, "tick at 3000 ms: the job runs on its original phase");
      late_core.tick(t0 + 3100 * kMs);
      ok &= check(late_core.auto_confirm_runs() == 2, "tick at 3100 ms: no second run inside one period");
    }
    std::filesystem::remove(late_db);
  }

  // A cleanup interval near the validation maximum must not overflow the due time.
  {
    namespace fmc = ros2_medkit_fault_manager::core;
    const int64_t epoch_now = 1790000000LL * 1000000000LL;
    const auto huge_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_huge.db";
    const auto corr_yaml = std::filesystem::temp_directory_path() / "check_cores_without_ros_corr_huge.yaml";
    std::filesystem::remove(huge_db);
    {
      std::ofstream out(corr_yaml.string());
      out << "correlation:\n  enabled: true\n  default_window_ms: 500\n  patterns:\n    sensor_errors:\n"
             "      codes: [\"SENSOR_*\"]\n  rules:\n    - id: r1\n      name: R1\n      mode: hierarchical\n"
             "      root_cause:\n        codes: [\"ESTOP_001\"]\n      symptoms:\n"
             "        - pattern: sensor_errors\n      window_ms: 2000\n      mute_symptoms: true\n";
    }
    StderrSink sink;
    fmc::FaultManagerConfig huge_config;
    huge_config.storage_type = "sqlite";
    huge_config.database_path = huge_db.string();
    huge_config.correlation_config_file = corr_yaml.string();
    huge_config.correlation_cleanup_interval_sec = 9.0e9;
    RecordingEventSink events;
    IgnoreCaptureHooks hooks;
    SettableWallClock clock;
    clock.now = epoch_now;
    {
      fmc::FaultManagerCore fault_core(huge_config, fmc::build_core_parts(huge_config, sink), events, hooks, clock,
                                       sink);
      fault_core.tick(epoch_now + 1000000000LL);
      ok &= check(fault_core.correlation_cleanup_runs() == 0, "cleanup interval 9e9 s: no run one second after start");
    }
    std::filesystem::remove(huge_db);
    std::filesystem::remove(corr_yaml);
  }

  // Gateway: REST server over minimal services answers /health.
  {
    MinimalServices services;
    const int port = free_port();
    ok &= check(port != 0, "an ephemeral port was found");
    gw::RestConfig rest;
    rest.server_name = "Hosted Server";
    rest.vendor_name = "hosted_vendor";
    rest.server_version = "9.8.7";
    gw::RESTServer server(services, nullptr, "127.0.0.1", port, gw::CorsConfig{}, gw::AuthConfig{},
                          gw::RateLimitConfig{}, rest);
    std::thread server_thread([&server] {
      server.start();
    });
    for (int i = 0; i < 100 && !server.is_running(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    httplib::Client client("127.0.0.1", port);
    client.set_connection_timeout(5);
    client.set_read_timeout(5);
    auto res = client.Get("/api/v1/health");
    const int status = res ? res->status : 0;
    ok &= check(status == 200, "GET /api/v1/health answers 200");
    if (status == 200) {
      std::printf("ok gateway health %d\n", status);
    }
    auto root = client.Get("/api/v1/");
    ok &= check(root && root->status == 200, "GET /api/v1/ answers 200");
    if (root && root->status == 200) {
      const auto body = nlohmann::json::parse(root->body, nullptr, false);
      ok &= check(body.is_object() && body.value("name", "") == "Hosted Server",
                  "GET /api/v1/ names the server the host set");
      ok &= check(body.is_object() && body.value("version", "") == "9.8.7",
                  "GET /api/v1/ reports the server version the host set");
    }
    auto info = client.Get("/api/v1/version-info");
    ok &= check(info && info->status == 200, "GET /api/v1/version-info answers 200");
    if (info && info->status == 200) {
      const auto body = nlohmann::json::parse(info->body, nullptr, false);
      const bool shaped = body.is_object() && body.contains("items") && body["items"].is_array() &&
                          !body["items"].empty() && body["items"][0].contains("vendor_info");
      ok &= check(shaped, "GET /api/v1/version-info carries items[0].vendor_info");
      if (shaped) {
        const auto & vendor = body["items"][0]["vendor_info"];
        ok &= check(vendor.value("name", "") == "hosted_vendor", "version-info vendor_info.name is the host's");
        ok &= check(vendor.value("version", "") == "9.8.7", "version-info vendor_info.version is the host's");
      }
    }
    server.stop();
    server_thread.join();
  }

  // A host that attaches the core freeze-frame capture to its own fault events gets frames in the fault detail.
  {
    MinimalServices services;
    auto plugin_app = hosted_app();
    plugin_app.id = "plugin_app";
    services.mutable_cache().update_apps({hosted_app(), plugin_app});
    auto & plugin = services.add_data_plugin({"plugin_app"});
    ControlledFaultEvents events;
    gw::FreezeFrameCaptureConfig capture_config;
    capture_config.max_faults = 2;
    auto capture = gw::make_freeze_frame_capture(capture_config, events, services.plugin_manager());
    ok &= check(capture != nullptr, "the core builds a freeze-frame capture over a plugin manager");
    gw::FreezeFrameCaptureConfig disabled;
    disabled.enabled = false;
    ok &= check(gw::make_freeze_frame_capture(disabled, events, services.plugin_manager()) == nullptr,
                "a disabled configuration builds no capture");
    if (capture) {
      services.set_freeze_frames(capture.get());
      auto & transport = services.fault_transport();
      for (const char * code : {"FF_1", "FF_2", "FF_3"}) {
        transport.serve_detail(code, "plugin_app");
      }
      transport.serve_detail("FF_HOSTED", "hosted_app");
      RunningServer running(services);
      auto client = make_client(running.port());
      const auto detail_of = [&](const std::string & app, const std::string & code) {
        auto res = client.Get("/api/v1/apps/" + app + "/faults/" + code);
        return res ? std::make_pair(res->status, res->body) : std::make_pair(0, std::string{});
      };

      // The value at the confirmation moment is the one served, not the later one.
      plugin.speed = 10;
      events.confirm("FF_1", "plugin_app");
      ok &= check(wait_until([&] {
                    return !capture->frames_for("FF_1").empty();
                  }),
                  "a fault confirmation for a plugin-served entity is captured");
      plugin.speed = 99;
      const auto first = detail_of("plugin_app", "FF_1");
      ok &= check(first.first == 200, "GET the fault detail answers 200");
      const auto frame = freeze_frame_of(first.second, "plugin_app");
      ok &= check(!frame.is_null(), "environment_data holds a freeze_frame named after the entity");
      ok &= check(!frame.is_null() && frame["x-medkit"]["full_data"].value("speed", -1) == 10,
                  "the freeze_frame holds the value from the confirmation moment, not the later one");
      ok &= check(!frame.is_null() && frame["x-medkit"].value("source", "") == "plugin_data_provider",
                  "the freeze_frame names the DataProvider as its source");

      // An entity no plugin serves gets no frame.
      events.confirm("FF_HOSTED", "hosted_app");
      events.confirm("FF_2", "plugin_app");
      ok &= check(wait_until([&] {
                    return !capture->frames_for("FF_2").empty();
                  }),
                  "a second fault is captured");
      ok &= check(capture->frames_for("FF_HOSTED").empty() &&
                      freeze_frame_of(detail_of("hosted_app", "FF_HOSTED").second, "hosted_app").is_null(),
                  "an entity no plugin serves gets no freeze_frame");

      // Past max_faults the oldest fault's frames are evicted.
      plugin.speed = 30;
      events.confirm("FF_3", "plugin_app");
      ok &= check(wait_until([&] {
                    return !capture->frames_for("FF_3").empty();
                  }),
                  "a third fault is captured");
      ok &= check(capture->frames_for("FF_1").empty() &&
                      freeze_frame_of(detail_of("plugin_app", "FF_1").second, "plugin_app").is_null(),
                  "past max_faults the oldest fault's frames are evicted");
      ok &= check(!capture->frames_for("FF_2").empty() && !capture->frames_for("FF_3").empty(),
                  "the newer faults keep their frames");

      // A re-confirmation takes a new frame.
      plugin.speed = 55;
      events.confirm("FF_3", "plugin_app");
      ok &= check(wait_until([&] {
                    const auto again = freeze_frame_of(detail_of("plugin_app", "FF_3").second, "plugin_app");
                    return !again.is_null() && again["x-medkit"]["full_data"].value("speed", -1) == 55;
                  }),
                  "a re-confirmation replaces the frame with the new value");
    }
  }

  // One upstream event source shared by the fault SSE stream and the capture.
  {
    const auto shared = share_one_event_source(true);
    ok &= check(shared.first, "through the fan-out the fault SSE stream carries the confirmation");
    ok &= check(shared.second, "through the fan-out the capture frames the same confirmation");
    const auto unshared = share_one_event_source(false);
    ok &= check(unshared.first != unshared.second,
                "without the fan-out one source serves one consumer only: the later subscriber replaces the earlier");
  }

  // A host that registers the samplers, a transport and a bulk-data store gets subscriptions and bulk data.
  {
    MinimalServices services;
    CountingTopicProvider topics;
    services.data_access_manager()->set_topic_data_provider(&topics);
    services.fault_transport().reports_faults = true;
    auto plugin_app = hosted_app();
    plugin_app.id = "plugin_app";
    services.mutable_cache().update_apps({hosted_app(), plugin_app});
    services.add_data_plugin({"plugin_app"});
    const auto store_dir = std::filesystem::temp_directory_path() / "check_cores_without_ros_bulk";
    std::filesystem::remove_all(store_dir);
    gw::BulkDataConfig bulk;
    bulk.storage_dir = store_dir.string();
    bulk.max_upload_size = 1024;
    bulk.categories = {"calibration", ""};
    services.set_bulk_store(gw::make_bulk_data_store(bulk));
    gw::register_builtin_samplers(services);
    add_sse_transport(services);
    {
      RunningServer running(services);
      auto client = make_client(running.port());

      // Cyclic subscription on a data resource.
      const auto data_sub = subscribe(client, "/api/v1/apps/hosted_app/data/temperature");
      ok &= check(!data_sub.second.empty(), "a cyclic subscription on a data resource is created");
      const auto data_events =
          data_sub.second.empty() ? std::vector<nlohmann::json>{} : read_events(client, data_sub.second, 2);
      ok &= check(data_events.size() == 2, "the data subscription streams two events");
      ok &= check(values_change(data_events,
                                [](const nlohmann::json & p) {
                                  return p.contains("data") ? p["data"].value("value", nlohmann::json())
                                                            : nlohmann::json();
                                }),
                  "the data value differs between the two events");
      ok &= check(!data_events.empty() && data_events[0].contains("payload") &&
                      data_events[0]["payload"].value("id", "") == "/temperature",
                  "the data payload names the resource");
      if (!data_sub.first.empty()) {
        client.Delete("/api/v1/apps/hosted_app/cyclic-subscriptions/" + data_sub.first);
      }

      // Cyclic subscription on a resource a plugin serves.
      const auto plugin_sub = subscribe(client, "/api/v1/apps/plugin_app/data/speed", "plugin_app");
      ok &= check(!plugin_sub.second.empty(), "a cyclic subscription on a plugin-served data resource is created");
      const auto plugin_events =
          plugin_sub.second.empty() ? std::vector<nlohmann::json>{} : read_events(client, plugin_sub.second, 2);
      ok &= check(plugin_events.size() == 2, "the plugin-data subscription streams two events");
      ok &= check(!plugin_events.empty() && plugin_events[0].contains("payload") &&
                      plugin_events[0]["payload"].value("source", "") == "plugin" &&
                      plugin_events[0]["payload"].value("name", "") == "speed",
                  "the plugin-data payload comes from the plugin's provider");
      ok &= check(values_change(plugin_events,
                                [](const nlohmann::json & p) {
                                  return p.value("value", nlohmann::json());
                                }),
                  "the plugin value differs between the two events");
      if (!plugin_sub.first.empty()) {
        client.Delete("/api/v1/apps/plugin_app/cyclic-subscriptions/" + plugin_sub.first);
      }

      // Cyclic subscription on faults.
      const auto fault_sub = subscribe(client, "/api/v1/apps/hosted_app/faults");
      ok &= check(!fault_sub.second.empty(), "a cyclic subscription on faults is created");
      const auto fault_events =
          fault_sub.second.empty() ? std::vector<nlohmann::json>{} : read_events(client, fault_sub.second, 2);
      ok &= check(fault_events.size() == 2, "the faults subscription streams two events");
      ok &= check(values_change(fault_events,
                                [](const nlohmann::json & p) {
                                  return p.contains("faults") && p["faults"].is_array() && !p["faults"].empty()
                                             ? p["faults"][0].value("occurrence_count", nlohmann::json())
                                             : nlohmann::json();
                                }),
                  "the fault occurrence_count differs between the two events");
      if (!fault_sub.first.empty()) {
        client.Delete("/api/v1/apps/hosted_app/cyclic-subscriptions/" + fault_sub.first);
      }

      // Bulk data: upload, list, download, delete.
      const std::string payload("bulk\0payload\xff\x01", 15);
      const std::string base = "/api/v1/apps/hosted_app/bulk-data";
      httplib::MultipartFormDataItems items = {{"file", payload, "cal.bin", "application/octet-stream"}};
      auto uploaded = client.Post(base + "/calibration", items);
      ok &= check(uploaded && uploaded->status == 201, "an upload to a configured category answers 201");
      const std::string item_id = uploaded && uploaded->status == 201
                                      ? nlohmann::json::parse(uploaded->body, nullptr, false).value("id", "")
                                      : "";
      auto listed = client.Get(base + "/calibration");
      const auto listed_json = listed ? nlohmann::json::parse(listed->body, nullptr, false) : nlohmann::json();
      ok &=
          check(listed && listed->status == 200 && listed_json.contains("items") && listed_json["items"].size() == 1 &&
                    listed_json["items"][0].value("id", "") == item_id && !item_id.empty(),
                "the uploaded item is listed");
      auto downloaded = client.Get(base + "/calibration/" + item_id);
      ok &= check(downloaded && downloaded->status == 200 && downloaded->body == payload, "the download is byte-equal");
      auto removed = client.Delete(base + "/calibration/" + item_id);
      ok &= check(removed && removed->status == 204, "the delete answers 204");
      auto relisted = client.Get(base + "/calibration");
      const auto relisted_json = relisted ? nlohmann::json::parse(relisted->body, nullptr, false) : nlohmann::json();
      ok &= check(relisted && relisted_json.contains("items") && relisted_json["items"].empty(),
                  "the deleted item is gone");
      auto too_big = client.Post(
          base + "/calibration",
          httplib::MultipartFormDataItems{{"file", std::string(2048, 'x'), "big.bin", "application/octet-stream"}});
      ok &= check(too_big && too_big->status >= 400, "an upload past max_upload_size is refused");

      // Categories: the store's own, and no rosbags when the fault manager serves none.
      auto categories = client.Get(base);
      const auto categories_json =
          categories ? nlohmann::json::parse(categories->body, nullptr, false) : nlohmann::json();
      const auto cat_items = categories_json.value("items", nlohmann::json::array());
      ok &= check(categories && categories->status == 200 &&
                      std::find(cat_items.begin(), cat_items.end(), nlohmann::json("calibration")) != cat_items.end(),
                  "the categories list the configured category");
      ok &= check(std::find(cat_items.begin(), cat_items.end(), nlohmann::json("rosbags")) == cat_items.end(),
                  "the categories do not list rosbags when the fault manager serves none");

      // A host without rosbags answers rosbags like any category that does not exist.
      const auto error_code_of = [](const httplib::Result & r) {
        return r ? nlohmann::json::parse(r->body, nullptr, false).value("error_code", "") : std::string{};
      };
      auto no_rosbag_list = client.Get(base + "/rosbags");
      auto no_such_list = client.Get(base + "/no_such_category");
      ok &= check(no_such_list && no_such_list->status == 404, "an unknown category list answers 404");
      ok &= check(no_rosbag_list && no_such_list && no_rosbag_list->status == no_such_list->status &&
                      error_code_of(no_rosbag_list) == error_code_of(no_such_list),
                  "listing rosbags without rosbags answers like an unknown category");
      auto no_rosbag_item = client.Get(base + "/rosbags/some-recording");
      auto no_such_item = client.Get(base + "/no_such_category/some-recording");
      ok &= check(no_rosbag_item && no_rosbag_item->body.find("Unknown category: rosbags") != std::string::npos,
                  "the rosbag download error names the unknown category");
      ok &= check(no_rosbag_item && no_such_item && no_rosbag_item->status == no_such_item->status &&
                      error_code_of(no_rosbag_item) == error_code_of(no_such_item),
                  "downloading a rosbag without rosbags answers like an unknown category");

      auto root = client.Get("/api/v1/");
      const auto caps =
          root ? nlohmann::json::parse(root->body, nullptr, false).value("capabilities", nlohmann::json::object())
               : nlohmann::json::object();
      ok &= check(caps.value("bulk_data", false), "GET / reports bulk_data true with a store");
      ok &= check(caps.value("cyclic_subscriptions", false),
                  "GET / reports cyclic_subscriptions true with samplers and a transport");
    }
    services.transport_registry()->shutdown_all(*services.subscription_manager());
    std::filesystem::remove_all(store_dir);
  }

  // Capabilities follow what the host provides.
  {
    MinimalServices bare;
    const auto caps = root_capabilities(bare);
    ok &=
        check(caps.contains("bulk_data") && !caps.value("bulk_data", true), "no store and no rosbags: bulk_data false");
    ok &= check(caps.contains("cyclic_subscriptions") && !caps.value("cyclic_subscriptions", true),
                "no transport and no sampler: cyclic_subscriptions false");

    MinimalServices transport_only;
    add_sse_transport(transport_only);
    const auto transport_caps = root_capabilities(transport_only);
    ok &= check(transport_caps.contains("cyclic_subscriptions") && !transport_caps.value("cyclic_subscriptions", true),
                "a transport without a sampler: cyclic_subscriptions false");
    transport_only.transport_registry()->shutdown_all(*transport_only.subscription_manager());

    MinimalServices sampler_only;
    sampler_only.sampler_registry()->register_sampler(
        "x-check", [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
          return nlohmann::json::object();
        });
    const auto sampler_caps = root_capabilities(sampler_only);
    ok &= check(sampler_caps.contains("cyclic_subscriptions") && !sampler_caps.value("cyclic_subscriptions", true),
                "a sampler without a transport: cyclic_subscriptions false");

    MinimalServices store_only;
    gw::BulkDataConfig store_only_config;
    store_only_config.storage_dir =
        (std::filesystem::temp_directory_path() / "check_cores_without_ros_store_only").string();
    store_only.set_bulk_store(gw::make_bulk_data_store(store_only_config));
    store_only.fault_transport().rosbags = false;
    const auto store_caps = root_capabilities(store_only);
    ok &= check(store_caps.value("bulk_data", false), "a store without rosbags: bulk_data true");
    std::filesystem::remove_all(std::filesystem::temp_directory_path() / "check_cores_without_ros_store_only");

    MinimalServices rosbags_only;
    rosbags_only.fault_transport().rosbags = true;
    const auto rosbag_caps = root_capabilities(rosbags_only);
    ok &= check(rosbag_caps.value("bulk_data", false), "rosbags without a store: bulk_data true");
  }

  // Host wiring the core owns: lock reaping, script manager, aggregation, lifecycle without a reader.
  ok &= check_lock_reaping();
  ok &= check_script_host();
  ok &= check_aggregation_host();
  ok &= check_lifecycle_without_reader();
  ok &= check_triggers();

  return ok ? 0 : 1;
}

int main() {
  try {
    return run();
  } catch (...) {
    std::printf("FAIL: the check threw an exception\n");
    return 1;
  }
}
