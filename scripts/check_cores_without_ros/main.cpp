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
#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/builtin_samplers.hpp"
#include "ros2_medkit_gateway/core/config.hpp"
#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/discovery/entity_discovery.hpp"
#include "ros2_medkit_gateway/core/gateway_services.hpp"
#include "ros2_medkit_gateway/core/http/handlers/sse_transport_provider.hpp"
#include "ros2_medkit_gateway/core/http/rate_limiter.hpp"
#include "ros2_medkit_gateway/core/http/rest_server.hpp"
#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/core/managers/configuration_manager.hpp"
#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/managers/operation_manager.hpp"
#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/core/subscription_transport.hpp"
#include "ros2_medkit_gateway/core/transports/action_transport.hpp"
#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"
#include "ros2_medkit_gateway/core/transports/service_transport.hpp"
#include "ros2_medkit_gateway/core/transports/topic_transport.hpp"
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
  gw::FaultWithEnvJsonResult get_fault_with_env(const std::string & /*fault_code*/,
                                                const std::string & /*source_id*/) override {
    return {false, "not supported", {}, gw::FaultFailure::Unavailable};
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
  std::atomic<bool> reports_faults{false};
  std::atomic<int> list_calls{0};

 private:
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
    return gw::dto::DataListResult{{{"items", gw::json::array()}}};
  }
  tl::expected<gw::dto::DataValue, gw::DataProviderErrorInfo> read_data(const std::string & /*entity_id*/,
                                                                        const std::string & resource_name) override {
    return gw::dto::DataValue{{{"source", "plugin"}, {"name", resource_name}, {"value", ++reads}}};
  }
  tl::expected<gw::dto::DataWriteResult, gw::DataProviderErrorInfo> write_data(const std::string & /*entity_id*/,
                                                                               const std::string & /*resource_name*/,
                                                                               const gw::json & /*value*/) override {
    return tl::make_unexpected(gw::DataProviderErrorInfo{gw::DataProviderError::ReadOnly, "read only", 405});
  }
  std::atomic<int> reads{0};
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
    return nullptr;
  }
  gw::LockManager * lock_manager() override {
    return nullptr;
  }
  gw::UpdateManager * update_manager() override {
    return nullptr;
  }
  gw::LogManager * log_manager() override {
    return nullptr;
  }
  gw::BulkDataStore * bulk_data_store() override {
    return nullptr;
  }
  gw::FaultTriggerEngine * fault_trigger_engine() override {
    return nullptr;
  }
  gw::FreezeFrameSource * freeze_frames() override {
    return nullptr;
  }
  gw::FaultEventSource * fault_events() override {
    return nullptr;
  }
  gw::TopicDataProvider * topic_data_provider() override {
    return nullptr;
  }
  gw::TriggerManager * trigger_manager() override {
    return nullptr;
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

 private:
  gw::ThreadSafeEntityCache cache_;
  EmptyDiscovery discovery_;
  NoTypeSchemas schemas_;
  std::shared_ptr<NoFaultTransport> fault_transport_;
  std::unique_ptr<gw::PluginManager> plugins_;
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
  explicit RunningServer(MinimalServices & services) : port_(free_port()) {
    gw::RestConfig rest;
    rest.http_thread_pool_size = 6;
    server_ = std::make_unique<gw::RESTServer>(services, nullptr, "127.0.0.1", port_, gw::CorsConfig{},
                                               gw::AuthConfig{}, gw::RateLimitConfig{}, rest);
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

gw::App hosted_app() {
  gw::App app;
  app.id = "hosted_app";
  app.name = "Hosted app";
  app.external = true;
  return app;
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

  // A host that registers the samplers and a transport gets cyclic subscriptions.
  {
    MinimalServices services;
    CountingTopicProvider topics;
    services.data_access_manager()->set_topic_data_provider(&topics);
    services.fault_transport().reports_faults = true;
    auto plugin_app = hosted_app();
    plugin_app.id = "plugin_app";
    services.mutable_cache().update_apps({hosted_app(), plugin_app});
    services.add_data_plugin({"plugin_app"});
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
    }
    services.transport_registry()->shutdown_all(*services.subscription_manager());
  }

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
