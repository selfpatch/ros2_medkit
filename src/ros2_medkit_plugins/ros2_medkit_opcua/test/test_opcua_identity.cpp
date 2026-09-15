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

// End-to-end against a live OPC-UA server (no HW): boot the test_alarm_server
// fixture and exercise the paths that only a real session can reach.
//
// INV2 identity: prove the asset-identity nameplate is filled from the server's
// device-info (ServerStatus/BuildInfo + the OPC-UA DI DeviceSet nameplate) with
// no manual entry, through both the raw OpcuaClient::read_device_info read and
// the full OpcuaPlugin::introspect() path that lands identity on the SOVD
// Component.
//
// Connection lifecycle: prove a successful connect clears the standing
// PLC_COMMS_LOST fault, which needs a connect that actually succeeds.

#include "ros2_medkit_opcua/device_identity.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"
#include "ros2_medkit_opcua/opcua_plugin.hpp"
#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/get_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

#ifndef MEDKIT_ALARM_SERVER_BIN
#define MEDKIT_ALARM_SERVER_BIN ""
#endif

// -- Stub PluginRequest/PluginResponse (mirrors test_opcua_plugin.cpp; the
//    plugin translation unit references them but the HTTP layer is not linked) --

namespace ros2_medkit_gateway {

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t) const {
  return {};
}
std::string PluginRequest::header(const std::string &) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
std::string PluginRequest::query_param(const std::string &) const {
  return {};
}

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json &) {
}
void PluginResponse::send_error(int, const std::string &, const std::string &, const nlohmann::json &) {
}

// -- FakePluginContext: node() is null (no ROS graph), just enough for
//    set_context() + introspect() to run. --

class FakePluginContext : public RosPluginContext {
 public:
  std::unordered_map<std::string, PluginEntityInfo> entities;

  rclcpp::Node * node() const override {
    return nullptr;
  }
  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities.find(id);
    return it != entities.end() ? std::optional<PluginEntityInfo>(it->second) : std::nullopt;
  }
  std::vector<PluginEntityInfo> get_child_apps(const std::string &) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string &) const override {
    // Contract: a bare JSON array of fault objects (empty for this fake).
    return nlohmann::json::array();
  }
  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest &, PluginResponse &,
                                                            const std::string & entity_id) const override {
    return get_entity(entity_id);
  }
  void register_capability(SovdEntityType, const std::string &) override {
  }
  void register_entity_capability(const std::string &, const std::string &) override {
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string &) const override {
    return {};
  }
  LockAccessResult check_lock(const std::string &, const std::string &, const std::string &) const override {
    return {true, "", "", ""};
  }
  tl::expected<LockInfo, LockError> acquire_lock(const std::string &, const std::string &,
                                                 const std::vector<std::string> &, int) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  tl::expected<void, LockError> release_lock(const std::string &, const std::string &) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  IntrospectionInput get_entity_snapshot() const override {
    return {};
  }
  nlohmann::json list_all_faults() const override {
    return nlohmann::json::object();
  }
  void register_sampler(
      const std::string &,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &)
      override {
  }
  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
};

namespace {

// Reserve an ephemeral loopback port and release it (best-effort; a race with
// the fixture bind is unlikely on a test host and retried by the caller).
int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  int port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
    socklen_t len = sizeof(addr);
    if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) == 0) {
      port = ntohs(addr.sin_port);
    }
  }
  close(sock);
  return port;
}

// Boots the test_alarm_server binary as a child process and blocks until it
// prints the "READY " handshake line on stdout. SIGTERM on teardown.
class AlarmServer {
 public:
  ~AlarmServer() {
    stop();
  }

  bool start(const std::string & binary, int port, const std::vector<std::string> & extra_args = {}) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
      return false;
    }
    int stdin_pipe[2];
    if (pipe(stdin_pipe) != 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      return false;
    }
    pid_ = fork();
    if (pid_ < 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
      return false;
    }
    if (pid_ == 0) {
      dup2(pipefd[1], STDOUT_FILENO);
      dup2(pipefd[1], STDERR_FILENO);
      dup2(stdin_pipe[0], STDIN_FILENO);
      close(pipefd[0]);
      close(pipefd[1]);
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
      std::string port_str = std::to_string(port);
      std::vector<const char *> argv_vec{binary.c_str(), "--port", port_str.c_str()};
      for (const auto & arg : extra_args) {
        argv_vec.push_back(arg.c_str());
      }
      argv_vec.push_back(nullptr);
      execv(binary.c_str(), const_cast<char * const *>(argv_vec.data()));
      _exit(127);
    }
    close(pipefd[1]);
    close(stdin_pipe[0]);
    read_fd_ = pipefd[0];
    write_fd_ = stdin_pipe[1];
    return wait_for_ready(15000);
  }

  // One CLI command ("fire Overpressure 750", "clear Overpressure", ...). The
  // fixture reads them line by line off stdin.
  bool send(const std::string & command) {
    if (write_fd_ < 0) {
      return false;
    }
    const std::string line = command + "\n";
    return write(write_fd_, line.c_str(), line.size()) == static_cast<ssize_t>(line.size());
  }

  void stop() {
    if (write_fd_ >= 0) {
      close(write_fd_);
      write_fd_ = -1;
    }
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      int status = 0;
      waitpid(pid_, &status, 0);
      pid_ = -1;
    }
    if (read_fd_ >= 0) {
      close(read_fd_);
      read_fd_ = -1;
    }
  }

 private:
  bool wait_for_ready(int timeout_ms) {
    std::string acc;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      pollfd pfd{read_fd_, POLLIN, 0};
      int remaining = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
      int rc = poll(&pfd, 1, remaining > 0 ? remaining : 0);
      if (rc <= 0) {
        continue;
      }
      char buf[256];
      ssize_t n = read(read_fd_, buf, sizeof(buf));
      if (n <= 0) {
        return false;  // EOF: child died before READY
      }
      acc.append(buf, static_cast<size_t>(n));
      if (acc.find("READY ") != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  pid_t pid_{-1};
  int read_fd_{-1};
  int write_fd_{-1};
};

std::string fixture_binary() {
  return std::string(MEDKIT_ALARM_SERVER_BIN);
}

bool fixture_available() {
  const std::string bin = fixture_binary();
  return !bin.empty() && access(bin.c_str(), X_OK) == 0;
}

}  // namespace

// -- Fixture that boots the alarm server once per test --

class OpcuaIdentityE2ETest : public ::testing::Test {
 protected:
  void SetUp() override {
    // The fixture is built by this package's own CMake and is a declared
    // dependency of this target; a missing binary means the build is broken,
    // so fail hard instead of skipping (run_ctest.py does the same).
    ASSERT_TRUE(fixture_available()) << "test_alarm_server fixture missing or not executable at '" << fixture_binary()
                                     << "'";
    port_ = reserve_local_port();
    ASSERT_NE(port_, 0);
    ASSERT_TRUE(server_.start(fixture_binary(), port_)) << "test_alarm_server did not signal READY";
    endpoint_ = "opc.tcp://127.0.0.1:" + std::to_string(port_);
    // The fixture prints READY before the OPC-UA listen socket is fully
    // accepting, so probe until a real connection succeeds. Once connectable it
    // stays so, making every per-test connect (and the plugin's) race-free.
    ASSERT_TRUE(wait_until_connectable()) << "fixture never became connectable at " << endpoint_;
  }

  void TearDown() override {
    server_.stop();
  }

  bool wait_until_connectable() {
    for (int attempt = 0; attempt < 50; ++attempt) {
      OpcuaClient probe;
      OpcuaClientConfig config;
      config.endpoint_url = endpoint_;
      config.connect_timeout = std::chrono::milliseconds(1000);
      if (probe.connect(config)) {
        probe.disconnect();
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  // Tear the fixture down and boot a fresh instance on the SAME port with
  // different CLI args (e.g. a new --serial). Simulates a PLC reboot /
  // device swap for the per-session identity refresh test.
  bool restart_server(const std::vector<std::string> & extra_args) {
    server_.stop();
    if (!server_.start(fixture_binary(), port_, extra_args)) {
      return false;
    }
    return wait_until_connectable();
  }

  AlarmServer server_;
  int port_{0};
  std::string endpoint_;
};

TEST_F(OpcuaIdentityE2ETest, ClientReadsServerBuildInfo) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto info = client.read_device_info();
  // The fixture pins explicit BuildInfo values.
  EXPECT_EQ(info.manufacturer_name, "SelfPatch Test Manufacturer");
  EXPECT_EQ(info.product_name, "SelfPatch Test PLC");
  EXPECT_EQ(info.software_version, "1.2.3");
  EXPECT_EQ(info.build_number, "build-4567");
  client.disconnect();
}

TEST_F(OpcuaIdentityE2ETest, ClientReadsDiNameplate) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto info = client.read_device_info();
  // The fixture exposes an OPC-UA DI DeviceSet nameplate.
  EXPECT_EQ(info.di_manufacturer, "SelfPatch Devices");
  EXPECT_EQ(info.di_model, "SPX-1000");
  EXPECT_EQ(info.di_serial_number, "SN-0001-TEST");
  EXPECT_EQ(info.di_hardware_revision, "HW-A2");
  EXPECT_EQ(info.di_software_revision, "SW-3.4.5");
  // OrderNumber lives in the vendor namespace (not DI) with leading + trailing
  // pad; it is matched by BrowseName across namespaces and edge-trimmed while the
  // load-bearing internal space in the MLFB is preserved.
  EXPECT_EQ(info.di_order_number, "6ES7 672-5SC11-0YA0");
  client.disconnect();
}

TEST_F(OpcuaIdentityE2ETest, MappedIdentityFromLiveServer) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto id = opcua_device_info_to_identity(client.read_device_info(), endpoint_);
  // DI nameplate wins over BuildInfo for manufacturer / model / software.
  EXPECT_EQ(id.manufacturer, "SelfPatch Devices");
  EXPECT_EQ(id.model, "SPX-1000");
  EXPECT_EQ(id.order_code, "6ES7 672-5SC11-0YA0");
  EXPECT_EQ(id.serial_number, "SN-0001-TEST");
  EXPECT_EQ(id.hardware_revision, "HW-A2");
  EXPECT_EQ(id.software_version, "SW-3.4.5");
  EXPECT_EQ(id.network_endpoint, endpoint_);
  EXPECT_EQ(id.extra.at("buildNumber"), "build-4567");
  EXPECT_EQ(id.provenance.at("serial_number"), "opcua");
  EXPECT_EQ(id.provenance.at("order_code"), "opcua");
  EXPECT_EQ(id.provenance.at("network_endpoint"), "opcua");
  client.disconnect();
}

namespace {

// Minimal node map so introspect() can name the area / component. The single
// node points at a nonexistent address-space node; the poller's failed reads
// do not drop the connection (BadNodeIdUnknown != disconnect).
std::string write_minimal_node_map() {
  const std::string yaml_path = "/tmp/test_opcua_identity_nodemap.yaml";
  std::ofstream f(yaml_path);
  f << R"(
area_id: test_plc
area_name: Test PLC Area
component_id: test_runtime
component_name: Test PLC Runtime
nodes:
  - node_id: "ns=2;i=9999"
    entity_id: tank
    data_name: level
    display_name: Tank Level
    data_type: float
    writable: false
)";
  return yaml_path;
}

}  // namespace

TEST_F(OpcuaIdentityE2ETest, PluginIntrospectPopulatesIdentity) {
  const std::string yaml_path = write_minimal_node_map();

  ros2_medkit_gateway::OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = endpoint_;
  plugin.configure(config);

  FakePluginContext ctx;
  plugin.set_context(ctx);

  auto result = plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(result.new_entities.components.empty());

  const auto & comp = result.new_entities.components.front();
  // The fixture session is unsecured (SecurityPolicy=None), so the component
  // gets the generic "plugin" tag: the spoofable nameplate may fill gaps in an
  // operator manifest but never override it. Per-field provenance still says
  // "opcua" so the read origin stays visible.
  EXPECT_EQ(comp.source, "plugin");
  ASSERT_FALSE(comp.identity.empty()) << "identity should be filled from the OPC-UA device-info";
  EXPECT_EQ(comp.identity.manufacturer, "SelfPatch Devices");
  EXPECT_EQ(comp.identity.model, "SPX-1000");
  EXPECT_EQ(comp.identity.serial_number, "SN-0001-TEST");
  EXPECT_EQ(comp.identity.network_endpoint, endpoint_);
  EXPECT_EQ(comp.identity.provenance.at("manufacturer"), "opcua");

  // Serialized SOVD JSON carries the nameplate under x-medkit.identity.
  auto j = comp.to_json();
  ASSERT_TRUE(j["x-medkit"].contains("identity"));
  EXPECT_EQ(j["x-medkit"]["identity"]["serialNumber"], "SN-0001-TEST");
  EXPECT_EQ(j["x-medkit"]["identity"]["_provenance"]["manufacturer"], "opcua");

  std::remove(yaml_path.c_str());
}

TEST_F(OpcuaIdentityE2ETest, PluginSourceTagIsTrustGated) {
  const std::string yaml_path = write_minimal_node_map();

  // Secured + certificate-validated profile: the protocol tag "opcua" is
  // stamped, giving the nameplate authority over the manifest. The cert paths
  // do not exist, so the connect fails fast without contacting a server - the
  // source tag is decided by configuration, not by connection state.
  ros2_medkit_gateway::OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = endpoint_;
  config["security_policy"] = "Basic256Sha256";
  config["security_mode"] = "SignAndEncrypt";
  config["client_cert_path"] = "/nonexistent/client_cert.der";
  config["client_key_path"] = "/nonexistent/client_key.pem";
  config["reject_untrusted"] = true;
  plugin.configure(config);

  FakePluginContext ctx;
  plugin.set_context(ctx);

  auto result = plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(result.new_entities.components.empty());
  EXPECT_EQ(result.new_entities.components.front().source, "opcua");

  // Same secured profile but accept-any server cert: a rogue endpoint would be
  // accepted, so the identity authority drops back to the generic "plugin" tag.
  ros2_medkit_gateway::OpcuaPlugin accept_any_plugin;
  config["reject_untrusted"] = false;
  accept_any_plugin.configure(config);
  FakePluginContext ctx2;
  accept_any_plugin.set_context(ctx2);
  auto accept_any_result = accept_any_plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(accept_any_result.new_entities.components.empty());
  EXPECT_EQ(accept_any_result.new_entities.components.front().source, "plugin");

  std::remove(yaml_path.c_str());
}

TEST_F(OpcuaIdentityE2ETest, IdentityRefreshedAfterReconnect) {
  const std::string yaml_path = write_minimal_node_map();

  ros2_medkit_gateway::OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = endpoint_;
  plugin.configure(config);

  FakePluginContext ctx;
  plugin.set_context(ctx);

  auto result = plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(result.new_entities.components.empty());
  ASSERT_EQ(result.new_entities.components.front().identity.serial_number, "SN-0001-TEST");

  // Reboot the "PLC" on the same port with a different nameplate. The plugin's
  // poller detects the drop and reconnects in the background; the next
  // introspect on the new session must re-read the device-info instead of
  // serving the value latched from the first session.
  ASSERT_TRUE(restart_server({"--serial", "SN-0002-RECONNECT"}));

  std::string observed_serial;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (std::chrono::steady_clock::now() < deadline) {
    auto refreshed = plugin.introspect(IntrospectionInput{});
    ASSERT_FALSE(refreshed.new_entities.components.empty());
    observed_serial = refreshed.new_entities.components.front().identity.serial_number;
    if (observed_serial == "SN-0002-RECONNECT") {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  EXPECT_EQ(observed_serial, "SN-0002-RECONNECT") << "identity not refreshed after reconnect";

  std::remove(yaml_path.c_str());
}

TEST_F(OpcuaIdentityE2ETest, ClientConnectionGenerationCountsSessions) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);

  EXPECT_EQ(client.connection_generation(), 0u);
  ASSERT_TRUE(client.connect(config));
  EXPECT_EQ(client.connection_generation(), 1u);
  // Re-connect on an already-open session is not a new session.
  ASSERT_TRUE(client.connect(config));
  EXPECT_EQ(client.connection_generation(), 1u);

  client.disconnect();
  ASSERT_TRUE(client.connect(config));
  EXPECT_EQ(client.connection_generation(), 2u);
  client.disconnect();
}

TEST_F(OpcuaIdentityE2ETest, DiNameplateReadFollowsBrowseContinuationPoints) {
  // Cap the server at 2 references per Browse result: every folder on the DI
  // nameplate path (ObjectsFolder, DeviceSet, TestDevice) now pages through
  // BrowseNext continuation points. Without continuation handling the
  // DeviceSet lookup truncates after the first two ObjectsFolder children and
  // the DI fields come back empty.
  ASSERT_TRUE(restart_server({"--max-refs-per-node", "2"}));

  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto info = client.read_device_info();
  EXPECT_EQ(info.di_manufacturer, "SelfPatch Devices");
  EXPECT_EQ(info.di_model, "SPX-1000");
  EXPECT_EQ(info.di_serial_number, "SN-0001-TEST");
  EXPECT_EQ(info.di_hardware_revision, "HW-A2");
  EXPECT_EQ(info.di_software_revision, "SW-3.4.5");
  EXPECT_EQ(info.di_order_number, "6ES7 672-5SC11-0YA0");
  client.disconnect();
}

// A gateway that restarts after a comms outage never raised PLC_COMMS_LOST in
// THIS process, yet the fault manager keys faults by fault_code alone and
// persists them, so the fault raised before the restart is still standing.
// The reconnect arm used to clear only when its own in-memory
// ``comms_lost_raised_`` flag was set, which no restart can satisfy, so the
// fault stayed CONFIRMED for good. The clear now goes out on every successful
// connect. Driven against the live fixture because the arm can only be reached
// by a connect that actually succeeds.
TEST_F(OpcuaIdentityE2ETest, SuccessfulConnectClearsCommsLostNeverRaisedHere) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  // Connect once to seed the client's stored config (what the poller reconnects
  // with), then drop the session so the poll loop starts in its reconnect arm -
  // the state a freshly started gateway is in while the PLC is already up.
  ASSERT_TRUE(client.connect(config));
  client.disconnect();
  ASSERT_FALSE(client.is_connected());

  NodeMap node_map;  // config-less: no entries, nothing to poll
  OpcuaPoller poller(client, node_map);

  std::mutex signals_mutex;
  std::vector<std::pair<std::string, bool>> signals;  // (fault_code, active)
  poller.set_alarm_callback(
      [&signals_mutex, &signals](const std::string &, const ros2_medkit::fault_detection::FaultSignal & signal) {
        std::lock_guard<std::mutex> lock(signals_mutex);
        signals.emplace_back(signal.fault_code, signal.active);
      });

  PollerConfig poller_config;
  poller_config.poll_interval = std::chrono::milliseconds(100);
  poller_config.reconnect_interval = std::chrono::milliseconds(100);
  poller_config.comms_lost_fault_enabled = true;
  poller.start(poller_config);

  bool cleared = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!cleared && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(signals_mutex);
      cleared = std::find(signals.begin(), signals.end(), std::make_pair(std::string(kCommsLostFaultCode), false)) !=
                signals.end();
    }
    if (!cleared) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  poller.stop();

  EXPECT_TRUE(cleared) << "a successful connect must clear PLC_COMMS_LOST even when this process never raised it";

  // Absence control on the same harness: the connect succeeded, so nothing may
  // have RAISED the fault. Without this a clear-everything-always regression
  // would still pass the assertion above.
  std::lock_guard<std::mutex> lock(signals_mutex);
  EXPECT_EQ(std::find(signals.begin(), signals.end(), std::make_pair(std::string(kCommsLostFaultCode), true)),
            signals.end())
      << "comms-lost must not be raised while the connection is up";
}

namespace {

// RAII rclcpp init/shutdown, tearing down only what it started.
struct ScopedRclcpp {
  const bool owned_;
  ScopedRclcpp() : owned_(!rclcpp::ok()) {
    if (owned_) {
      rclcpp::init(0, nullptr);
    }
  }
  ~ScopedRclcpp() {
    if (owned_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
  ScopedRclcpp(const ScopedRclcpp &) = delete;
  ScopedRclcpp & operator=(const ScopedRclcpp &) = delete;
};

// Spins an executor on its own thread and guarantees cancel -> join on every
// exit path. A gtest ASSERT_* returns from the middle of the test body, so a
// bare std::thread would be destroyed while still joinable, and that calls
// std::terminate: the run ends in SIGABRT and the assertion message that says
// what actually failed never reaches the report.
class ScopedExecutorSpin {
 public:
  using CancelFn = std::function<void()>;

  // The cancel is injectable so a test can make it fail. It is a callable
  // because rclcpp::Executor::cancel() is virtual on jazzy and later but NOT on
  // humble, where a subclass's cancel() would neither compile with `override`
  // nor be the one called through a base reference.
  explicit ScopedExecutorSpin(rclcpp::executors::MultiThreadedExecutor & executor, CancelFn cancel = nullptr)
    : executor_(executor)
    , cancel_(cancel ? std::move(cancel) : CancelFn([this]() {
      executor_.cancel();
    }))
    , thread_([this]() {
      executor_.spin();
      spin_returned_.store(true);
    }) {
  }

  ~ScopedExecutorSpin() {
    // A destructor is implicitly noexcept, and both cancel() and join() can
    // throw, so an escape here would be the std::terminate this class exists
    // to prevent. Swallowing is right in a destructor: by this point the test
    // has either passed or recorded its failure, and that verdict is what the
    // run has to report.
    try {
      stop();
    } catch (const std::exception & e) {
      std::cerr << "ScopedExecutorSpin teardown failed: " << e.what() << "\n";
    } catch (...) {
      std::cerr << "ScopedExecutorSpin teardown failed\n";
    }
  }

  // Idempotent, so a test can end the spin at the point it wants the executor
  // quiet and still be covered on the paths that never get there.
  //
  // The join is unconditional on the cancel's outcome. cancel() throws if the
  // guard condition cannot be triggered, and letting that skip the join would
  // move the terminate from this class's destructor - where the catch above can
  // report it - into the std::thread member's destructor, which no catch here
  // can reach: ~std::thread calls std::terminate on a joinable thread. The
  // flag goes last so a failed teardown leaves the object willing to try again.
  void stop() {
    if (stopped_) {
      return;
    }
    // cancel() refuses with an exception while the executor is not spinning,
    // and a cancel that lands before spin() has begun is simply lost - the
    // thread then spins for good and the join below never returns. So the
    // cancel is re-issued until the spin function has actually returned, which
    // only the spin thread can report.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!spin_returned_.load()) {
      try {
        cancel_();
      } catch (const std::exception &) {
        // Not spinning yet, or the guard condition could not be triggered. The
        // next attempt is what resolves either case.
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        std::cerr << "ScopedExecutorSpin: executor still spinning after 10s of cancel attempts\n";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (thread_.joinable()) {
      // Nothing is left to try if this throws: the thread stays joinable and
      // its own destructor ends the process. That is a clean abort with a
      // reason, which is the honest outcome - detaching instead would leave a
      // live executor thread running against nodes about to be destroyed.
      thread_.join();
    }
    stopped_ = true;
  }

  ScopedExecutorSpin(const ScopedExecutorSpin &) = delete;
  ScopedExecutorSpin & operator=(const ScopedExecutorSpin &) = delete;
  ScopedExecutorSpin(ScopedExecutorSpin &&) = delete;
  ScopedExecutorSpin & operator=(ScopedExecutorSpin &&) = delete;

 private:
  rclcpp::executors::MultiThreadedExecutor & executor_;
  CancelFn cancel_;
  std::atomic<bool> spin_returned_{false};
  std::thread thread_;
  bool stopped_{false};
};

// The plugin only builds its fault-service clients when the context hands it a
// real node, which is what makes the ClearFault request observable on the wire.
class RealNodePluginContext : public FakePluginContext {
 public:
  explicit RealNodePluginContext(rclcpp::Node * node) : node_(node) {
  }
  rclcpp::Node * node() const override {
    return node_;
  }

 private:
  rclcpp::Node * node_;
};

// Poll until an OPC-UA session can be opened at ``endpoint``. A fixture prints
// READY before its listen socket is accepting, so nothing may rely on one
// before this returns.
bool wait_for_connectable(const std::string & endpoint) {
  for (int attempt = 0; attempt < 50; ++attempt) {
    OpcuaClient probe;
    OpcuaClientConfig config;
    config.endpoint_url = endpoint;
    config.connect_timeout = std::chrono::milliseconds(1000);
    if (probe.connect(config)) {
      probe.disconnect();
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

// The ApplicationUri the server at ``endpoint`` publishes, read the same way
// the plugin reads it off a live session. Empty when the session cannot be
// opened or the server publishes none.
std::string live_application_uri(const std::string & endpoint) {
  OpcuaClient probe;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint;
  config.connect_timeout = std::chrono::milliseconds(5000);
  if (!probe.connect(config)) {
    return {};
  }
  const std::string uri = probe.read_server_application_uri();
  probe.disconnect();
  return uri;
}

// The component id a config-less plugin derives from the server at
// ``endpoint``, read over a throwaway session so no test pins the fixture's
// nameplate spelling.
std::string device_derived_component_id(const std::string & endpoint) {
  OpcuaClient probe;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint;
  config.connect_timeout = std::chrono::milliseconds(5000);
  if (!probe.connect(config)) {
    return {};
  }
  const std::string id = derive_component_identity(probe.read_device_info(), endpoint).id;
  probe.disconnect();
  return id;
}

// A stand-in fault manager carrying the three services the plugin talks to,
// keyed the way the real one is: one row per fault code holding the set of
// sources that reported it, a read that answers from those rows, and a clear
// that removes the whole row - ClearFault has no source field.
//
// The services are created one at a time so a test can decide in which order
// the plugin discovers them; ``open_reads(false)`` parks GetFault requests
// unanswered until ``release_reads()``.
class FaultStoreStub {
 public:
  explicit FaultStoreStub(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  }

  void open_reports() {
    report_srv_ = node_->create_service<ros2_medkit_msgs::srv::ReportFault>(
        "/fault_manager/report_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> req,
                                              std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> res) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            reported_.push_back(req->fault_code);
            auto & sources = rows_[req->fault_code];
            if (std::find(sources.begin(), sources.end(), req->source_id) == sources.end()) {
              sources.push_back(req->source_id);
            }
          }
          res->accepted = true;
        });
  }

  void open_clears() {
    clear_srv_ = node_->create_service<ros2_medkit_msgs::srv::ClearFault>(
        "/fault_manager/clear_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> req,
                                             std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> res) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            cleared_.push_back(*req);
            rows_.erase(req->fault_code);  // ClearFault carries no source: the row goes
          }
          res->success = true;
        });
  }

  void open_reads(bool answer_immediately = true) {
    answer_reads_.store(answer_immediately);
    read_srv_ = node_->create_service<ros2_medkit_msgs::srv::GetFault>(
        "/fault_manager/get_fault", [this](const std::shared_ptr<rmw_request_id_t> header,
                                           const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> req) {
          if (answer_reads_.load()) {
            answer_read(*header, req->fault_code);
            return;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          parked_reads_.emplace_back(*header, req->fault_code);
        });
  }

  /// Answer the OLDEST parked read and keep parking the ones that follow, so a
  /// test can let a probe the plugin has already given up on answer late.
  bool release_one_read() {
    std::pair<rmw_request_id_t, std::string> parked;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (parked_reads_.empty()) {
        return false;
      }
      parked = parked_reads_.front();
      parked_reads_.erase(parked_reads_.begin());
    }
    answer_read(parked.first, parked.second);
    return true;
  }

  /// Answer every parked read and keep answering the ones that follow.
  void release_reads() {
    std::vector<std::pair<rmw_request_id_t, std::string>> parked;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      parked.swap(parked_reads_);
    }
    answer_reads_.store(true);
    for (auto & entry : parked) {
      answer_read(entry.first, entry.second);
    }
  }

  size_t parked_read_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return parked_reads_.size();
  }

  void seed(const std::string & fault_code, const std::vector<std::string> & sources) {
    std::lock_guard<std::mutex> lock(mutex_);
    rows_[fault_code] = sources;
  }

  std::vector<std::string> sources_of(const std::string & fault_code) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = rows_.find(fault_code);
    return it == rows_.end() ? std::vector<std::string>{} : it->second;
  }

  std::vector<std::string> cleared_codes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> codes;
    codes.reserve(cleared_.size());
    for (const auto & req : cleared_) {
      codes.push_back(req.fault_code);
    }
    return codes;
  }

  std::vector<ros2_medkit_msgs::srv::ClearFault::Request> cleared() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cleared_;
  }

  std::vector<std::string> reported() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return reported_;
  }

 private:
  void answer_read(const rmw_request_id_t & header, const std::string & fault_code) {
    ros2_medkit_msgs::srv::GetFault::Response response;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = rows_.find(fault_code);
      response.success = it != rows_.end() && !it->second.empty();
      if (response.success) {
        response.fault.fault_code = fault_code;
        response.fault.reporting_sources = it->second;
      }
    }
    rmw_request_id_t id = header;
    read_srv_->send_response(id, response);
  }

  rclcpp::Node::SharedPtr node_;
  mutable std::mutex mutex_;
  std::map<std::string, std::vector<std::string>> rows_;
  std::vector<ros2_medkit_msgs::srv::ClearFault::Request> cleared_;
  std::vector<std::string> reported_;
  std::vector<std::pair<rmw_request_id_t, std::string>> parked_reads_;
  std::atomic<bool> answer_reads_{true};
  rclcpp::Service<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetFault>::SharedPtr read_srv_;
};

}  // namespace

// A cancel() that throws must not cost the join. If it does, the guard's thread
// member is destroyed while joinable and ~std::thread calls std::terminate, so
// this test does not fail - it takes the whole binary down with SIGABRT, which
// is why it asserts on having reached the end at all.
TEST(ScopedExecutorSpinTest, AThrowingCancelStillJoinsTheThread) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("scoped_spin_throwing_cancel");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  {
    // Fails the way rclcpp documents cancel() can - the guard condition cannot
    // be triggered - after actually stopping the spin, so what is under test is
    // the join and not a hang. Injected because cancel() is not virtual on every
    // distro this builds on.
    ScopedExecutorSpin spin(executor, [&executor]() {
      executor.cancel();
      throw std::runtime_error("cancel failed");
    });
    // stop() contains the throw itself, so the explicit teardown a test does at
    // the point it wants the executor quiet stays usable.
    EXPECT_NO_THROW(spin.stop());
  }

  executor.remove_node(node);
  SUCCEED() << "the guard joined its thread despite cancel() throwing";
}

// The connect-time clear, read off the wire. The decision is only reachable
// through a connect that SUCCEEDS, so it needs the live fixture, and the flag it
// sets is only observable with a real fault-manager service on the other end. A
// correlation rule may name PLC_COMMS_LOST as the root cause of every symptom an
// outage produced, and the link coming back is not an operator resolving those,
// so this clear must not cascade.
TEST_F(OpcuaIdentityE2ETest, ConnectTimeCommsLostClearSkipsTheCorrelationCascade) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_connect_clear");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_connect_clear_faultmgr");

  FaultStoreStub store(fault_manager);
  // A PLC_COMMS_LOST this bridge raised before the process restarted.
  store.seed(kCommsLostFaultCode, {"test_runtime"});
  store.open_reports();
  store.open_clears();
  store.open_reads();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  // The connect inside set_context() succeeds against the fixture, which is the
  // only way to reach the connect-time decision.
  plugin.set_context(ctx);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (store.cleared().empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  const auto cleared = store.cleared();
  ASSERT_FALSE(cleared.empty()) << "a successful connect sent no ClearFault at all";
  EXPECT_EQ(cleared.front().fault_code, std::string(kCommsLostFaultCode));
  EXPECT_TRUE(cleared.front().skip_correlation_auto_clear)
      << "the connect-time clear cascade-cleared the symptoms of the outage it ended";
}

// The other side of the same rule, also on the wire: when the DEVICE reports its
// condition inactive, that IS a resolution at the source, so the correlation
// engine may act on it and the flag stays off. Only a live AlarmCondition
// lifecycle reaches on_event_alarm's ClearFault arm, so this drives the
// fixture's own CLI to fire and then clear a condition.
TEST_F(OpcuaIdentityE2ETest, DeviceReportedAlarmClearKeepsTheCorrelationCascade) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_device_clear");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_device_clear_faultmgr");

  // Config-less: the component names itself from the device, and that is the id
  // the gate asks the store about. Read it the same way the plugin will, so no
  // test pins the fixture's nameplate spelling.
  const std::string component_id = device_derived_component_id(endpoint_);
  ASSERT_FALSE(component_id.empty());

  FaultStoreStub store(fault_manager);
  store.seed(kCommsLostFaultCode, {component_id});
  store.open_reports();
  store.open_clears();
  store.open_reads();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["poll_interval_ms"] = 100;
  // Zero-config native A&C on the Server EventNotifier, with auto_clear so the
  // condition going inactive clears the fault without an operator ack/confirm.
  config["auto_alarms"] = nlohmann::json{{"enabled", true}, {"auto_clear", true}};
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  plugin.set_context(ctx);

  const auto reported_count = [&store]() {
    return store.reported().size();
  };
  // The connect-time PLC_COMMS_LOST clear also lands here (this connect
  // succeeded), so a clear is looked up by the code it names.
  const auto clear_for = [&store](const std::string & code) -> std::optional<bool> {
    for (const auto & req : store.cleared()) {
      if (req.fault_code == code) {
        return req.skip_correlation_auto_clear;
      }
    }
    return std::nullopt;
  };

  // Fire until the event subscription is up and a report lands. The retry is the
  // subscription handshake, not flakiness in the assertion: an event fired
  // before the subscribe simply is not delivered.
  const auto fire_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (reported_count() == 0 && std::chrono::steady_clock::now() < fire_deadline) {
    ASSERT_TRUE(server_.send("fire Overpressure 750"));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  ASSERT_GT(reported_count(), 0u) << "the fixture's AlarmCondition never reached the fault manager";

  const std::string alarm_code = store.reported().front();
  ASSERT_TRUE(server_.send("clear Overpressure"));
  const auto clear_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!clear_for(alarm_code).has_value() && std::chrono::steady_clock::now() < clear_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spin.stop();
  plugin.shutdown();

  const auto device_clear_skips = clear_for(alarm_code);
  ASSERT_TRUE(device_clear_skips.has_value())
      << "the device reporting condition " << alarm_code << " inactive sent no ClearFault";
  EXPECT_FALSE(*device_clear_skips) << "a clear the device itself reported must keep the correlation cascade";

  // The connect-time clear travelled the same wire in the same test, and it is
  // the opposite case: not an operator resolving anything, so it does not
  // cascade. Having both here is what makes the flag above a decision rather
  // than a constant.
  const auto link_state_clear_skips = clear_for(kCommsLostFaultCode);
  ASSERT_TRUE(link_state_clear_skips.has_value()) << "the connect-time clear never arrived";
  EXPECT_TRUE(*link_state_clear_skips);
}

// One gateway may bridge two field buses, both raising PLC_COMMS_LOST, and
// ClearFault carries no source: it clears the whole row. A row this bridge
// shares with another is not this link's to clear, whether the foreign id is the
// only source or one of several. The sibling test above is the positive control
// - identical setup, the row naming only this bridge, and the clear goes out.
TEST_F(OpcuaIdentityE2ETest, ASharedCommsLostRowIsLeftStanding) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_foreign_clear");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_foreign_clear_faultmgr");

  FaultStoreStub store(fault_manager);
  // Two bridges hold the row: the other one's link says nothing about ours.
  store.seed(kCommsLostFaultCode, {"beckhoff_cx5140", "test_runtime"});
  store.open_reports();
  store.open_clears();
  store.open_reads();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  plugin.set_context(ctx);

  // The probe is asked, answered and acted on within a couple of poll cycles;
  // the sibling test's clear lands well inside this window on the same harness,
  // so an empty result here is a decision and not a missed deadline.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  bool probe_answered = false;
  while (!probe_answered && std::chrono::steady_clock::now() < deadline) {
    probe_answered = plugin.comms_lost_probe_count_for_test() > 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // Give any (wrong) clear the time the sibling test's right one needs.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  EXPECT_TRUE(probe_answered) << "the store was never asked who holds " << kCommsLostFaultCode;
  for (const auto & req : store.cleared()) {
    EXPECT_NE(req.fault_code, std::string(kCommsLostFaultCode))
        << "this link coming back cleared a row another bridge also holds";
  }
  EXPECT_EQ(store.sources_of(kCommsLostFaultCode).size(), 2u) << "the shared row was cleared";
}

// A link-state clear the bounded buffer refuses is owed, not abandoned. A
// gateway restarting with a persisted PLC_COMMS_LOST while the fault manager is
// still down has no reconnect coming - its connect SUCCEEDED - so nothing
// re-derives that clear and the fault stands CONFIRMED against a healthy link.
// The decision is taken again once the buffer has drained, and its clear lands
// behind everything it must not overtake.
TEST_F(OpcuaIdentityE2ETest, AnOwedLinkStateClearIsSentAfterTheBufferDrains) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_owed_clear");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_owed_clear_faultmgr");

  FaultStoreStub store(fault_manager);
  store.seed(kCommsLostFaultCode, {"test_runtime"});
  // Reads and clears are reachable; REPORTS are not, which is what holds the
  // pending buffer full so the link-state clear is refused by it.
  store.open_clears();
  store.open_reads();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  plugin.set_context(ctx);

  // More one-shot dispatches than the buffer can hold. Each is an operator's
  // scoped clear: nothing re-derives one, so they outrank the link-state clear,
  // which the buffer gives up first.
  const size_t queued = OpcuaPlugin::kMaxPendingDispatches + 44;
  for (size_t i = 0; i < queued; ++i) {
    static_cast<void>(plugin.clear_fault("tank", "PLC_OPERATOR_" + std::to_string(i)));
  }

  // The refusal needs a decision to have happened against the full buffer.
  const auto refusal_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (plugin.comms_lost_probe_count_for_test() == 0 && std::chrono::steady_clock::now() < refusal_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_GT(plugin.comms_lost_probe_count_for_test(), 0u) << "the store was never asked while the buffer was full";
  EXPECT_TRUE(store.cleared_codes().empty()) << "the buffer dispatched while the report sink was unreachable";

  store.open_reports();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  const auto comms_lost_position = [&store]() -> std::optional<size_t> {
    const auto codes = store.cleared_codes();
    for (size_t i = 0; i < codes.size(); ++i) {
      if (codes[i] == kCommsLostFaultCode) {
        return i;
      }
    }
    return std::nullopt;
  };
  while (!comms_lost_position().has_value() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  const auto codes = store.cleared_codes();
  const auto position = comms_lost_position();
  ASSERT_TRUE(position.has_value()) << "the owed " << kCommsLostFaultCode
                                    << " clear was never re-issued, so the fault stands against a live link";
  EXPECT_EQ(*position, codes.size() - 1) << "the owed clear overtook the buffered dispatches";
  EXPECT_EQ(std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode)), 1)
      << "the owed clear was re-issued more than once";
}

// A decision is taken only while the session is up. A clear decided against a
// link that is down would clear a fault that is genuinely standing, which is the
// state the poller has just reported. The probe predicate carries that term and
// CommsLostProbeDue is where it is falsified; this drives the same promise
// through the whole plugin.
TEST_F(OpcuaIdentityE2ETest, NoLinkStateClearIsDecidedWhileTheLinkIsDown) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_link_down");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_link_down_faultmgr");

  FaultStoreStub store(fault_manager);
  store.seed(kCommsLostFaultCode, {"test_runtime"});

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  config["comms_lost_debounce_ms"] = 200;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  // Connect succeeds, so a decision is owed; no fault services exist yet, so it
  // cannot be taken.
  plugin.set_context(ctx);

  server_.stop();  // the PLC goes away with the decision still owed
  store.open_reports();
  store.open_clears();
  store.open_reads();

  // The link being down is what the poller reports, and that report is the
  // control: it proves the harness is live while no clear travels.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  const auto reported_comms_lost = [&store]() {
    const auto reported = store.reported();
    return std::find(reported.begin(), reported.end(), std::string(kCommsLostFaultCode)) != reported.end();
  };
  while (!reported_comms_lost() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  EXPECT_TRUE(reported_comms_lost()) << "the dead link was never reported, so this proves nothing";
  const auto codes = store.cleared_codes();
  EXPECT_EQ(std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode)), 0)
      << "a clear was decided against a link that is down";
}

// A store that never answers must not hold the decision for the life of the
// process. The probe is dropped once it outlives fault_service_timeout_ms, the
// decision is owed again, and the clear goes out on the answer that does come.
TEST_F(OpcuaIdentityE2ETest, AnUnansweredProbeIsDroppedAndTheDecisionIsTakenAgain) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_probe_timeout");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_probe_timeout_faultmgr");

  FaultStoreStub store(fault_manager);
  store.seed(kCommsLostFaultCode, {"test_runtime"});
  store.open_reports();
  store.open_clears();
  store.open_reads(/*answer_immediately=*/false);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  config["fault_service_timeout_ms"] = 1000;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  plugin.set_context(ctx);

  const auto probe_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (store.parked_read_count() < 2 && std::chrono::steady_clock::now() < probe_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  const size_t probes_before_release = store.parked_read_count();

  store.release_reads();
  const auto clear_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (store.cleared_codes().empty() && std::chrono::steady_clock::now() < clear_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  EXPECT_GE(probes_before_release, 2u) << "a probe the store never answered held the decision for good";
  const auto codes = store.cleared_codes();
  EXPECT_GE(std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode)), 1)
      << "the decision was never taken once the store answered";
}

// A probe the poll thread has given up on may still be answered by the store.
// That answer describes the question asked before the timeout, and the decision
// waiting now belongs to the probe that replaced it, so the late one is ignored
// and only the current probe's answer decides.
//
// Two mechanisms carry that, and neither is reachable alone from here:
// remove_pending_request takes the entry out of the client, and the generation
// the timeout branch moves on catches a callback that won the race against that
// erase (rclcpp erases before it invokes the callback, outside its mutex). The
// race is not reproducible on demand, so this pins the behaviour and the unit
// test on the consume-side check is what discriminates the generation.
TEST_F(OpcuaIdentityE2ETest, AnAnswerToATimedOutProbeIsNotReadAsTheNextOnes) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_stale_answer");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_stale_answer_faultmgr");

  FaultStoreStub store(fault_manager);
  store.seed(kCommsLostFaultCode, {"test_runtime"});
  store.open_reports();
  store.open_clears();
  store.open_reads(/*answer_immediately=*/false);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  config["fault_service_timeout_ms"] = 1000;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  plugin.set_context(ctx);

  // Probe A parks, times out, and probe B parks behind it.
  const auto probe_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (store.parked_read_count() < 2 && std::chrono::steady_clock::now() < probe_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_GE(store.parked_read_count(), 2u) << "the probe never timed out, so there is no stale answer to ignore";

  // A answers late.
  ASSERT_TRUE(store.release_one_read());
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_TRUE(store.cleared_codes().empty()) << "an answer to a probe already given up on decided the clear";

  // B answers, and that is the answer the decision is waiting for.
  store.release_reads();
  const auto clear_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (store.cleared_codes().empty() && std::chrono::steady_clock::now() < clear_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  const auto codes = store.cleared_codes();
  EXPECT_EQ(std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode)), 1)
      << "the current probe's answer did not decide exactly one clear";
}

// An answer parked before the link dropped is not applied after the reconnect.
//
// The decision is driven from publish_values, which the poll loop reaches only
// while connected, so an answer parked just before a drop is first looked at on
// the tick AFTER the reconnect - when the link reads as up and the store's
// answer describes a store from before an outage the poller has since reported.
// Here a second bridge raises the shared code during that outage, so acting on
// the stale answer would clear a row another bridge now holds.
TEST_F(OpcuaIdentityE2ETest, AnAnswerParkedBeforeAnOutageIsNotAppliedAfterTheReconnect) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_stale_session");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_stale_session_faultmgr");

  FaultStoreStub store(fault_manager);
  store.seed(kCommsLostFaultCode, {"test_runtime"});
  store.open_reports();
  store.open_clears();
  store.open_reads(/*answer_immediately=*/false);  // the first probe parks

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  config["comms_lost_debounce_ms"] = 200;
  // Long enough that the probe does not time out while the link is down: the
  // answer has to survive to the tick after the reconnect, which is the case
  // under test.
  config["fault_service_timeout_ms"] = 120000;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  plugin.set_context(ctx);

  const auto wait_for = [](const std::function<bool()> & done, std::chrono::seconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return done();
  };

  // The connect-time probe is parked, unanswered.
  ASSERT_TRUE(wait_for(
      [&store]() {
        return store.parked_read_count() >= 1;
      },
      std::chrono::seconds(30)))
      << "the connect never asked the store, so this proves nothing";

  // The link drops with the probe still outstanding. The poll loop sits in its
  // reconnect arm from here, so nothing consumes an answer until it is back.
  server_.stop();
  ASSERT_TRUE(wait_for(
      [&store]() {
        const auto reported = store.reported();
        return std::find(reported.begin(), reported.end(), std::string(kCommsLostFaultCode)) != reported.end();
      },
      std::chrono::seconds(30)))
      << "the outage was never reported, so this proves nothing";

  // The store answers the outstanding probe now, describing the row as it was
  // when the probe was sent: ours alone.
  store.release_reads();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // A second bridge reports the same code while the outage lasts, so the row
  // the parked answer describes is not the row standing now.
  store.seed(kCommsLostFaultCode, {"test_runtime", "beckhoff_cx5140"});

  // The PLC comes back. The first tick after the reconnect is where the stale
  // answer would be consumed.
  ASSERT_TRUE(server_.start(fixture_binary(), port_)) << "the fixture did not come back";
  ASSERT_TRUE(wait_until_connectable());
  ASSERT_TRUE(wait_for(
      [&plugin]() {
        return plugin.comms_lost_probe_count_for_test() >= 2;
      },
      std::chrono::seconds(30)))
      << "the decision was never taken again on the new session";
  std::this_thread::sleep_for(std::chrono::seconds(2));

  spin.stop();
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  const auto codes = store.cleared_codes();
  EXPECT_EQ(std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode)), 0)
      << "an answer from the session before the outage cleared a row two bridges now hold";
  EXPECT_EQ(store.sources_of(kCommsLostFaultCode).size(), 2u) << "the shared row was cleared";
}

// The config-less restart heal, end to end. A gateway that starts while its PLC
// is down names the component after the endpoint and raises PLC_COMMS_LOST under
// that stand-in. When the PLC returns the device names itself, so the id the
// store holds is no id the component still carries - and the fault this very
// process raised has to heal all the same.
TEST_F(OpcuaIdentityE2ETest, AFaultRaisedUnderTheStandInHealsAfterTheDeviceNamesItself) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_standin_heal");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_standin_heal_faultmgr");

  const std::string nameplate_id = device_derived_component_id(endpoint_);
  ASSERT_FALSE(nameplate_id.empty());

  FaultStoreStub store(fault_manager);
  store.open_reports();
  store.open_clears();
  store.open_reads();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  server_.stop();  // the PLC is down when the gateway starts

  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;  // config-less: no node map, so the device names the component
  config["poll_interval_ms"] = 100;
  config["comms_lost_debounce_ms"] = 200;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  plugin.set_context(ctx);

  // The outage is reported under the endpoint-derived stand-in.
  const auto raise_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (store.sources_of(kCommsLostFaultCode).empty() && std::chrono::steady_clock::now() < raise_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  const auto raised_sources = store.sources_of(kCommsLostFaultCode);
  ASSERT_EQ(raised_sources.size(), 1u) << "the outage was never reported, so this proves nothing";
  EXPECT_EQ(raised_sources.front(), derive_component_identity(OpcuaClient::DeviceInfo{}, endpoint_).id)
      << "expected the endpoint-derived stand-in, got '" << raised_sources.front() << "'";
  EXPECT_NE(raised_sources.front(), nameplate_id);

  ASSERT_TRUE(server_.start(fixture_binary(), port_)) << "the fixture did not come back";
  ASSERT_TRUE(wait_until_connectable());

  const auto heal_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
  while (store.sources_of(kCommsLostFaultCode).size() != 0 && std::chrono::steady_clock::now() < heal_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spin.stop();
  plugin.shutdown();

  const auto codes = store.cleared_codes();
  EXPECT_GE(std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode)), 1)
      << "a fault this process raised under its stand-in was never healed after the device named itself";
  EXPECT_TRUE(store.sources_of(kCommsLostFaultCode).empty());
}

// The binding, driven end to end against live fixtures.
//
// The sweep is substituted (which is what the injected discovery I/O is for) so
// the test decides which address is open and which identity is served there;
// every session, identity read and connect is real. Discovery identifies OPC-UA
// only on 4840 (network_discovery.cpp), so the fixtures listen there and it is
// the ADDRESS that varies - a fixture binds every interface, so one process is
// reachable at 127.0.0.1 and 127.0.0.2 alike, which is what makes "the same
// server at a new address" and "a different server at the bound address" both
// reachable.
//
// Three claims: a live, reachable foreign server is not adopted by a sweep; a
// live foreign server at the bound address is dropped at connect; the bound
// server at a new address is re-adopted.
TEST_F(OpcuaIdentityE2ETest, TheBridgeStaysBoundToItsOwnServerAcrossAddressAndSwap) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_binding");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_binding_faultmgr");

  server_.stop();  // the base class's fixture does not take part
  constexpr int kOpcuaPort = 4840;
  const std::string bound_address = "127.0.0.1";
  const std::string other_address = "127.0.0.2";
  const std::string foreign_uri = "urn:test:a-different-plc";
  const auto url_for = [](const std::string & ip) {
    return "opc.tcp://" + ip + ":4840";
  };

  // What the sweep reports: one address, and the identity served there.
  std::mutex sweep_mutex;
  std::string open_address = bound_address;
  std::string served_uri;
  const auto scan = [&sweep_mutex, &open_address](const std::string & ip, uint16_t port, int) {
    std::lock_guard<std::mutex> lock(sweep_mutex);
    return port == kOpcuaPort && ip == open_address;
  };
  const auto identify = [&sweep_mutex, &served_uri](const std::string & url, int) {
    IdentifyResult result;
    result.ok = true;
    result.advertised_url = url;
    {
      std::lock_guard<std::mutex> lock(sweep_mutex);
      result.application_uri = served_uri;
    }
    result.application_name = "Test PLC";
    result.application_type = 0;  // Server
    result.anonymous_none_available = true;
    return result;
  };
  const auto sweep_reports = [&sweep_mutex, &open_address, &served_uri](const std::string & ip,
                                                                        const std::string & uri) {
    std::lock_guard<std::mutex> lock(sweep_mutex);
    open_address = ip;
    served_uri = uri;
  };

  AlarmServer bound_server;
  ASSERT_TRUE(bound_server.start(fixture_binary(), kOpcuaPort))
      << "this test needs TCP 4840 on loopback, the only port discovery identifies OPC-UA on";
  ASSERT_TRUE(wait_for_connectable(url_for(bound_address)));
  // The identity the fixture actually serves is what the binding becomes, so
  // the sweep reports the same one and selection has something to look for.
  const std::string bound_uri = live_application_uri(url_for(bound_address));
  ASSERT_FALSE(bound_uri.empty()) << "the fixture publishes no ApplicationUri, so nothing can bind to it";
  ASSERT_NE(bound_uri, foreign_uri);
  sweep_reports(bound_address, bound_uri);

  FaultStoreStub store(fault_manager);
  // A standing outage, so every accepted session produces a ClearFault: that
  // clear is how the test sees which server the plugin is polling.
  store.seed(kCommsLostFaultCode, {"test_runtime"});
  store.open_reports();
  store.open_clears();
  store.open_reads();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  ScopedExecutorSpin spin(executor);

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  plugin.set_discovery_io_for_test(scan, identify);
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  config["comms_lost_debounce_ms"] = 200;
  config["discovery"] = nlohmann::json{{"enabled", true},
                                       {"subnets", nlohmann::json::array({"127.0.0.0/30"})},
                                       {"ports", nlohmann::json::array({kOpcuaPort})},
                                       {"interval_s", 2}};
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  plugin.set_context(ctx);

  const auto cleared_count = [&store]() {
    const auto codes = store.cleared_codes();
    return std::count(codes.begin(), codes.end(), std::string(kCommsLostFaultCode));
  };
  const auto wait_for = [](const std::function<bool()> & done, std::chrono::seconds budget) {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return done();
  };
  const auto outage_reported = [&store]() {
    const auto reported = store.reported();
    return std::find(reported.begin(), reported.end(), std::string(kCommsLostFaultCode)) != reported.end();
  };

  ASSERT_TRUE(wait_for(
      [&]() {
        return cleared_count() > 0;
      },
      std::chrono::seconds(30)))
      << "the startup scan never adopted the fixture, so this proves nothing";
  const auto clears_after_binding = cleared_count();

  // ---- a LIVE foreign server, reachable at the bound address and at the one
  //      the sweep offers, is neither adopted nor polled ---------------------
  bound_server.stop();
  AlarmServer foreign_server;
  ASSERT_TRUE(foreign_server.start(fixture_binary(), kOpcuaPort, {"--app-uri", foreign_uri}));
  ASSERT_TRUE(wait_for_connectable(url_for(bound_address))) << "the foreign fixture never became connectable";
  ASSERT_EQ(live_application_uri(url_for(bound_address)), foreign_uri);
  sweep_reports(other_address, foreign_uri);

  ASSERT_TRUE(wait_for(outage_reported, std::chrono::seconds(30)))
      << "the outage was never reported, so this proves nothing";
  // The sweep saw the foreign server and found no hit carrying the binding.
  ASSERT_TRUE(wait_for(
      [&]() {
        return plugin.rescan_refused_count_for_test() > 0;
      },
      std::chrono::seconds(30)))
      << "the sweep never reported the bound server missing, so nothing here is about the binding";
  // ... and the connect the reconnect arm keeps attempting at the bound address
  // reaches that same foreign server, which the session's own identity catches.
  ASSERT_TRUE(wait_for(
      [&]() {
        return plugin.binding_mismatch_count_for_test() > 0;
      },
      std::chrono::seconds(30)))
      << "a different server at the bound address was polled as if it were the bound one";
  std::this_thread::sleep_for(std::chrono::seconds(3));

  EXPECT_EQ(cleared_count(), clears_after_binding) << "a server this bridge is not bound to cleared the outage";
  EXPECT_FALSE(store.sources_of(kCommsLostFaultCode).empty())
      << "the outage was healed by a server this bridge is not bound to";

  // ---- the bound server, at the address the foreign one was offered on, is
  //      re-adopted ----------------------------------------------------------
  foreign_server.stop();
  AlarmServer moved_server;
  ASSERT_TRUE(moved_server.start(fixture_binary(), kOpcuaPort));
  ASSERT_TRUE(wait_for_connectable(url_for(other_address))) << "the moved fixture never became connectable";
  sweep_reports(other_address, bound_uri);

  const bool recovered = wait_for(
      [&]() {
        return cleared_count() > clears_after_binding;
      },
      std::chrono::seconds(60));

  spin.stop();
  plugin.shutdown();
  moved_server.stop();
  std::remove(yaml_path.c_str());

  EXPECT_TRUE(recovered) << "the bound server at " << url_for(other_address)
                         << " was not re-adopted, so the outage never ended";
  EXPECT_TRUE(store.sources_of(kCommsLostFaultCode).empty()) << "the outage was cleared but the row still stands";
}

// The order the whole rename fix rests on: PollerConfig::on_connected runs
// before the link-state edge and before anything is subscribed, so whatever it
// renames is what the event path is handed. apply_condition_state pins a fault's
// entity at the first sighting of its ConditionId, and the ConditionRefresh
// burst that follows the subscribe is that first sighting for every condition
// the device had standing - so a rename after it files those faults under an
// entity the rename then drops.
TEST_F(OpcuaIdentityE2ETest, TheConnectedHookRunsBeforeTheEventRoutingIsCopied) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  NodeMap node_map;  // config-less: named after the endpoint until a device answers
  node_map.set_component_identity("opcua-127_0_0_1", "opcua-127_0_0_1");
  node_map.mutable_auto_alarms().enabled = true;
  ASSERT_TRUE(node_map.finalize_auto_alarms_overlay());
  ASSERT_EQ(node_map.auto_alarms().entity_id, "opcua-127_0_0_1_alarms");

  OpcuaPoller poller(client, node_map);
  std::atomic<int> hook_calls{0};
  PollerConfig poller_config;
  poller_config.poll_interval = std::chrono::milliseconds(100);
  poller_config.on_connected = [&hook_calls, &node_map]() {
    hook_calls.fetch_add(1);
    // Exactly what the plugin's hook does once the adopted device names itself.
    node_map.mutable_auto_alarms().entity_id.clear();
    node_map.set_component_identity("siemens_ag_cpu_1505sp_f", "Siemens AG CPU 1505SP F");
    node_map.finalize_auto_alarms_overlay();
    return true;
  };
  poller.start(poller_config);

  const auto routing = poller.alarm_routing();
  poller.stop();
  client.disconnect();

  EXPECT_EQ(hook_calls.load(), 1) << "the connected hook never fired on a session that was already up";
  ASSERT_TRUE(routing) << "no event subscription was made, so nothing was copied";
  EXPECT_EQ(routing->auto_alarms.entity_id, "siemens_ag_cpu_1505sp_f_alarms")
      << "the routing was copied before the rename, so every condition replayed on this session "
         "would be pinned under an entity the rename drops";
}

}  // namespace ros2_medkit_gateway
