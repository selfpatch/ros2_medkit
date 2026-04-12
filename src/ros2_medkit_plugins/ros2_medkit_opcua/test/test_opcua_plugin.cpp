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

// Unit tests for OpcuaPlugin's DataProvider/OperationProvider/FaultProvider
// methods. These test the provider layer directly (no HTTP, no OPC-UA server)
// by constructing the plugin with a pre-loaded NodeMap and a fake poller
// snapshot. The gateway's PluginContext is stubbed with a minimal
// FakePluginContext that returns entities and faults on demand.

#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"

// -- Stub PluginRequest/PluginResponse (same pattern as graph_provider test) --

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

// -- FakePluginContext --

class FakePluginContext : public PluginContext {
 public:
  std::unordered_map<std::string, PluginEntityInfo> entities;
  nlohmann::json all_faults = nlohmann::json::object();

  rclcpp::Node * node() const override {
    return nullptr;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities.find(id);
    if (it != entities.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string &) const override {
    return {};
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    nlohmann::json result;
    result["faults"] = nlohmann::json::array();
    if (all_faults.contains("faults")) {
      for (const auto & f : all_faults["faults"]) {
        if (f.value("source_id", "") == entity_id) {
          result["faults"].push_back(f);
        }
      }
    }
    return result;
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
    return {true, "", ""};
  }
  tl::expected<LockInfo, LockError> acquire_lock(const std::string &, const std::string &,
                                                 const std::vector<std::string> &, int) override {
    return tl::make_unexpected(LockError{"not supported", ""});
  }
  tl::expected<void, LockError> release_lock(const std::string &, const std::string &) override {
    return tl::make_unexpected(LockError{"not supported", ""});
  }
  IntrospectionInput get_entity_snapshot() const override {
    return {};
  }
  nlohmann::json list_all_faults() const override {
    return all_faults;
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

// -- Test fixture --

class OpcuaPluginTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Write a test node map YAML
    yaml_path_ = "/tmp/test_opcua_plugin_nodemap.yaml";
    std::ofstream f(yaml_path_);
    f << R"(
area_id: test_plc
component_id: test_runtime
nodes:
  - node_id: "ns=2;i=1"
    entity_id: tank
    data_name: level
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: true
    min_value: 0.0
    max_value: 1000.0
  - node_id: "ns=2;i=2"
    entity_id: tank
    data_name: pressure
    display_name: Tank Pressure
    unit: bar
    data_type: float
    writable: false
)";
    f.close();

    // Configure plugin with the test node map
    nlohmann::json config;
    config["node_map_path"] = yaml_path_;
    config["endpoint_url"] = "opc.tcp://nonexistent:4840";
    plugin_.configure(config);

    // Set up fake context with PLC entities
    ctx_.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
    ctx_.entities["test_runtime"] = {SovdEntityType::COMPONENT, "test_runtime", "/test_plc", "/test_plc/test_runtime"};
    plugin_.set_context(ctx_);
  }

  std::string yaml_path_;
  OpcuaPlugin plugin_;
  FakePluginContext ctx_;
};

// -- DataProvider tests --

TEST_F(OpcuaPluginTest, ListDataReturnsItems) {
  auto result = plugin_.list_data("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->contains("items"));
  EXPECT_EQ((*result)["items"].size(), 2u);
  EXPECT_EQ((*result)["items"][0]["id"], "level");
  EXPECT_EQ((*result)["items"][1]["id"], "pressure");
}

TEST_F(OpcuaPluginTest, ListDataEntityNotFound) {
  auto result = plugin_.list_data("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::EntityNotFound);
  EXPECT_EQ(result.error().http_status, 404);
}

TEST_F(OpcuaPluginTest, ReadDataReturnsValue) {
  auto result = plugin_.read_data("tank", "level");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["id"], "level");
  EXPECT_EQ((*result)["unit"], "mm");
  EXPECT_EQ((*result)["data_type"], "float");
  EXPECT_EQ((*result)["writable"], true);
}

TEST_F(OpcuaPluginTest, ReadDataNotFound) {
  auto result = plugin_.read_data("tank", "nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::ResourceNotFound);
}

TEST_F(OpcuaPluginTest, WriteDataReadOnly) {
  nlohmann::json body = {{"value", 5.0}};
  auto result = plugin_.write_data("tank", "pressure", body);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::ReadOnly);
  EXPECT_EQ(result.error().http_status, 400);
}

TEST_F(OpcuaPluginTest, WriteDataMissingValue) {
  nlohmann::json body = {{"not_value", 42}};
  auto result = plugin_.write_data("tank", "level", body);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, DataProviderError::InvalidValue);
}

// -- OperationProvider tests --

TEST_F(OpcuaPluginTest, ListOperationsOnlyWritable) {
  auto result = plugin_.list_operations("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->contains("items"));
  // Only 'level' is writable, 'pressure' is read-only
  EXPECT_EQ((*result)["items"].size(), 1u);
  EXPECT_EQ((*result)["items"][0]["id"], "set_level");
}

TEST_F(OpcuaPluginTest, ListOperationsEntityNotFound) {
  auto result = plugin_.list_operations("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::EntityNotFound);
}

TEST_F(OpcuaPluginTest, ExecuteOperationMissingValue) {
  nlohmann::json params = {{"not_value", 42}};
  auto result = plugin_.execute_operation("tank", "set_level", params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::InvalidParameters);
}

TEST_F(OpcuaPluginTest, ExecuteOperationReadOnly) {
  nlohmann::json params = {{"value", 5.0}};
  auto result = plugin_.execute_operation("tank", "set_pressure", params);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OperationProviderError::Rejected);
}

// -- FaultProvider tests --

TEST_F(OpcuaPluginTest, ListFaultsEmpty) {
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->contains("items"));
  EXPECT_TRUE((*result)["items"].empty());
}

TEST_F(OpcuaPluginTest, ListFaultsWithData) {
  ctx_.all_faults = {{"faults", {{{"fault_code", "PLC_LOW_LEVEL"}, {"source_id", "tank"}, {"severity", 2}}}}};
  auto result = plugin_.list_faults("tank");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["items"].size(), 1u);
  EXPECT_EQ((*result)["items"][0]["code"], "PLC_LOW_LEVEL");
}

TEST_F(OpcuaPluginTest, GetFaultNotFound) {
  auto result = plugin_.get_fault("tank", "NONEXISTENT");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, FaultProviderError::FaultNotFound);
}

}  // namespace ros2_medkit_gateway
