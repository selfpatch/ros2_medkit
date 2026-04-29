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

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"
#include "ros2_medkit_gateway/core/managers/configuration_manager.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"

namespace ros2_medkit_gateway {
namespace {

class MockParameterTransport : public ParameterTransport {
 public:
  MockParameterTransport() = default;
  ~MockParameterTransport() override = default;
  MockParameterTransport(const MockParameterTransport &) = delete;
  MockParameterTransport & operator=(const MockParameterTransport &) = delete;
  MockParameterTransport(MockParameterTransport &&) = delete;
  MockParameterTransport & operator=(MockParameterTransport &&) = delete;

  bool is_self_node(const std::string & node_name) const override {
    return node_name == self_node_;
  }

  ParameterResult list_parameters(const std::string & node_name) override {
    last_list_node_ = node_name;
    ++list_calls_;
    ParameterResult r;
    r.success = list_success_;
    r.data = list_data_;
    r.error_message = list_error_;
    r.error_code = list_error_code_;
    return r;
  }

  ParameterResult get_parameter(const std::string & node_name, const std::string & param_name) override {
    last_get_node_ = node_name;
    last_get_name_ = param_name;
    ++get_calls_;
    ParameterResult r;
    r.success = get_success_;
    r.data = get_data_;
    r.error_message = get_error_;
    r.error_code = get_error_code_;
    return r;
  }

  ParameterResult set_parameter(const std::string & node_name, const std::string & param_name,
                                const json & value) override {
    last_set_node_ = node_name;
    last_set_name_ = param_name;
    last_set_value_ = value;
    ++set_calls_;
    set_history_.push_back({node_name, param_name, value});
    ParameterResult r;
    auto override_it = set_failures_.find(param_name);
    if (override_it != set_failures_.end()) {
      r.success = false;
      r.error_message = override_it->second;
      r.error_code = ParameterErrorCode::INVALID_VALUE;
      return r;
    }
    r.success = set_success_;
    if (set_success_) {
      json obj;
      obj["name"] = param_name;
      obj["value"] = value;
      obj["type"] = "string";
      r.data = obj;
    } else {
      r.error_message = set_error_;
      r.error_code = set_error_code_;
    }
    return r;
  }

  ParameterResult list_own_parameters() override {
    ++own_list_calls_;
    ParameterResult r;
    r.success = true;
    r.data = json::array({json{{"name", "self_param"}, {"value", 1}, {"type", "int"}}});
    return r;
  }

  ParameterResult get_own_parameter(const std::string & param_name) override {
    last_own_get_name_ = param_name;
    ++own_get_calls_;
    ParameterResult r;
    r.success = true;
    r.data = json{{"name", param_name}, {"value", 7}, {"type", "int"}};
    return r;
  }

  ParameterResult get_default(const std::string & node_name, const std::string & param_name) override {
    last_get_default_node_ = node_name;
    last_get_default_name_ = param_name;
    ++get_default_calls_;
    ParameterResult r;
    auto node_it = defaults_.find(node_name);
    if (node_it == defaults_.end()) {
      r.success = false;
      r.error_message = "no defaults";
      r.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
      return r;
    }
    auto val_it = node_it->second.find(param_name);
    if (val_it == node_it->second.end()) {
      r.success = false;
      r.error_message = "no default for param";
      r.error_code = ParameterErrorCode::NOT_FOUND;
      return r;
    }
    r.success = true;
    r.data = val_it->second;
    return r;
  }

  ParameterResult list_defaults(const std::string & node_name) override {
    last_list_defaults_node_ = node_name;
    ++list_defaults_calls_;
    ParameterResult r;
    auto node_it = defaults_.find(node_name);
    if (node_it == defaults_.end()) {
      r.success = false;
      r.error_message = "no defaults";
      r.error_code = ParameterErrorCode::NO_DEFAULTS_CACHED;
      return r;
    }
    json arr = json::array();
    for (const auto & [name, value] : node_it->second) {
      json entry;
      entry["name"] = name;
      entry["value"] = value;
      entry["type"] = "string";
      arr.push_back(entry);
    }
    r.success = true;
    r.data = arr;
    return r;
  }

  bool is_node_available(const std::string &) const override {
    return true;
  }

  void invalidate(const std::string & node_name) override {
    invalidated_.push_back(node_name);
  }

  void shutdown() override {
    ++shutdown_calls_;
  }

  // Mock state knobs ---------------------------------------------------------
  std::string self_node_ = "/self";
  bool list_success_ = true;
  json list_data_ = json::array();
  std::string list_error_;
  ParameterErrorCode list_error_code_ = ParameterErrorCode::NONE;

  bool get_success_ = true;
  json get_data_ = json::object();
  std::string get_error_;
  ParameterErrorCode get_error_code_ = ParameterErrorCode::NONE;

  bool set_success_ = true;
  std::string set_error_;
  ParameterErrorCode set_error_code_ = ParameterErrorCode::NONE;
  std::map<std::string, std::string> set_failures_;  // param_name -> reason

  std::map<std::string, std::map<std::string, json>> defaults_;

  // Recording state ----------------------------------------------------------
  struct SetCall {
    std::string node;
    std::string param;
    json value;
  };

  std::string last_list_node_;
  std::string last_get_node_, last_get_name_;
  std::string last_set_node_, last_set_name_;
  json last_set_value_;
  std::string last_own_get_name_;
  std::string last_get_default_node_, last_get_default_name_;
  std::string last_list_defaults_node_;
  std::vector<SetCall> set_history_;
  std::vector<std::string> invalidated_;

  int list_calls_ = 0;
  int get_calls_ = 0;
  int set_calls_ = 0;
  int own_list_calls_ = 0;
  int own_get_calls_ = 0;
  int get_default_calls_ = 0;
  int list_defaults_calls_ = 0;
  int shutdown_calls_ = 0;
};

}  // namespace

TEST(ConfigurationManagerRoutingTest, ListParametersDelegatesToTransportForRemoteNode) {
  auto mock = std::make_shared<MockParameterTransport>();
  mock->list_data_ = json::array({json{{"name", "speed"}, {"value", 42}, {"type", "int"}}});
  ConfigurationManager mgr(mock);
  auto r = mgr.list_parameters("/remote/node");
  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->last_list_node_, "/remote/node");
  EXPECT_EQ(mock->list_calls_, 1);
  EXPECT_EQ(mock->own_list_calls_, 0);
  ASSERT_TRUE(r.data.is_array());
  ASSERT_EQ(r.data.size(), 1u);
  EXPECT_EQ(r.data[0]["name"], "speed");
}

TEST(ConfigurationManagerRoutingTest, ListParametersShortCircuitsForSelfNode) {
  auto mock = std::make_shared<MockParameterTransport>();
  mock->self_node_ = "/gateway";
  ConfigurationManager mgr(mock);
  auto r = mgr.list_parameters("/gateway");
  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->own_list_calls_, 1);
  EXPECT_EQ(mock->list_calls_, 0);  // no IPC delegation
}

TEST(ConfigurationManagerRoutingTest, GetParameterShortCircuitsForSelfNode) {
  auto mock = std::make_shared<MockParameterTransport>();
  mock->self_node_ = "/gateway";
  ConfigurationManager mgr(mock);
  auto r = mgr.get_parameter("/gateway", "log_level");
  EXPECT_TRUE(r.success);
  EXPECT_EQ(mock->own_get_calls_, 1);
  EXPECT_EQ(mock->last_own_get_name_, "log_level");
  EXPECT_EQ(mock->get_calls_, 0);
}

TEST(ConfigurationManagerRoutingTest, SetParameterDelegatesAndPropagatesError) {
  auto mock = std::make_shared<MockParameterTransport>();
  mock->set_success_ = false;
  mock->set_error_ = "invalid value";
  mock->set_error_code_ = ParameterErrorCode::INVALID_VALUE;
  ConfigurationManager mgr(mock);
  auto r = mgr.set_parameter("/node", "p", 1);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error_code, ParameterErrorCode::INVALID_VALUE);
  EXPECT_EQ(r.error_message, "invalid value");
  EXPECT_EQ(mock->last_set_node_, "/node");
  EXPECT_EQ(mock->last_set_name_, "p");
}

TEST(ConfigurationManagerRoutingTest, ResetParameterUsesCachedDefaultThenSet) {
  auto mock = std::make_shared<MockParameterTransport>();
  mock->defaults_["/node"]["p"] = 99;
  ConfigurationManager mgr(mock);
  auto r = mgr.reset_parameter("/node", "p");
  ASSERT_TRUE(r.success);
  EXPECT_EQ(mock->get_default_calls_, 1);
  EXPECT_EQ(mock->last_get_default_node_, "/node");
  EXPECT_EQ(mock->last_get_default_name_, "p");
  EXPECT_EQ(mock->set_calls_, 1);
  EXPECT_EQ(mock->last_set_value_, 99);
  ASSERT_TRUE(r.data.is_object());
  EXPECT_EQ(r.data["reset_to_default"], true);
}

TEST(ConfigurationManagerRoutingTest, ResetParameterPropagatesNoDefaults) {
  auto mock = std::make_shared<MockParameterTransport>();
  ConfigurationManager mgr(mock);
  auto r = mgr.reset_parameter("/node", "p");
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error_code, ParameterErrorCode::NO_DEFAULTS_CACHED);
  EXPECT_EQ(mock->set_calls_, 0);  // never reached set
}

TEST(ConfigurationManagerRoutingTest, ResetAllParametersFanOutsAndAggregates) {
  auto mock = std::make_shared<MockParameterTransport>();
  mock->defaults_["/node"]["a"] = 1;
  mock->defaults_["/node"]["b"] = 2;
  mock->defaults_["/node"]["c"] = 3;
  mock->set_failures_["b"] = "read-only";  // one of three fails
  ConfigurationManager mgr(mock);
  auto r = mgr.reset_all_parameters("/node");
  EXPECT_FALSE(r.success);  // partial failure
  EXPECT_EQ(mock->list_defaults_calls_, 1);
  EXPECT_EQ(mock->set_calls_, 3);
  ASSERT_TRUE(r.data.is_object());
  EXPECT_EQ(r.data["reset_count"], 2u);
  EXPECT_EQ(r.data["failed_count"], 1u);
  ASSERT_TRUE(r.data["failed_parameters"].is_array());
  EXPECT_EQ(r.data["failed_parameters"][0], "b");
}

TEST(ConfigurationManagerRoutingTest, ShutdownIsIdempotentAndForwardsToTransport) {
  auto mock = std::make_shared<MockParameterTransport>();
  ConfigurationManager mgr(mock);
  mgr.shutdown();
  mgr.shutdown();
  mgr.shutdown();
  EXPECT_EQ(mock->shutdown_calls_, 1);  // forwarded once
  // Subsequent operations short-circuit with SHUT_DOWN.
  auto r = mgr.list_parameters("/node");
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error_code, ParameterErrorCode::SHUT_DOWN);
  EXPECT_EQ(mock->list_calls_, 0);
}

}  // namespace ros2_medkit_gateway
