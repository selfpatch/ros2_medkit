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

#include "ros2_medkit_opcua/opcua_client.hpp"

#include <algorithm>
#include <atomic>
#include <deque>
#include <iostream>
#include <thread>
#include <type_traits>

namespace ros2_medkit_gateway {

namespace {

/// Set the connected flag to false when the BadStatus code indicates a
/// terminal connection loss (as opposed to e.g. BadNodeIdUnknown which is a
/// per-node issue). Called from read_value, read_values and write_value so
/// that OpcuaPoller's reconnect logic (which keys off is_connected()) fires
/// regardless of which operation detected the drop first.
void maybe_mark_disconnected(std::atomic<bool> & connected_flag, const opcua::BadStatus & e) {
  const auto code = e.code();
  if (code == UA_STATUSCODE_BADCONNECTIONCLOSED || code == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
      code == UA_STATUSCODE_BADNOTCONNECTED) {
    connected_flag = false;
  }
}

OpcuaValue variant_to_value(const opcua::Variant & var) {
  if (var.isEmpty()) {
    return std::string("<empty>");
  }
  if (var.isType<bool>()) {
    return var.getScalarCopy<bool>();
  }
  if (var.isType<int16_t>()) {
    return static_cast<int32_t>(var.getScalarCopy<int16_t>());
  }
  if (var.isType<uint16_t>()) {
    return static_cast<int32_t>(var.getScalarCopy<uint16_t>());
  }
  if (var.isType<int32_t>()) {
    return var.getScalarCopy<int32_t>();
  }
  if (var.isType<uint32_t>()) {
    return static_cast<int64_t>(var.getScalarCopy<uint32_t>());
  }
  if (var.isType<int64_t>()) {
    return var.getScalarCopy<int64_t>();
  }
  if (var.isType<float>()) {
    return var.getScalarCopy<float>();
  }
  if (var.isType<double>()) {
    return var.getScalarCopy<double>();
  }
  if (var.isType<opcua::String>()) {
    return std::string(var.getScalarCopy<opcua::String>());
  }
  return std::string("<unsupported type>");
}

opcua::Variant value_to_variant(const OpcuaValue & val) {
  return std::visit(
      [](auto && v) -> opcua::Variant {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::string>) {
          return opcua::Variant::fromScalar(opcua::String(v));
        } else if constexpr (std::is_same_v<T, float>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, double>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, bool>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, int32_t>) {
          return opcua::Variant::fromScalar(v);
        } else if constexpr (std::is_same_v<T, int64_t>) {
          return opcua::Variant::fromScalar(v);
        } else {
          return opcua::Variant::fromScalar(v);
        }
      },
      val);
}

}  // namespace

struct OpcuaClient::Impl {
  opcua::Client client;
  OpcuaClientConfig config;
  std::atomic<bool> connected{false};
  mutable std::mutex client_mutex;
  std::string server_desc;

  // Subscription tracking - store actual Subscription objects
  struct SubInfo {
    opcua::Subscription<opcua::Client> sub;
    DataChangeCallback callback;
  };
  std::deque<SubInfo> subscriptions;  // deque for stable references (callbacks capture &cb)
  std::mutex sub_mutex;
};

OpcuaClient::OpcuaClient() : impl_(std::make_unique<Impl>()) {
}

OpcuaClient::~OpcuaClient() {
  disconnect();
}

bool OpcuaClient::connect(const OpcuaClientConfig & config) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  impl_->config = config;

  try {
    if (impl_->client.isConnected()) {
      impl_->connected = true;
      return true;
    }
    impl_->client.config().setTimeout(static_cast<uint32_t>(config.connect_timeout.count()));
    impl_->client.connect(config.endpoint_url);
    impl_->connected = true;

    try {
      opcua::Node node(impl_->client, opcua::NodeId(0, UA_NS0ID_SERVER_SERVERSTATUS_BUILDINFO_PRODUCTNAME));
      auto val = node.readValue();
      if (val.isType<opcua::String>()) {
        impl_->server_desc = std::string(val.getScalarCopy<opcua::String>());
      }
    } catch (...) {
      impl_->server_desc = "OPC-UA Server";
    }

    return true;
  } catch (const opcua::BadStatus &) {
    impl_->connected = false;
    return false;
  }
}

void OpcuaClient::disconnect() {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  if (impl_->connected) {
    try {
      // Delete subscriptions first
      {
        std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
        for (auto & info : impl_->subscriptions) {
          try {
            info.sub.deleteSubscription();
          } catch (...) {
          }
        }
        impl_->subscriptions.clear();
      }
      impl_->client.disconnect();
    } catch (...) {
    }
    impl_->connected = false;
  }
}

bool OpcuaClient::is_connected() const {
  return impl_->connected.load();
}

OpcuaClientConfig OpcuaClient::current_config() const {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  return impl_->config;
}

std::string OpcuaClient::endpoint_url() const {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  return impl_->config.endpoint_url;
}

std::vector<std::string> OpcuaClient::browse(const opcua::NodeId & parent_node) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::vector<std::string> result;

  if (!impl_->connected) {
    return result;
  }

  try {
    opcua::Node node(impl_->client, parent_node);
    auto children = node.browseChildren();
    for (auto & child : children) {
      result.push_back(child.id().toString());
    }
  } catch (const opcua::BadStatus &) {
  }

  return result;
}

ReadResult OpcuaClient::read_value(const opcua::NodeId & node_id) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  ReadResult result;
  result.node_id = node_id.toString();
  result.timestamp = std::chrono::system_clock::now();

  if (!impl_->connected) {
    result.good = false;
    return result;
  }

  try {
    opcua::Node node(impl_->client, node_id);
    auto val = node.readValue();
    result.value = variant_to_value(val);
    result.good = true;
  } catch (const opcua::BadStatus & e) {
    result.good = false;
    maybe_mark_disconnected(impl_->connected, e);
  }

  return result;
}

std::vector<ReadResult> OpcuaClient::read_values(const std::vector<opcua::NodeId> & node_ids) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::vector<ReadResult> results;
  results.reserve(node_ids.size());
  auto now = std::chrono::system_clock::now();

  if (!impl_->connected) {
    for (const auto & nid : node_ids) {
      results.push_back({nid.toString(), {}, now, false});
    }
    return results;
  }

  for (const auto & nid : node_ids) {
    ReadResult r;
    r.node_id = nid.toString();
    r.timestamp = now;
    try {
      opcua::Node node(impl_->client, nid);
      r.value = variant_to_value(node.readValue());
      r.good = true;
    } catch (const opcua::BadStatus & e) {
      r.good = false;
      maybe_mark_disconnected(impl_->connected, e);
    }
    results.push_back(std::move(r));
  }
  return results;
}

tl::expected<void, OpcuaClient::WriteErrorInfo>
OpcuaClient::write_value(const opcua::NodeId & node_id, const OpcuaValue & value, const std::string & data_type_hint) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return tl::make_unexpected(WriteErrorInfo{WriteError::NotConnected, "Not connected to OPC-UA server"});
  }

  // Helper: coerce OpcuaValue to double for numeric writes
  auto to_double = [](const OpcuaValue & v) -> double {
    return std::visit(
        [](auto && x) -> double {
          if constexpr (std::is_arithmetic_v<std::decay_t<decltype(x)>>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        v);
  };

  // Helper: coerce OpcuaValue to int32 for integer writes
  auto to_int32 = [](const OpcuaValue & v) -> int32_t {
    return std::visit(
        [](auto && x) -> int32_t {
          if constexpr (std::is_arithmetic_v<std::decay_t<decltype(x)>>) {
            return static_cast<int32_t>(x);
          }
          return 0;
        },
        v);
  };

  // Helper: coerce OpcuaValue to bool
  auto to_bool = [](const OpcuaValue & v) -> bool {
    return std::visit(
        [](auto && x) -> bool {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_integral_v<T>) {
            return x != 0;
          } else if constexpr (std::is_floating_point_v<T>) {
            return x > T{0} || x < T{0};
          }
          return false;
        },
        v);
  };

  try {
    opcua::Node node(impl_->client, node_id);

    // Fast path: use the data_type_hint from NodeMap instead of probing
    // the server with a readValue round-trip. Halves mutex hold time.
    if (!data_type_hint.empty()) {
      if (data_type_hint == "float") {
        node.writeValueScalar(static_cast<float>(to_double(value)));
      } else if (data_type_hint == "int") {
        node.writeValueScalar(to_int32(value));
      } else if (data_type_hint == "bool") {
        node.writeValueScalar(to_bool(value));
      } else if (data_type_hint == "string") {
        auto sval = std::visit(
            [](auto && x) -> std::string {
              using T = std::decay_t<decltype(x)>;
              if constexpr (std::is_same_v<T, std::string>) {
                return x;
              } else {
                return std::to_string(x);
              }
            },
            value);
        node.writeValueScalar(opcua::String(sval));
      } else {
        // Unknown hint - fall through to probe path
        node.writeValueScalar(static_cast<float>(to_double(value)));
      }
      return {};
    }

    // Slow path: read current value to discover expected type, then write.
    auto current = node.readValue();

    if (current.isType<float>()) {
      node.writeValueScalar(static_cast<float>(to_double(value)));
    } else if (current.isType<double>()) {
      node.writeValueScalar(to_double(value));
    } else if (current.isType<int32_t>()) {
      node.writeValueScalar(to_int32(value));
    } else if (current.isType<bool>()) {
      node.writeValueScalar(to_bool(value));
    } else {
      auto var = value_to_variant(value);
      node.writeValue(var);
    }
    return {};
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, e);
    auto code = e.code();
    if (code == UA_STATUSCODE_BADTYPEMISMATCH) {
      return tl::make_unexpected(WriteErrorInfo{WriteError::TypeMismatch, e.what()});
    }
    if (code == UA_STATUSCODE_BADUSERACCESSDENIED || code == UA_STATUSCODE_BADNOTWRITABLE) {
      return tl::make_unexpected(WriteErrorInfo{WriteError::AccessDenied, e.what()});
    }
    if (code == UA_STATUSCODE_BADNODEIDUNKNOWN) {
      return tl::make_unexpected(WriteErrorInfo{WriteError::NodeNotFound, e.what()});
    }
    return tl::make_unexpected(WriteErrorInfo{WriteError::TransportError, e.what()});
  }
}

uint32_t OpcuaClient::create_subscription(double publish_interval_ms, DataChangeCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return 0;
  }

  try {
    opcua::SubscriptionParameters params{};
    params.publishingInterval = publish_interval_ms;

    auto sub = impl_->client.createSubscription(params);
    uint32_t sub_id = sub.subscriptionId();

    std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
    impl_->subscriptions.push_back({std::move(sub), std::move(callback)});

    return sub_id;
  } catch (const opcua::BadStatus &) {
    return 0;
  }
}

bool OpcuaClient::add_monitored_item(uint32_t subscription_id, const opcua::NodeId & node_id) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return false;
  }

  std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);
  // Find the subscription by ID
  for (auto & info : impl_->subscriptions) {
    if (info.sub.subscriptionId() == subscription_id) {
      try {
        auto nid_str = node_id.toString();
        auto cb_copy = info.callback;  // capture by value to avoid use-after-free

        info.sub.subscribeDataChange(
            node_id, opcua::AttributeId::Value,
            [nid_str, cb_copy](uint32_t /*sub_id*/, uint32_t /*mon_id*/, const opcua::DataValue & dv) {
              if (dv.hasValue()) {
                auto val = variant_to_value(dv.getValue());
                cb_copy(nid_str, val);
              }
            });
        return true;
      } catch (const opcua::BadStatus &) {
        return false;
      }
    }
  }
  return false;
}

void OpcuaClient::remove_subscriptions() {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  std::lock_guard<std::mutex> sub_lock(impl_->sub_mutex);

  for (auto & info : impl_->subscriptions) {
    try {
      info.sub.deleteSubscription();
    } catch (...) {
    }
  }
  impl_->subscriptions.clear();
}

std::string OpcuaClient::server_description() const {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  return impl_->server_desc;
}

}  // namespace ros2_medkit_gateway
