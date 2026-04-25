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
#include <cstring>
#include <deque>
#include <iostream>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <open62541/client_subscriptions.h>
#include <open62541/types.h>

namespace ros2_medkit_gateway {

// Forward declaration - defined after Impl so the trampoline can call back into
// the client. Static linkage keeps the symbol private to this translation unit.
struct EventCallbackContext;
static void on_event_trampoline_c(UA_Client * client, UA_UInt32 sub_id, void * sub_ctx, UA_UInt32 mon_id,
                                  void * mon_ctx, size_t n_fields, UA_Variant * fields);

/// Heap-owned context passed to the open62541 C event callback. Lifetime is
/// owned by ``Impl::event_callbacks`` (unique_ptr); the raw pointer handed to C
/// is valid until ``remove_event_monitored_item`` or ``remove_subscriptions``
/// erases the entry. The ``generation_snapshot`` lets the trampoline drop
/// callbacks that fire from a defunct subscription after a reconnect.
struct EventCallbackContext {
  OpcuaClient * owner{nullptr};
  uint64_t generation_snapshot{0};
  uint32_t subscription_id{0};
  uint32_t monitored_item_id{0};  // populated after createEvent returns
  OpcuaClient::EventCallback callback;
};

namespace {

/// Set the connected flag to false when the BadStatus code indicates a
/// terminal connection loss (as opposed to e.g. BadNodeIdUnknown which is a
/// per-node issue). Called from read_value, read_values and write_value so
/// that OpcuaPoller's reconnect logic (which keys off is_connected()) fires
/// regardless of which operation detected the drop first. Also bumps the
/// subscription generation so any in-flight event callbacks from the dying
/// subscription are filtered out by the trampoline.
void maybe_mark_disconnected(std::atomic<bool> & connected_flag, std::atomic<uint64_t> & generation,
                             const opcua::BadStatus & e) {
  const auto code = e.code();
  if (code == UA_STATUSCODE_BADCONNECTIONCLOSED || code == UA_STATUSCODE_BADSECURECHANNELCLOSED ||
      code == UA_STATUSCODE_BADNOTCONNECTED) {
    if (connected_flag.exchange(false)) {
      generation.fetch_add(1, std::memory_order_release);
    }
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

  // Issue #386: native OPC-UA AlarmCondition event subscription support.
  //
  // generation increments whenever the connection drops or is closed. The
  // event trampoline captures a snapshot at createEvent time and drops
  // notifications when the snapshot diverges from the live counter, so
  // late-arriving events from a defunct subscription cannot reach user code.
  std::atomic<uint64_t> generation{0};

  // Heap-owned contexts for raw-C event monitored items. unique_ptr keeps
  // the ctx alive exactly as long as the entry sits in the map; we release
  // entries only after the server ACKs DeleteMonitoredItem (or when we tear
  // down the entire client in disconnect()). Keyed by monitored_item_id; we
  // store sub_id alongside in the ctx so cleanup can address the right
  // subscription.
  std::unordered_map<uint32_t, std::unique_ptr<EventCallbackContext>> event_callbacks;
  std::mutex event_callbacks_mutex;
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
    // Bump generation FIRST so any in-flight event callbacks fired from the
    // dying subscription drop their work in the trampoline (they read
    // generation atomically) before we touch the storage they reference.
    impl_->generation.fetch_add(1, std::memory_order_release);
    try {
      // Issue #386: clear event monitored items BEFORE deleting subscriptions.
      // open62541's deleteSubscription cleans up server-side, but our
      // EventCallbackContext objects (held in unique_ptr) must outlive any
      // pending C callbacks; the generation bump above already filters them
      // out, so it is safe to drop the contexts here.
      {
        std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
        for (auto & [mi_id, ctx] : impl_->event_callbacks) {
          UA_Client_MonitoredItems_deleteSingle(impl_->client.handle(), ctx->subscription_id, mi_id);
        }
        impl_->event_callbacks.clear();
      }
      // Delete subscriptions
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
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
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
      maybe_mark_disconnected(impl_->connected, impl_->generation, e);
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
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
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
  // Issue #386: bump generation so the trampoline drops any callback fired
  // from the now-defunct subscription before we erase its EventCallbackContext.
  impl_->generation.fetch_add(1, std::memory_order_release);

  // Clear event monitored items BEFORE the open62541pp Subscription destructor
  // runs (subscription deletion cascades server-side, but the C callback's
  // userdata must remain valid until its last possible invocation).
  {
    std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
    for (auto & [mi_id, ctx] : impl_->event_callbacks) {
      UA_Client_MonitoredItems_deleteSingle(impl_->client.handle(), ctx->subscription_id, mi_id);
    }
    impl_->event_callbacks.clear();
  }

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

// ----------------------------------------------------------------------------
// Issue #386: native OPC-UA AlarmCondition event subscription primitives.
// open62541pp v0.16 has no native EventFilter / event subscription API, so the
// raw open62541 C API is used here. The free functions below build the
// EventFilter and trampoline; OpcuaClient owns the per-monitored-item context
// in unique_ptr storage so lifetime is explicit.
// ----------------------------------------------------------------------------

namespace {

/// Build an OPC-UA EventFilter from per-field SimpleAttributeOperand specs.
/// open62541 servers reject SAOs whose BrowsePath does not resolve directly
/// from the supplied ``typeDefinitionId`` (verified against open62541 1.4.6
/// with FULL ns0). Inheritance traversal is NOT performed during
/// validation, so ``AlarmConditionType+EventType`` returns
/// ``BadNodeIdUnknown`` even though EventType is inherited. The caller must
/// pass each field with the type that *directly* defines its first browse
/// segment.
///
/// Auto-prepends 3 fixed clauses so the trampoline can extract them
/// positionally:
///   [0] EventType  - BaseEventType property
///   [1] SourceNode - BaseEventType property
///   [2] ConditionId - ConditionType, empty BrowsePath, AttributeId=NodeId
///                    (Part 9 §5.5.2.13 special case)
UA_EventFilter make_event_filter(const std::vector<OpcuaClient::EventFieldSpec> & user_specs) {
  std::vector<OpcuaClient::EventFieldSpec> all_specs;
  all_specs.reserve(user_specs.size() + 3);
  all_specs.push_back({opcua::NodeId(0, UA_NS0ID_BASEEVENTTYPE), {{0, "EventType"}}, UA_ATTRIBUTEID_VALUE});
  all_specs.push_back({opcua::NodeId(0, UA_NS0ID_BASEEVENTTYPE), {{0, "SourceNode"}}, UA_ATTRIBUTEID_VALUE});
  all_specs.push_back({opcua::NodeId(0, UA_NS0ID_CONDITIONTYPE), {}, UA_ATTRIBUTEID_NODEID});
  for (const auto & s : user_specs) {
    all_specs.push_back(s);
  }

  UA_EventFilter filter;
  UA_EventFilter_init(&filter);
  filter.selectClausesSize = all_specs.size();
  filter.selectClauses = static_cast<UA_SimpleAttributeOperand *>(
      UA_Array_new(filter.selectClausesSize, &UA_TYPES[UA_TYPES_SIMPLEATTRIBUTEOPERAND]));

  for (size_t i = 0; i < all_specs.size(); ++i) {
    UA_SimpleAttributeOperand & sao = filter.selectClauses[i];
    UA_SimpleAttributeOperand_init(&sao);
    UA_NodeId_copy(all_specs[i].type_definition_id.handle(), &sao.typeDefinitionId);
    sao.attributeId = all_specs[i].attribute_id;
    const auto & path = all_specs[i].browse_path;
    sao.browsePathSize = path.size();
    if (sao.browsePathSize > 0) {
      sao.browsePath =
          static_cast<UA_QualifiedName *>(UA_Array_new(sao.browsePathSize, &UA_TYPES[UA_TYPES_QUALIFIEDNAME]));
      for (size_t j = 0; j < path.size(); ++j) {
        sao.browsePath[j] = UA_QUALIFIEDNAME_ALLOC(path[j].namespace_index, path[j].name.c_str());
      }
    }
  }
  return filter;
}

}  // namespace

// C-linkage trampoline matching ``UA_Client_EventNotificationCallback``.
// Defined at namespace scope so its address is a stable function pointer.
static void on_event_trampoline_c(UA_Client * /*client*/, UA_UInt32 sub_id, void * /*sub_ctx*/, UA_UInt32 mon_id,
                                  void * mon_ctx, size_t n_fields, UA_Variant * fields) {
  std::cerr << "[opcua_client] TRAMPOLINE FIRED sub=" << sub_id << " mon=" << mon_id << " n_fields=" << n_fields
            << std::endl;
  auto * ctx = static_cast<EventCallbackContext *>(mon_ctx);
  if (ctx == nullptr || ctx->owner == nullptr) {
    std::cerr << "[opcua_client] TRAMPOLINE: ctx null" << std::endl;
    return;
  }
  // Stale callback from a defunct subscription - ctx is still valid (we only
  // free contexts after the generation has already moved past), but the
  // payload no longer reflects live state. Drop silently.
  if (ctx->generation_snapshot != ctx->owner->current_generation()) {
    return;
  }

  // Copy the UA_Variant fields into open62541pp wrappers. UA_Variant_copy
  // duplicates the underlying buffer, which the opcua::Variant destructor
  // will free.
  std::vector<opcua::Variant> values;
  values.reserve(n_fields);
  for (size_t i = 0; i < n_fields; ++i) {
    UA_Variant copy;
    UA_Variant_init(&copy);
    UA_Variant_copy(&fields[i], &copy);
    values.emplace_back(opcua::Variant{std::move(copy)});
  }

  // Auto-prepended positions (matching make_event_filter):
  //   [0] EventType, [1] SourceNode, [2] ConditionId
  opcua::NodeId event_type;
  opcua::NodeId source_node;
  opcua::NodeId condition_id;
  if (n_fields >= 1 && values[0].isType<opcua::NodeId>()) {
    event_type = values[0].getScalarCopy<opcua::NodeId>();
  }
  if (n_fields >= 2 && values[1].isType<opcua::NodeId>()) {
    source_node = values[1].getScalarCopy<opcua::NodeId>();
  }
  if (n_fields >= 3 && values[2].isType<opcua::NodeId>()) {
    condition_id = values[2].getScalarCopy<opcua::NodeId>();
  }

  std::vector<opcua::Variant> user_values;
  if (n_fields > 3) {
    user_values.reserve(n_fields - 3);
    for (size_t i = 3; i < n_fields; ++i) {
      user_values.push_back(std::move(values[i]));
    }
  }

  if (ctx->callback) {
    ctx->callback(user_values, source_node, event_type, condition_id);
  }
}

uint64_t OpcuaClient::current_generation() const {
  return impl_->generation.load(std::memory_order_acquire);
}

void OpcuaClient::run_iterate(uint16_t timeout_ms) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);
  if (!impl_->connected) {
    return;
  }
  try {
    impl_->client.runIterate(timeout_ms);
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
  }
}

uint32_t OpcuaClient::add_event_monitored_item(uint32_t subscription_id, const opcua::NodeId & source_node,
                                               const std::vector<EventFieldSpec> & select_specs,
                                               EventCallback callback) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return 0;
  }

  // Heap-allocate context. Ownership stays in event_callbacks; the C API gets
  // a non-owning raw pointer until remove_event_monitored_item or cleanup
  // erases the entry.
  auto ctx = std::make_unique<EventCallbackContext>();
  ctx->owner = this;
  ctx->generation_snapshot = impl_->generation.load(std::memory_order_acquire);
  ctx->subscription_id = subscription_id;
  ctx->callback = std::move(callback);
  EventCallbackContext * raw_ctx = ctx.get();

  UA_EventFilter filter = make_event_filter(select_specs);

  UA_MonitoredItemCreateRequest item;
  UA_MonitoredItemCreateRequest_init(&item);
  // Deep-copy the source NodeId so the request struct owns its string
  // buffer (if any). Cleared by UA_MonitoredItemCreateRequest_clear after
  // the call.
  UA_NodeId_copy(source_node.handle(), &item.itemToMonitor.nodeId);
  item.itemToMonitor.attributeId = UA_ATTRIBUTEID_EVENTNOTIFIER;
  item.monitoringMode = UA_MONITORINGMODE_REPORTING;
  item.requestedParameters.samplingInterval = 0.0;
  item.requestedParameters.discardOldest = true;
  item.requestedParameters.queueSize = 100;
  UA_ExtensionObject_setValueNoDelete(&item.requestedParameters.filter, &filter, &UA_TYPES[UA_TYPES_EVENTFILTER]);

  // Debug log so integration-test failures surface the exact NodeId we
  // hand to the server. Trace-level diagnostic; can be tightened to a
  // ROS RCLCPP_DEBUG once the issue #386 server interop is stable.
  std::cerr << "[opcua_client] add_event_monitored_item: subId=" << subscription_id
            << " nodeId=" << source_node.toString() << " selectClauses=" << (select_specs.size() + 3)
            << std::endl;

  UA_MonitoredItemCreateResult result =
      UA_Client_MonitoredItems_createEvent(impl_->client.handle(), subscription_id, UA_TIMESTAMPSTORETURN_BOTH, item,
                                           raw_ctx, on_event_trampoline_c, /*deleteCallback=*/nullptr);

  // Free the locally-allocated request members (the ExtensionObject does
  // NOT own the filter because we used setValueNoDelete; clear it
  // separately below). UA_MonitoredItemCreateRequest_clear walks the
  // struct including itemToMonitor.nodeId.
  // Detach the filter from item.requestedParameters before clearing the
  // request, otherwise UA_*_clear would try to free our stack filter.
  // Re-init the ExtensionObject to a valid empty state.
  UA_ExtensionObject_init(&item.requestedParameters.filter);
  UA_MonitoredItemCreateRequest_clear(&item);
  UA_EventFilter_clear(&filter);

  std::cerr << "[opcua_client] createEvent result: status=" << UA_StatusCode_name(result.statusCode)
            << " miId=" << result.monitoredItemId << std::endl;

  if (result.statusCode != UA_STATUSCODE_GOOD) {
    UA_MonitoredItemCreateResult_clear(&result);
    return 0;
  }

  uint32_t mi_id = result.monitoredItemId;
  ctx->monitored_item_id = mi_id;
  UA_MonitoredItemCreateResult_clear(&result);

  {
    std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
    impl_->event_callbacks.emplace(mi_id, std::move(ctx));
  }
  return mi_id;
}

bool OpcuaClient::remove_event_monitored_item(uint32_t subscription_id, uint32_t mi_id) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  std::unique_ptr<EventCallbackContext> retired;
  {
    std::lock_guard<std::mutex> ev_lock(impl_->event_callbacks_mutex);
    auto it = impl_->event_callbacks.find(mi_id);
    if (it == impl_->event_callbacks.end() || it->second->subscription_id != subscription_id) {
      return false;
    }
    retired = std::move(it->second);
    impl_->event_callbacks.erase(it);
  }

  // Bump generation BEFORE freeing the ctx so any in-flight trampoline call
  // captured the old generation and will drop its work.
  impl_->generation.fetch_add(1, std::memory_order_release);

  if (impl_->connected) {
    UA_StatusCode status = UA_Client_MonitoredItems_deleteSingle(impl_->client.handle(), subscription_id, mi_id);
    (void)status;  // best effort; on failure the server may already have dropped it
  }
  // ``retired`` falls out of scope here, freeing the EventCallbackContext.
  return true;
}

tl::expected<std::vector<opcua::Variant>, OpcuaClient::MethodErrorInfo>
OpcuaClient::call_method(const opcua::NodeId & object_id, const opcua::NodeId & method_id,
                         const std::vector<opcua::Variant> & input_args) {
  std::lock_guard<std::mutex> lock(impl_->client_mutex);

  if (!impl_->connected) {
    return tl::make_unexpected(MethodErrorInfo{MethodError::NotConnected, "Not connected to OPC-UA server"});
  }

  // Helper: map an OPC-UA status code to a MethodError category.
  auto status_to_error = [](UA_StatusCode code, const std::string & msg) -> MethodErrorInfo {
    if (code == UA_STATUSCODE_BADMETHODINVALID || code == UA_STATUSCODE_BADNODEIDUNKNOWN ||
        code == UA_STATUSCODE_BADNOTSUPPORTED) {
      return {MethodError::MethodNotFound, msg};
    }
    if (code == UA_STATUSCODE_BADARGUMENTSMISSING || code == UA_STATUSCODE_BADINVALIDARGUMENT ||
        code == UA_STATUSCODE_BADTYPEMISMATCH || code == UA_STATUSCODE_BADTOOMANYARGUMENTS) {
      return {MethodError::InvalidArgument, msg};
    }
    if (code == UA_STATUSCODE_BADTIMEOUT) {
      return {MethodError::MethodTimeout, msg};
    }
    return {MethodError::TransportError, msg};
  };

  try {
    opcua::CallMethodResult result =
        opcua::services::call(impl_->client, object_id, method_id, opcua::Span<const opcua::Variant>(input_args));
    UA_StatusCode code = result.getStatusCode().get();
    std::cerr << "[opcua_client] call_method object=" << object_id.toString() << " method=" << method_id.toString()
              << " statusCode=" << UA_StatusCode_name(code);
    auto arg_results = result.getInputArgumentResults();
    for (size_t i = 0; i < arg_results.size(); ++i) {
      std::cerr << " arg" << i << "=" << UA_StatusCode_name(arg_results[i].get());
    }
    std::cerr << std::endl;
    if (code != UA_STATUSCODE_GOOD) {
      return tl::make_unexpected(status_to_error(code, UA_StatusCode_name(code)));
    }
    // Per OPC-UA Part 4 §5.11.2, even when the overall call statusCode is
    // Good the server may report per-argument validation failures via
    // inputArgumentResults. AlarmConditionType.Acknowledge surfaces
    // BadEventIdUnknown here when the EventId we tracked has been
    // superseded by a newer event from the server. Treat any non-Good
    // per-arg result as a transport error so the SOVD layer returns 502
    // instead of falsely reporting success.
    for (size_t i = 0; i < arg_results.size(); ++i) {
      UA_StatusCode arg_code = arg_results[i].get();
      if (arg_code != UA_STATUSCODE_GOOD) {
        std::string msg = std::string(UA_StatusCode_name(arg_code)) + " on input arg " + std::to_string(i);
        return tl::make_unexpected(status_to_error(arg_code, msg));
      }
    }
    auto outputs_span = result.getOutputArguments();
    std::vector<opcua::Variant> outputs;
    outputs.reserve(outputs_span.size());
    for (const auto & v : outputs_span) {
      outputs.push_back(v);  // Variant is copyable
    }
    return outputs;
  } catch (const opcua::BadStatus & e) {
    maybe_mark_disconnected(impl_->connected, impl_->generation, e);
    return tl::make_unexpected(status_to_error(e.code(), e.what()));
  }
}

}  // namespace ros2_medkit_gateway
