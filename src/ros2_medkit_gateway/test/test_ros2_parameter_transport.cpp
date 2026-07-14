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

// Regression coverage for #531: a discoverable-but-unresponsive node's
// parameter service must not be able to hang Ros2ParameterTransport forever.
// Before the fix, client->list_parameters(...)/get_parameters(...)/etc. were
// called with no timeout, so rclcpp defaulted to waiting forever once
// wait_for_service() had already succeeded (i.e. the service exists in the
// ROS graph but its callback never replies in time). That held spin_mutex_
// and an HTTP worker forever, eventually exhausting the whole gateway thread
// pool.

#include <gtest/gtest.h>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

#include <cstddef>

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"

using namespace ros2_medkit_gateway;

namespace {

constexpr const char * kUnresponsiveNodeName = "unresponsive_param_node";

}  // namespace

class TestRos2ParameterTransportUnresponsiveNode : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // The node under test: parameter services are NOT auto-started by
    // rclcpp::Node (start_parameter_services(false)); instead we hand-roll a
    // ListParameters service whose callback blocks well past the transport's
    // service timeout, simulating a node that IS discoverable (the service
    // exists in the ROS graph, so wait_for_service() succeeds immediately)
    // but never actually answers a request in time.
    //
    // The callback blocks on release_blocking_callback_ rather than a fixed
    // sleep: it never completes on its own during the assertions below
    // (proving the client-side call is bounded purely by
    // Ros2ParameterTransport's own timeout, not by how long the server takes
    // to reply), while still letting TearDown release it immediately so the
    // test suite doesn't pay a long fixed delay every run.
    rclcpp::NodeOptions unresponsive_options;
    unresponsive_options.start_parameter_services(false);
    unresponsive_node_ = std::make_shared<rclcpp::Node>(kUnresponsiveNodeName, unresponsive_options);

    list_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::ListParameters>(
        "~/list_parameters", [this](const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> /*response*/) {
          while (!release_blocking_callback_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        });

    // The remaining parameter services just need to exist in the ROS graph
    // so that wait_for_service() (which only probes service discovery, not
    // responsiveness) succeeds regardless of which underlying client it
    // checks. They are not expected to be invoked by the scenarios below
    // since list_parameters is the first remote call in the round trip and
    // it never returns within the bounded timeout.
    get_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::GetParameters>(
        "~/get_parameters", [](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> /*request*/,
                               std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> /*response*/) {});
    set_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::SetParameters>(
        "~/set_parameters", [](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> /*request*/,
                               std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> /*response*/) {});
    describe_parameters_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::DescribeParameters>(
        "~/describe_parameters",
        [](const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> /*response*/) {});
    get_parameter_types_service_ = unresponsive_node_->create_service<rcl_interfaces::srv::GetParameterTypes>(
        "~/get_parameter_types", [](const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> /*response*/) {});
    set_parameters_atomically_service_ =
        unresponsive_node_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
            "~/set_parameters_atomically",
            [](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> /*request*/,
               std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> /*response*/) {});

    // Client-side node handed to the transport (used only for logging / the
    // self-node FQN short-circuit). Ros2ParameterTransport creates and owns
    // its own internal node for the actual SyncParametersClient IPC, so this
    // node does not need to be spun for parameter round trips to proceed.
    client_node_ = std::make_shared<rclcpp::Node>("test_param_transport_client_node");

    // Small service_timeout_sec so the test proves boundedness quickly.
    // Generous negative_cache_ttl_sec so the negative-cache scenario below
    // is guaranteed to hit the cache rather than re-attempting IPC.
    transport_ = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

    // Spin the unresponsive node so its service callback can be dispatched
    // when a request arrives.
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(unresponsive_node_);
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });

    // Give the service time to register in the ROS graph before the test
    // starts issuing requests (avoids a flaky wait_for_service failure that
    // would mask the scenario under test).
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown() override {
    // Release any in-flight blocking service callback first so the spin loop
    // (spin_some, not a blocking spin()) can drain and the spin thread exit.
    release_blocking_callback_ = true;
    // Documented MultiThreadedExecutor teardown order: cancel -> join spin
    // thread -> reset executor -> reset nodes. cancel() before join is safe
    // here because the loop is a spin_some poll gated on spin_thread_running_,
    // so it exits regardless; cancel() also unblocks any in-flight spin_some.
    executor_->cancel();
    spin_thread_running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(unresponsive_node_);
    transport_.reset();
    executor_.reset();
    list_parameters_service_.reset();
    get_parameters_service_.reset();
    set_parameters_service_.reset();
    describe_parameters_service_.reset();
    get_parameter_types_service_.reset();
    set_parameters_atomically_service_.reset();
    unresponsive_node_.reset();
    client_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> unresponsive_node_;
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_service_;
  std::atomic<bool> release_blocking_callback_{false};

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<ros2::Ros2ParameterTransport> transport_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// GREEN (post-fix) behavior: a request against a discoverable-but-never-
// responding node's parameter service returns a bounded-time error instead
// of hanging forever.
//
// RED (pre-fix) would have been: this call blocks until the service replies,
// which in this fixture is only released by TearDown - i.e. it would not
// return during the test body at all. That cannot be safely exercised in a
// test that must terminate on its own (the point of #531 is precisely that
// nothing bounds it), so instead this asserts the call completes well within
// a small multiple of service_timeout_sec (0.5s) - which is only possible
// because get_service_timeout() is now threaded through every
// SyncParametersClient call in the round trip.
//
// list_parameters() also short-circuits after its internal
// cache_default_values() round-trip marks the node unavailable, so the whole
// call is bounded by ~1x service_timeout_sec (one round-trip), not ~2x (#531).
// The tighter 2s assertion documents that single-round-trip cap; the 5s bound
// is the documented outer safety limit.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, ListParametersReturnsBoundedErrorInsteadOfHanging) {
  auto start = std::chrono::steady_clock::now();
  auto result = transport_->list_parameters(kUnresponsiveNodeName);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.success);
  EXPECT_LT(elapsed, std::chrono::seconds(5))
      << "list_parameters() against an unresponsive-but-discoverable node must be bounded by "
         "service_timeout_sec, not block until the server replies (or forever, pre-fix)";
  EXPECT_LT(elapsed, std::chrono::seconds(2))
      << "list_parameters() must cap the spin_mutex_ hold at ~1x service_timeout_sec (single "
         "round-trip) by short-circuiting once cache_default_values marks the node unavailable";
}

// A second, immediate call against the same node must short-circuit via the
// negative cache (mark_node_unavailable() populated it on the first call's
// round-trip failure) rather than paying the timeout again.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, SecondCallShortCircuitsViaNegativeCache) {
  auto first_result = transport_->list_parameters(kUnresponsiveNodeName);
  ASSERT_FALSE(first_result.success);
  ASSERT_FALSE(transport_->is_node_available(kUnresponsiveNodeName))
      << "first round-trip failure must negative-cache the node (#531)";

  auto start = std::chrono::steady_clock::now();
  auto second_result = transport_->list_parameters(kUnresponsiveNodeName);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(second_result.success);
  EXPECT_LT(elapsed, std::chrono::milliseconds(100))
      << "second call must hit the negative cache and fail fast, not re-attempt the 0.5s IPC timeout";
}

// A round-trip timeout against an unresponsive node must map to a
// node-unavailable error code (TIMEOUT / SERVICE_UNAVAILABLE -> HTTP 503), NOT
// INTERNAL_ERROR (-> HTTP 500). Before the fix the inner round-trip catch
// rethrew into the operation's outer catch, which set INTERNAL_ERROR and thus
// wrongly surfaced a 500 for what is really "the node isn't answering" (#531).
TEST_F(TestRos2ParameterTransportUnresponsiveNode, GetParameterTimeoutMapsTo503NotInternalError) {
  auto start = std::chrono::steady_clock::now();
  auto result = transport_->get_parameter(kUnresponsiveNodeName, "some_param");
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.success);
  EXPECT_LT(elapsed, std::chrono::seconds(5)) << "get_parameter() must be bounded by service_timeout_sec";
  EXPECT_NE(result.error_code, ParameterErrorCode::INTERNAL_ERROR)
      << "a node round-trip timeout must not be reported as INTERNAL_ERROR (would map to HTTP 500)";
  EXPECT_TRUE(result.error_code == ParameterErrorCode::TIMEOUT ||
              result.error_code == ParameterErrorCode::SERVICE_UNAVAILABLE)
      << "an unresponsive node must map to a 503 (node-unavailable) error code, got "
      << static_cast<int>(result.error_code);
}

// The list_parameters single-round-trip cap must hold even with the negative
// cache DISABLED (negative_cache_ttl_sec == 0, a documented in-range value).
// is_node_unavailable() is always false when the TTL is 0, so gating the cap on
// it would silently let list_parameters do a SECOND round-trip. The cap is
// instead driven by cache_default_values' own return value, which is
// TTL-independent (#531).
//
// Timing-INDEPENDENT assertion (does not flake under TSan/ASan, which triple
// ctest timeouts): the error CLASS is the proxy for which path ran.
// cache_default_values short-circuits with SERVICE_UNAVAILABLE, so seeing
// SERVICE_UNAVAILABLE proves the short-circuit fired (single round-trip). If the
// cap were defeated (second round-trip), list_parameters' own catch would set
// TIMEOUT instead. A generous outer time bound is kept only as a sanity net.
TEST_F(TestRos2ParameterTransportUnresponsiveNode, ListParametersCapHoldsWithNegativeCacheDisabled) {
  constexpr double kServiceTimeoutSec = 1.0;
  constexpr double kNegativeCacheDisabled = 0.0;
  auto ttl0_transport =
      std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), kServiceTimeoutSec, kNegativeCacheDisabled);

  auto start = std::chrono::steady_clock::now();
  auto result = ttl0_transport->list_parameters(kUnresponsiveNodeName);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE)
      << "with the negative cache disabled the cache_default_values short-circuit must still fire "
         "(SERVICE_UNAVAILABLE); TIMEOUT would mean the cap was defeated and a second round-trip ran";
  EXPECT_LT(elapsed, std::chrono::seconds(5)) << "sanity: still bounded";
}

// ---------------------------------------------------------------------------
// list_parameters RESPONDS, get_parameters / set_parameters BLOCK.
//
// This is the coverage the fully-unresponsive fixture above CANNOT provide:
// SyncParametersClient::get_parameters and set_parameters do NOT throw on
// timeout - they return an EMPTY vector (only list_parameters throws). So the
// mark-on-timeout catch blocks never fire for get/set; the timeout has to be
// detected via "empty response when names were requested". A node that answers
// list_parameters (so the transport gets past the first call) but stalls on the
// value read/write is exactly what exercises those empty-result paths (#531).
// ---------------------------------------------------------------------------
class TestRos2ParameterTransportListRespondsGetSetStuck : public ::testing::Test {
 protected:
  static constexpr const char * kNodeName = "list_responds_getset_stuck_node";
  static constexpr const char * kParamName = "sample_param";

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node_ = std::make_shared<rclcpp::Node>(kNodeName, options);

    // list_parameters ALWAYS responds with one name, so the transport always
    // gets past the first round-trip call and reaches get/set.
    list_parameters_service_ = node_->create_service<rcl_interfaces::srv::ListParameters>(
        "~/list_parameters", [](const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> /*request*/,
                                std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response) {
          response->result.names.push_back(kParamName);
        });

    // get_parameters: blocks while block_get_ is set (client times out -> empty),
    // otherwise returns one INTEGER value per requested name (responsive).
    get_parameters_service_ = node_->create_service<rcl_interfaces::srv::GetParameters>(
        "~/get_parameters", [this](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
                                   std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
          while (block_get_.load() && !release_all_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          if (release_all_.load()) {
            return;
          }
          for (size_t i = 0; i < request->names.size(); ++i) {
            rcl_interfaces::msg::ParameterValue value;
            value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            value.integer_value = 42;
            response->values.push_back(value);
          }
        });

    // set_parameters: blocks while block_set_ is set (client times out -> empty),
    // otherwise returns one successful result per param (responsive).
    set_parameters_service_ = node_->create_service<rcl_interfaces::srv::SetParameters>(
        "~/set_parameters", [this](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
                                   std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
          while (block_set_.load() && !release_all_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
          if (release_all_.load()) {
            return;
          }
          for (size_t i = 0; i < request->parameters.size(); ++i) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            response->results.push_back(result);
          }
        });

    // describe_parameters / get_parameter_types / set_parameters_atomically only
    // need to exist so wait_for_service() succeeds; they respond instantly empty.
    describe_parameters_service_ = node_->create_service<rcl_interfaces::srv::DescribeParameters>(
        "~/describe_parameters",
        [](const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> /*response*/) {});
    get_parameter_types_service_ = node_->create_service<rcl_interfaces::srv::GetParameterTypes>(
        "~/get_parameter_types", [](const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> /*response*/) {});
    set_parameters_atomically_service_ = node_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
        "~/set_parameters_atomically",
        [](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> /*response*/) {});

    client_node_ = std::make_shared<rclcpp::Node>("test_listresponds_client_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown() override {
    release_all_ = true;  // unstick any blocked callback first
    executor_->cancel();
    spin_thread_running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(node_);
    executor_.reset();
    list_parameters_service_.reset();
    get_parameters_service_.reset();
    set_parameters_service_.reset();
    describe_parameters_service_.reset();
    get_parameter_types_service_.reset();
    set_parameters_atomically_service_.reset();
    node_.reset();
    client_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_service_;

  std::atomic<bool> block_get_{false};
  std::atomic<bool> block_set_{false};
  std::atomic<bool> release_all_{false};

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// (a) get_parameter: list responds (param exists) but get_parameters returns
// empty (timeout). Must map to a 503-class code (TIMEOUT), NOT INTERNAL_ERROR
// (500), and negative-cache the node. Before this fix the empty-get branch was
// INTERNAL_ERROR because get_parameters never throws on timeout (#531).
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, GetParameterEmptyGetAfterFoundNameIsTimeout) {
  block_get_ = true;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto result = transport->get_parameter(kNodeName, kParamName);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::TIMEOUT)
      << "empty get_parameters after a found name is a value-read timeout, got " << static_cast<int>(result.error_code);
  EXPECT_NE(result.error_code, ParameterErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(transport->is_node_available(kNodeName)) << "the get timeout must negative-cache the node";
}

// (b) set_parameter: pre-read responds (fast type hint), but set_parameters
// returns empty (timeout). Must map to TIMEOUT (503) + negative-cache, NOT
// INTERNAL_ERROR (500). This is the #531-fully-live-on-writes path: pre-fix a
// fully-unresponsive node's set returned 500 with no negative cache.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, SetParameterEmptyResultsIsTimeoutNotInternalError) {
  block_get_ = false;  // pre-read (type hint) responds quickly
  block_set_ = true;   // the actual write stalls
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto result = transport->set_parameter(kNodeName, kParamName, nlohmann::json(7));

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error_code, ParameterErrorCode::TIMEOUT)
      << "empty set_parameters results is a write timeout, got " << static_cast<int>(result.error_code);
  EXPECT_NE(result.error_code, ParameterErrorCode::INTERNAL_ERROR);
  EXPECT_FALSE(transport->is_node_available(kNodeName)) << "the set timeout must negative-cache the node";
}

// (c) list_parameters op: defaults already cached (so cache_default_values does
// not short-circuit), then get_parameters stalls on the MAIN round-trip. The
// all-empty-after-names result must be reported as TIMEOUT, not a misleading
// success=[] (#531). Asserting TIMEOUT (not SERVICE_UNAVAILABLE) pins the main
// round-trip path specifically, distinct from the cache_default_values gate.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, ListParametersEmptyGetAfterNamesIsTimeoutNotEmptySuccess) {
  block_get_ = false;  // fully responsive first
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), 0.5, 60.0);

  auto first = transport->list_parameters(kNodeName);
  ASSERT_TRUE(first.success) << "responsive node must list successfully and cache defaults first";

  block_get_ = true;  // now stall value reads
  auto second = transport->list_parameters(kNodeName);

  EXPECT_FALSE(second.success);
  EXPECT_EQ(second.error_code, ParameterErrorCode::TIMEOUT)
      << "list returned names but all gets came back empty: must be TIMEOUT (main round-trip), got "
      << static_cast<int>(second.error_code);
}

// Recovery: a node negative-cached on a get timeout must recover once it becomes
// responsive again and the TTL expires - it must NOT be 503'd forever. Also
// pins Fix 4 (no empty-defaults poison): cache_default_values must NOT have
// committed an empty defaults map on the timeout, or get_default would return
// NOT_FOUND forever; after recovery get_default returns the real value.
TEST_F(TestRos2ParameterTransportListRespondsGetSetStuck, MarkedNodeRecoversAfterTtlExpires) {
  constexpr double kServiceTimeoutSec = 0.5;
  constexpr double kShortTtlSec = 0.4;
  auto transport = std::make_shared<ros2::Ros2ParameterTransport>(client_node_.get(), kServiceTimeoutSec, kShortTtlSec);

  block_get_ = true;
  auto stalled = transport->list_parameters(kNodeName);
  EXPECT_FALSE(stalled.success);
  EXPECT_FALSE(transport->is_node_available(kNodeName)) << "get timeout must negative-cache the node";

  // Node becomes responsive again; wait past the negative-cache TTL.
  block_get_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(700));

  auto recovered = transport->list_parameters(kNodeName);
  EXPECT_TRUE(recovered.success) << "a marked node must recover after the TTL expires, not stay 503 forever";

  // Fix 4: defaults were NOT poisoned with an empty map on the earlier timeout,
  // so after recovery the real default is retrievable (not NOT_FOUND forever).
  auto def = transport->get_default(kNodeName, kParamName);
  EXPECT_TRUE(def.success) << "get_default must return the real default after recovery, not a poisoned empty cache";
}
