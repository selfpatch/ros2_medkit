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

/**
 * @file ghost_node_injector.cpp
 * @brief Puts nodes into the ROS graph that no running participant stands behind
 *
 * Test fixture. The DDS RMWs create a graph node entry for any participant GID a
 * `ros_discovery_info` message names.
 *
 *   --ghost <fqn>     a message for an unowned GID (starts 00 00) listing the node with
 *                     no endpoints: the shape of a node behind a DDS router.
 *   --backed <fqn>    the same for a GID starting 00 01 that names a publisher this
 *                     process owns: a node whose participant announces no enclave.
 *   --leftover <fqn>  leaves a leftover behind a real node:
 *       1. creates the node in its own context and captures its participant's last
 *          discovery message, then prints `ghost_node_injector: leftover_ready
 *          participant_gid=<hex>`;
 *       2. with --announce <n>, on an `announce` stdin line, republishes it with <n>
 *          more nodes `<name>_<NNNNN>` and prints `ghost_node_injector:
 *          leftover_announced count=<n> acked=<true|false>`. Send it only after the
 *          target graph lists the node, or the original message can replace it;
 *       3. on a `leave` line, destroys the node and its context and waits until this
 *          process's graph drops it;
 *       4. after --delay <seconds> (default 0), or on a `publish` line after `leave`,
 *          republishes the message and prints `ghost_node_injector:
 *          leftover_published matched_subscriptions=<n> acked=<true|false>`.
 *
 * The participant GID is the node's publisher GUID prefix plus entity id 00 00 01 c1.
 * Step 1 waits for a discovery message with that GID, so a wrong derivation fails there.
 * --ghost and --backed print `ghost_node_injector: matched_subscriptions=<n> entries=<k>
 * acked=<true|false>`, and send one message per node in command-line order. Messages go
 * through the rmw layer on a reliable, transient-local, keep-all writer, so late graphs
 * receive them in that order and they outlive this process. rcl would rename the topic to
 * `/ros_discovery_info`, which the RMWs do not read.
 *
 * Runs until SIGINT or SIGTERM.
 */

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <rcl/node.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw_dds_common/msg/participant_entities_info.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <std_msgs/msg/empty.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

namespace {

using ParticipantEntitiesInfo = rmw_dds_common::msg::ParticipantEntitiesInfo;
using NodeEntitiesInfo = rmw_dds_common::msg::NodeEntitiesInfo;
using Gid = rmw_dds_common::msg::Gid;

constexpr const char * kUsage =
    "usage: ghost_node_injector [--ghost <fqn>]... [--backed <fqn>]... "
    "[--leftover <fqn> [--announce <n>] [--delay <seconds>]]";

struct InjectedNode {
  std::string name;
  std::string ns;
  bool backed;
};

struct Arguments {
  std::vector<InjectedNode> injected;
  std::optional<InjectedNode> leftover;
  size_t announce{0};
  double delay_sec{0.0};
};

InjectedNode parse_fqn(const std::string & fqn, bool backed) {
  const auto slash = fqn.rfind('/');
  if (fqn.empty() || fqn.front() != '/' || slash == std::string::npos || slash + 1 == fqn.size()) {
    throw std::invalid_argument("not a fully qualified node name: '" + fqn + "'");
  }
  std::string ns = slash == 0 ? "/" : fqn.substr(0, slash);
  return InjectedNode{fqn.substr(slash + 1), std::move(ns), backed};
}

Arguments parse_arguments(const std::vector<std::string> & args) {
  Arguments parsed;
  bool announce_given = false;
  bool delay_given = false;
  // args[0] is the program name.
  for (size_t i = 1; i < args.size(); ++i) {
    const auto & flag = args[i];
    if (i + 1 >= args.size()) {
      throw std::invalid_argument(kUsage);
    }
    const auto & value = args[++i];
    if (flag == "--ghost" || flag == "--backed") {
      parsed.injected.push_back(parse_fqn(value, flag == "--backed"));
    } else if (flag == "--leftover" && !parsed.leftover) {
      parsed.leftover = parse_fqn(value, false);
    } else if (flag == "--announce") {
      parsed.announce = std::stoul(value);
      announce_given = true;
    } else if (flag == "--delay") {
      parsed.delay_sec = std::stod(value);
      // In-range test negated so NaN fails it. Do not apply clang-tidy's De Morgan rewrite.
      if (!(parsed.delay_sec >= 0.0 && parsed.delay_sec <= 3600.0)) {  // NOLINT(readability-simplify-boolean-expr)
        throw std::invalid_argument("--delay must be between 0 and 3600 seconds");
      }
      delay_given = true;
    } else {
      throw std::invalid_argument(kUsage);
    }
  }
  if ((parsed.injected.empty() && !parsed.leftover) || ((announce_given || delay_given) && !parsed.leftover)) {
    throw std::invalid_argument(kUsage);
  }
  return parsed;
}

std::string to_hex(const Gid & gid) {
  static constexpr std::array<char, 16> kDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                                '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  std::string hex;
  for (const auto byte : gid.data) {
    hex.push_back(kDigits[static_cast<size_t>(byte >> 4U)]);
    hex.push_back(kDigits[static_cast<size_t>(byte & 0x0FU)]);
  }
  return hex;
}

class GhostNodeInjector : public rclcpp::Node {
 public:
  explicit GhostNodeInjector(Arguments args) : Node("_ghost_node_injector"), args_(std::move(args)) {
    backing_publisher_ = create_publisher<std_msgs::msg::Empty>("~/backing", rclcpp::QoS(1));

    rmw_node_ = rcl_node_get_rmw_handle(get_node_base_interface()->get_rcl_node_handle());
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos.avoid_ros_namespace_conventions = true;
    const auto * type_support =
        rosidl_typesupport_cpp::get_message_type_support_handle<rmw_dds_common::msg::ParticipantEntitiesInfo>();
    rmw_publisher_options_t publisher_options = rmw_get_default_publisher_options();
    discovery_publisher_ =
        rmw_create_publisher(rmw_node_, type_support, "ros_discovery_info", &qos, &publisher_options);
    if (discovery_publisher_ == nullptr) {
      throw std::runtime_error(std::string("rmw_create_publisher(ros_discovery_info) failed: ") +
                               rmw_get_error_string().str);
    }

    if (args_.leftover) {
      rmw_subscription_options_t subscription_options = rmw_get_default_subscription_options();
      discovery_subscription_ =
          rmw_create_subscription(rmw_node_, type_support, "ros_discovery_info", &qos, &subscription_options);
      if (discovery_subscription_ == nullptr) {
        throw std::runtime_error(std::string("rmw_create_subscription(ros_discovery_info) failed: ") +
                                 rmw_get_error_string().str);
      }
      start_leftover_node();
      if (fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK) != 0) {
        throw std::runtime_error("could not make stdin non-blocking");
      }
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      tick_injected();
      tick_leftover();
    });
  }

  ~GhostNodeInjector() override {
    timer_.reset();
    stop_leftover_node();
    if (discovery_subscription_ != nullptr) {
      if (rmw_destroy_subscription(rmw_node_, discovery_subscription_) != RMW_RET_OK) {
        std::fprintf(stderr, "rmw_destroy_subscription(ros_discovery_info) failed: %s\n", rmw_get_error_string().str);
        rmw_reset_error();
      }
      discovery_subscription_ = nullptr;
    }
    if (discovery_publisher_ != nullptr) {
      if (rmw_destroy_publisher(rmw_node_, discovery_publisher_) != RMW_RET_OK) {
        std::fprintf(stderr, "rmw_destroy_publisher(ros_discovery_info) failed: %s\n", rmw_get_error_string().str);
        rmw_reset_error();
      }
      discovery_publisher_ = nullptr;
    }
    backing_publisher_.reset();
  }

  GhostNodeInjector(const GhostNodeInjector &) = delete;
  GhostNodeInjector & operator=(const GhostNodeInjector &) = delete;
  GhostNodeInjector(GhostNodeInjector &&) = delete;
  GhostNodeInjector & operator=(GhostNodeInjector &&) = delete;

 private:
  enum class LeftoverStep { kCapturing, kReady, kRemoving, kDelaying, kPublished, kFailed };

  // ---- --ghost / --backed --------------------------------------------------------------

  /// Publish once something is listening, then report once every matched reader has acknowledged.
  void tick_injected() {
    if (args_.injected.empty() || injected_reported_) {
      return;
    }
    if (!injected_published_) {
      size_t matched = 0;
      if (rmw_publisher_count_matched_subscriptions(discovery_publisher_, &matched) != RMW_RET_OK || matched == 0 ||
          !resolve_backing_gid()) {
        if (std::chrono::steady_clock::now() - started_ < kMatchTimeout) {
          return;
        }
        report_injected(matched, false);
        return;
      }
      publish_injected();
      injected_matched_ = matched;
      injected_published_ = true;
      injected_published_at_ = std::chrono::steady_clock::now();
    }
    const bool acked = rmw_publisher_wait_for_all_acked(discovery_publisher_, rmw_time_t{0, 0}) == RMW_RET_OK;
    if (acked || std::chrono::steady_clock::now() - injected_published_at_ >= kAckTimeout) {
      report_injected(injected_matched_, acked);
    }
  }

  /// The backing publisher's graph GID. Not PublisherBase::get_gid(): Cyclone returns a local id there.
  bool resolve_backing_gid() {
    const auto gid = endpoint_gid(*this, backing_publisher_->get_topic_name());
    if (!gid) {
      return false;
    }
    backing_gid_ = *gid;
    return true;
  }

  void publish_injected() {
    std::random_device seed;
    std::mt19937 generator(seed());
    std::uniform_int_distribution<int> byte(0, 255);

    for (const auto & node : args_.injected) {
      ParticipantEntitiesInfo message;
      for (auto & b : message.gid.data) {
        b = static_cast<std::remove_reference_t<decltype(b)>>(byte(generator));
      }
      message.gid.data[0] = 0x00;
      message.gid.data[1] = node.backed ? 0x01 : 0x00;
      NodeEntitiesInfo info;
      info.node_namespace = node.ns;
      info.node_name = node.name;
      if (node.backed) {
        info.writer_gid_seq.push_back(backing_gid_);
      }
      message.node_entities_info_seq.push_back(std::move(info));
      publish(message);
    }
  }

  void report_injected(size_t matched, bool acked) {
    std::printf("ghost_node_injector: matched_subscriptions=%zu entries=%zu acked=%s\n", matched, args_.injected.size(),
                acked ? "true" : "false");
    std::fflush(stdout);
    injected_reported_ = true;
  }

  // ---- --leftover ----------------------------------------------------------------------

  void start_leftover_node() {
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging(false);
    leftover_context_ = std::make_shared<rclcpp::Context>();
    leftover_context_->init(0, nullptr, init_options);
    leftover_node_ = std::make_shared<rclcpp::Node>(args_.leftover->name, args_.leftover->ns,
                                                    rclcpp::NodeOptions().context(leftover_context_));
    leftover_marker_ = leftover_node_->create_publisher<std_msgs::msg::Empty>("~/leftover_marker", rclcpp::QoS(1));
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = leftover_context_;
    leftover_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    leftover_executor_->add_node(leftover_node_);
    leftover_spin_thread_ = std::thread([executor = leftover_executor_]() {
      executor->spin();
    });
  }

  /// Destroy the node, which removes its participant, and shut its context down.
  void stop_leftover_node() {
    if (!leftover_context_) {
      return;
    }
    leftover_executor_->cancel();
    if (leftover_spin_thread_.joinable()) {
      leftover_spin_thread_.join();
    }
    leftover_executor_->remove_node(leftover_node_);
    leftover_executor_.reset();
    leftover_marker_.reset();
    leftover_node_.reset();
    leftover_context_->shutdown("leftover node leaves");
    leftover_context_.reset();
  }

  void tick_leftover() {
    if (!args_.leftover) {
      return;
    }
    const auto & target = *args_.leftover;
    if (leftover_step_ != LeftoverStep::kCapturing) {
      // Keep the keep-all reader empty; only the capture step reads what it takes.
      drain_discovery_messages();
    }
    switch (leftover_step_) {
      case LeftoverStep::kCapturing:
        capture_leftover_message();
        if (captured_) {
          std::printf("ghost_node_injector: leftover_ready participant_gid=%s\n", to_hex(captured_->gid).c_str());
          std::fflush(stdout);
          leftover_step_ = LeftoverStep::kReady;
        } else if (std::chrono::steady_clock::now() - started_ >= kMatchTimeout) {
          fail("no discovery message for participant GID " + (derived_gid_ ? to_hex(*derived_gid_) : "(none)") +
               " lists " + target.ns + "/" + target.name + " with its marker publisher");
        }
        break;
      case LeftoverStep::kReady:
        read_stdin();
        if (!announced_ && args_.announce > 0 && stdin_buffer_.find("announce\n") != std::string::npos) {
          announce();
          // No graph in this process sees its own participant's writes; reader acks stand in.
          std::printf("ghost_node_injector: leftover_announced count=%zu acked=%s\n", args_.announce,
                      wait_for_acks() ? "true" : "false");
          std::fflush(stdout);
          announced_ = true;
        }
        if (stdin_buffer_.find("leave\n") != std::string::npos) {
          stop_leftover_node();
          left_at_ = std::chrono::steady_clock::now();
          leftover_step_ = LeftoverStep::kRemoving;
        }
        break;
      case LeftoverStep::kRemoving:
        if (!own_graph_lists(target)) {
          removed_at_ = std::chrono::steady_clock::now();
          std::printf("ghost_node_injector: leftover_removed after_ms=%lld\n",
                      static_cast<long long>(
                          std::chrono::duration_cast<std::chrono::milliseconds>(removed_at_ - left_at_).count()));
          std::fflush(stdout);
          leftover_step_ = LeftoverStep::kDelaying;
        } else if (std::chrono::steady_clock::now() - left_at_ >= kMatchTimeout) {
          fail("this process's graph still lists the node after its context was shut down");
        }
        break;
      case LeftoverStep::kDelaying:
        read_stdin();
        if (std::chrono::steady_clock::now() - removed_at_ >= std::chrono::duration<double>(args_.delay_sec) ||
            stdin_buffer_.find("publish\n", stdin_buffer_.find("leave\n")) != std::string::npos) {
          size_t matched = 0;
          if (rmw_publisher_count_matched_subscriptions(discovery_publisher_, &matched) != RMW_RET_OK) {
            matched = 0;
          }
          publish(stale_message_);
          const bool acked = wait_for_acks();
          std::printf("ghost_node_injector: leftover_published matched_subscriptions=%zu acked=%s\n", matched,
                      acked ? "true" : "false");
          std::fflush(stdout);
          leftover_step_ = LeftoverStep::kPublished;
        }
        break;
      case LeftoverStep::kPublished:
      case LeftoverStep::kFailed:
        break;
    }
  }

  /// Take the discovery messages received so far; keep the leftover participant's one that lists
  /// the marker publisher, the node's last endpoint.
  void capture_leftover_message() {
    if (!derived_gid_) {
      marker_gid_ = endpoint_gid(*leftover_node_, leftover_marker_->get_topic_name());
      if (!marker_gid_) {
        return;
      }
      Gid participant_gid;
      std::fill(participant_gid.data.begin(), participant_gid.data.end(), 0);
      std::copy_n(marker_gid_->data.begin(), kGuidPrefixSize, participant_gid.data.begin());
      std::copy(kParticipantEntityId.begin(), kParticipantEntityId.end(),
                participant_gid.data.begin() + kGuidPrefixSize);
      derived_gid_ = participant_gid;
    }
    const auto & target = *args_.leftover;
    while (true) {
      ParticipantEntitiesInfo message;
      bool taken = false;
      if (rmw_take(discovery_subscription_, &message, &taken, nullptr) != RMW_RET_OK) {
        rmw_reset_error();
        return;
      }
      if (!taken) {
        return;
      }
      if (message.gid != *derived_gid_) {
        continue;
      }
      const auto & nodes = message.node_entities_info_seq;
      const bool lists_target = std::any_of(nodes.begin(), nodes.end(), [this, &target](const NodeEntitiesInfo & info) {
        const auto & writers = info.writer_gid_seq;
        return info.node_name == target.name && info.node_namespace == target.ns &&
               std::find(writers.begin(), writers.end(), *marker_gid_) != writers.end();
      });
      if (lists_target && !captured_) {
        captured_ = message;
        stale_message_ = message;
      }
    }
  }

  void drain_discovery_messages() {
    ParticipantEntitiesInfo message;
    bool taken = true;
    while (taken) {
      if (rmw_take(discovery_subscription_, &message, &taken, nullptr) != RMW_RET_OK) {
        rmw_reset_error();
        return;
      }
    }
  }

  void announce() {
    stale_message_ = *captured_;
    for (size_t i = 0; i < args_.announce; ++i) {
      NodeEntitiesInfo info;
      info.node_namespace = args_.leftover->ns;
      info.node_name = announced_name(i);
      stale_message_.node_entities_info_seq.push_back(std::move(info));
    }
    publish(stale_message_);
  }

  std::string announced_name(size_t index) const {
    auto digits = std::to_string(index);
    if (digits.size() < kAnnouncedIndexWidth) {
      digits.insert(0, kAnnouncedIndexWidth - digits.size(), '0');
    }
    return args_.leftover->name + "_" + digits;
  }

  /// Whether this process's graph lists the node with an enclave.
  bool own_graph_lists(const InjectedNode & node) {
    for (const auto & [name, ns, enclave] : get_node_graph_interface()->get_node_names_with_enclaves()) {
      if (name == node.name && ns == node.ns && !enclave.empty()) {
        return true;
      }
    }
    return false;
  }

  void read_stdin() {
    std::array<char, 256> buffer{};
    while (true) {
      const auto count = ::read(STDIN_FILENO, buffer.data(), buffer.size());
      if (count <= 0) {
        return;
      }
      stdin_buffer_.append(buffer.data(), static_cast<size_t>(count));
    }
  }

  /// Whether every matched reader acknowledged what was published, within kAckTimeout.
  bool wait_for_acks() {
    return rmw_publisher_wait_for_all_acked(discovery_publisher_,
                                            rmw_time_t{static_cast<uint64_t>(kAckTimeout.count()), 0}) == RMW_RET_OK;
  }

  void fail(const std::string & reason) {
    std::printf("ghost_node_injector: leftover_failed %s\n", reason.c_str());
    std::fflush(stdout);
    leftover_step_ = LeftoverStep::kFailed;
  }

  // ---- shared --------------------------------------------------------------------------

  /// The graph GID of the publisher `node` owns on `topic`, as `node`'s own graph keys it.
  static std::optional<Gid> endpoint_gid(rclcpp::Node & node, const std::string & topic) {
    for (const auto & info : node.get_publishers_info_by_topic(topic)) {
      if (info.node_name() == node.get_name() && info.node_namespace() == node.get_namespace()) {
        Gid gid;
        std::fill(gid.data.begin(), gid.data.end(), 0);
        const auto & endpoint = info.endpoint_gid();
        std::copy_n(endpoint.begin(), std::min(endpoint.size(), gid.data.size()), gid.data.begin());
        return gid;
      }
    }
    return std::nullopt;
  }

  void publish(const ParticipantEntitiesInfo & message) {
    if (rmw_publish(discovery_publisher_, &message, nullptr) != RMW_RET_OK) {
      RCLCPP_ERROR(get_logger(), "rmw_publish failed: %s", rmw_get_error_string().str);
      rmw_reset_error();
    }
  }

  static constexpr std::chrono::seconds kMatchTimeout{30};
  static constexpr std::chrono::seconds kAckTimeout{10};
  static constexpr size_t kGuidPrefixSize = 12;
  static constexpr size_t kAnnouncedIndexWidth = 5;
  static constexpr std::array<uint8_t, 4> kParticipantEntityId{0x00, 0x00, 0x01, 0xc1};

  Arguments args_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr backing_publisher_;
  Gid backing_gid_;
  rmw_node_t * rmw_node_{nullptr};
  rmw_publisher_t * discovery_publisher_{nullptr};
  rmw_subscription_t * discovery_subscription_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_;
  const std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};

  std::chrono::steady_clock::time_point injected_published_at_;
  size_t injected_matched_{0};
  bool injected_published_{false};
  bool injected_reported_{false};

  rclcpp::Context::SharedPtr leftover_context_;
  rclcpp::Node::SharedPtr leftover_node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr leftover_marker_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> leftover_executor_;
  std::thread leftover_spin_thread_;
  LeftoverStep leftover_step_{LeftoverStep::kCapturing};
  std::optional<Gid> marker_gid_;
  std::optional<Gid> derived_gid_;
  std::optional<ParticipantEntitiesInfo> captured_;
  bool announced_{false};
  ParticipantEntitiesInfo stale_message_;
  std::string stdin_buffer_;
  std::chrono::steady_clock::time_point left_at_;
  std::chrono::steady_clock::time_point removed_at_;
};

}  // namespace

int main(int argc, char ** argv) {
  Arguments args;
  try {
    args = parse_arguments(rclcpp::remove_ros_arguments(argc, argv));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 2;
  }
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, [&args]() -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<GhostNodeInjector>(std::move(args));
  });
}
