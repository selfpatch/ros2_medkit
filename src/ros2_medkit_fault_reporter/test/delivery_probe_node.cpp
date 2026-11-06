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

// Test node for the delivery tests: reports faults through FaultReporter in the pattern named by
// its `mode` parameter and logs what it did, so a launch test can compare it with the FaultManager.

#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

using ros2_medkit_fault_reporter::FaultReporter;
using Clock = std::chrono::steady_clock;

namespace {

void poll_until(const rclcpp::Node::SharedPtr & node, std::chrono::seconds limit, const std::function<bool()> & done) {
  const auto deadline = Clock::now() + limit;
  while (rclcpp::ok() && !done() && Clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// `ctor` "raw" builds the reporters from node interfaces without a timers interface.
std::vector<std::unique_ptr<FaultReporter>>
fresh_reporters(const rclcpp::Node::SharedPtr & node, size_t count, const std::string & ctor = "node",
                const std::string & service = "/fault_manager/report_fault") {
  std::vector<std::unique_ptr<FaultReporter>> reporters;
  for (size_t i = 0; i < count; ++i) {
    if (ctor == "raw") {
      reporters.push_back(std::make_unique<FaultReporter>(
          node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_services_interface(),
          node->get_node_parameters_interface(), node->get_logger(), "/delivery_probe", service));
    } else {
      reporters.push_back(std::make_unique<FaultReporter>(node, "/delivery_probe", service));
    }
  }
  return reporters;
}

bool all_ready(const std::vector<std::unique_ptr<FaultReporter>> & reporters) {
  for (const auto & r : reporters) {
    if (!r->is_service_ready()) {
      return false;
    }
  }
  return true;
}

size_t total_pending(const std::vector<std::unique_ptr<FaultReporter>> & reporters) {
  size_t n = 0;
  for (const auto & r : reporters) {
    n += r->pending_reports();
  }
  return n;
}

}  // namespace

int run(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("delivery_probe");
  const auto mode = node->declare_parameter<std::string>("mode", "burst");
  const auto count = static_cast<size_t>(node->declare_parameter<int64_t>("count", 24));
  const int rounds = static_cast<int>(node->declare_parameter<int64_t>("rounds", 1));
  const auto prefix = node->declare_parameter<std::string>("prefix", "P");
  constexpr auto kLimit = std::chrono::seconds(60);
  const auto severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;

  if (mode == "config") {
    FaultReporter reporter(node, "/delivery_probe");
    const auto & c = reporter.delivery_config();
    std::printf("CONFIG ack=%g attempts=%d pending=%d\n", c.ack_timeout_sec, c.max_attempts, c.max_pending);
  } else if (mode == "burst") {
    // Fresh reporters, each reporting the moment its client sees the service.
    for (int r = 0; r < rounds; ++r) {
      auto reporters = fresh_reporters(node, count);
      std::vector<bool> sent(count, false);
      poll_until(node, kLimit, [&]() {
        bool all = true;
        for (size_t i = 0; i < count; ++i) {
          if (!sent[i] && reporters[i]->is_service_ready()) {
            reporters[i]->report(prefix + "_" + std::to_string(r) + "_" + std::to_string(i), severity, "burst");
            sent[i] = true;
          }
          all = all && sent[i];
        }
        return all;
      });
      poll_until(node, kLimit, [&]() {
        return total_pending(reporters) == 0;
      });
      std::printf("ROUND %d pending=%zu\n", r, total_pending(reporters));
    }
    std::printf("DONE\n");
  } else if (mode == "order") {
    // FAILED then PASSED at once, on a fresh reporter per code.
    auto reporters = fresh_reporters(node, count);
    poll_until(node, kLimit, [&]() {
      return all_ready(reporters);
    });
    for (size_t i = 0; i < count; ++i) {
      const auto code = prefix + "_" + std::to_string(i);
      reporters[i]->report(code, severity, "order");
      reporters[i]->report_passed(code);
    }
    poll_until(node, kLimit, [&]() {
      return total_pending(reporters) == 0;
    });
    std::printf("DONE pending=%zu\n", total_pending(reporters));
  } else if (mode == "stalled") {
    // The manager is stopped by the test: report and then log the pending count as it changes.
    // Code i goes to reporter i % reporters. With `destroy` the reporters are destroyed after reporting.
    const auto reporter_count =
        static_cast<size_t>(node->declare_parameter<int64_t>("reporters", static_cast<int64_t>(count)));
    const auto ctor = node->declare_parameter<std::string>("ctor", "node");
    const auto service = node->declare_parameter<std::string>("service", "/fault_manager/report_fault");
    const bool destroy = node->declare_parameter<bool>("destroy", false);
    auto reporters = fresh_reporters(node, reporter_count, ctor, service);
    const auto go_file = node->declare_parameter<std::string>("go_file", "");
    poll_until(node, kLimit, [&]() {
      return all_ready(reporters);
    });
    // The test stops the manager after READY and then creates go_file.
    std::printf("READY\n");
    std::fflush(stdout);
    poll_until(node, kLimit, [&]() {
      return std::ifstream(go_file).good();
    });
    for (size_t i = 0; i < count; ++i) {
      reporters[i % reporter_count]->report(prefix + "_" + std::to_string(i), severity, "stalled");
    }
    std::printf("REPORTED pending=%zu\n", total_pending(reporters));
    std::fflush(stdout);
    size_t last = total_pending(reporters);
    if (destroy) {
      reporters.clear();
      last = 0;
      std::printf("DESTROYED\n");
      std::fflush(stdout);
    }
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      const size_t now = total_pending(reporters);
      if (now != last) {
        last = now;
        std::printf("PENDING %zu\n", now);
        std::fflush(stdout);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  std::fflush(stdout);
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char * argv[]) {
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "delivery_probe_node: %s\n", e.what());
    return 1;
  }
}
