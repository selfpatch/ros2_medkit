// Copyright 2025 mfaferek93
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

#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

#include "rclcpp/node_interfaces/node_timers.hpp"
#include "rclcpp/version.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_medkit_fault_reporter {

namespace {
using Clock = std::chrono::steady_clock;
using ReportFault = ros2_medkit_msgs::srv::ReportFault;
}  // namespace

// Confirms delivery. Per fault code the reports form a queue and only the front is in flight, so the
// manager receives the events of one code in the order they were reported. The front is sent again
// when no response arrives within the ack timeout, up to max_attempts requests.
struct FaultReporter::Delivery : std::enable_shared_from_this<FaultReporter::Delivery> {
  struct Entry {
    std::shared_ptr<ReportFault::Request> request;
    int attempts{0};
    Clock::time_point sent_at;
    int64_t request_id{-1};
  };

  // Deliveries whose reporter is destroyed while they hold reports. One per context, owned by it;
  // the context's shutdown drops what is left, with a warning per report.
  struct Orphans : std::enable_shared_from_this<Orphans> {
    void watch(rclcpp::Context & owner) {
      std::call_once(watched, [this, &owner]() {
        std::weak_ptr<Orphans> weak = weak_from_this();
        owner.add_on_shutdown_callback([weak]() {
          if (auto self = weak.lock()) {
            self->drop_all();
          }
        });
      });
    }

    void adopt(const std::shared_ptr<Delivery> & delivery) {
      std::lock_guard<std::mutex> lock(mutex);
      if (delivery->pending_count() > 0) {
        held[delivery.get()] = delivery;
      }
    }

    void forget(const Delivery * delivery) {
      std::shared_ptr<Delivery> last;  // destroyed after the lock is released
      std::lock_guard<std::mutex> lock(mutex);
      auto it = held.find(delivery);
      if (it != held.end()) {
        last = std::move(it->second);
        held.erase(it);
      }
    }

    void drop_all() {
      std::map<const Delivery *, std::shared_ptr<Delivery>> taken;
      {
        std::lock_guard<std::mutex> lock(mutex);
        taken.swap(held);
      }
      for (auto & entry : taken) {
        entry.second->drop_pending();
      }
    }

    std::mutex mutex;
    std::map<const Delivery *, std::shared_ptr<Delivery>> held;
    std::once_flag watched;
  };

  Delivery(rclcpp::Client<ReportFault>::SharedPtr c, const rclcpp::Logger & l, const core::DeliveryConfig & cfg,
           const rclcpp::Context::SharedPtr & ctx)
    : client(std::move(c)), logger(l), config(cfg), context(ctx) {
  }

  void enqueue(const std::shared_ptr<ReportFault::Request> & request) {
    std::lock_guard<std::mutex> lock(mutex);
    if (pending >= static_cast<size_t>(config.max_pending)) {
      RCLCPP_WARN(logger, "%zu reports wait for the FaultManager, dropping the report for '%s'", pending,
                  request->fault_code.c_str());
      return;
    }
    auto & queue = queues[request->fault_code];
    Entry entry;
    entry.request = request;
    queue.push_back(std::move(entry));
    ++pending;
    if (queue.size() == 1) {
      send_front_locked(request->fault_code);
    }
    arm_timer_locked();
  }

  size_t pending_count() {
    std::lock_guard<std::mutex> lock(mutex);
    return pending;
  }

  // The reporter is destroyed. Pending reports stay here, held by the context, and are sent until
  // confirmed or dropped like any other.
  void release() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (pending == 0) {
        stop_timer_locked();
        return;
      }
    }
    auto ctx = context.lock();
    if (!ctx || !ctx->is_valid()) {
      drop_pending();
      return;
    }
    auto registry = ctx->get_sub_context<Orphans>();
    registry->watch(*ctx);
    {
      std::lock_guard<std::mutex> lock(mutex);
      orphans = registry;
    }
    registry->adopt(shared_from_this());
    if (!ctx->is_valid()) {
      registry->drop_all();  // the shutdown ran before the adoption
    }
  }

  // Drops every pending report with a warning that names its fault code.
  void drop_pending() {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & [code, queue] : queues) {
      if (!queue.empty() && queue.front().request_id >= 0) {
        client->remove_pending_request(queue.front().request_id);
      }
      for (size_t i = 0; i < queue.size(); ++i) {
        RCLCPP_WARN(logger, "Shut down before the FaultManager answered, dropping the report for '%s'", code.c_str());
      }
    }
    queues.clear();
    pending = 0;
    stop_timer_locked();
  }

  void on_response(const std::string & code, const std::shared_ptr<ReportFault::Request> & request) {
    std::shared_ptr<Orphans> done;
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto it = queues.find(code);
      if (it == queues.end() || it->second.empty() || it->second.front().request != request) {
        return;  // the answer to an attempt that was already replaced
      }
      pop_front_locked(code);
      done = drained_orphans_locked();
    }
    if (done) {
      done->forget(this);
    }
  }

  void on_timer() {
    std::shared_ptr<Orphans> done;
    {
      std::lock_guard<std::mutex> lock(mutex);
      retry_due_locked();
      if (pending == 0 && timer) {
        timer->cancel();
      }
      done = drained_orphans_locked();
    }
    if (done) {
      done->forget(this);
    }
  }

  std::mutex mutex;
  std::map<std::string, std::deque<Entry>> queues;
  size_t pending{0};
  rclcpp::TimerBase::SharedPtr timer;

 private:
  void arm_timer_locked() {
    if (timer && timer->is_canceled()) {
      timer->reset();
    }
  }

  void stop_timer_locked() {
    if (timer) {
      timer->cancel();
      timer.reset();
    }
  }

  // The registry that holds this delivery once it has nothing left to send, else null.
  std::shared_ptr<Orphans> drained_orphans_locked() {
    return pending == 0 ? orphans.lock() : nullptr;
  }

  void send_front_locked(const std::string & code) {
    Entry & entry = queues[code].front();
    ++entry.attempts;
    entry.sent_at = Clock::now();
    std::weak_ptr<Delivery> weak = weak_from_this();
    auto request = entry.request;
    entry.request_id =
        client
            ->async_send_request(
                request,
                [weak, code,
                 request](rclcpp::Client<ReportFault>::SharedFuture) {  // NOLINT(performance-unnecessary-value-param)
                  if (auto self = weak.lock()) {
                    self->on_response(code, request);
                  }
                })
            .request_id;
  }

  void pop_front_locked(const std::string & code) {
    auto & queue = queues[code];
    queue.pop_front();
    --pending;
    if (queue.empty()) {
      queues.erase(code);
    } else {
      send_front_locked(code);
    }
  }

  void retry_due_locked() {
    const auto now = Clock::now();
    const auto timeout =
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config.ack_timeout_sec));
    std::vector<std::string> codes;
    for (const auto & [code, queue] : queues) {
      if (!queue.empty() && now - queue.front().sent_at >= timeout) {
        codes.push_back(code);
      }
    }
    for (const auto & code : codes) {
      Entry & entry = queues[code].front();
      client->remove_pending_request(entry.request_id);
      if (entry.attempts >= config.max_attempts) {
        RCLCPP_WARN(logger, "No response from the FaultManager after %d attempts, dropping the report for '%s'",
                    entry.attempts, code.c_str());
        pop_front_locked(code);
      } else {
        RCLCPP_DEBUG(logger, "No response for the report for '%s', sending attempt %d of %d (service %s)", code.c_str(),
                     entry.attempts + 1, config.max_attempts, client->service_is_ready() ? "ready" : "not ready");
        send_front_locked(code);
      }
    }
  }

  rclcpp::Client<ReportFault>::SharedPtr client;
  rclcpp::Logger logger;
  core::DeliveryConfig config;
  std::weak_ptr<rclcpp::Context> context;
  std::weak_ptr<Orphans> orphans;
};

FaultReporter::FaultReporter(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
                             const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & node_graph,
                             const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr & node_services,
                             const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
                             const rclcpp::Logger & logger, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node_base, node_graph, node_services, node_params, nullptr, logger, source_id, service_name) {
}

FaultReporter::FaultReporter(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
                             const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & node_graph,
                             const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr & node_services,
                             const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
                             const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr & node_timers,
                             const rclcpp::Logger & logger, const std::string & source_id,
                             const std::string & service_name)
  : source_id_(source_id)
  , logger_(logger)
  , delivery_config_(load_delivery_config(node_params, logger))
  , core_(source_id, load_parameters(node_params, logger),
          [this](const ros2_medkit::fault_detection::FaultReport & report) {
            send_report(report);
          }) {
  // Validate source_id
  if (source_id_.empty()) {
    RCLCPP_WARN(logger_, "FaultReporter created with empty source_id, fault origins will be difficult to trace");
  }

  // Create service client. It outlives the node handle used to construct it, but still relies on the
  // ROS context and the node interfaces staying valid - reporting after node teardown / rclcpp shutdown
  // is not safe.
#if RCLCPP_VERSION_GTE(29, 0, 0)  // Kilted and later
  client_ = rclcpp::create_client<ros2_medkit_msgs::srv::ReportFault>(node_base, node_graph, node_services,
                                                                      service_name, rclcpp::ServicesQoS());
#else  // Humble, Jazzy
  client_ = rclcpp::create_client<ros2_medkit_msgs::srv::ReportFault>(
      node_base, node_graph, node_services, service_name, rmw_qos_profile_services_default, nullptr);
#endif

  delivery_ = std::make_shared<Delivery>(client_, logger_, delivery_config_, node_base->get_context());
  std::weak_ptr<Delivery> weak = delivery_;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delivery_config_.ack_timeout_sec / 2.0));
  delivery_->timer = std::make_shared<rclcpp::WallTimer<std::function<void()>>>(
      period,
      [weak]() {
        if (auto self = weak.lock()) {
          self->on_timer();
        }
      },
      node_base->get_context());
  // Without a timers interface the timer goes to the node's default callback group all the same.
  if (node_timers) {
    node_timers->add_timer(delivery_->timer, nullptr);
  } else {
    rclcpp::node_interfaces::NodeTimers(node_base.get()).add_timer(delivery_->timer, nullptr);
  }
  delivery_->timer->cancel();

  RCLCPP_DEBUG(logger_, "FaultReporter initialized for source: %s", source_id_.c_str());
}

FaultReporter::~FaultReporter() {
  try {
    delivery_->release();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Could not keep the pending reports of a destroyed reporter: %s", e.what());
  }
}

size_t FaultReporter::pending_reports() const {
  return delivery_->pending_count();
}

FaultReporter::FaultReporter(const rclcpp::Node::SharedPtr & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node->get_node_base_interface(), node->get_node_graph_interface(),
                  node->get_node_services_interface(), node->get_node_parameters_interface(),
                  node->get_node_timers_interface(), node->get_logger(), source_id, service_name) {
}

FaultReporter::FaultReporter(rclcpp::Node & node, const std::string & source_id, const std::string & service_name)
  : FaultReporter(node.get_node_base_interface(), node.get_node_graph_interface(), node.get_node_services_interface(),
                  node.get_node_parameters_interface(), node.get_node_timers_interface(), node.get_logger(), source_id,
                  service_name) {
}

FaultReporter::FaultReporter(rclcpp_lifecycle::LifecycleNode & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node.get_node_base_interface(), node.get_node_graph_interface(), node.get_node_services_interface(),
                  node.get_node_parameters_interface(), node.get_node_timers_interface(), node.get_logger(), source_id,
                  service_name) {
}

FaultReporter::FaultReporter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node->get_node_base_interface(), node->get_node_graph_interface(),
                  node->get_node_services_interface(), node->get_node_parameters_interface(),
                  node->get_node_timers_interface(), node->get_logger(), source_id, service_name) {
}

FilterConfig
FaultReporter::load_parameters(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
                               const rclcpp::Logger & logger) {
  const FilterConfig defaults;

  // Declare parameters with defaults if not already declared
  if (!node_params->has_parameter("fault_reporter.local_filtering.enabled")) {
    node_params->declare_parameter("fault_reporter.local_filtering.enabled", rclcpp::ParameterValue(defaults.enabled));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.default_threshold")) {
    node_params->declare_parameter("fault_reporter.local_filtering.default_threshold",
                                   rclcpp::ParameterValue(defaults.default_threshold));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.default_window_sec")) {
    node_params->declare_parameter("fault_reporter.local_filtering.default_window_sec",
                                   rclcpp::ParameterValue(defaults.default_window_sec));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.bypass_severity")) {
    node_params->declare_parameter("fault_reporter.local_filtering.bypass_severity",
                                   rclcpp::ParameterValue(static_cast<int>(defaults.bypass_severity)));
  }

  // Parameters are int64: core::validate_filter_params checks the range before narrowing.
  std::vector<std::string> warnings;
  const FilterConfig config = core::validate_filter_params(
      node_params->get_parameter("fault_reporter.local_filtering.enabled").as_bool(),
      node_params->get_parameter("fault_reporter.local_filtering.default_threshold").as_int(),
      node_params->get_parameter("fault_reporter.local_filtering.default_window_sec").as_double(),
      node_params->get_parameter("fault_reporter.local_filtering.bypass_severity").as_int(), &warnings);
  for (const auto & warning : warnings) {
    RCLCPP_WARN(logger, "%s", warning.c_str());
  }

  RCLCPP_DEBUG(logger, "Filter config: enabled=%d, threshold=%d, window=%.1fs, bypass_severity=%d", config.enabled,
               config.default_threshold, config.default_window_sec, config.bypass_severity);
  return config;
}

core::DeliveryConfig
FaultReporter::load_delivery_config(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
                                    const rclcpp::Logger & logger) {
  const core::DeliveryConfig defaults;
  if (!node_params->has_parameter("fault_reporter.delivery.ack_timeout_sec")) {
    node_params->declare_parameter("fault_reporter.delivery.ack_timeout_sec",
                                   rclcpp::ParameterValue(defaults.ack_timeout_sec));
  }
  if (!node_params->has_parameter("fault_reporter.delivery.max_attempts")) {
    node_params->declare_parameter("fault_reporter.delivery.max_attempts",
                                   rclcpp::ParameterValue(static_cast<int64_t>(defaults.max_attempts)));
  }
  if (!node_params->has_parameter("fault_reporter.delivery.max_pending")) {
    node_params->declare_parameter("fault_reporter.delivery.max_pending",
                                   rclcpp::ParameterValue(static_cast<int64_t>(defaults.max_pending)));
  }

  // Parameters are int64: the range is checked before narrowing.
  std::vector<std::string> warnings;
  const auto config = core::validate_delivery_params(
      node_params->get_parameter("fault_reporter.delivery.ack_timeout_sec").as_double(),
      node_params->get_parameter("fault_reporter.delivery.max_attempts").as_int(),
      node_params->get_parameter("fault_reporter.delivery.max_pending").as_int(), &warnings);
  for (const auto & warning : warnings) {
    RCLCPP_WARN(logger, "%s", warning.c_str());
  }
  return config;
}

void FaultReporter::report(const std::string & fault_code, uint8_t severity, const std::string & description) {
  // Validate fault_code
  if (fault_code.empty()) {
    RCLCPP_WARN(logger_, "Attempted to report fault with empty fault_code, ignoring");
    return;
  }

  if (!core_.report(fault_code, severity, description)) {
    RCLCPP_DEBUG(logger_, "Fault '%s' filtered (threshold not met)", fault_code.c_str());
  }
}

void FaultReporter::report_passed(const std::string & fault_code) {
  if (fault_code.empty()) {
    RCLCPP_WARN(logger_, "Attempted to report PASSED with empty fault_code, ignoring");
    return;
  }

  if (!core_.report_passed(fault_code)) {
    RCLCPP_DEBUG(logger_, "PASSED for '%s' filtered (threshold not met)", fault_code.c_str());
  }
}

bool FaultReporter::is_service_ready() const {
  return client_->service_is_ready();
}

void FaultReporter::send_report(const ros2_medkit::fault_detection::FaultReport & report) {
  using Request = ros2_medkit_msgs::srv::ReportFault::Request;
  static_assert(static_cast<uint8_t>(ros2_medkit::fault_detection::ReportKind::kFailed) == Request::EVENT_FAILED,
                "ReportKind::kFailed must match ReportFault EVENT_FAILED");
  static_assert(static_cast<uint8_t>(ros2_medkit::fault_detection::ReportKind::kPassed) == Request::EVENT_PASSED,
                "ReportKind::kPassed must match ReportFault EVENT_PASSED");

  const bool failed = report.kind == ros2_medkit::fault_detection::ReportKind::kFailed;
  if (!client_->service_is_ready()) {
    // Use WARN level for high-severity faults that would bypass filtering
    if (failed && report.severity >= core_.filter().config().bypass_severity) {
      RCLCPP_WARN(logger_, "FaultManager service not available, dropping high-severity fault '%s'",
                  report.fault_code.c_str());
    } else {
      RCLCPP_DEBUG(logger_, "FaultManager service not available, skipping report for '%s'", report.fault_code.c_str());
    }
    return;
  }

  auto request = std::make_shared<Request>();
  request->fault_code = report.fault_code;
  request->event_type = static_cast<uint8_t>(report.kind);
  request->severity = report.severity;
  request->description = report.description;
  request->source_id = source_id_;

  delivery_->enqueue(request);

  if (failed) {
    RCLCPP_DEBUG(logger_, "Reported FAILED: %s (severity=%d)", report.fault_code.c_str(), report.severity);
  } else {
    RCLCPP_DEBUG(logger_, "Reported PASSED: %s", report.fault_code.c_str());
  }
}

}  // namespace ros2_medkit_fault_reporter
