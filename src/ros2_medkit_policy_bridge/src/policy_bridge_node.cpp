// Copyright 2026 ahmedsleem109
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

#include "ros2_medkit_policy_bridge/policy_bridge_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

namespace ros2_medkit_policy_bridge {

namespace {

// Fault code length limit used across medkit bridges.
constexpr size_t kMaxCodeLen = 64;

std::string chunk_disagreement_text(double spread, double threshold) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "Policy predictions stopped agreeing: chunk spread %.4g above %.4g", spread,
                threshold);
  return buf;
}

std::string slow_inference_text(double ms, double max_ms) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "Policy inference took %.1f ms, above %.1f ms", ms, max_ms);
  return buf;
}

// Parameters are declared with dynamic typing and read with these helpers, so
// a value of the wrong type (an integer for a threshold, a string for a count)
// is named in a warning and replaced by the default instead of throwing out of
// the constructor.
class ParameterReader {
 public:
  ParameterReader(rclcpp::Node & node, std::vector<std::string> & warnings) : node_(node), warnings_(warnings) {
  }

  double number(const std::string & name, double fallback) {
    const auto v = declare(name, rclcpp::ParameterValue(fallback));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return v.get<double>();
    }
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(v.get<int64_t>());
    }
    wrong_type(name, v, "a number", core::number_text(fallback));
    return fallback;
  }

  int64_t integer(const std::string & name, int64_t fallback) {
    const auto v = declare(name, rclcpp::ParameterValue(fallback));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return v.get<int64_t>();
    }
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double d = v.get<double>();
      // A whole number written as 3.0 is taken; the range check that follows
      // in validate_config sees the real value.
      double whole = 0.0;
      if (std::isfinite(d) && std::fabs(d) < 9.0e15 && !(std::fabs(std::modf(d, &whole)) > 0.0)) {
        return static_cast<int64_t>(whole);
      }
    }
    wrong_type(name, v, "an integer", std::to_string(fallback));
    return fallback;
  }

  bool flag(const std::string & name, bool fallback) {
    const auto v = declare(name, rclcpp::ParameterValue(fallback));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      return v.get<bool>();
    }
    wrong_type(name, v, "true or false", fallback ? "true" : "false");
    return fallback;
  }

  std::string text(const std::string & name, const std::string & fallback) {
    const auto v = declare(name, rclcpp::ParameterValue(fallback));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return v.get<std::string>();
    }
    wrong_type(name, v, "a string", "\"" + fallback + "\"");
    return fallback;
  }

  std::vector<double> numbers(const std::string & name) {
    const auto v = declare(name, rclcpp::ParameterValue(std::vector<double>{}));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      return v.get<std::vector<double>>();
    }
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto & ints = v.get<std::vector<int64_t>>();
      return std::vector<double>(ints.begin(), ints.end());
    }
    wrong_type(name, v, "a list of numbers", "[]");
    return {};
  }

 private:
  rclcpp::ParameterValue declare(const std::string & name, const rclcpp::ParameterValue & fallback) {
    rcl_interfaces::msg::ParameterDescriptor any;
    any.dynamic_typing = true;
    return node_.declare_parameter(name, fallback, any);
  }

  void wrong_type(const std::string & name, const rclcpp::ParameterValue & v, const char * expected,
                  const std::string & used) {
    warnings_.push_back(name + " is " + rclcpp::to_string(v.get_type()) + ", expected " + expected + ": using " + used);
  }

  rclcpp::Node & node_;
  std::vector<std::string> & warnings_;
};

}  // namespace

PolicyBridgeNode::PolicyBridgeNode(const rclcpp::NodeOptions & options) : Node("policy_bridge", options) {
  load_parameters();
  monitor_ = std::make_unique<core::PolicyMonitor>(config_);
  sync_ = core::FaultSync({chunk_disagreement_code_, slow_inference_code_});

  // Best effort matches both reliable and best-effort publishers. A dropped
  // message is a missing reading: it neither counts toward a raise nor resets
  // one, so it cannot create or clear a fault by itself. An invalid topic name
  // throws here; main() logs it and exits.
  sub_ = create_subscription<ros2_medkit_policy_msgs::msg::PolicyIntrospection>(
      topic_param_, rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
      [this](const ros2_medkit_policy_msgs::msg::PolicyIntrospection::ConstSharedPtr & msg) {
        on_introspection(msg);
      });

  // Checks the FaultManager service and sends what is due. It runs for the
  // life of the node: it also notices the FaultManager coming back.
  sync_timer_ = create_wall_timer(std::chrono::duration<double>(sync_period_sec_), [this]() {
    sync();
  });

  if (config_.spread_threshold <= 0.0 && config_.max_inference_ms <= 0.0) {
    RCLCPP_INFO(get_logger(),
                "PolicyBridge on %s has no check enabled: set spread_threshold and/or max_inference_ms "
                "(measure them on successful runs of the policy)",
                sub_->get_topic_name());
  } else {
    RCLCPP_INFO(get_logger(),
                "PolicyBridge watching %s (spread_threshold=%.4g, max_inference_ms=%.4g, raise_after=%u, "
                "clear_after=%u, clear_on_new_episode=%s)",
                sub_->get_topic_name(), config_.spread_threshold, config_.max_inference_ms, config_.raise_after,
                config_.clear_after, config_.clear_on_new_episode ? "true" : "false");
  }
}

PolicyBridgeNode::~PolicyBridgeNode() {
  sync_timer_.reset();
  sub_.reset();
}

void PolicyBridgeNode::load_parameters() {
  std::vector<std::string> warnings;
  ParameterReader read(*this, warnings);

  topic_param_ = read.text("introspection_topic", "/policy/introspection");
  source_id_ = read.text("source_id", "");
  const auto prefix = read.text("code_prefix", "POLICY");

  core::RawConfig raw;
  raw.spread_threshold = read.number("spread_threshold", 0.0);
  raw.spread_history = read.integer("spread_history", 4);
  raw.spread_horizon = read.integer("spread_horizon", 20);
  raw.joint_scale = read.numbers("joint_scale");
  raw.max_inference_ms = read.number("max_inference_ms", 0.0);
  raw.raise_after = read.integer("raise_after", 3);
  raw.clear_after = read.integer("clear_after", 0);
  raw.clear_on_new_episode = read.flag("clear_on_new_episode", true);
  config_ = core::validate_config(raw, &warnings);

  auto severity = [&warnings](const char * name, int64_t value) {
    if (value < 0 || value > 3) {
      warnings.push_back(std::string(name) + " " + std::to_string(value) + " is outside [0, 3], using 1 (WARN)");
      return uint8_t{1};
    }
    return static_cast<uint8_t>(value);
  };
  chunk_disagreement_severity_ =
      severity("chunk_disagreement_severity", read.integer("chunk_disagreement_severity", 1));
  slow_inference_severity_ = severity("slow_inference_severity", read.integer("slow_inference_severity", 1));

  const double period = read.number("retry_period_sec", 0.1);
  if (std::isfinite(period) && period >= 0.01 && period <= 10.0) {
    sync_period_sec_ = period;
  } else {
    warnings.push_back("retry_period_sec " + core::number_text(period) + " is outside [0.01, 10], using 0.1");
    sync_period_sec_ = 0.1;
  }

  for (const auto & w : warnings) {
    RCLCPP_WARN(get_logger(), "%s", w.c_str());
  }

  chunk_disagreement_code_ = fault_code(prefix, "CHUNK_DISAGREEMENT");
  slow_inference_code_ = fault_code(prefix, "SLOW_INFERENCE");

  // The bridge sends one report per change of state, so the reporter's local
  // occurrence filter would hold back a single raise and every pass. Declared
  // off here; a value given for it (YAML or command line) is kept.
  if (!has_parameter("fault_reporter.local_filtering.enabled")) {
    declare_parameter("fault_reporter.local_filtering.enabled", false);
  }
}

std::string PolicyBridgeNode::fault_code(const std::string & prefix, const std::string & suffix) {
  std::string head;
  for (char ch : prefix) {
    const auto c = static_cast<unsigned char>(ch);
    head += std::isalnum(c) ? static_cast<char>(std::toupper(c)) : '_';
  }
  // Collapse runs of '_' and trim them from both ends.
  std::string clean;
  for (char ch : head) {
    if (ch == '_' && (clean.empty() || clean.back() == '_')) {
      continue;
    }
    clean += ch;
  }
  while (!clean.empty() && clean.back() == '_') {
    clean.pop_back();
  }
  // Keep the suffix whole: the prefix gives way when the code is too long.
  const size_t room = kMaxCodeLen - suffix.size() - 1;
  if (clean.size() > room) {
    clean.resize(room);
    while (!clean.empty() && clean.back() == '_') {
      clean.pop_back();
    }
  }
  return clean.empty() ? suffix : clean + "_" + suffix;
}

core::PolicySample PolicyBridgeNode::to_sample(const ros2_medkit_policy_msgs::msg::PolicyIntrospection & msg) {
  core::PolicySample s;
  s.episode = msg.episode;
  s.step = msg.step;
  s.chunk_length = msg.chunk_length;
  s.action_dim = msg.action_dim;
  if (msg.chunk.size() > core::kMaxChunkValues) {
    s.dropped_chunk_values = msg.chunk.size();  // not copied: it cannot be used
  } else {
    s.chunk = msg.chunk;
  }
  s.inference_ms = msg.inference_ms;
  return s;
}

void PolicyBridgeNode::on_introspection(const ros2_medkit_policy_msgs::msg::PolicyIntrospection::ConstSharedPtr & msg) {
  const auto result = monitor_->update(to_sample(*msg));
  log_rejection(result, *msg);
  for (const auto & notice : result.notices) {
    RCLCPP_WARN(get_logger(), "%s: %s", sub_->get_topic_name(), notice.c_str());
  }
  if (result.transitions.empty()) {
    return;
  }
  for (const auto & t : result.transitions) {
    const bool low = t.signal == core::MonitorResult::Signal::kChunkDisagreement;
    std::string description;
    if (t.raise) {
      description = low ? chunk_disagreement_text(t.value, config_.spread_threshold)
                        : slow_inference_text(t.value, config_.max_inference_ms);
      description += " for " + std::to_string(config_.raise_after) + " readings in a row (episode " +
                     std::to_string(msg->episode) + ", step " + std::to_string(msg->step) + ")";
    }
    sync_.set(low ? chunk_disagreement_code_ : slow_inference_code_, t.raise,
              low ? chunk_disagreement_severity_ : slow_inference_severity_, std::move(description));
  }
  sync();
}

void PolicyBridgeNode::log_rejection(const core::MonitorResult & result,
                                     const ros2_medkit_policy_msgs::msg::PolicyIntrospection & msg) {
  const auto now = std::chrono::steady_clock::now();
  if (result.rejection != core::Rejection::kNone) {
    auto [it, first] = rejections_.try_emplace(result.rejection);
    auto & log = it->second;
    if (first) {
      RCLCPP_WARN(get_logger(),
                  "Ignored a message on %s (episode %u, step %u): %s; more messages ignored for this reason are "
                  "counted and logged at most every %lld s",
                  sub_->get_topic_name(), msg.episode, msg.step, result.rejected.c_str(),
                  static_cast<long long>(kRejectionSummaryPeriod.count()));
      log.last_logged = now;
    } else {
      ++log.unlogged;
    }
    log.last_reason = result.rejected;
  }
  for (auto & [kind, log] : rejections_) {
    if (log.unlogged > 0 && now - log.last_logged >= kRejectionSummaryPeriod) {
      RCLCPP_WARN(get_logger(), "Ignored %llu more messages on %s since the last report, the latest: %s",
                  static_cast<unsigned long long>(log.unlogged), sub_->get_topic_name(), log.last_reason.c_str());
      log.unlogged = 0;
      log.last_logged = now;
    }
  }
}

ros2_medkit_fault_reporter::FaultReporter * PolicyBridgeNode::reporter() {
  if (reporter_) {
    return reporter_.get();
  }
  const std::string topic = sub_->get_topic_name();  // after remapping
  std::string source = source_id_;
  if (source.empty()) {
    // Attribute the faults to the node that publishes the policy's
    // introspection, so they show up on that entity in the gateway. During
    // discovery rcl can report placeholder names; those are not used.
    std::vector<rclcpp::TopicEndpointInfo> publishers;
    try {
      publishers = get_publishers_info_by_topic(topic);
    } catch (const std::exception & e) {
      // A shutdown (SIGINT) can land while the timer runs: then this fails
      // because the context is gone, and there is nothing left to report.
      if (rclcpp::ok(get_node_base_interface()->get_context()) && !graph_error_logged_) {
        RCLCPP_WARN(get_logger(), "Cannot list the publishers of %s, trying again: %s", topic.c_str(), e.what());
        graph_error_logged_ = true;
      }
      return nullptr;
    }
    for (const auto & info : publishers) {
      const auto & name = info.node_name();
      const auto & ns = info.node_namespace();
      if (name.empty() || name == "_NODE_NAME_UNKNOWN_" || ns == "_NODE_NAMESPACE_UNKNOWN_") {
        continue;
      }
      source = (ns.empty() || ns == "/") ? std::string() : ns;
      source += "/";
      source += name;
      break;
    }
  }
  if (source.empty()) {
    // Give discovery time to name the publisher before falling back.
    const auto now = std::chrono::steady_clock::now();
    if (!source_wait_since_) {
      source_wait_since_ = now;
    }
    if (now - *source_wait_since_ < kSourceResolveGrace) {
      return nullptr;
    }
    source = topic;
    RCLCPP_WARN(get_logger(), "No named publisher on %s after %lld s, reporting policy faults with source_id %s",
                topic.c_str(), static_cast<long long>(kSourceResolveGrace.count()), source.c_str());
  }
  reporter_ = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(*this, source, kReportService);
  return reporter_.get();
}

bool PolicyBridgeNode::new_fault_manager_seen() {
  // A FaultManager that was killed and started again at once never looks
  // gone: the dead one's endpoints stay in the graph until its DDS lease ends,
  // so the service stays ready. The new one is a new publisher on the event
  // topic straight away.
  std::vector<rclcpp::TopicEndpointInfo> publishers;
  try {
    publishers = get_publishers_info_by_topic(kFaultManagerEvents);
  } catch (const std::exception &) {
    return false;  // shutting down, or the graph is busy: try again next tick
  }
  std::set<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> now;
  for (const auto & info : publishers) {
    now.insert(info.endpoint_gid());
  }
  const bool seen_new = std::any_of(now.begin(), now.end(), [this](const auto & gid) {
    return fault_manager_instances_.count(gid) == 0;
  });
  fault_manager_instances_ = std::move(now);
  return seen_new;
}

void PolicyBridgeNode::sync() {
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {
    return;  // shutting down
  }
  auto * rep = reporter();
  if (rep == nullptr) {
    return;  // the source is not resolved yet; due reports wait
  }
  const bool ready = rep->is_service_ready();
  const bool new_instance = new_fault_manager_seen();
  if (ready && (!fault_manager_ready_ || new_instance)) {
    // The FaultManager appeared, for the first time or after a restart that
    // may have lost what it held: send the state of every code again.
    sync_.forget_sent();
    if (missing_logged_) {
      RCLCPP_INFO(get_logger(), "FaultManager reachable again, sending the state of %s and %s",
                  chunk_disagreement_code_.c_str(), slow_inference_code_.c_str());
    }
    missing_logged_ = false;
    raise_waits_since_.reset();
  }
  fault_manager_ready_ = ready;
  if (!ready) {
    // A raise nobody can receive is worth one line; passes are not.
    const auto due = sync_.due();
    const bool raise_due = std::any_of(due.begin(), due.end(), [](const auto & r) {
      return r.raise;
    });
    const auto now = std::chrono::steady_clock::now();
    if (!raise_due) {
      raise_waits_since_.reset();
    } else if (!raise_waits_since_) {
      raise_waits_since_ = now;
    } else if (!missing_logged_ && now - *raise_waits_since_ >= kFaultManagerMissingWarn) {
      RCLCPP_WARN(get_logger(),
                  "FaultManager service not reachable for %lld s while a policy fault is raised; it is sent when "
                  "the service appears",
                  static_cast<long long>(kFaultManagerMissingWarn.count()));
      missing_logged_ = true;
    }
    return;
  }
  for (const auto & r : sync_.due()) {
    if (r.raise) {
      rep->report(r.code, r.severity, r.description);
    } else {
      rep->report_passed(r.code);
    }
    sync_.mark_sent(r);
  }
}

}  // namespace ros2_medkit_policy_bridge
