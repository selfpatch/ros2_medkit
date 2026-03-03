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

#include "ros2_medkit_gateway/log_manager.hpp"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "ros2_medkit_gateway/plugins/plugin_manager.hpp"

namespace ros2_medkit_gateway {

namespace {
// Log level aliases matching rcl_interfaces::msg::Log constants
using Log = rcl_interfaces::msg::Log;
}  // namespace

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

std::string LogManager::level_to_severity(uint8_t level) {
  switch (level) {
    case Log::DEBUG:
      return "debug";
    case Log::INFO:
      return "info";
    case Log::WARN:
      return "warning";
    case Log::ERROR:
      return "error";
    case Log::FATAL:
      return "fatal";
    default:
      return "debug";
  }
}

uint8_t LogManager::severity_to_level(const std::string & severity) {
  if (severity == "debug") {
    return Log::DEBUG;
  }
  if (severity == "info") {
    return Log::INFO;
  }
  if (severity == "warning") {
    return Log::WARN;
  }
  if (severity == "error") {
    return Log::ERROR;
  }
  if (severity == "fatal") {
    return Log::FATAL;
  }
  return 0;
}

bool LogManager::is_valid_severity(const std::string & severity) {
  return severity_to_level(severity) != 0;
}

std::string LogManager::normalize_fqn(const std::string & fqn) {
  if (!fqn.empty() && fqn[0] == '/') {
    return fqn.substr(1);
  }
  return fqn;
}

json LogManager::entry_to_json(const LogEntry & e) {
  // Format as ISO 8601 UTC
  std::time_t t = static_cast<std::time_t>(e.stamp_sec);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);
  std::ostringstream ts;
  ts << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%S");
  ts << "." << std::setfill('0') << std::setw(9) << e.stamp_nanosec << "Z";

  json j;
  j["id"] = "log_" + std::to_string(e.id);
  j["timestamp"] = ts.str();
  j["severity"] = level_to_severity(e.level);
  j["message"] = e.msg;

  json ctx;
  ctx["node"] = e.name;
  if (!e.function.empty()) {
    ctx["function"] = e.function;
  }
  if (!e.file.empty()) {
    ctx["file"] = e.file;
  }
  if (e.line != 0) {
    ctx["line"] = e.line;
  }
  j["context"] = ctx;

  return j;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LogManager::LogManager(rclcpp::Node * node, PluginManager * plugin_mgr, size_t max_buffer_size)
  : node_(node), plugin_mgr_(plugin_mgr), max_buffer_size_(max_buffer_size) {
  rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100), [this](const rcl_interfaces::msg::Log::ConstSharedPtr & msg) {
        on_rosout(msg);
      });

  RCLCPP_INFO(node_->get_logger(), "LogManager: subscribed to /rosout (buffer_size=%zu)", max_buffer_size_);
}

// ---------------------------------------------------------------------------
// /rosout callback
// ---------------------------------------------------------------------------

void LogManager::on_rosout(const rcl_interfaces::msg::Log::ConstSharedPtr & msg) {
  LogEntry entry;
  entry.id = next_id_.fetch_add(1, std::memory_order_relaxed);
  entry.stamp_sec = msg->stamp.sec;
  entry.stamp_nanosec = msg->stamp.nanosec;
  entry.level = msg->level;
  entry.name = msg->name;  // already without leading slash per rcl convention
  entry.msg = msg->msg;
  entry.function = msg->function;
  entry.file = msg->file;
  entry.line = msg->line;

  // Notify all LogProvider observers — they may forward to OTel, DB, etc.
  // Exceptions from plugins are caught to prevent a misbehaving plugin from
  // crashing the gateway's ROS 2 subscription callback.
  bool suppress_buffer = false;
  if (plugin_mgr_) {
    for (auto * observer : plugin_mgr_->get_log_observers()) {
      try {
        if (observer->on_log_entry(entry)) {
          suppress_buffer = true;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "LogProvider::on_log_entry threw: %s", e.what());
      } catch (...) {
        RCLCPP_WARN(node_->get_logger(), "LogProvider::on_log_entry threw unknown exception");
      }
    }
  }

  if (!suppress_buffer) {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    auto & buf = buffers_[entry.name];
    buf.push_back(std::move(entry));
    if (buf.size() > max_buffer_size_) {
      buf.pop_front();
    }
  }
}

// ---------------------------------------------------------------------------
// inject_entry_for_testing
// ---------------------------------------------------------------------------

void LogManager::inject_entry_for_testing(LogEntry entry) {
  std::lock_guard<std::mutex> lock(buffers_mutex_);
  auto & buf = buffers_[entry.name];
  buf.push_back(std::move(entry));
  if (buf.size() > max_buffer_size_) {
    buf.pop_front();
  }
}

// ---------------------------------------------------------------------------
// effective_provider
// ---------------------------------------------------------------------------

LogProvider * LogManager::effective_provider() const {
  if (plugin_mgr_) {
    return plugin_mgr_->get_log_provider();
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// get_logs
// ---------------------------------------------------------------------------

json LogManager::get_logs(const std::vector<std::string> & node_fqns, bool prefix_match,
                          const std::string & min_severity, const std::string & context_filter,
                          const std::string & entity_id) {
  // Delegate to plugin if one is registered
  if (auto * provider = effective_provider()) {
    // Normalize FQNs before passing to plugin (strip leading '/')
    std::vector<std::string> normalized;
    normalized.reserve(node_fqns.size());
    for (const auto & fqn : node_fqns) {
      normalized.push_back(normalize_fqn(fqn));
    }
    return provider->get_logs(normalized, prefix_match, min_severity, context_filter, entity_id);
  }

  // Default: query local ring buffer
  LogConfig cfg = get_config(entity_id);

  // Effective minimum severity: stricter of entity config and query-param override
  uint8_t min_level = severity_to_level(cfg.severity_filter);
  if (!min_severity.empty() && is_valid_severity(min_severity)) {
    uint8_t query_level = severity_to_level(min_severity);
    if (query_level > min_level) {
      min_level = query_level;
    }
  }

  std::vector<LogEntry> collected;

  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    for (const auto & [buf_name, buf] : buffers_) {
      bool matches = false;
      for (const auto & fqn : node_fqns) {
        const std::string norm = normalize_fqn(fqn);
        if (prefix_match) {
          // Match exactly OR as a namespace prefix (must be followed by '/')
          // e.g. norm="powertrain/engine" must NOT match "powertrain/engine_control"
          matches = (buf_name == norm) || (buf_name.rfind(norm + "/", 0) == 0);
        } else {
          matches = (buf_name == norm);
        }
        if (matches) {
          break;
        }
      }
      if (!matches) {
        continue;
      }

      for (const auto & entry : buf) {
        if (entry.level < min_level) {
          continue;
        }
        if (!context_filter.empty() && entry.name.find(context_filter) == std::string::npos) {
          continue;
        }
        collected.push_back(entry);
      }
    }
  }

  // Sort by id ascending
  std::sort(collected.begin(), collected.end(), [](const LogEntry & a, const LogEntry & b) {
    return a.id < b.id;
  });

  // Cap to max_entries (most recent N)
  if (collected.size() > cfg.max_entries) {
    collected.erase(collected.begin(), collected.begin() + static_cast<ptrdiff_t>(collected.size() - cfg.max_entries));
  }

  json items = json::array();
  for (const auto & e : collected) {
    items.push_back(entry_to_json(e));
  }
  return items;
}

// ---------------------------------------------------------------------------
// Config management
// ---------------------------------------------------------------------------

LogConfig LogManager::get_config(const std::string & entity_id) const {
  std::lock_guard<std::mutex> lock(configs_mutex_);
  auto it = configs_.find(entity_id);
  if (it != configs_.end()) {
    return it->second;
  }
  return LogConfig{};
}

std::string LogManager::update_config(const std::string & entity_id, const std::optional<std::string> & severity_filter,
                                      const std::optional<size_t> & max_entries) {
  if (severity_filter.has_value() && !is_valid_severity(*severity_filter)) {
    return "Invalid severity_filter '" + *severity_filter + "'. Must be one of: debug, info, warning, error, fatal";
  }
  if (max_entries.has_value() && *max_entries == 0) {
    return "max_entries must be greater than 0";
  }

  // If a plugin provider is registered, delegate config to it
  if (auto * provider = effective_provider()) {
    return provider->update_config(entity_id, severity_filter, max_entries);
  }

  std::lock_guard<std::mutex> lock(configs_mutex_);
  auto & cfg = configs_[entity_id];
  if (severity_filter.has_value()) {
    cfg.severity_filter = *severity_filter;
  }
  if (max_entries.has_value()) {
    cfg.max_entries = *max_entries;
  }
  return "";
}

}  // namespace ros2_medkit_gateway
