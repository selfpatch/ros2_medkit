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

#include "ros2_medkit_gateway/core/managers/log_manager.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"

namespace ros2_medkit_gateway {

namespace {

// SOVD log severity levels mirror the rcl_interfaces::msg::Log uint8 constants
// (DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50). These are part of the
// neutral data contract carried in LogEntry::level - the source adapter
// performs the ROS-side mapping; the manager body operates on numeric levels
// only.
constexpr uint8_t kLevelDebug = 10;
constexpr uint8_t kLevelInfo = 20;
constexpr uint8_t kLevelWarn = 30;
constexpr uint8_t kLevelError = 40;
constexpr uint8_t kLevelFatal = 50;

}  // namespace

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

std::string LogManager::level_to_severity(uint8_t level) {
  switch (level) {
    case kLevelDebug:
      return "debug";
    case kLevelInfo:
      return "info";
    case kLevelWarn:
      return "warning";
    case kLevelError:
      return "error";
    case kLevelFatal:
      return "fatal";
    default:
      return "debug";
  }
}

uint8_t LogManager::severity_to_level(const std::string & severity) {
  if (severity == "debug") {
    return kLevelDebug;
  }
  if (severity == "info") {
    return kLevelInfo;
  }
  if (severity == "warning") {
    return kLevelWarn;
  }
  if (severity == "error") {
    return kLevelError;
  }
  if (severity == "fatal") {
    return kLevelFatal;
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
// Constructor / destructor
// ---------------------------------------------------------------------------

LogManager::LogManager(std::shared_ptr<LogSource> source, LogProviderRegistry * provider_registry,
                       size_t max_buffer_size)
  : source_(std::move(source)), provider_registry_(provider_registry), max_buffer_size_(max_buffer_size) {
  auto * provider = effective_provider();
  if (provider && provider->manages_ingestion()) {
    // Primary LogProvider owns the entire pipeline - never start the source,
    // so no subscription is created.
    return;
  }

  if (source_) {
    source_->start([this](const LogEntry & entry) {
      on_log_entry(entry);
    });
  }
}

LogManager::~LogManager() {
  // Stop the source before any of our members destruct so no callback can
  // fire on a half-destroyed manager. The source's stop() is idempotent and
  // also runs from its own destructor; calling it here gives a deterministic
  // teardown order regardless of how the shared_ptr's last reference drops.
  if (source_) {
    source_->stop();
  }
}

// ---------------------------------------------------------------------------
// Source-emitted entry handler
// ---------------------------------------------------------------------------

void LogManager::on_log_entry(const LogEntry & source_entry) {
  // Stamp a monotonic id (the source emits entries without one; we are the
  // single point of id assignment for both source-driven and programmatic
  // injection paths).
  LogEntry entry = source_entry;
  entry.id = next_id_.fetch_add(1, std::memory_order_relaxed);

  // Notify all LogProvider observers - they may forward to OTel, DB, etc.
  // Exceptions from plugins are caught to prevent a misbehaving plugin from
  // crashing the source's delivery callback.
  bool suppress_buffer = false;
  if (provider_registry_) {
    for (auto * observer : provider_registry_->log_observers()) {
      try {
        if (observer->on_log_entry(entry)) {
          suppress_buffer = true;
        }
      } catch (const std::exception & e) {
        std::cerr << "LogProvider::on_log_entry threw: " << e.what() << '\n';
      } catch (...) {
        std::cerr << "LogProvider::on_log_entry threw unknown exception\n";
      }
    }
  }

  if (!suppress_buffer) {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    // Cap the number of distinct node buffers to prevent unbounded growth.
    // Uses max_buffer_size_ * 10 as the cap (e.g., 200 entries -> 2000 distinct nodes).
    if (buffers_.find(entry.name) == buffers_.end() && buffers_.size() >= max_buffer_size_ * 10) {
      // Silently drop logs from new nodes beyond the cap
    } else {
      auto & buf = buffers_[entry.name];
      buf.push_back(entry);
      if (buf.size() > max_buffer_size_) {
        buf.pop_front();
      }
    }
  }

  // Notify triggers about the log event (resolve logger name to entity ID)
  if (notifier_) {
    std::string entity_id;

    // Try resolving via node_to_app mapping (manifest/hybrid mode)
    if (node_to_entity_resolver_) {
      // entry.name has no leading '/' (rcl convention), but the mapping
      // may use FQNs with leading '/' - try both forms
      entity_id = node_to_entity_resolver_("/" + entry.name);
      if (entity_id.empty()) {
        entity_id = node_to_entity_resolver_(entry.name);
      }
    }

    // Fallback: extract last path segment (runtime_only mode)
    if (entity_id.empty()) {
      auto last_slash = entry.name.rfind('/');
      entity_id = (last_slash != std::string::npos) ? entry.name.substr(last_slash + 1) : entry.name;
    }

    if (!entity_id.empty()) {
      notifier_->notify("logs", entity_id, "", entry_to_json(entry), ChangeType::CREATED);
    }
  }
}

// ---------------------------------------------------------------------------
// inject_entry_for_testing
// ---------------------------------------------------------------------------

void LogManager::inject_entry_for_testing(LogEntry entry) {
  std::lock_guard<std::mutex> lock(buffers_mutex_);
  // Respect buffer cap (same logic as on_log_entry) for consistent test behavior
  if (buffers_.find(entry.name) == buffers_.end() && buffers_.size() >= max_buffer_size_ * 10) {
    return;  // Silently drop logs from new nodes beyond the cap
  }
  auto & buf = buffers_[entry.name];
  buf.push_back(std::move(entry));
  if (buf.size() > max_buffer_size_) {
    buf.pop_front();
  }
}

// ---------------------------------------------------------------------------
// add_log_entry (programmatic injection for trigger log_settings)
// ---------------------------------------------------------------------------

void LogManager::add_log_entry(const std::string & entity_id, const std::string & severity, const std::string & message,
                               const nlohmann::json & metadata) {
  std::string normalized = normalize_fqn(entity_id);

  // Build a LogEntry using the same structure as source-emitted entries.
  LogEntry entry;
  entry.id = next_id_.fetch_add(1, std::memory_order_relaxed);

  // Use wall-clock time for programmatic entries
  auto now = std::chrono::system_clock::now();
  auto epoch = now.time_since_epoch();
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
  auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);

  entry.stamp_sec = secs.count();
  entry.stamp_nanosec = static_cast<uint32_t>(nsecs.count());
  entry.level = severity_to_level(severity);
  if (entry.level == 0) {
    entry.level = kLevelInfo;  // fallback for invalid severity
  }
  entry.name = normalized;
  entry.line = 0;

  // Build message with metadata suffix
  if (metadata.is_object() && !metadata.empty()) {
    entry.msg = message + " " + metadata.dump();
  } else {
    entry.msg = message;
  }

  // Push to ring buffer (same path as source-emitted entries)
  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    auto & buf = buffers_[entry.name];
    buf.push_back(entry);
    if (buf.size() > max_buffer_size_) {
      buf.pop_front();
    }
  }

  // Notify observers if a notifier is set
  if (notifier_) {
    notifier_->notify("logs", normalized, "", entry_to_json(entry), ChangeType::CREATED);
  }
}

// ---------------------------------------------------------------------------
// set_notifier
// ---------------------------------------------------------------------------

void LogManager::set_notifier(ResourceChangeNotifier * notifier) {
  notifier_ = notifier;
}

void LogManager::set_node_to_entity_resolver(NodeToEntityFn resolver) {
  node_to_entity_resolver_ = std::move(resolver);
}

// ---------------------------------------------------------------------------
// effective_provider
// ---------------------------------------------------------------------------

LogProvider * LogManager::effective_provider() const {
  if (provider_registry_) {
    return provider_registry_->primary_log_provider();
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// get_logs
// ---------------------------------------------------------------------------

tl::expected<json, std::string> LogManager::get_logs(const std::vector<std::string> & node_fqns, bool prefix_match,
                                                     const std::string & min_severity,
                                                     const std::string & context_filter,
                                                     const std::string & entity_id) {
  // Delegate to plugin if one is registered
  if (auto * provider = effective_provider()) {
    // Normalize FQNs before passing to plugin (strip leading '/')
    std::vector<std::string> normalized;
    normalized.reserve(node_fqns.size());
    for (const auto & fqn : node_fqns) {
      normalized.push_back(normalize_fqn(fqn));
    }
    try {
      auto entries = provider->get_logs(normalized, prefix_match, min_severity, context_filter, entity_id);
      json items = json::array();
      for (const auto & e : entries) {
        items.push_back(entry_to_json(e));
      }
      return items;
    } catch (const std::exception & e) {
      std::cerr << "LogProvider::get_logs threw: " << e.what() << '\n';
      return tl::make_unexpected(std::string("LogProvider plugin error: ") + e.what());
    } catch (...) {
      std::cerr << "LogProvider::get_logs threw unknown exception\n";
      return tl::make_unexpected(std::string("LogProvider plugin error: unknown exception"));
    }
  }

  // Default: query local ring buffer
  // get_config() only fails on plugin errors; without a plugin (local path) it always succeeds.
  auto cfg_result = get_config(entity_id);
  if (!cfg_result) {
    return tl::make_unexpected(cfg_result.error());
  }
  LogConfig cfg = *cfg_result;

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
        // ROS 2 logger names use '.' as separator (e.g. "powertrain.engine.temp_sensor")
        // while entity FQNs use '/' (e.g. "powertrain/engine/temp_sensor").
        // Try both slash-format and dot-format so the default ring buffer matches either.
        std::string norm_dot = norm;
        std::replace(norm_dot.begin(), norm_dot.end(), '/', '.');
        if (prefix_match) {
          // Match exactly OR as a namespace prefix.
          // Slash-format: prefix must be followed by '/'
          // Dot-format:   prefix must be followed by '.'
          matches = (buf_name == norm) || (buf_name.rfind(norm + "/", 0) == 0) || (buf_name == norm_dot) ||
                    (buf_name.rfind(norm_dot + ".", 0) == 0);
        } else {
          matches = (buf_name == norm) || (buf_name == norm_dot);
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

tl::expected<LogConfig, std::string> LogManager::get_config(const std::string & entity_id) const {
  if (auto * provider = effective_provider()) {
    try {
      return provider->get_config(entity_id);
    } catch (const std::exception & e) {
      std::cerr << "LogProvider::get_config threw: " << e.what() << '\n';
      return tl::make_unexpected(std::string("LogProvider plugin error: ") + e.what());
    } catch (...) {
      std::cerr << "LogProvider::get_config threw unknown exception\n";
      return tl::make_unexpected(std::string("LogProvider plugin error: unknown exception"));
    }
  }

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
    try {
      return provider->update_config(entity_id, severity_filter, max_entries);
    } catch (const std::exception & e) {
      std::cerr << "LogProvider::update_config threw: " << e.what() << '\n';
      return std::string("LogProvider plugin error: ") + e.what();
    } catch (...) {
      std::cerr << "LogProvider::update_config threw unknown exception\n";
      return "LogProvider plugin error: unknown exception";
    }
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
