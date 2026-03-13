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

#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"
#include "ros2_medkit_linux_introspection/proc_reader.hpp"

#include <httplib.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <unistd.h>

using namespace ros2_medkit_gateway;  // NOLINT(build/namespaces)

class ProcfsPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "procfs_introspection";
  }

  void configure(const nlohmann::json & config) override {
    std::chrono::seconds ttl{10};
    if (config.contains("pid_cache_ttl_seconds")) {
      ttl = std::chrono::seconds{config["pid_cache_ttl_seconds"].get<int>()};
    }
    if (config.contains("proc_root")) {
      proc_root_ = config["proc_root"].get<std::string>();
    }
    pid_cache_ = std::make_unique<ros2_medkit_linux_introspection::PidCache>(ttl);
  }

  void set_context(PluginContext & ctx) override {
    ctx_ = &ctx;
    ctx.register_capability(SovdEntityType::APP, "x-medkit-procfs");
    ctx.register_capability(SovdEntityType::COMPONENT, "x-medkit-procfs");
  }

  void register_routes(httplib::Server & server, const std::string & api_prefix) override {
    // App-level endpoint
    server.Get((api_prefix + R"(/apps/([^/]+)/x-medkit-procfs)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_app_request(req, res);
               });

    // Component-level aggregation endpoint
    server.Get((api_prefix + R"(/components/([^/]+)/x-medkit-procfs)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_component_request(req, res);
               });
  }

  IntrospectionResult introspect(const IntrospectionInput & input) override {
    IntrospectionResult result;
    pid_cache_->refresh(proc_root_);

    for (const auto & app : input.apps) {
      auto fqn = app.effective_fqn();
      if (fqn.empty()) {
        continue;
      }

      auto pid_opt = pid_cache_->lookup(fqn, proc_root_);
      if (!pid_opt) {
        continue;
      }

      auto proc_info = ros2_medkit_linux_introspection::read_process_info(*pid_opt, proc_root_);
      if (!proc_info) {
        continue;
      }

      result.metadata[app.id] = process_info_to_json(*proc_info);
    }
    return result;
  }

 private:
  PluginContext * ctx_{nullptr};
  std::unique_ptr<ros2_medkit_linux_introspection::PidCache> pid_cache_ =
      std::make_unique<ros2_medkit_linux_introspection::PidCache>();
  std::string proc_root_{"/"};

  static nlohmann::json process_info_to_json(const ros2_medkit_linux_introspection::ProcessInfo & info) {
    // Calculate uptime from start_time_ticks
    long ticks_per_sec = sysconf(_SC_CLK_TCK);
    double uptime_sec = 0.0;
    if (ticks_per_sec > 0 && info.start_time_ticks > 0) {
      std::ifstream uptime_file("/proc/uptime");
      double system_uptime = 0.0;
      if (uptime_file >> system_uptime) {
        double start_sec = static_cast<double>(info.start_time_ticks) / static_cast<double>(ticks_per_sec);
        uptime_sec = system_uptime - start_sec;
        if (uptime_sec < 0) {
          uptime_sec = 0;
        }
      }
    }

    return {{"pid", info.pid},
            {"ppid", info.ppid},
            {"exe", info.exe_path},
            {"cmdline", info.cmdline},
            {"rss_bytes", info.rss_bytes},
            {"vm_size_bytes", info.vm_size_bytes},
            {"threads", info.num_threads},
            {"cpu_user_ticks", info.cpu_user_ticks},
            {"cpu_system_ticks", info.cpu_system_ticks},
            {"uptime_seconds", static_cast<uint64_t>(uptime_sec)}};
  }

  void handle_app_request(const httplib::Request & req, httplib::Response & res) {
    auto entity_id = req.matches[1].str();
    auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto pid_opt = pid_cache_->lookup(entity->fqn, proc_root_);
    if (!pid_opt) {
      PluginContext::send_error(res, 503, "x-medkit-pid-lookup-failed", "PID lookup failed for node " + entity->fqn);
      return;
    }

    auto proc_info = ros2_medkit_linux_introspection::read_process_info(*pid_opt, proc_root_);
    if (!proc_info) {
      PluginContext::send_error(res, 503, "x-medkit-proc-read-failed", proc_info.error());
      return;
    }

    PluginContext::send_json(res, process_info_to_json(*proc_info));
  }

  void handle_component_request(const httplib::Request & req, httplib::Response & res) {
    auto entity_id = req.matches[1].str();
    auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto child_apps = ctx_->get_child_apps(entity_id);
    std::map<pid_t, nlohmann::json> processes;  // Deduplicate by PID

    for (const auto & app : child_apps) {
      auto pid_opt = pid_cache_->lookup(app.fqn, proc_root_);
      if (!pid_opt) {
        continue;
      }

      if (processes.find(*pid_opt) == processes.end()) {
        auto proc_info = ros2_medkit_linux_introspection::read_process_info(*pid_opt, proc_root_);
        if (!proc_info) {
          continue;
        }
        auto j = process_info_to_json(*proc_info);
        j["node_ids"] = nlohmann::json::array();
        processes[*pid_opt] = std::move(j);
      }
      processes[*pid_opt]["node_ids"].push_back(app.id);
    }

    nlohmann::json result;
    result["processes"] = nlohmann::json::array();
    for (auto & [_, proc_json] : processes) {
      result["processes"].push_back(std::move(proc_json));
    }
    PluginContext::send_json(res, result);
  }
};

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}
extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new ProcfsPlugin();
}
extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * p) {
  return static_cast<ProcfsPlugin *>(p);
}
