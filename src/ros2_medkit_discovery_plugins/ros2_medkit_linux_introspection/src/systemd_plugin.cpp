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

#include "ros2_medkit_gateway/discovery/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "ros2_medkit_linux_introspection/plugin_config.hpp"
#include "ros2_medkit_linux_introspection/proc_reader.hpp"
#include "ros2_medkit_linux_introspection/systemd_utils.hpp"

#include <nlohmann/json.hpp>
#include <systemd/sd-bus.h>
#include <systemd/sd-login.h>

#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <string>

using namespace ros2_medkit_gateway;  // NOLINT(build/namespaces)

namespace {

// RAII wrapper for sd_bus
struct SdBusDeleter {
  void operator()(sd_bus * bus) const {
    sd_bus_unref(bus);
  }
};
using SdBusPtr = std::unique_ptr<sd_bus, SdBusDeleter>;

struct UnitInfo {
  std::string unit;
  std::string unit_type;
  std::string active_state;
  std::string sub_state;
  uint32_t restart_count{0};
  uint64_t watchdog_usec{0};
};

// Query unit properties via sd-bus
std::optional<UnitInfo> query_unit_info(const std::string & unit_name, sd_bus * shared_bus = nullptr) {
  SdBusPtr owned_bus;
  sd_bus * bus = shared_bus;
  if (!bus) {
    sd_bus * raw_bus = nullptr;
    if (sd_bus_open_system(&raw_bus) < 0) {
      return std::nullopt;
    }
    owned_bus.reset(raw_bus);
    bus = raw_bus;
  }

  UnitInfo info;
  info.unit = unit_name;

  auto dot_pos = unit_name.rfind('.');
  if (dot_pos != std::string::npos) {
    info.unit_type = unit_name.substr(dot_pos + 1);
  }

  auto obj_path = "/org/freedesktop/systemd1/unit/" + ros2_medkit_linux_introspection::escape_unit_for_dbus(unit_name);
  const char * iface = "org.freedesktop.systemd1.Unit";

  // ActiveState
  char * value = nullptr;
  if (sd_bus_get_property_string(bus, "org.freedesktop.systemd1", obj_path.c_str(), iface, "ActiveState", nullptr,
                                 &value) >= 0 &&
      value) {
    info.active_state = value;
    free(value);  // NOLINT(cppcoreguidelines-no-malloc)
  }

  // SubState
  value = nullptr;
  if (sd_bus_get_property_string(bus, "org.freedesktop.systemd1", obj_path.c_str(), iface, "SubState", nullptr,
                                 &value) >= 0 &&
      value) {
    info.sub_state = value;
    free(value);  // NOLINT(cppcoreguidelines-no-malloc)
  }

  // NRestarts (Service-specific property)
  const char * svc_iface = "org.freedesktop.systemd1.Service";
  sd_bus_error error = SD_BUS_ERROR_NULL;
  uint32_t nrestarts = 0;
  if (sd_bus_get_property_trivial(bus, "org.freedesktop.systemd1", obj_path.c_str(), svc_iface, "NRestarts", &error,
                                  'u', &nrestarts) >= 0) {
    info.restart_count = nrestarts;
  }
  sd_bus_error_free(&error);

  // WatchdogUSec (Service-specific property)
  uint64_t watchdog = 0;
  error = SD_BUS_ERROR_NULL;
  if (sd_bus_get_property_trivial(bus, "org.freedesktop.systemd1", obj_path.c_str(), svc_iface, "WatchdogUSec", &error,
                                  't', &watchdog) >= 0) {
    info.watchdog_usec = watchdog;
  }
  sd_bus_error_free(&error);

  return info;
}

}  // namespace

class SystemdPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "systemd_introspection";
  }

  void configure(const nlohmann::json & config) override {
    auto cfg = ros2_medkit_linux_introspection::parse_introspection_config(config);
    pid_cache_ = std::move(cfg.pid_cache);
    proc_root_ = std::move(cfg.proc_root);
  }

  void set_context(PluginContext & ctx) override {
    ctx_ = &ctx;
    ctx.register_capability(SovdEntityType::APP, "x-medkit-systemd");
    ctx.register_capability(SovdEntityType::COMPONENT, "x-medkit-systemd");
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-medkit-systemd)",
         [this](const PluginRequest & req, PluginResponse & res) {
           handle_app_request(req, res);
         }},
        {"GET", R"(components/([^/]+)/x-medkit-systemd)",
         [this](const PluginRequest & req, PluginResponse & res) {
           handle_component_request(req, res);
         }},
    };
  }

  IntrospectionResult introspect(const IntrospectionInput & input) override {
    IntrospectionResult result;
    pid_cache_->refresh(proc_root_);

    sd_bus * raw_bus = nullptr;
    SdBusPtr request_bus;
    if (sd_bus_open_system(&raw_bus) >= 0) {
      request_bus.reset(raw_bus);
    }

    for (const auto & app : input.apps) {
      auto fqn = app.effective_fqn();
      if (fqn.empty()) {
        continue;
      }

      auto pid_opt = pid_cache_->lookup(fqn, proc_root_);
      if (!pid_opt) {
        continue;
      }

      char * unit_cstr = nullptr;
      if (sd_pid_get_unit(*pid_opt, &unit_cstr) < 0 || !unit_cstr) {
        continue;
      }
      std::string unit_name(unit_cstr);
      free(unit_cstr);  // NOLINT(cppcoreguidelines-no-malloc)

      auto unit_info = query_unit_info(unit_name, request_bus.get());
      if (!unit_info) {
        continue;
      }

      result.metadata[app.id] = unit_info_to_json(*unit_info);
    }
    return result;
  }

 private:
  PluginContext * ctx_{nullptr};
  std::unique_ptr<ros2_medkit_linux_introspection::PidCache> pid_cache_ =
      std::make_unique<ros2_medkit_linux_introspection::PidCache>();
  std::string proc_root_{"/"};

  static nlohmann::json unit_info_to_json(const UnitInfo & info) {
    return {
        {"unit", info.unit},           {"unit_type", info.unit_type},         {"active_state", info.active_state},
        {"sub_state", info.sub_state}, {"restart_count", info.restart_count}, {"watchdog_usec", info.watchdog_usec}};
  }

  void handle_app_request(const PluginRequest & req, PluginResponse & res) {
    auto entity_id = req.path_param(1);
    auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto pid_opt = pid_cache_->lookup(entity->fqn, proc_root_);
    if (!pid_opt) {
      res.send_error(404, "x-medkit-pid-lookup-failed", "Process not found for entity " + entity_id);
      return;
    }

    char * unit_cstr = nullptr;
    if (sd_pid_get_unit(*pid_opt, &unit_cstr) < 0 || !unit_cstr) {
      res.send_error(404, "x-medkit-not-in-systemd-unit", "Entity " + entity_id + " is not managed by a systemd unit");
      return;
    }
    std::string unit_name(unit_cstr);
    free(unit_cstr);  // NOLINT(cppcoreguidelines-no-malloc)

    auto unit_info = query_unit_info(unit_name);
    if (!unit_info) {
      res.send_error(503, "x-medkit-systemd-query-failed",
                     "Failed to query systemd properties for entity " + entity_id);
      return;
    }

    res.send_json(unit_info_to_json(*unit_info));
  }

  void handle_component_request(const PluginRequest & req, PluginResponse & res) {
    auto entity_id = req.path_param(1);
    auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto child_apps = ctx_->get_child_apps(entity_id);
    std::map<std::string, nlohmann::json> units;  // Deduplicate by unit name

    sd_bus * raw_bus = nullptr;
    SdBusPtr request_bus;
    if (sd_bus_open_system(&raw_bus) >= 0) {
      request_bus.reset(raw_bus);
    }

    for (const auto & app : child_apps) {
      auto pid_opt = pid_cache_->lookup(app.fqn, proc_root_);
      if (!pid_opt) {
        continue;
      }

      char * unit_cstr = nullptr;
      if (sd_pid_get_unit(*pid_opt, &unit_cstr) < 0 || !unit_cstr) {
        continue;
      }
      std::string unit_name(unit_cstr);
      free(unit_cstr);  // NOLINT(cppcoreguidelines-no-malloc)

      if (units.find(unit_name) == units.end()) {
        auto unit_info = query_unit_info(unit_name, request_bus.get());
        if (!unit_info) {
          continue;
        }
        auto j = unit_info_to_json(*unit_info);
        j["node_ids"] = nlohmann::json::array();
        units[unit_name] = std::move(j);
      }
      units[unit_name]["node_ids"].push_back(app.id);
    }

    nlohmann::json result;
    result["units"] = nlohmann::json::array();
    for (auto & [_, unit_json] : units) {
      result["units"].push_back(std::move(unit_json));
    }
    res.send_json(result);
  }
};

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}
extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new SystemdPlugin();
}
extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * p) {
  return static_cast<SystemdPlugin *>(p);
}
