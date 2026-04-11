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

#include "ros2_medkit_gateway/aggregation/peer_client.hpp"

#include <algorithm>
#include <cctype>
#include <iterator>
#include <set>
#include <string>
#include <utility>

#include "ros2_medkit_gateway/http/error_codes.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Vendor error code for peer unavailable (connection failure)
constexpr const char * ERR_X_MEDKIT_PEER_UNAVAILABLE = "x-medkit-peer-unavailable";

/// API prefix for SOVD endpoints
constexpr const char * API_PREFIX = "/api/v1";

/// Maximum response body size to accept from peers (10MB)
constexpr size_t MAX_PEER_RESPONSE_SIZE = 10 * 1024 * 1024;

/// Maximum number of entities per collection (areas, components, apps, functions)
/// before detail fetches are skipped to prevent excessive HTTP requests.
constexpr size_t MAX_ENTITIES_PER_COLLECTION = 1000;

/**
 * @brief Percent-encode a query parameter key or value (RFC 3986)
 *
 * Unreserved characters (A-Z, a-z, 0-9, '-', '.', '_', '~') pass through;
 * everything else is encoded as %XX. This avoids depending on
 * httplib::detail internals.
 */
std::string encode_query_param(const std::string & value) {
  std::string result;
  result.reserve(value.size());
  for (char ch : value) {
    auto c = static_cast<unsigned char>(ch);
    if (std::isalnum(c) || c == '-' || c == '.' || c == '_' || c == '~') {
      result += static_cast<char>(c);
    } else {
      static const char hex[] = "0123456789ABCDEF";
      result += '%';
      result += hex[c >> 4];
      result += hex[c & 0x0F];
    }
  }
  return result;
}

/**
 * @brief Reconstruct path with query string from httplib request
 *
 * httplib::Request::path does not include the query string; query parameters
 * are stored separately in req.params. This helper reconstructs the full
 * request target (path + "?key=val&...") so forwarded requests preserve
 * filtering/pagination parameters.
 */
std::string path_with_query(const httplib::Request & req) {
  if (req.params.empty()) {
    return req.path;
  }
  std::string result = req.path + "?";
  bool first = true;
  for (const auto & param : req.params) {
    if (!first) {
      result += "&";
    }
    result += encode_query_param(param.first);
    result += "=";
    result += encode_query_param(param.second);
    first = false;
  }
  return result;
}

/**
 * @brief Validate an entity ID for safe use in URL paths
 *
 * Rejects IDs with path traversal characters (/, ..), null bytes, or other
 * characters that could be used for SSRF or path injection. Matches the same
 * rules as HandlerContext::validate_entity_id: alphanumeric + underscore + hyphen,
 * max 256 chars.
 */
bool is_valid_entity_id(const std::string & id) {
  if (id.empty() || id.size() > 256) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
  });
}

/**
 * @brief Build a SOVD GenericError JSON body
 */
nlohmann::json make_error_body(const std::string & error_code, const std::string & message,
                               const std::string & vendor_code = "") {
  nlohmann::json body;
  if (!vendor_code.empty()) {
    body["error_code"] = ERR_VENDOR_ERROR;
    body["vendor_code"] = vendor_code;
  } else {
    body["error_code"] = error_code;
  }
  body["message"] = message;
  return body;
}

/**
 * @brief Parse an entity collection from a JSON response
 *
 * Expects the response to contain an "items" array. Each item is parsed
 * by the provided parser function.
 */
template <typename T, typename Parser>
std::vector<T> parse_collection(const nlohmann::json & response_json, Parser parser) {
  std::vector<T> result;
  if (response_json.contains("items") && response_json["items"].is_array()) {
    for (const auto & item : response_json["items"]) {
      result.push_back(parser(item));
    }
  }
  return result;
}

/**
 * @brief Parse an Area from JSON
 */
Area parse_area(const nlohmann::json & j) {
  Area area;
  area.id = j.value("id", "");
  area.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    area.namespace_path = xm.value("namespace", "");
    area.description = xm.value("description", "");
    area.source = xm.value("source", "");
  }
  if (j.contains("translationId")) {
    area.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    area.tags = j["tags"].get<std::vector<std::string>>();
  }
  return area;
}

/**
 * @brief Parse a Component from JSON
 */
Component parse_component(const nlohmann::json & j) {
  Component comp;
  comp.id = j.value("id", "");
  comp.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    comp.namespace_path = xm.value("namespace", "");
    comp.fqn = xm.value("fqn", "");
    comp.area = xm.value("area", "");
    comp.source = xm.value("source", "");
    comp.description = xm.value("description", "");
    comp.variant = xm.value("variant", "");
    comp.parent_component_id = xm.value("parentComponentId", "");
    if (xm.contains("dependsOn") && xm["dependsOn"].is_array()) {
      comp.depends_on = xm["dependsOn"].get<std::vector<std::string>>();
    }
  }
  if (j.contains("translationId")) {
    comp.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    comp.tags = j["tags"].get<std::vector<std::string>>();
  }
  return comp;
}

/**
 * @brief Extract the component ID from an ``is-located-on`` URI.
 *
 * SOVD exposes the app-to-component binding as a standard relationship
 * URI (ISO 17978-3, §7.6). Peers may emit it as either an absolute URL
 * (``http://host:port/api/v1/components/{id}``) or a path-only reference
 * (``/api/v1/components/{id}``). Extract the trailing segment after the
 * ``/components/`` marker.
 */
std::string component_id_from_located_on(const std::string & uri) {
  static const std::string kMarker = "/components/";
  auto pos = uri.rfind(kMarker);
  if (pos == std::string::npos) {
    return "";
  }
  auto id_start = pos + kMarker.size();
  if (id_start >= uri.size()) {
    return "";
  }
  auto id_end = uri.find_first_of("/?#", id_start);
  return uri.substr(id_start, id_end - id_start);
}

/**
 * @brief Parse an App from JSON.
 *
 * The app-to-component binding is recovered from SOVD's standard
 * ``is-located-on`` relationship (body field or ``_links`` entry),
 * which every SOVD-compliant peer emits. Vendor fallback:
 * ``x-medkit.component_id`` (gateway's own extension).
 */
App parse_app(const nlohmann::json & j) {
  App app;
  app.id = j.value("id", "");
  app.name = j.value("name", "");
  app.description = j.value("description", "");

  // SOVD-standard: is-located-on relationship (ISO 17978-3, §7.6)
  if (j.contains("is-located-on") && j["is-located-on"].is_string()) {
    app.component_id = component_id_from_located_on(j["is-located-on"].get<std::string>());
  }
  if (app.component_id.empty() && j.contains("_links") && j["_links"].is_object()) {
    const auto & links = j["_links"];
    if (links.contains("is-located-on") && links["is-located-on"].is_string()) {
      app.component_id = component_id_from_located_on(links["is-located-on"].get<std::string>());
    }
  }

  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    // Vendor fallback: gateway emits x-medkit.component_id (snake_case) via
    // XMedkit builder in discovery_handlers.cpp. Only used if the SOVD
    // standard is-located-on field is absent.
    if (app.component_id.empty()) {
      app.component_id = xm.value("component_id", "");
    }
    app.source = xm.value("source", "");
    app.is_online = xm.value("is_online", false);
    if (app.description.empty()) {
      app.description = xm.value("description", "");
    }
  }
  if (j.contains("translationId")) {
    app.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    app.tags = j["tags"].get<std::vector<std::string>>();
  }
  return app;
}

/**
 * @brief Parse a Function from JSON
 */
Function parse_function(const nlohmann::json & j) {
  Function func;
  func.id = j.value("id", "");
  func.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    func.source = xm.value("source", "");
    func.description = xm.value("description", "");
    if (xm.contains("hosts") && xm["hosts"].is_array()) {
      func.hosts = xm["hosts"].get<std::vector<std::string>>();
    }
  }
  if (j.contains("translationId")) {
    func.translation_id = j["translationId"].get<std::string>();
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    func.tags = j["tags"].get<std::vector<std::string>>();
  }
  return func;
}

}  // namespace

PeerClient::PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth)
  : url_(url), name_(name), timeout_ms_(timeout_ms), forward_auth_(forward_auth) {
}

const std::string & PeerClient::url() const {
  return url_;
}

const std::string & PeerClient::name() const {
  return name_;
}

bool PeerClient::is_healthy() const {
  return healthy_.load();
}

void PeerClient::ensure_client() {
  if (!client_) {
    client_ = std::make_unique<httplib::Client>(url_);
    client_->set_connection_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);
    client_->set_read_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);
    // Note: cpp-httplib Client does not expose set_payload_max_length (server-only).
    // Response size is enforced post-download in forward_request() and
    // forward_and_get_json() via MAX_PEER_RESPONSE_SIZE body length checks.
    // The read timeout provides a secondary defense against slow-drip attacks.
  }
}

void PeerClient::check_health() {
  std::lock_guard<std::mutex> lock(client_mutex_);
  ensure_client();
  auto result = client_->Get(std::string(API_PREFIX) + "/health");
  healthy_.store(result && result->status == 200);
}

tl::expected<PeerEntities, std::string> PeerClient::fetch_entities() {
  // Use a dedicated client for this long-running operation (4 sequential HTTP
  // requests, up to 8s with 2s timeout) to avoid blocking health checks and
  // forwarding on the shared client_mutex_.
  httplib::Client cli(url_);
  cli.set_connection_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);
  cli.set_read_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);

  PeerEntities entities;
  const std::string peer_source = "peer:" + name_;

  // Fetch areas
  {
    auto result = cli.Get(std::string(API_PREFIX) + "/areas");
    if (!result) {
      return tl::unexpected<std::string>("Failed to connect to peer '" + name_ + "' at " + url_);
    }
    if (result->status != 200) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned status " + std::to_string(result->status) +
                                         " for /areas");
    }
    if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
      return tl::unexpected<std::string>("Response from peer '" + name_ + "' for /areas exceeds size limit");
    }
    auto response_json = nlohmann::json::parse(result->body, nullptr, false);
    if (response_json.is_discarded()) {
      return tl::unexpected<std::string>("Invalid JSON from peer '" + name_ + "' for /areas");
    }
    entities.areas = parse_collection<Area>(response_json, parse_area);
    // Validate entity IDs and enforce per-collection limit
    entities.areas.erase(std::remove_if(entities.areas.begin(), entities.areas.end(),
                                        [](const Area & a) {
                                          return !is_valid_entity_id(a.id);
                                        }),
                         entities.areas.end());
    if (entities.areas.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(entities.areas.size()) +
                                         " areas (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    for (auto & area : entities.areas) {
      area.source = peer_source;
    }

    // Fetch subareas for each top-level area (list endpoint filters them out).
    // Collect into a separate vector first to avoid push_back during iteration
    // (which can invalidate references if the vector reallocates).
    std::vector<Area> all_subareas;
    for (const auto & area : entities.areas) {
      auto sub_result = cli.Get(std::string(API_PREFIX) + "/areas/" + area.id + "/subareas");
      if (sub_result && sub_result->status == 200 && sub_result->body.size() <= MAX_PEER_RESPONSE_SIZE) {
        auto sub_json = nlohmann::json::parse(sub_result->body, nullptr, false);
        if (!sub_json.is_discarded()) {
          auto subareas = parse_collection<Area>(sub_json, parse_area);
          for (auto & sub : subareas) {
            if (!is_valid_entity_id(sub.id)) {
              continue;
            }
            sub.source = peer_source;
            all_subareas.push_back(std::move(sub));
          }
        }
      }
    }
    entities.areas.insert(entities.areas.end(), std::make_move_iterator(all_subareas.begin()),
                          std::make_move_iterator(all_subareas.end()));
  }

  // Fetch components (list then detail per entity for full relationship data)
  {
    auto result = cli.Get(std::string(API_PREFIX) + "/components");
    if (!result) {
      return tl::unexpected<std::string>("Failed to connect to peer '" + name_ + "' at " + url_);
    }
    if (result->status != 200) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned status " + std::to_string(result->status) +
                                         " for /components");
    }
    if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
      return tl::unexpected<std::string>("Response from peer '" + name_ + "' for /components exceeds size limit");
    }
    auto response_json = nlohmann::json::parse(result->body, nullptr, false);
    if (response_json.is_discarded()) {
      return tl::unexpected<std::string>("Invalid JSON from peer '" + name_ + "' for /components");
    }
    // Parse IDs from list, then fetch detail per entity for relationships
    auto comp_list = parse_collection<Component>(response_json, parse_component);
    // Validate entity IDs and enforce per-collection limit
    comp_list.erase(std::remove_if(comp_list.begin(), comp_list.end(),
                                   [](const Component & c) {
                                     return !is_valid_entity_id(c.id);
                                   }),
                    comp_list.end());
    if (comp_list.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(comp_list.size()) +
                                         " components (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    for (auto & comp : comp_list) {
      auto detail = cli.Get(std::string(API_PREFIX) + "/components/" + comp.id);
      if (detail && detail->status == 200) {
        auto detail_json = nlohmann::json::parse(detail->body, nullptr, false);
        if (!detail_json.is_discarded()) {
          comp = parse_component(detail_json);
        }
      }
      comp.source = peer_source;
    }
    // Fetch subcomponents for each top-level component (list endpoint filters them out).
    // Collect into a separate vector first to avoid push_back during iteration
    // (which can invalidate references if the vector reallocates).
    std::vector<Component> all_subcomps;
    for (const auto & comp : comp_list) {
      auto sub_result = cli.Get(std::string(API_PREFIX) + "/components/" + comp.id + "/subcomponents");
      if (sub_result && sub_result->status == 200 && sub_result->body.size() <= MAX_PEER_RESPONSE_SIZE) {
        auto sub_json = nlohmann::json::parse(sub_result->body, nullptr, false);
        if (!sub_json.is_discarded()) {
          auto subcomps = parse_collection<Component>(sub_json, parse_component);
          for (auto & sub : subcomps) {
            if (!is_valid_entity_id(sub.id)) {
              continue;
            }
            // Fetch detail for each subcomponent to get full relationships
            auto detail = cli.Get(std::string(API_PREFIX) + "/components/" + sub.id);
            if (detail && detail->status == 200) {
              auto detail_json = nlohmann::json::parse(detail->body, nullptr, false);
              if (!detail_json.is_discarded()) {
                sub = parse_component(detail_json);
              }
            }
            sub.source = peer_source;
            all_subcomps.push_back(std::move(sub));
          }
        }
      }
    }
    comp_list.insert(comp_list.end(), std::make_move_iterator(all_subcomps.begin()),
                     std::make_move_iterator(all_subcomps.end()));

    entities.components = std::move(comp_list);
  }

  // Fetch apps
  {
    auto result = cli.Get(std::string(API_PREFIX) + "/apps");
    if (!result) {
      return tl::unexpected<std::string>("Failed to connect to peer '" + name_ + "' at " + url_);
    }
    if (result->status != 200) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned status " + std::to_string(result->status) +
                                         " for /apps");
    }
    if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
      return tl::unexpected<std::string>("Response from peer '" + name_ + "' for /apps exceeds size limit");
    }
    auto response_json = nlohmann::json::parse(result->body, nullptr, false);
    if (response_json.is_discarded()) {
      return tl::unexpected<std::string>("Invalid JSON from peer '" + name_ + "' for /apps");
    }
    entities.apps = parse_collection<App>(response_json, parse_app);
    // Validate entity IDs and enforce per-collection limit
    entities.apps.erase(std::remove_if(entities.apps.begin(), entities.apps.end(),
                                       [](const App & a) {
                                         return !is_valid_entity_id(a.id);
                                       }),
                        entities.apps.end());
    if (entities.apps.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(entities.apps.size()) +
                                         " apps (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    for (auto & app : entities.apps) {
      app.source = peer_source;
    }
    // Filter ROS 2 internal nodes (underscore prefix convention) at source.
    // These are noise nodes like _param_client_node that should never appear
    // as SOVD entities.
    entities.apps.erase(std::remove_if(entities.apps.begin(), entities.apps.end(),
                                       [](const App & app) {
                                         return !app.id.empty() && app.id[0] == '_';
                                       }),
                        entities.apps.end());
  }

  // Fetch functions (list then detail per entity for hosts data)
  {
    auto result = cli.Get(std::string(API_PREFIX) + "/functions");
    if (!result) {
      return tl::unexpected<std::string>("Failed to connect to peer '" + name_ + "' at " + url_);
    }
    if (result->status != 200) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned status " + std::to_string(result->status) +
                                         " for /functions");
    }
    if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
      return tl::unexpected<std::string>("Response from peer '" + name_ + "' for /functions exceeds size limit");
    }
    auto response_json = nlohmann::json::parse(result->body, nullptr, false);
    if (response_json.is_discarded()) {
      return tl::unexpected<std::string>("Invalid JSON from peer '" + name_ + "' for /functions");
    }
    // Parse IDs from list, then fetch detail per entity for hosts
    auto func_list = parse_collection<Function>(response_json, parse_function);
    // Validate entity IDs and enforce per-collection limit
    func_list.erase(std::remove_if(func_list.begin(), func_list.end(),
                                   [](const Function & f) {
                                     return !is_valid_entity_id(f.id);
                                   }),
                    func_list.end());
    if (func_list.size() > MAX_ENTITIES_PER_COLLECTION) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(func_list.size()) +
                                         " functions (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    for (auto & func : func_list) {
      auto detail = cli.Get(std::string(API_PREFIX) + "/functions/" + func.id);
      if (detail && detail->status == 200) {
        auto detail_json = nlohmann::json::parse(detail->body, nullptr, false);
        if (!detail_json.is_discarded()) {
          func = parse_function(detail_json);
        }
      }
      func.source = peer_source;
    }
    entities.functions = std::move(func_list);
  }

  return entities;
}

void PeerClient::forward_request(const httplib::Request & req, httplib::Response & res) {
  // Create a dedicated client per forwarding call to avoid holding client_mutex_
  // during potentially long I/O operations. The shared client_ is reserved for
  // short health checks only.
  httplib::Client cli(url_);
  cli.set_connection_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);
  cli.set_read_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);

  httplib::Headers headers;
  // Forward Authorization header only when explicitly enabled (forward_auth).
  // Default is off to prevent token leakage to untrusted/mDNS-discovered peers.
  if (forward_auth_ && req.has_header("Authorization")) {
    headers.emplace("Authorization", req.get_header_value("Authorization"));
  }

  httplib::Result result{nullptr, httplib::Error::Unknown};
  const std::string path = path_with_query(req);
  const std::string content_type = req.get_header_value("Content-Type");

  if (req.method == "GET") {
    result = cli.Get(path, headers);
  } else if (req.method == "POST") {
    result = cli.Post(path, headers, req.body, content_type);
  } else if (req.method == "PUT") {
    result = cli.Put(path, headers, req.body, content_type);
  } else if (req.method == "DELETE") {
    result = cli.Delete(path, headers);
  } else if (req.method == "PATCH") {
    result = cli.Patch(path, headers, req.body, content_type);
  }

  if (!result) {
    res.status = 502;
    auto error_body = make_error_body(ERR_VENDOR_ERROR, "Peer '" + name_ + "' at " + url_ + " is unavailable",
                                      ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
    res.status = 502;
    auto error_body = make_error_body(ERR_VENDOR_ERROR, "Response from peer '" + name_ + "' exceeds size limit",
                                      ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Copy response from peer - only forward safe headers.
  // The x-medkit header allowlist must match headers the gateway actually produces.
  // Currently the only x-medkit HTTP header is X-Medkit-Local-Only (fault_handlers.cpp).
  // Update this list when adding new x-medkit HTTP response headers.
  static const std::set<std::string> allowed_headers = {"content-type", "etag", "cache-control", "last-modified",
                                                        "x-medkit-local-only"};

  res.status = result->status;
  res.body = result->body;
  for (const auto & header : result->headers) {
    std::string lower_name = header.first;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), [](unsigned char c) {
      return std::tolower(c);
    });
    if (allowed_headers.count(lower_name) > 0) {
      res.set_header(header.first, header.second);
    }
  }
}

tl::expected<nlohmann::json, std::string> PeerClient::forward_and_get_json(const std::string & method,
                                                                           const std::string & path,
                                                                           const std::string & auth_header) {
  // Create a dedicated client per call to avoid holding client_mutex_ during I/O.
  // The shared client_ is reserved for short health checks only.
  httplib::Client cli(url_);
  cli.set_connection_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);
  cli.set_read_timeout(timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000);

  httplib::Headers headers;
  // Only forward auth header when forward_auth is enabled
  if (forward_auth_ && !auth_header.empty()) {
    headers.emplace("Authorization", auth_header);
  }

  // Accumulate body with streaming size enforcement via ContentReceiver.
  // This prevents a malicious peer from pushing hundreds of MB before the
  // post-download check. The receiver returns false to abort the download
  // as soon as MAX_PEER_RESPONSE_SIZE is exceeded.
  std::string accumulated_body;
  bool size_exceeded = false;
  int response_status = 0;

  auto content_receiver = [&](const char * data, size_t data_length) -> bool {
    if (accumulated_body.size() + data_length > MAX_PEER_RESPONSE_SIZE) {
      size_exceeded = true;
      return false;  // Abort download
    }
    accumulated_body.append(data, data_length);
    return true;
  };

  httplib::Result result{nullptr, httplib::Error::Unknown};

  if (method == "GET") {
    result = cli.Get(
        path, headers,
        [&response_status](const httplib::Response & resp) -> bool {
          response_status = resp.status;
          return true;  // Continue to receive body
        },
        content_receiver);
  } else if (method == "POST") {
    result = cli.Post(path, headers, "", "application/json");
  } else if (method == "PUT") {
    result = cli.Put(path, headers, "", "application/json");
  } else if (method == "DELETE") {
    result = cli.Delete(path, headers);
  }

  // For non-GET methods, fall back to post-download size check since
  // cpp-httplib does not offer ContentReceiver overloads for all methods.
  bool used_streaming = (method == "GET");

  if (used_streaming) {
    if (size_exceeded) {
      return tl::unexpected<std::string>("Response from peer '" + name_ + "' exceeds size limit for " + method + " " +
                                         path);
    }
    if (!result) {
      return tl::unexpected<std::string>("Failed to connect to peer '" + name_ + "' at " + url_);
    }
    if (response_status < 200 || response_status >= 300) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned status " + std::to_string(response_status) +
                                         " for " + method + " " + path);
    }
  } else {
    if (!result) {
      return tl::unexpected<std::string>("Failed to connect to peer '" + name_ + "' at " + url_);
    }
    if (result->status < 200 || result->status >= 300) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned status " + std::to_string(result->status) +
                                         " for " + method + " " + path);
    }
    if (result->body.size() > MAX_PEER_RESPONSE_SIZE) {
      return tl::unexpected<std::string>("Response from peer '" + name_ + "' exceeds size limit for " + method + " " +
                                         path);
    }
    accumulated_body = std::move(result->body);
  }

  auto parsed = nlohmann::json::parse(accumulated_body, nullptr, false);
  if (parsed.is_discarded()) {
    return tl::unexpected<std::string>("Invalid JSON response from peer '" + name_ + "' for " + method + " " + path);
  }

  return parsed;
}

}  // namespace ros2_medkit_gateway
