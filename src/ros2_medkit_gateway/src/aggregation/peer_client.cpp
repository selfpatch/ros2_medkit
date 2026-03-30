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
 * @brief Parse an App from JSON
 */
App parse_app(const nlohmann::json & j) {
  App app;
  app.id = j.value("id", "");
  app.name = j.value("name", "");
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    app.component_id = xm.value("componentId", "");
    app.source = xm.value("source", "");
    app.description = xm.value("description", "");
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

PeerClient::PeerClient(const std::string & url, const std::string & name, int timeout_ms)
  : url_(url), name_(name), timeout_ms_(timeout_ms) {
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
    for (auto & area : entities.areas) {
      area.source = peer_source;
    }
  }

  // Fetch components
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
    entities.components = parse_collection<Component>(response_json, parse_component);
    for (auto & comp : entities.components) {
      comp.source = peer_source;
    }
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
    for (auto & app : entities.apps) {
      app.source = peer_source;
    }
  }

  // Fetch functions
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
    entities.functions = parse_collection<Function>(response_json, parse_function);
    for (auto & func : entities.functions) {
      func.source = peer_source;
    }
  }

  return entities;
}

void PeerClient::forward_request(const httplib::Request & req, httplib::Response & res) {
  std::lock_guard<std::mutex> lock(client_mutex_);
  ensure_client();

  httplib::Headers headers;
  // Forward Authorization header if present
  if (req.has_header("Authorization")) {
    headers.emplace("Authorization", req.get_header_value("Authorization"));
  }

  httplib::Result result{nullptr, httplib::Error::Unknown};
  const std::string & path = req.path;
  const std::string content_type = req.get_header_value("Content-Type");

  if (req.method == "GET") {
    result = client_->Get(path, headers);
  } else if (req.method == "POST") {
    result = client_->Post(path, headers, req.body, content_type);
  } else if (req.method == "PUT") {
    result = client_->Put(path, headers, req.body, content_type);
  } else if (req.method == "DELETE") {
    result = client_->Delete(path, headers);
  } else if (req.method == "PATCH") {
    result = client_->Patch(path, headers, req.body, content_type);
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

  // Copy response from peer - only forward safe headers
  static const std::set<std::string> allowed_headers = {"content-type", "etag", "cache-control", "last-modified"};

  res.status = result->status;
  res.body = result->body;
  for (const auto & header : result->headers) {
    std::string lower_name = header.first;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
    if (allowed_headers.count(lower_name) > 0 || lower_name.find("x-medkit") == 0) {
      res.set_header(header.first, header.second);
    }
  }
}

tl::expected<nlohmann::json, std::string> PeerClient::forward_and_get_json(const std::string & method,
                                                                           const std::string & path,
                                                                           const std::string & auth_header) {
  std::lock_guard<std::mutex> lock(client_mutex_);
  ensure_client();

  httplib::Headers headers;
  if (!auth_header.empty()) {
    headers.emplace("Authorization", auth_header);
  }

  httplib::Result result{nullptr, httplib::Error::Unknown};

  if (method == "GET") {
    result = client_->Get(path, headers);
  } else if (method == "POST") {
    result = client_->Post(path, headers, "", "application/json");
  } else if (method == "PUT") {
    result = client_->Put(path, headers, "", "application/json");
  } else if (method == "DELETE") {
    result = client_->Delete(path, headers);
  }

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

  auto parsed = nlohmann::json::parse(result->body, nullptr, false);
  if (parsed.is_discarded()) {
    return tl::unexpected<std::string>("Invalid JSON response from peer '" + name_ + "' for " + method + " " + path);
  }

  return parsed;
}

}  // namespace ros2_medkit_gateway
