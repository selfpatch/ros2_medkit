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

#include "ros2_medkit_gateway/core/aggregation/peer_client.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iterator>
#include <map>
#include <set>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"

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

/// Deepest level of subareas or subcomponents read below a top-level Area or
/// Component. The top-level lists leave nested entities out, so each level
/// costs one request per entity of the level above; the bound keeps a deep or
/// cyclic tree from turning one refresh into an unbounded walk.
constexpr size_t MAX_NESTING_DEPTH = 8;

/// Most operations, and most data items naming a topic, kept for one App a
/// peer reports. Each data item names one topic, so an App records at most
/// this many topics it publishes and this many it subscribes to. The largest
/// App of a gateway serving the demo nodes has 1 operation and 4 such data
/// items.
constexpr size_t MAX_PEER_ENTRIES_PER_APP = 1000;

/// Most dependencies kept for one Component or Function a peer reports: no
/// more than the entities of one collection there can be to depend on.
constexpr size_t MAX_PEER_DEPENDENCIES = MAX_ENTITIES_PER_COLLECTION;

/// Most records, and most bytes of text in them, kept from one peer in one
/// refresh. A record is an entity, an operation, a topic an App publishes or
/// subscribes to, or one entry of a list an entity carries (tags, hosts,
/// dependencies, identity fields). Every response of a refresh stays within
/// the size and value bounds, but a refresh makes one request per entity, and
/// 1000 Apps answering 100000 operations each would keep some 100 million
/// records. A gateway serving the demo nodes reports 16 entities, 3 operations
/// and 33 topics, in under 1 kB of text; a peer of 245 Apps with 1000
/// operations each, just inside the record bound, took the gateway to about
/// 200 MB.
constexpr size_t MAX_PEER_FETCH_RECORDS = 250 * 1000;
constexpr size_t MAX_PEER_FETCH_TEXT = 16 * 1024 * 1024;

/// A peer's answer that is past one of the bounds above: what it does, after
/// the peer's name. Thrown by the parsers, where the bound is reached, and
/// turned into the fetch's failure by fetch_entities.
class PeerFetchRefused : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// What one record costs against MAX_PEER_FETCH_RECORDS and MAX_PEER_FETCH_TEXT.
struct RecordCost {
  size_t records{0};
  size_t text{0};
};

size_t text_of(const std::vector<std::string> & strings) {
  size_t text = 0;
  for (const auto & s : strings) {
    text += s.size();
  }
  return text;
}

size_t text_of(const std::map<std::string, std::string> & fields) {
  size_t text = 0;
  for (const auto & [key, value] : fields) {
    text += key.size() + value.size();
  }
  return text;
}

/// The records and text an entity keeps of what its peer said. An App's
/// operations and topics are charged one by one as they are read.
RecordCost cost_of(const Area & a) {
  return {1 + a.tags.size(), a.id.size() + a.name.size() + a.description.size() + a.namespace_path.size() +
                                 a.source.size() + a.parent_area_id.size() + a.translation_id.size() + text_of(a.tags)};
}

RecordCost cost_of(const Component & c) {
  const auto & id = c.identity;
  return {1 + c.tags.size() + c.depends_on.size() + id.extra.size() + id.provenance.size(),
          c.id.size() + c.name.size() + c.description.size() + c.namespace_path.size() + c.fqn.size() + c.area.size() +
              c.source.size() + c.variant.size() + c.parent_component_id.size() + c.translation_id.size() +
              text_of(c.tags) + text_of(c.depends_on) + id.manufacturer.size() + id.model.size() +
              id.order_code.size() + id.serial_number.size() + id.hardware_revision.size() +
              id.firmware_version.size() + id.software_version.size() + id.network_endpoint.size() + id.role.size() +
              text_of(id.extra) + text_of(id.provenance)};
}

RecordCost cost_of(const App & a) {
  return {1 + a.tags.size(), a.id.size() + a.name.size() + a.description.size() + a.component_id.size() +
                                 a.source.size() + a.translation_id.size() + text_of(a.tags)};
}

RecordCost cost_of(const Function & f) {
  return {1 + f.tags.size() + f.hosts.size() + f.depends_on.size(),
          f.id.size() + f.name.size() + f.description.size() + f.source.size() + f.translation_id.size() +
              text_of(f.tags) + text_of(f.hosts) + text_of(f.depends_on)};
}

/// What one refresh has kept of one peer, against MAX_PEER_FETCH_RECORDS and
/// MAX_PEER_FETCH_TEXT. Charged before a record is kept; past either bound
/// the fetch is refused.
class FetchBudget {
 public:
  void take(RecordCost cost) {
    records_ += cost.records;
    text_ += cost.text;
    if (records_ > MAX_PEER_FETCH_RECORDS) {
      throw PeerFetchRefused("reports more than " + std::to_string(MAX_PEER_FETCH_RECORDS) +
                             " records in one refresh (entities, operations, topics and the lists they carry)");
    }
    if (text_ > MAX_PEER_FETCH_TEXT) {
      throw PeerFetchRefused("reports more than " + std::to_string(MAX_PEER_FETCH_TEXT / (1024 * 1024)) +
                             " MiB of text in one refresh");
    }
  }

  /// Hand back what a record that is being replaced cost, before its
  /// replacement is taken.
  void give_back(RecordCost cost) {
    records_ -= std::min(records_, cost.records);
    text_ -= std::min(text_, cost.text);
  }

  template <class Entity>
  void charge(const Entity & entity) {
    take(cost_of(entity));
  }

  /// `kept` is replaced by `replacement`: the detail of an entity its list
  /// named.
  template <class Entity>
  void replace(Entity & kept, Entity replacement) {
    give_back(cost_of(kept));
    take(cost_of(replacement));
    kept = std::move(replacement);
  }

 private:
  size_t records_{0};
  size_t text_{0};
};

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

/// The `items` array of a collection a peer answered, or nullptr when it has none.
const nlohmann::json * items_of(const nlohmann::json & response_json) {
  if (!response_json.is_object()) {
    return nullptr;
  }
  const auto items = response_json.find("items");
  return items != response_json.end() && items->is_array() ? &*items : nullptr;
}

/// The entity id an item of a peer's collection names, as the entity parsers
/// read it, or nullptr when it names none.
const std::string * item_id(const nlohmann::json & item) {
  if (!item.is_object()) {
    return nullptr;
  }
  const auto id = item.find("id");
  return id != item.end() && id->is_string() ? &id->get_ref<const std::string &>() : nullptr;
}

/// True for an item of a peer's collection that names a valid entity id. No
/// entity is built for any other item.
bool names_valid_entity(const nlohmann::json & item) {
  const auto * id = item_id(item);
  return id != nullptr && is_valid_entity_id(*id);
}

/**
 * @brief Parse the entities of a collection a peer answered, if there are at
 * most `max` of them.
 *
 * Only the items naming a valid entity id are built, and they are counted
 * before any is: an empty object costs the peer three bytes and the gateway a
 * whole entity, so building first and checking the cap after let a body
 * within the size limit allocate gigabytes.
 *
 * Each entity is charged to `budget` before it is kept.
 *
 * @return The entities in the order the peer listed them, or the number of
 *   items naming one when it is past `max`.
 */
template <typename T, typename Parser>
tl::expected<std::vector<T>, size_t> parse_collection(const nlohmann::json & response_json, Parser parser, size_t max,
                                                      FetchBudget & budget) {
  std::vector<T> result;
  const auto * items = items_of(response_json);
  if (items == nullptr) {
    return result;
  }
  const auto count = static_cast<size_t>(std::count_if(items->begin(), items->end(), names_valid_entity));
  if (count > max) {
    return tl::unexpected(count);
  }
  result.reserve(count);
  for (const auto & item : *items) {
    if (names_valid_entity(item)) {
      auto entity = parser(item);
      budget.charge(entity);
      result.push_back(std::move(entity));
    }
  }
  return result;
}

/// The string under `key`, or empty when the key is absent or holds something else.
std::string string_at(const nlohmann::json & j, const char * key) {
  auto it = j.find(key);
  return (it != j.end() && it->is_string()) ? it->get<std::string>() : std::string{};
}

/// A field a gateway serves under `x-medkit.ros2.<ros2_key>`, falling back to
/// `x-medkit.<legacy_key>`, where this parser used to look for it.
std::string ros2_field(const nlohmann::json & xm, const char * ros2_key, const char * legacy_key) {
  auto ros2 = xm.find("ros2");
  if (ros2 != xm.end() && ros2->is_object()) {
    auto value = string_at(*ros2, ros2_key);
    if (!value.empty()) {
      return value;
    }
  }
  return string_at(xm, legacy_key);
}

/// The description a gateway serves at the top level of an entity, falling back
/// to `x-medkit.description`, which only some routes carry.
std::string description_of(const nlohmann::json & j) {
  auto description = string_at(j, "description");
  if (description.empty()) {
    auto xm = j.find("x-medkit");
    if (xm != j.end() && xm->is_object()) {
      description = string_at(*xm, "description");
    }
  }
  return description;
}

/**
 * @brief Parse an Area from JSON
 */
Area parse_area(const nlohmann::json & j) {
  Area area;
  area.id = j.value("id", "");
  area.name = j.value("name", "");
  area.description = description_of(j);
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    area.namespace_path = ros2_field(xm, "namespace", "namespace");
    area.source = xm.value("source", "");
    // Served on the detail only. A subarea read off its parent's subareas
    // route takes its parent from the route instead (see fetch_entities).
    area.parent_area_id = string_at(xm, "parent_area_id");
    if (!is_valid_entity_id(area.parent_area_id)) {
      area.parent_area_id.clear();
    }
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
  comp.description = description_of(j);
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    comp.namespace_path = ros2_field(xm, "namespace", "namespace");
    comp.fqn = ros2_field(xm, "node", "fqn");
    comp.area = xm.value("area", "");
    comp.source = xm.value("source", "");
    comp.variant = xm.value("variant", "");
    comp.parent_component_id = xm.value("parentComponentId", "");
    if (xm.contains("dependsOn") && xm["dependsOn"].is_array()) {
      // Counted before anything is built from it, like the links of a Function.
      if (xm["dependsOn"].size() > MAX_PEER_DEPENDENCIES) {
        throw PeerFetchRefused("reports more than " + std::to_string(MAX_PEER_DEPENDENCIES) +
                               " dependencies for Component '" + comp.id + "'");
      }
      comp.depends_on = xm["dependsOn"].get<std::vector<std::string>>();
    }
    // Asset-identity nameplate (with per-field provenance). Emitted by peers under
    // x-medkit.identity only when populated; parse it back so identity survives
    // aggregation instead of being silently dropped.
    if (xm.contains("identity") && xm["identity"].is_object()) {
      comp.identity = AssetIdentity::from_json(xm["identity"]);
    }
    // A peer emits `available` only to say false, so absence is the peer
    // stating the entity is reachable and the default has to be true. A
    // Component carries no second signal - an App has `is_online` - so this is
    // the only way an unreachable one stays unreachable past another hop.
    comp.available = xm.value("available", true);
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
 * @brief Extract the entity ID that follows `marker` (e.g. ``/components/``) in a relationship URI.
 *
 * Peers may emit a relationship either as an absolute URL
 * (``http://host:port/api/v1/components/{id}``) or as a path-only reference
 * (``/api/v1/components/{id}``). Extract the segment after the last
 * `marker` and validate it with ``is_valid_entity_id`` so that a malformed
 * or hostile peer URI cannot smuggle path traversal or percent-encoded junk
 * into the aggregator's entity cache.
 * Returns an empty string if the URI contains no marker or if the
 * extracted segment fails validation.
 */
std::string entity_id_after(const std::string & uri, const std::string & marker) {
  auto pos = uri.rfind(marker);
  if (pos == std::string::npos) {
    return "";
  }
  auto id_start = pos + marker.size();
  if (id_start >= uri.size()) {
    return "";
  }
  auto id_end = uri.find_first_of("/?#", id_start);
  auto candidate = uri.substr(id_start, id_end - id_start);
  return is_valid_entity_id(candidate) ? candidate : std::string{};
}

/**
 * @brief Extract the component ID from an ``is-located-on`` URI.
 *
 * SOVD exposes the app-to-component binding as a standard relationship
 * URI (ISO 17978-3, §7.6).
 */
std::string component_id_from_located_on(const std::string & uri) {
  return entity_id_after(uri, "/components/");
}

/**
 * @brief Read a Function's ``depends-on`` links into Function ids.
 *
 * The Function detail serves its dependencies only as ``_links["depends-on"]``,
 * an array of ``/api/v1/functions/{id}`` references (a HAL ``{"href": ...}``
 * object is read too). Each id is extracted and validated the way an
 * ``is-located-on`` reference is; a link that yields no valid id is skipped.
 *
 * Each id is kept once, in the order the peer listed it: a set beside the list
 * says whether it is already there, since searching the list for every link
 * cost the square of its length. A Function past MAX_PEER_DEPENDENCIES
 * distinct dependencies, `owner`, refuses the fetch.
 */
std::vector<std::string> function_ids_from_depends_on(const nlohmann::json & j, const std::string & owner) {
  std::vector<std::string> ids;
  auto links = j.find("_links");
  if (links == j.end() || !links->is_object()) {
    return ids;
  }
  auto depends_on = links->find("depends-on");
  if (depends_on == links->end() || !depends_on->is_array()) {
    return ids;
  }
  std::unordered_set<std::string> seen;
  for (const auto & link : *depends_on) {
    std::string uri = link.is_string() ? link.get<std::string>() : (link.is_object() ? string_at(link, "href") : "");
    auto id = entity_id_after(uri, "/functions/");
    if (id.empty() || seen.count(id) > 0u) {
      continue;
    }
    if (ids.size() >= MAX_PEER_DEPENDENCIES) {
      throw PeerFetchRefused("reports more than " + std::to_string(MAX_PEER_DEPENDENCIES) +
                             " dependencies for Function '" + owner + "'");
    }
    seen.insert(id);
    ids.push_back(std::move(id));
  }
  return ids;
}

/**
 * @brief Read a peer's `/operations` collection into an App's service/action lists.
 *
 * The wire id is not used as the name: on the peer it is already qualified when
 * that peer saw a collision among its own members, and the qualifier is the
 * peer's business, not ours. `name` is always the bare short name, which is what
 * ambiguity is keyed on, and the full ROS path comes from x-medkit.ros2 so two
 * same-named operations stay distinguishable.
 *
 * Each operation is charged to `budget` before it is kept, and an App past
 * MAX_PEER_ENTRIES_PER_APP operations refuses the fetch.
 */
void parse_operations_into(const nlohmann::json & j, App & app, FetchBudget & budget) {
  if (!j.contains("items") || !j["items"].is_array()) {
    return;
  }
  for (const auto & item : j["items"]) {
    if (!item.is_object()) {
      continue;
    }
    const std::string name = item.value("name", "");
    if (name.empty()) {
      continue;
    }
    std::string full_path;
    std::string type;
    bool is_action = item.value("asynchronous_execution", false);
    if (item.contains("x-medkit") && item["x-medkit"].is_object()) {
      const auto & xm = item["x-medkit"];
      if (xm.contains("ros2") && xm["ros2"].is_object()) {
        const auto & ros2 = xm["ros2"];
        type = ros2.value("type", "");
        full_path = ros2.value("service", "");
        if (full_path.empty()) {
          full_path = ros2.value("action", "");
          is_action = is_action || !full_path.empty();
        }
        const std::string kind = ros2.value("kind", "");
        if (!kind.empty()) {
          is_action = kind == "action";
        }
      }
    }
    if (full_path.empty()) {
      continue;
    }
    if (app.services.size() + app.actions.size() >= MAX_PEER_ENTRIES_PER_APP) {
      throw PeerFetchRefused("reports more than " + std::to_string(MAX_PEER_ENTRIES_PER_APP) + " operations for App '" +
                             app.id + "'");
    }
    budget.take({1, name.size() + full_path.size() + type.size()});
    if (is_action) {
      app.actions.push_back(ActionInfo{name, full_path, type, std::nullopt});
    } else {
      app.services.push_back(ServiceInfo{name, full_path, type, std::nullopt});
    }
  }
}

/**
 * @brief Read a peer's data collection into the App's topics.
 *
 * A data item names its ROS topic by full path in ``x-medkit.ros2.topic``. An
 * item without one describes something that is not a topic - a plugin's data
 * point, for instance - and carries nothing this gateway can address as one, so
 * it is skipped rather than recorded under its wire id.
 *
 * ``direction`` decides which side of the topic the App is on, and only the
 * three values a gateway emits are accepted. An item that says anything else
 * leaves the App unattributed for that topic instead of being recorded as a
 * publisher it may not be: the ownership built out of these lists is what a
 * bare item id is dispatched by, and a guess there sends a read to a gateway
 * that does not have the topic.
 *
 * Each topic is charged to `budget` before it is kept, and an App past
 * MAX_PEER_ENTRIES_PER_APP data items naming a topic refuses the fetch.
 */
void parse_data_items_into(const nlohmann::json & j, App & app, FetchBudget & budget) {
  if (!j.contains("items") || !j["items"].is_array()) {
    return;
  }
  size_t topic_items = 0;
  for (const auto & item : j["items"]) {
    if (!item.is_object() || !item.contains("x-medkit") || !item["x-medkit"].is_object()) {
      continue;
    }
    const auto & xm = item["x-medkit"];
    if (!xm.contains("ros2") || !xm["ros2"].is_object()) {
      continue;
    }
    const std::string topic = xm["ros2"].value("topic", "");
    const std::string direction = xm["ros2"].value("direction", "");
    if (topic.empty()) {
      continue;
    }
    if (++topic_items > MAX_PEER_ENTRIES_PER_APP) {
      throw PeerFetchRefused("reports more than " + std::to_string(MAX_PEER_ENTRIES_PER_APP) + " data items for App '" +
                             app.id + "'");
    }
    if (direction == "publish" || direction == "both") {
      budget.take({1, topic.size()});
      app.topics.publishes.push_back(topic);
    }
    if (direction == "subscribe" || direction == "both") {
      budget.take({1, topic.size()});
      app.topics.subscribes.push_back(topic);
    }
  }
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
    } else if (links.contains("is-located-on") && links["is-located-on"].is_object()) {
      // HAL+JSON object form: {"href": "/api/v1/components/ecu-a"}
      auto href = links["is-located-on"].value("href", "");
      if (!href.empty()) {
        app.component_id = component_id_from_located_on(href);
      }
    }
  }

  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    // Vendor fallback: gateway emits x-medkit.component_id (snake_case) in
    // discovery_handlers.cpp. Only used if the SOVD standard is-located-on
    // field is absent. Validated for the same reasons as
    // component_id_from_located_on - the value is peer-provided.
    if (app.component_id.empty()) {
      auto candidate = xm.value("component_id", "");
      if (is_valid_entity_id(candidate)) {
        app.component_id = candidate;
      }
    }
    app.source = xm.value("source", "");
    app.is_online = xm.value("is_online", false);
    // Emitted only to say false, so absence means reachable. Read back for the
    // same reason `is_online` is: a leaf that went quiet several hops away is
    // described by the gateway that still holds its declaration, and that
    // description is the only account of it this gateway can get.
    app.available = xm.value("available", true);
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
  func.description = description_of(j);
  if (j.contains("x-medkit") && j["x-medkit"].is_object()) {
    const auto & xm = j["x-medkit"];
    func.source = xm.value("source", "");
    if (xm.contains("hosts") && xm["hosts"].is_array()) {
      func.hosts = xm["hosts"].get<std::vector<std::string>>();
    }
  }
  // The Function detail serves `translation_id`; `translationId` is what this
  // parser read before.
  func.translation_id = string_at(j, "translation_id");
  if (func.translation_id.empty()) {
    func.translation_id = string_at(j, "translationId");
  }
  if (j.contains("tags") && j["tags"].is_array()) {
    func.tags = j["tags"].get<std::vector<std::string>>();
  }
  func.depends_on = function_ids_from_depends_on(j, func.id);
  return func;
}

/**
 * @brief What a non-200 on a sub-request means, which depends on the route.
 */
enum class RouteKind {
  /// A top-level collection: ``/areas``, ``/components``, ``/apps``,
  /// ``/functions``. Every peer serves these, so nothing but 200 describes one.
  kCollection,
  /// A nested collection: ``/areas/{id}/subareas``, ``/components/{id}/subcomponents``,
  /// ``/apps/{id}/operations``. A gateway old enough not to have the route answers
  /// 404, and aggregation has to keep working across that version boundary, so a
  /// 404 here means "not offered" rather than "could not be read".
  ///
  /// It also carries ``504 not-responding``, and for the same reason the detail
  /// of an addressable entity does: the route hangs off an entity, and a peer
  /// holding a declaration for a gateway that went quiet answers 504 on every
  /// route of that entity, this one included. Read as a hole in the picture it
  /// would abort the whole fetch, and one unreachable member anywhere behind a
  /// peer would freeze this gateway's view of everything that peer holds.
  kNestedCollection,
  /// The detail of an entity that carries availability of its own (a Component).
  /// ``504 not-responding`` is the peer describing that entity as unreachable -
  /// a statement about the entity, which an aggregating peer makes whenever it
  /// is itself holding a declaration for a gateway that went quiet. It is
  /// carried, not read as a hole in the picture.
  kAddressableDetail,
  /// The detail of a grouping entity (a Function). It has no availability of its
  /// own to carry, and its members are named nowhere else, so nothing but 200
  /// describes it.
  kGroupingDetail,
};

/**
 * @brief A peer's answer to one request, read no further than
 * MAX_PEER_RESPONSE_SIZE.
 */
struct PeerReply {
  httplib::Result result;                 ///< The answer with its body, or why there is none
  bool too_large{false};                  ///< The read stopped at the limit; `result` is then empty
  bool past_deadline{false};              ///< The request ran into its deadline and was stopped; `result` is empty
  std::chrono::milliseconds deadline{0};  ///< The deadline the request was armed with
};

/// The key a request deadline is configured by and its value, for a message.
std::string describe_deadline(std::chrono::milliseconds deadline) {
  return "aggregation.request_deadline_ms (" + std::to_string(deadline.count()) + "ms for this request)";
}

/// What a call against the peer is admitted by: the owner's admission mutex
/// and its shutdown flag (see PeerClient::admission_mutex_), and the registry
/// that stops a request still running at `deadline`.
struct Admission {
  std::shared_mutex & mutex;
  const std::atomic<bool> & shutdown_requested;
  PeerCallRegistry & calls;
  std::chrono::milliseconds deadline;
};

/**
 * @brief Send `req` to the peer, reading its body no further than
 * MAX_PEER_RESPONSE_SIZE, unless shutdown has begun.
 *
 * httplib's own calls read the whole body into memory before they return it,
 * so a size check on what they return runs after the gateway has already held
 * everything the peer chose to send. Here the bytes are counted as they arrive
 * and the read stops at the limit, which holds whatever the status is.
 *
 * Once shutdown has begun the call is not made and fails as canceled. The
 * admission lock taken for that check is released when the call's socket
 * exists, from the socket-options callback, which cpp-httplib runs while it
 * holds the lock its stop() takes; a call that never gets that far releases
 * it on return.
 *
 * The request is armed with the admission's deadline as it starts, before it
 * connects, and stopped by the registry if it is still running then.
 */
PeerReply send_capped(httplib::Client & cli, httplib::Request & req, const Admission & admission) {
  PeerReply reply;
  std::shared_lock<std::shared_mutex> admitted(admission.mutex);
  if (admission.shutdown_requested.load(std::memory_order_acquire)) {
    reply.result = httplib::Result{nullptr, httplib::Error::Canceled};
    return reply;
  }
  cli.set_socket_options([&admitted](socket_t /*sock*/) {
    if (admitted.owns_lock()) {
      admitted.unlock();
    }
  });
  std::string body;
  req.content_receiver = [&body, &reply](const char * data, size_t length, uint64_t /*offset*/, uint64_t /*total*/) {
    if (length > MAX_PEER_RESPONSE_SIZE - body.size()) {
      reply.too_large = true;
      return false;
    }
    body.append(data, length);
    return true;
  };
  auto res = std::make_unique<httplib::Response>();
  auto error = httplib::Error::Success;
  admission.calls.arm(&cli, admission.deadline);
  const bool sent = cli.send(req, *res, error);
  // A stop() that landed after the answer was complete changed nothing.
  reply.past_deadline = admission.calls.disarm(&cli) && !sent;
  reply.deadline = admission.deadline;
  // The callback refers to `admitted`, which ends with this call.
  cli.set_socket_options(nullptr);
  if (sent) {
    res->body = std::move(body);
    reply.result = httplib::Result{std::move(res), error};
  } else {
    reply.result = httplib::Result{nullptr, error};
  }
  return reply;
}

/// A GET to the peer, read no further than MAX_PEER_RESPONSE_SIZE.
PeerReply get_capped(httplib::Client & cli, const Admission & admission, const std::string & path,
                     const httplib::Headers & headers = {}) {
  httplib::Request req;
  req.method = "GET";
  req.path = path;
  req.headers = headers;
  return send_capped(cli, req, admission);
}

/**
 * @brief What one sub-request issued while describing a peer produced.
 */
struct SubResponse {
  enum class Kind {
    kBody,               ///< `body` holds the parsed response
    kRouteAbsent,        ///< the peer does not offer this route; carry on without it
    kEntityUnreachable,  ///< the peer named this entity and says it cannot be reached
    kIncomplete,         ///< part of the picture could not be read; `error` says why
  };

  Kind kind{Kind::kIncomplete};
  nlohmann::json body;
  std::string error;
};

/**
 * @brief True for a SOVD error body carrying the ``not-responding`` code.
 */
bool says_not_responding(const std::string & body) {
  if (judge_peer_json(body) != PeerJsonShape::kAcceptable) {
    return false;
  }
  auto parsed = nlohmann::json::parse(body, nullptr, false);
  return !parsed.is_discarded() && parsed.is_object() && parsed.value("error_code", "") == ERR_NOT_RESPONDING;
}

/**
 * @brief Classify one sub-request of a peer fetch.
 *
 * @param reply What the call produced
 * @param peer_name Peer the call was made against, for the error text
 * @param path Route that was called, for the error text
 * @param kind What a non-200 means on this route
 */
SubResponse read_sub_response(const PeerReply & reply, const std::string & peer_name, const std::string & path,
                              RouteKind kind) {
  SubResponse out;
  if (reply.too_large) {
    out.error = "Response from peer '" + peer_name + "' for " + path + " exceeds size limit";
    return out;
  }
  if (reply.past_deadline) {
    out.error =
        "Peer '" + peer_name + "' did not finish answering " + path + " within " + describe_deadline(reply.deadline);
    return out;
  }
  const auto & result = reply.result;
  if (!result) {
    out.error = result.error() == httplib::Error::Canceled
                    ? "Call to peer '" + peer_name + "' for " + path + " canceled, shutdown in progress"
                    : "Failed to connect to peer '" + peer_name + "' for " + path;
    return out;
  }
  if (result->status == 404 && kind == RouteKind::kNestedCollection) {
    out.kind = SubResponse::Kind::kRouteAbsent;
    return out;
  }
  if (result->status == 504 && says_not_responding(result->body) &&
      (kind == RouteKind::kAddressableDetail || kind == RouteKind::kNestedCollection)) {
    out.kind = SubResponse::Kind::kEntityUnreachable;
    return out;
  }
  if (result->status != 200) {
    out.error = "Peer '" + peer_name + "' returned status " + std::to_string(result->status) + " for " + path;
    return out;
  }
  if (const auto shape = judge_peer_json(result->body); shape != PeerJsonShape::kAcceptable) {
    out.error = "Response from peer '" + peer_name + "' for " + path + " " + describe_peer_json_shape(shape);
    return out;
  }
  auto parsed = nlohmann::json::parse(result->body, nullptr, false);
  if (parsed.is_discarded()) {
    out.error = "Invalid JSON from peer '" + peer_name + "' for " + path;
    return out;
  }
  out.kind = SubResponse::Kind::kBody;
  out.body = std::move(parsed);
  return out;
}

}  // namespace

namespace {

/// Name and value of the budget an error was measured against, for a message a
/// client can act on. The READ budget differs by call: a forward spends
/// `aggregation.forward_timeout_ms`, a fan-out spends `aggregation.timeout_ms`,
/// so the caller names its own rather than have this guess.
std::string describe_budget(PeerBudget budget, const PeerTimeouts & timeouts, const char * read_key, int read_ms) {
  switch (budget) {
    case PeerBudget::kConnect:
      return "aggregation.timeout_ms (" + std::to_string(timeouts.connect_ms) + "ms, connect)";
    case PeerBudget::kRead:
      return std::string(read_key) + " (" + std::to_string(read_ms) + "ms, read)";
    case PeerBudget::kWrite:
      return "aggregation.write_timeout_ms (" + std::to_string(timeouts.write_ms) + "ms, write)";
    case PeerBudget::kNone:
      break;
  }
  return "its budget";
}

}  // namespace

PeerBudget peer_budget_for(httplib::Error error) {
  switch (error) {
    case httplib::Error::ConnectionTimeout:
      return PeerBudget::kConnect;
    case httplib::Error::Read:
      return PeerBudget::kRead;
    case httplib::Error::Write:
      return PeerBudget::kWrite;
    case httplib::Error::Success:
    case httplib::Error::Unknown:
    case httplib::Error::Connection:
    case httplib::Error::BindIPAddress:
    case httplib::Error::ExceedRedirectCount:
    case httplib::Error::Canceled:
    case httplib::Error::SSLConnection:
    case httplib::Error::SSLLoadingCerts:
    case httplib::Error::SSLServerVerification:
    case httplib::Error::UnsupportedMultipartBoundaryChars:
    case httplib::Error::Compression:
    case httplib::Error::ProxyConnection:
    case httplib::Error::SSLPeerCouldBeClosed_:
      return PeerBudget::kNone;
  }
  return PeerBudget::kNone;
}

PeerFailureKind classify_peer_error(httplib::Error error, std::chrono::milliseconds elapsed,
                                    std::chrono::milliseconds read_budget, std::chrono::milliseconds write_budget) {
  // Every enumerator listed rather than a default, because -Wswitch-enum is an
  // error here: a cpp-httplib upgrade that adds a failure mode has to be
  // classified deliberately instead of falling into "unreachable" unnoticed.
  switch (error) {
    case httplib::Error::ConnectionTimeout:
      return PeerFailureKind::kTimeout;
    case httplib::Error::Canceled:
      return PeerFailureKind::kCanceled;
    case httplib::Error::Read:
      // The budget is always set on these clients, so a call that consumed it
      // ran out of time. One that came back sooner broke for another reason -
      // most often the peer process dying with the answer half sent - and
      // reporting that as a timeout would tell an operator to wait for a peer
      // that is gone.
      //
      // `elapsed` covers connect plus read, because cpp-httplib reports no
      // separate connect-completion time and its socket-options hook runs
      // BEFORE the connect. A slow handshake therefore counts towards the read
      // budget. The connect budget is capped well below the read budgets (see
      // AggregationConfig::peer_timeouts) so that window is bounded by the cap
      // rather than by the whole metadata budget, and a handshake slower than
      // the cap fails as ConnectionTimeout instead, which is attributed
      // correctly. Removing the window entirely needs the vendored client to
      // expose a post-connect callback.
      return elapsed >= read_budget ? PeerFailureKind::kTimeout : PeerFailureKind::kUnreachable;
    case httplib::Error::Write:
      // Against the WRITE budget, which is its own and smaller than the read
      // budget by default. Measured against the read budget, a write that ran
      // out at 5 s inside a 15 s forward budget looks like a peer that died,
      // and the one key whose whole purpose is bounding a slow upload could
      // never report its own expiry.
      return elapsed >= write_budget ? PeerFailureKind::kTimeout : PeerFailureKind::kUnreachable;
    case httplib::Error::Success:
      // Not reachable through the callers, which classify only a null result,
      // but the enumerator exists and "no failure" is the only honest answer.
      return PeerFailureKind::kNone;
    case httplib::Error::Unknown:
    case httplib::Error::Connection:
    case httplib::Error::BindIPAddress:
    case httplib::Error::ExceedRedirectCount:
    case httplib::Error::SSLConnection:
    case httplib::Error::SSLLoadingCerts:
    case httplib::Error::SSLServerVerification:
    case httplib::Error::UnsupportedMultipartBoundaryChars:
    case httplib::Error::Compression:
    case httplib::Error::ProxyConnection:
    case httplib::Error::SSLPeerCouldBeClosed_:
      return PeerFailureKind::kUnreachable;
  }
  return PeerFailureKind::kUnreachable;
}

const char * peer_failure_reason(PeerFailureKind kind) {
  switch (kind) {
    case PeerFailureKind::kNone:
      return "none";
    case PeerFailureKind::kTimeout:
      return "timeout";
    case PeerFailureKind::kUnreachable:
      return "unreachable";
    case PeerFailureKind::kCanceled:
      return "canceled";
    case PeerFailureKind::kErrorStatus:
      return "error-status";
    case PeerFailureKind::kTooLarge:
      return "too-large";
    case PeerFailureKind::kInvalidResponse:
      return "invalid-response";
    case PeerFailureKind::kEntityNotFound:
      return "entity-not-found";
  }
  return "unreachable";
}

PeerClient::PeerClient(const std::string & url, const std::string & name, PeerTimeouts timeouts, bool forward_auth,
                       std::string peer_auth_header)
  : url_(url)
  , name_(name)
  , timeouts_(timeouts)
  , forward_auth_(forward_auth)
  , peer_auth_header_(std::move(peer_auth_header)) {
}

PeerClient::PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth,
                       std::string peer_auth_header)
  : PeerClient(url, name, PeerTimeouts::uniform(timeout_ms), forward_auth, std::move(peer_auth_header)) {
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

void PeerClient::check_health() {
  // A per-call client, as every other call makes, so shutdown() stops one in
  // flight. The read timeout bounds each read, not the answer: a peer that
  // sends its answer a byte at a time, each inside the timeout, would hold the
  // call and the refresh waiting on it for as long as it kept sending, so the
  // request ends at its deadline, the two capped budgets added up. A shared
  // client kept for health checks was left out of shutdown(), so such a peer
  // held the executor that runs the refresh past SIGTERM.
  //
  // Both budgets are capped at 1 s whatever is configured: an unresponsive
  // peer is unhealthy whether the gateway waits one second or five, and a
  // refresh checks every peer before it reads any.
  const int health_ms = std::min(timeouts_.metadata_read_ms, 1000);
  const int connect_ms = std::min(timeouts_.connect_ms, health_ms);
  ScopedClient scoped(*this, url_, health_ms);
  auto & cli = *scoped;
  cli.set_connection_timeout(connect_ms / 1000, (connect_ms % 1000) * 1000);
  // The health check is this gateway asking on its own behalf, so it carries
  // the gateway's own credential. Without it a peer that requires auth on
  // reads answers 401, is recorded as unhealthy, and then appears in no
  // fan-out and no relay - the peer is reachable and correctly configured, and
  // the aggregator reports it as down.
  httplib::Headers headers;
  if (!peer_auth_header_.empty()) {
    headers.emplace("Authorization", peer_auth_header_);
  }
  // A health answer past the size limit is no answer: the peer is unhealthy.
  const auto reply = get_capped(
      cli, Admission{admission_mutex_, shutdown_requested_, calls_, std::chrono::milliseconds(connect_ms + health_ms)},
      std::string(API_PREFIX) + "/health", headers);
  const auto & result = reply.result;
  const bool healthy = result && result->status == 200;
  std::string instance_id;
  if (healthy && judge_peer_json(result->body) == PeerJsonShape::kAcceptable) {
    auto body = nlohmann::json::parse(result->body, nullptr, false);
    if (body.is_object() && body.contains("x-medkit-instance-id") && body["x-medkit-instance-id"].is_string()) {
      instance_id = body["x-medkit-instance-id"].get<std::string>();
    }
  }
  {
    std::lock_guard<std::mutex> id_lock(instance_id_mutex_);
    instance_id_ = std::move(instance_id);
  }
  healthy_.store(healthy);
}

std::string PeerClient::instance_id() const {
  std::lock_guard<std::mutex> lock(instance_id_mutex_);
  return instance_id_;
}

// The whole body is a function-try-block. The parsers read fields with the
// JSON library's typed accessors, which throw on a value of the wrong type -
// `"name": 5`, a tag that is a number - and nothing below the cache refresh
// caught that: one such peer failed every refresh of the cache, the local
// entities included. Here it fails this peer's fetch, like any other answer
// that cannot be read, and the peer's last complete declaration stays.
tl::expected<PeerEntities, std::string> PeerClient::fetch_entities() try {
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return tl::unexpected<std::string>("Peer '" + name_ + "': fetch_entities skipped, shutdown in progress");
  }
  // A dedicated client for this call, as for every call against the peer.
  // ScopedClient registers with the active-client registry so shutdown() can
  // stop() the in-flight call.
  ScopedClient scoped(*this, url_, timeouts_.metadata_read_ms);
  auto & cli = *scoped;
  const Admission admission{admission_mutex_, shutdown_requested_, calls_,
                            request_deadline(timeouts_.metadata_read_ms)};
  // Fetching the peer's entities is this gateway asking on its own behalf, so
  // it carries the gateway's own credential. Set once on the client rather
  // than on each of the requests below - that is also what stops the next
  // route added here from silently going out unauthenticated and reducing the
  // peer to "offers no such route".
  if (!peer_auth_header_.empty()) {
    cli.set_default_headers({{"Authorization", peer_auth_header_}});
  }

  PeerEntities entities;
  const std::string peer_source = "peer:" + name_;
  // Every record kept below is charged to this before it is kept: each
  // response is bounded on its own, but a refresh makes a request per entity.
  FetchBudget budget;

  // A route the peer does not offer is a property of the peer, not of the
  // entity that happened to hit it first, so it is recorded once however many
  // entities ask for it.
  auto note_absent_route = [&entities](const std::string & route) {
    if (std::find(entities.absent_routes.begin(), entities.absent_routes.end(), route) ==
        entities.absent_routes.end()) {
      entities.absent_routes.push_back(route);
    }
  };

  // Fetch areas
  {
    auto response = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + "/areas"), name_, "/areas",
                                      RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    auto areas = parse_collection<Area>(response.body, parse_area, MAX_ENTITIES_PER_COLLECTION, budget);
    if (!areas) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(areas.error()) +
                                         " areas (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    entities.areas = std::move(*areas);
    for (auto & area : entities.areas) {
      area.declared_source = area.source;
      area.source = peer_source;
    }

    // Subareas, read level by level below the top-level Areas (the list leaves
    // them out), down to MAX_NESTING_DEPTH. Collected into a separate vector so
    // nothing is pushed into the vector being iterated. A subarea's list item
    // names no parent, so the route it was read from does: it is a child of the
    // Area that route belongs to. An id already read is not read again, which
    // is what ends a tree that loops back on itself.
    std::vector<Area> all_subareas;
    std::unordered_set<std::string> seen_areas;
    std::vector<std::string> area_level;
    for (const auto & area : entities.areas) {
      seen_areas.insert(area.id);
      area_level.push_back(area.id);
    }
    for (size_t depth = 0; depth < MAX_NESTING_DEPTH && !area_level.empty(); ++depth) {
      std::vector<std::string> next_level;
      for (const auto & parent_id : area_level) {
        const std::string route = "/areas/" + parent_id + "/subareas";
        auto sub = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + route), name_, route,
                                     RouteKind::kNestedCollection);
        if (sub.kind == SubResponse::Kind::kIncomplete) {
          return tl::unexpected<std::string>(sub.error);
        }
        if (sub.kind == SubResponse::Kind::kRouteAbsent) {
          note_absent_route("/areas/{id}/subareas");
          continue;
        }
        if (sub.kind == SubResponse::Kind::kEntityUnreachable) {
          // The peer holds this Area's id but the gateway contributing it is
          // silent, so its members are out of reach. That is one Area's worth of
          // detail missing, not a failed read of the peer.
          continue;
        }
        const auto * items = items_of(sub.body);
        if (items == nullptr) {
          continue;
        }
        for (const auto & item : *items) {
          if (!names_valid_entity(item) || !seen_areas.insert(*item_id(item)).second) {
            continue;
          }
          // The cap is checked before each subarea is built, not once a level
          // is done: a level is as wide as the peer makes it, and every
          // subarea taken costs a request for its own subareas.
          if (entities.areas.size() + all_subareas.size() >= MAX_ENTITIES_PER_COLLECTION) {
            return tl::unexpected<std::string>("Peer '" + name_ + "' returned more than " +
                                               std::to_string(MAX_ENTITIES_PER_COLLECTION) + " areas and subareas");
          }
          auto subarea = parse_area(item);
          budget.charge(subarea);
          subarea.parent_area_id = parent_id;
          subarea.declared_source = subarea.source;
          subarea.source = peer_source;
          next_level.push_back(subarea.id);
          all_subareas.push_back(std::move(subarea));
        }
      }
      area_level = std::move(next_level);
    }
    entities.areas.insert(entities.areas.end(), std::make_move_iterator(all_subareas.begin()),
                          std::make_move_iterator(all_subareas.end()));
  }

  // Fetch components (list then detail per entity for full relationship data)
  {
    auto response = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + "/components"), name_,
                                      "/components", RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    // Parse IDs from list, then fetch detail per entity for relationships
    auto components = parse_collection<Component>(response.body, parse_component, MAX_ENTITIES_PER_COLLECTION, budget);
    if (!components) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(components.error()) +
                                         " components (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    auto comp_list = std::move(*components);
    // The detail response carries the relationships (parent, dependencies,
    // identity) the list omits, so a Component built from the list alone is a
    // Component asserted to have none.
    for (auto & comp : comp_list) {
      const std::string route = "/components/" + comp.id;
      auto detail = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + route), name_, route,
                                      RouteKind::kAddressableDetail);
      if (detail.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(detail.error);
      }
      if (detail.kind == SubResponse::Kind::kBody) {
        budget.replace(comp, parse_component(detail.body));
      } else {
        // The peer holds this Component's id but cannot describe it: whoever
        // contributes it has gone quiet. What the list gave is everything the
        // peer can still say, and the availability is what it is saying.
        comp.available = false;
      }
      comp.declared_source = comp.source;
      comp.source = peer_source;
    }
    // Subcomponents, level by level below the top-level Components (the list
    // leaves them out), down to MAX_NESTING_DEPTH, the same way as subareas.
    // Collected into a separate vector so nothing is pushed into the vector
    // being iterated.
    std::vector<Component> all_subcomps;
    std::unordered_set<std::string> seen_components;
    std::vector<std::string> component_level;
    for (const auto & comp : comp_list) {
      seen_components.insert(comp.id);
      component_level.push_back(comp.id);
    }
    for (size_t depth = 0; depth < MAX_NESTING_DEPTH && !component_level.empty(); ++depth) {
      std::vector<std::string> next_level;
      for (const auto & parent_id : component_level) {
        const std::string route = "/components/" + parent_id + "/subcomponents";
        auto sub = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + route), name_, route,
                                     RouteKind::kNestedCollection);
        if (sub.kind == SubResponse::Kind::kIncomplete) {
          return tl::unexpected<std::string>(sub.error);
        }
        if (sub.kind == SubResponse::Kind::kRouteAbsent) {
          note_absent_route("/components/{id}/subcomponents");
          continue;
        }
        if (sub.kind == SubResponse::Kind::kEntityUnreachable) {
          // Same as for subareas: the parent is retained and unreachable, so what
          // it contains cannot be read. The parent itself already carries that.
          continue;
        }
        const auto * items = items_of(sub.body);
        if (items == nullptr) {
          continue;
        }
        for (const auto & item : *items) {
          if (!names_valid_entity(item) || !seen_components.insert(*item_id(item)).second) {
            continue;
          }
          // Checked before the subcomponent is built and before its detail is
          // asked for: a level is as wide as the peer makes it, and checking
          // once it is done let one level cost a request per child of every
          // parent in it.
          if (comp_list.size() + all_subcomps.size() >= MAX_ENTITIES_PER_COLLECTION) {
            return tl::unexpected<std::string>("Peer '" + name_ + "' returned more than " +
                                               std::to_string(MAX_ENTITIES_PER_COLLECTION) +
                                               " components and subcomponents");
          }
          auto subcomp = parse_component(item);
          budget.charge(subcomp);
          // Fetch detail for each subcomponent to get full relationships
          const std::string sub_id = subcomp.id;
          const std::string detail_route = "/components/" + sub_id;
          auto detail = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + detail_route), name_,
                                          detail_route, RouteKind::kAddressableDetail);
          if (detail.kind == SubResponse::Kind::kIncomplete) {
            return tl::unexpected<std::string>(detail.error);
          }
          if (detail.kind == SubResponse::Kind::kBody) {
            budget.replace(subcomp, parse_component(detail.body));
          } else {
            subcomp.available = false;
          }
          // The route this id came from is itself the statement that the
          // Component it belongs to is its parent, so the tree keeps its shape
          // even when the entity's own description is out of reach.
          if (subcomp.parent_component_id.empty()) {
            subcomp.parent_component_id = parent_id;
          }
          subcomp.declared_source = subcomp.source;
          subcomp.source = peer_source;
          next_level.push_back(sub_id);
          all_subcomps.push_back(std::move(subcomp));
        }
      }
      component_level = std::move(next_level);
    }
    comp_list.insert(comp_list.end(), std::make_move_iterator(all_subcomps.begin()),
                     std::make_move_iterator(all_subcomps.end()));

    entities.components = std::move(comp_list);
  }

  // Fetch apps
  {
    auto response = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + "/apps"), name_, "/apps",
                                      RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    auto apps = parse_collection<App>(response.body, parse_app, MAX_ENTITIES_PER_COLLECTION, budget);
    if (!apps) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(apps.error()) +
                                         " apps (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    entities.apps = std::move(*apps);
    for (auto & app : entities.apps) {
      app.declared_source = app.source;
      app.source = peer_source;
    }
    // No internal-node filter here: the peer applied its own, and an id that
    // starts with '_' can name a visible node whose name collided.

    // Fetch each app's operations and data. Neither is ever declared in a
    // manifest - both are discovered from the ROS graph - so the only record of
    // what a peer's app exposes is what the peer reports. Without the
    // operations the aggregator cannot tell that two members share an operation
    // short name except by asking at request time, and an answer that depends
    // on who is reachable is not an answer a client can rely on. Without the
    // topics it holds no record of which member owns one, so an item id that
    // names no member - the form a topic with a single provider is listed under
    // - cannot be routed to the gateway that has the topic.
    //
    // `X-Medkit-No-Fan-Out` keeps the peer from re-asking ITS peers: each
    // gateway reports what it holds, and the hop that owns the entity is the
    // hop that answers for it. It is also what makes this terminate.
    //
    // A route that is absent or answers for an unreachable entity is skipped on
    // its own, never for the app: the two collections are read independently
    // and one missing must not cost the other.
    for (auto & app : entities.apps) {
      const httplib::Headers no_fan_out{{"X-Medkit-No-Fan-Out", "1"}};

      const std::string ops_route = "/apps/" + app.id + "/operations";
      auto ops = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + ops_route, no_fan_out), name_,
                                   ops_route, RouteKind::kNestedCollection);
      if (ops.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(ops.error);
      }
      if (ops.kind == SubResponse::Kind::kRouteAbsent) {
        note_absent_route("/apps/{id}/operations");
      } else if (ops.kind == SubResponse::Kind::kBody) {
        parse_operations_into(ops.body, app, budget);
      }
      // kEntityUnreachable: the App is retained and its gateway is silent, so
      // the peer answers for it rather than proxying. It keeps what the peer
      // already reported; there is nothing further to read.

      const std::string data_route = "/apps/" + app.id + "/data";
      auto data = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + data_route, no_fan_out), name_,
                                    data_route, RouteKind::kNestedCollection);
      if (data.kind == SubResponse::Kind::kIncomplete) {
        return tl::unexpected<std::string>(data.error);
      }
      if (data.kind == SubResponse::Kind::kRouteAbsent) {
        note_absent_route("/apps/{id}/data");
      } else if (data.kind == SubResponse::Kind::kBody) {
        parse_data_items_into(data.body, app, budget);
      }
    }
  }

  // Fetch functions (list then detail per entity for hosts data)
  {
    auto response = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + "/functions"), name_,
                                      "/functions", RouteKind::kCollection);
    if (response.kind != SubResponse::Kind::kBody) {
      return tl::unexpected<std::string>(response.error);
    }
    // Parse IDs from list, then fetch detail per entity for hosts
    auto functions = parse_collection<Function>(response.body, parse_function, MAX_ENTITIES_PER_COLLECTION, budget);
    if (!functions) {
      return tl::unexpected<std::string>("Peer '" + name_ + "' returned " + std::to_string(functions.error()) +
                                         " functions (max " + std::to_string(MAX_ENTITIES_PER_COLLECTION) + ")");
    }
    auto func_list = std::move(*functions);
    // A Function's hosts live only in its detail response: the list carries
    // none. A Function built from the list alone is a Function asserted to
    // group nothing, which is a statement the peer never made.
    for (auto & func : func_list) {
      const std::string route = "/functions/" + func.id;
      auto detail = read_sub_response(get_capped(cli, admission, std::string(API_PREFIX) + route), name_, route,
                                      RouteKind::kGroupingDetail);
      if (detail.kind != SubResponse::Kind::kBody) {
        return tl::unexpected<std::string>(detail.error);
      }
      budget.replace(func, parse_function(detail.body));
      func.declared_source = func.source;
      func.source = peer_source;
    }
    entities.functions = std::move(func_list);
  }

  return entities;
} catch (const nlohmann::json::exception & e) {
  return tl::unexpected<std::string>("Response from peer '" + name_ + "' could not be read: " + e.what());
} catch (const PeerFetchRefused & e) {
  // Past a bound on what one refresh keeps of a peer: refused like an
  // oversized answer, and the last complete declaration stays.
  return tl::unexpected<std::string>("Peer '" + name_ + "' " + e.what());
}

void PeerClient::forward_request(const httplib::Request & req, httplib::Response & res) {
  const auto refuse_for_shutdown = [&res]() {
    res.status = 503;
    auto error_body =
        make_error_body(ERR_VENDOR_ERROR, "Gateway shutting down; peer forward refused", ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
  };
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    refuse_for_shutdown();
    return;
  }
  // A dedicated client per forwarding call. ScopedClient registers with the
  // active-client registry so shutdown() can stop() the in-flight call.
  //
  // The forward budget, not the metadata one: what comes back is whatever the
  // peer had to do to answer, which for a synchronous operation is that
  // gateway's own service budget and for a large resource is the transfer.
  ScopedClient scoped(*this, url_, timeouts_.forward_read_ms);
  auto & cli = *scoped;

  httplib::Headers headers;
  // Forward Authorization header only when explicitly enabled (forward_auth).
  // Default is off to prevent token leakage to untrusted/mDNS-discovered peers.
  if (forward_auth_ && req.has_header("Authorization")) {
    headers.emplace("Authorization", req.get_header_value("Authorization"));
  } else if (!peer_auth_header_.empty()) {
    // The gateway's own credential, used when there is no client credential to
    // pass on. A client's own token wins where forwarding is enabled, so
    // forward_auth keeps naming the end user to the peer.
    headers.emplace("Authorization", peer_auth_header_);
  }
  // Propagate fan-out suppression header to prevent recursive loops when
  // a forwarded request bounces back to the origin via bidirectional peering.
  if (req.has_header("X-Medkit-No-Fan-Out")) {
    headers.emplace("X-Medkit-No-Fan-Out", "1");
  }
  // The client's identity, which is what a lock is held against. The peer owns
  // the entity, so the peer holds the lock and judges every request against the
  // name it recorded; a request that arrives anonymous is a different caller
  // than the one that took the lock, whoever sent it. It is not a credential -
  // authority travels in Authorization, which is forwarded only when the
  // deployment says the peer is trusted with it.
  if (req.has_header("X-Client-Id")) {
    headers.emplace("X-Client-Id", req.get_header_value("X-Client-Id"));
  }

  httplib::Result result{nullptr, httplib::Error::Unknown};
  const std::string path = path_with_query(req);
  const std::string content_type = req.get_header_value("Content-Type");

  const auto started_at = std::chrono::steady_clock::now();

  bool too_large = false;
  bool past_deadline = false;
  std::chrono::milliseconds deadline{0};
  if (req.method == "GET" || req.method == "DELETE" || req.method == "POST" || req.method == "PUT" ||
      req.method == "PATCH") {
    httplib::Request out;
    out.method = req.method;
    out.path = path;
    out.headers = std::move(headers);
    if (req.method == "POST" || req.method == "PUT" || req.method == "PATCH") {
      if (!content_type.empty()) {
        out.set_header("Content-Type", content_type);
      }
      out.body = req.body;
    }
    auto reply = send_capped(
        cli, out,
        Admission{admission_mutex_, shutdown_requested_, calls_, request_deadline(timeouts_.forward_read_ms)});
    result = std::move(reply.result);
    too_large = reply.too_large;
    past_deadline = reply.past_deadline;
    deadline = reply.deadline;
  }

  if (too_large) {
    res.status = 502;
    auto error_body = make_error_body(ERR_VENDOR_ERROR, "Response from peer '" + name_ + "' exceeds size limit",
                                      ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // A call refused at admission, or stopped in flight, because shutdown began
  // after the check above is the same answer as one refused by that check.
  // Classified as a transport failure it read as the peer being unavailable.
  if (!result && shutdown_requested_.load(std::memory_order_acquire)) {
    refuse_for_shutdown();
    return;
  }

  // The peer kept sending, each read inside its budget, and the answer was
  // still not complete at the request's deadline. Like a budget that ran out,
  // the peer may still be working, so it answers as a timeout naming the key
  // that bounds it.
  if (past_deadline) {
    res.status = 504;
    auto error_body = make_error_body(ERR_NOT_RESPONDING,
                                      "Peer '" + name_ + "' at " + url_ + " did not finish answering within " +
                                          describe_deadline(deadline) + ". The request may still be running there.");
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  if (!result) {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at);
    const auto kind = classify_peer_error(result.error(), elapsed, std::chrono::milliseconds(timeouts_.forward_read_ms),
                                          std::chrono::milliseconds(timeouts_.write_ms));
    if (kind == PeerFailureKind::kTimeout) {
      // The peer is answering someone; this gateway stopped waiting. Saying it
      // is unavailable describes a different system state and points an
      // operator at the wrong box.
      //
      // The message names the budget that actually expired. A connect timeout
      // is bounded by aggregation.timeout_ms, not by the forward budget the
      // call was hoping to spend, and telling an operator to raise the forward
      // budget would not move a peer that never accepted the connection.
      const auto expired = describe_budget(peer_budget_for(result.error()), timeouts_, "aggregation.forward_timeout_ms",
                                           timeouts_.forward_read_ms);
      res.status = 504;
      auto error_body =
          make_error_body(ERR_NOT_RESPONDING, "Peer '" + name_ + "' at " + url_ + " did not answer within " + expired +
                                                  ". The request may still be running there.");
      res.set_content(error_body.dump(), "application/json");
      return;
    }
    res.status = 502;
    auto error_body = make_error_body(ERR_VENDOR_ERROR,
                                      "Peer '" + name_ + "' at " + url_ + " is unavailable (" +
                                          std::string(peer_failure_reason(kind)) + ")",
                                      ERR_X_MEDKIT_PEER_UNAVAILABLE);
    res.set_content(error_body.dump(), "application/json");
    return;
  }

  // Copy response from peer - only forward safe headers.
  // The x-medkit header allowlist must match headers the gateway actually produces.
  // Currently the only x-medkit HTTP header is X-Medkit-Local-Only (fault_handlers.cpp).
  // Update this list when adding new x-medkit HTTP response headers.
  //
  // `location` is what a 201 or 202 hands the client to address the resource it
  // just created. The peer builds it from the request it received, so it names
  // the member's own route - a path this gateway resolves to the same member,
  // and the only address of a resource that lives on the other side. Dropping it
  // leaves the client a status with nothing to follow.
  static const std::set<std::string> allowed_headers = {"content-type",  "etag",     "cache-control",
                                                        "last-modified", "location", "x-medkit-local-only"};

  res.status = result->status;
  res.body = std::move(result->body);
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

tl::expected<nlohmann::json, PeerError> PeerClient::forward_and_get_json(const std::string & method,
                                                                         const std::string & path,
                                                                         const std::string & auth_header,
                                                                         const httplib::Headers & extra_headers) {
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return tl::unexpected(PeerError{PeerFailureKind::kCanceled,
                                    "Peer '" + name_ + "': forward_and_get_json skipped, shutdown in progress"});
  }
  // A dedicated client per call. ScopedClient registers with the active-client
  // registry so shutdown() can stop() the in-flight call.
  //
  // The metadata budget: a fan-out runs against every peer at once with a
  // client waiting on the slowest of them, so this is bounded by what a
  // listing is worth, not by what one peer's work might cost.
  ScopedClient scoped(*this, url_, timeouts_.metadata_read_ms);
  auto & cli = *scoped;

  httplib::Headers headers;
  // Only forward auth header when forward_auth is enabled
  if (forward_auth_ && !auth_header.empty()) {
    headers.emplace("Authorization", auth_header);
  } else if (!peer_auth_header_.empty()) {
    headers.emplace("Authorization", peer_auth_header_);
  }
  for (const auto & [key, value] : extra_headers) {
    headers.emplace(key, value);
  }

  httplib::Request out;
  out.method = method;
  out.path = path;
  out.headers = std::move(headers);
  if (method == "POST" || method == "PUT") {
    out.set_header("Content-Type", "application/json");
  }
  const auto started_at = std::chrono::steady_clock::now();
  PeerReply reply;
  if (method == "GET" || method == "POST" || method == "PUT" || method == "DELETE") {
    reply = send_capped(
        cli, out,
        Admission{admission_mutex_, shutdown_requested_, calls_, request_deadline(timeouts_.metadata_read_ms)});
  }
  const auto & result = reply.result;

  auto transport_failure = [&]() {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started_at);
    const auto kind =
        classify_peer_error(result.error(), elapsed, std::chrono::milliseconds(timeouts_.metadata_read_ms),
                            std::chrono::milliseconds(timeouts_.write_ms));
    std::string message = kind == PeerFailureKind::kTimeout
                              ? "Peer '" + name_ + "' at " + url_ + " did not answer " + method + " " + path +
                                    " within " +
                                    describe_budget(peer_budget_for(result.error()), timeouts_,
                                                    "aggregation.timeout_ms", timeouts_.metadata_read_ms)
                              : "Failed to reach peer '" + name_ + "' at " + url_ + " for " + method + " " + path;
    return PeerError{kind, std::move(message)};
  };

  // A 404 carrying `entity-not-found` is the peer saying it holds nothing by the
  // id in the path. A caller asking several peers about one entity reads that
  // as "nothing from this peer", not as a call that failed.
  auto status_failure = [&](int status, const std::string & body) {
    auto kind = PeerFailureKind::kErrorStatus;
    if (status == 404 && judge_peer_json(body) == PeerJsonShape::kAcceptable) {
      auto error = nlohmann::json::parse(body, nullptr, false);
      if (error.is_object() && string_at(error, "error_code") == ERR_ENTITY_NOT_FOUND) {
        kind = PeerFailureKind::kEntityNotFound;
      }
    }
    return PeerError{kind,
                     "Peer '" + name_ + "' returned status " + std::to_string(status) + " for " + method + " " + path};
  };

  // The size guard stops the read from inside the content receiver, which
  // cpp-httplib surfaces as a cancelled read. Checked before the transport
  // error so an oversized body is reported as one rather than as a broken
  // connection.
  if (reply.too_large) {
    return tl::unexpected(PeerError{PeerFailureKind::kTooLarge, "Response from peer '" + name_ +
                                                                    "' exceeds size limit for " + method + " " + path});
  }
  if (reply.past_deadline) {
    return tl::unexpected(PeerError{PeerFailureKind::kTimeout, "Peer '" + name_ + "' at " + url_ +
                                                                   " did not finish answering " + method + " " + path +
                                                                   " within " + describe_deadline(reply.deadline)});
  }
  if (!result) {
    return tl::unexpected(transport_failure());
  }
  if (result->status < 200 || result->status >= 300) {
    return tl::unexpected(status_failure(result->status, result->body));
  }
  const std::string & body_text = result->body;

  // The caller copies, merges and serializes what comes back, all of which
  // recurse once per level of nesting, and the parsed body costs many times
  // its text: a body nested or filled past the bounds is not the JSON any
  // route promises, and is refused before it is parsed.
  if (const auto shape = judge_peer_json(body_text); shape != PeerJsonShape::kAcceptable) {
    return tl::unexpected(PeerError{PeerFailureKind::kInvalidResponse, "Response from peer '" + name_ + "' for " +
                                                                           method + " " + path + " " +
                                                                           describe_peer_json_shape(shape)});
  }
  auto parsed = nlohmann::json::parse(body_text, nullptr, false);
  if (parsed.is_discarded()) {
    return tl::unexpected(PeerError{PeerFailureKind::kInvalidResponse,
                                    "Invalid JSON response from peer '" + name_ + "' for " + method + " " + path});
  }

  return parsed;
}

void PeerClient::shutdown() {
  if (shutdown_requested_.exchange(true)) {
    return;
  }
  // Every client in flight is stopped with the registry mutex held, as the
  // reaper stops one at its deadline (PeerCallRegistry::stop_all).
  //
  // Exclusive admission waits out every call between its check of the flag
  // and its socket, so each call is either refused or in flight below. A
  // call still resolving or connecting holds its client's socket lock, so
  // the wait, and the stop() of that client, last until the connect ends,
  // which aggregation.timeout_ms bounds.
  std::unique_lock<std::shared_mutex> admission(admission_mutex_);
  calls_.stop_all();
}

std::chrono::milliseconds PeerClient::request_deadline(int read_ms) const {
  return std::chrono::milliseconds(
      std::max(timeouts_.request_deadline_ms, timeouts_.connect_ms + timeouts_.write_ms + read_ms));
}

PeerCallRegistry::PeerCallRegistry()
  : reaper_([this] {
    reap();
  }) {
}

PeerCallRegistry::~PeerCallRegistry() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  reaper_.join();
}

void PeerCallRegistry::add(httplib::Client * cli) {
  std::lock_guard<std::mutex> lock(mutex_);
  calls_.emplace(cli, Call{});
}

void PeerCallRegistry::remove(httplib::Client * cli) {
  std::lock_guard<std::mutex> lock(mutex_);
  calls_.erase(cli);
}

void PeerCallRegistry::arm(httplib::Client * cli, std::chrono::milliseconds deadline) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto call = calls_.find(cli);
    if (call == calls_.end()) {
      return;
    }
    call->second = Call{std::chrono::steady_clock::now() + deadline, false};
  }
  wake_.notify_all();
}

bool PeerCallRegistry::disarm(httplib::Client * cli) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto call = calls_.find(cli);
  if (call == calls_.end()) {
    return false;
  }
  const bool expired = call->second.expired;
  call->second = Call{};
  return expired;
}

void PeerCallRegistry::stop_all() {
  // stop() runs with the mutex held. A worker removes its client in
  // ~ScopedClient, before the client itself is destroyed, and that removal
  // takes the same mutex - so a client cannot be destroyed while this loop is
  // calling stop() on it. Calling stop() on pointers copied out from under the
  // mutex left exactly that window open: a worker whose call returned in
  // between destroyed the client the loop was about to stop. Holding the mutex
  // costs a worker that starts or ends a call meanwhile a wait of one stop()
  // per active client; stop() only shuts the socket down and never waits for
  // the call it cancels, and nothing that stop() locks is held by a worker
  // while it waits for this mutex.
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & call : calls_) {
    call.first->stop();
  }
}

void PeerCallRegistry::reap() {
  // A stop() that comes before the request has its socket shuts nothing, so
  // a request stopped at its deadline is stopped again this often until it
  // returns and disarms. The deadline is never shorter than the connect
  // budget, and stop() waits for a connect in progress, so this is a guard,
  // not the path a request normally takes.
  constexpr auto kRestop = std::chrono::milliseconds(100);
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stopping_) {
    const auto now = std::chrono::steady_clock::now();
    auto next = std::chrono::steady_clock::time_point::max();
    for (auto & [cli, call] : calls_) {
      if (call.deadline <= now) {
        cli->stop();
        call.expired = true;
        call.deadline = now + kRestop;
      }
      next = std::min(next, call.deadline);
    }
    if (next == std::chrono::steady_clock::time_point::max()) {
      wake_.wait(lock);
    } else {
      wake_.wait_until(lock, next);
    }
  }
}

PeerClient::ScopedClient::ScopedClient(PeerClient & owner, const std::string & url, int read_timeout_ms)
  : owner_(owner), cli_(url) {
  const int connect_ms = owner.timeouts_.connect_ms;
  const int write_ms = owner.timeouts_.write_ms;
  cli_.set_connection_timeout(connect_ms / 1000, (connect_ms % 1000) * 1000);
  cli_.set_read_timeout(read_timeout_ms / 1000, (read_timeout_ms % 1000) * 1000);
  // Without this the write budget is cpp-httplib's own 5 s default and follows
  // no configured value at all, so a large request body to a slow peer is cut
  // off however high the read budget is set.
  cli_.set_write_timeout(write_ms / 1000, (write_ms % 1000) * 1000);
  owner_.calls_.add(&cli_);
}

PeerClient::ScopedClient::~ScopedClient() {
  owner_.calls_.remove(&cli_);
}

}  // namespace ros2_medkit_gateway
