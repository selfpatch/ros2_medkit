// Copyright 2026 Bartlomiej Burda
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

#include "ros2_medkit_gateway/http/handlers/script_handlers.hpp"

#include <algorithm>
#include <cctype>

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

static constexpr size_t kMaxScriptSizeBytes = 10 * 1024 * 1024;  // 10 MB

ScriptHandlers::ScriptHandlers(HandlerContext & ctx, ScriptManager * script_manager)
  : ctx_(ctx), script_mgr_(script_manager) {
}

bool ScriptHandlers::check_backend(httplib::Response & res) {
  if (!script_mgr_ || !script_mgr_->has_backend()) {
    HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Scripts backend not configured");
    return false;
  }
  return true;
}

bool ScriptHandlers::is_valid_resource_id(const std::string & id) {
  if (id.empty() || id.size() > 256) {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}

void ScriptHandlers::send_script_error(httplib::Response & res, const ScriptBackendErrorInfo & err) {
  switch (err.code) {
    case ScriptBackendError::NotFound:
      HandlerContext::send_error(res, 404, ERR_SCRIPT_NOT_FOUND, err.message);
      break;
    case ScriptBackendError::AlreadyExists:
      HandlerContext::send_error(res, 409, ERR_SCRIPT_ALREADY_EXISTS, err.message);
      break;
    case ScriptBackendError::ManagedScript:
      HandlerContext::send_error(res, 409, ERR_SCRIPT_MANAGED, err.message);
      break;
    case ScriptBackendError::AlreadyRunning:
      HandlerContext::send_error(res, 409, ERR_SCRIPT_RUNNING, err.message);
      break;
    case ScriptBackendError::NotRunning:
      HandlerContext::send_error(res, 409, ERR_SCRIPT_NOT_RUNNING, err.message);
      break;
    case ScriptBackendError::InvalidInput:
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, err.message);
      break;
    case ScriptBackendError::UnsupportedType:
      HandlerContext::send_error(res, 400, ERR_SCRIPT_UNSUPPORTED_TYPE, err.message);
      break;
    case ScriptBackendError::FileTooLarge:
      HandlerContext::send_error(res, 413, ERR_SCRIPT_FILE_TOO_LARGE, err.message);
      break;
    case ScriptBackendError::ConcurrencyLimit:
      HandlerContext::send_error(res, 429, ERR_SCRIPT_CONCURRENCY_LIMIT, err.message);
      break;
    case ScriptBackendError::Internal:
    default:
      HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, err.message);
      break;
  }
}

json ScriptHandlers::script_info_to_json(const ScriptInfo & info, const std::string & base_path) {
  json obj;
  obj["id"] = info.id;
  obj["name"] = info.name;
  obj["description"] = info.description;
  obj["href"] = api_path(base_path + "/scripts/" + info.id);
  obj["managed"] = info.managed;
  obj["proximity_proof_required"] = info.proximity_proof_required;
  if (info.parameters_schema.has_value()) {
    obj["parameters_schema"] = info.parameters_schema.value();
  } else {
    obj["parameters_schema"] = nullptr;
  }
  return obj;
}

json ScriptHandlers::execution_info_to_json(const ExecutionInfo & info) {
  json obj;
  obj["id"] = info.id;
  obj["status"] = info.status;
  if (info.progress.has_value()) {
    obj["progress"] = info.progress.value();
  } else {
    obj["progress"] = nullptr;
  }
  if (info.started_at.has_value()) {
    obj["started_at"] = info.started_at.value();
  } else {
    obj["started_at"] = nullptr;
  }
  if (info.completed_at.has_value()) {
    obj["completed_at"] = info.completed_at.value();
  } else {
    obj["completed_at"] = nullptr;
  }
  if (info.output_parameters.has_value()) {
    obj["parameters"] = info.output_parameters.value();
  } else {
    obj["parameters"] = nullptr;
  }
  if (info.error.has_value()) {
    obj["error"] = info.error.value();
  } else {
    obj["error"] = nullptr;
  }
  return obj;
}

void ScriptHandlers::handle_list_scripts(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    std::string entity_type_segment = (req.path.find("/components/") != std::string::npos) ? "components" : "apps";
    auto base_path = "/" + entity_type_segment + "/" + entity_id;

    auto result = script_mgr_->list_scripts(entity_id);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    json items = json::array();
    for (const auto & info : *result) {
      items.push_back(script_info_to_json(info, base_path));
    }
    HandlerContext::send_json(res, json{{"items", items}});
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_upload_script(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto file_it = req.files.find("file");
    if (file_it == req.files.end()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Missing required multipart field: file");
      return;
    }

    const auto & file_part = file_it->second;
    if (file_part.content.size() > kMaxScriptSizeBytes) {
      HandlerContext::send_error(res, 413, ERR_SCRIPT_FILE_TOO_LARGE, "Script file exceeds maximum allowed size");
      return;
    }

    std::optional<json> metadata;
    auto meta_it = req.files.find("metadata");
    if (meta_it != req.files.end()) {
      try {
        metadata = json::parse(meta_it->second.content);
      } catch (const json::parse_error &) {
        HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON in metadata field");
        return;
      }
    }

    auto result = script_mgr_->upload_script(entity_id, file_part.filename, file_part.content, metadata);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    std::string entity_type_segment = (req.path.find("/components/") != std::string::npos) ? "components" : "apps";
    auto script_path = "/api/v1/" + entity_type_segment + "/" + entity_id + "/scripts/" + result->id;

    HandlerContext::send_json(res, json{{"id", result->id}, {"name", result->name}});
    res.status = 201;
    res.set_header("Location", script_path);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_get_script(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto script_id = req.matches[2].str();
    if (!is_valid_resource_id(script_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid script ID format");
      return;
    }
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    std::string entity_type_segment = (req.path.find("/components/") != std::string::npos) ? "components" : "apps";
    auto base_path = "/" + entity_type_segment + "/" + entity_id;

    auto result = script_mgr_->get_script(entity_id, script_id);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    HandlerContext::send_json(res, script_info_to_json(*result, base_path));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_delete_script(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto script_id = req.matches[2].str();
    if (!is_valid_resource_id(script_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid script ID format");
      return;
    }
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto result = script_mgr_->delete_script(entity_id, script_id);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    res.status = 204;
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_start_execution(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto script_id = req.matches[2].str();
    if (!is_valid_resource_id(script_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid script ID format");
      return;
    }
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error &) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON body");
      return;
    }

    if (!body.contains("execution_type") || !body["execution_type"].is_string() ||
        body["execution_type"].get<std::string>().empty()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Missing required field: execution_type");
      return;
    }

    ExecutionRequest exec_req;
    exec_req.execution_type = body["execution_type"].get<std::string>();

    if (body.contains("parameters") && !body["parameters"].is_null()) {
      exec_req.parameters = body["parameters"];
    }

    if (body.contains("proximity_response") && body["proximity_response"].is_string()) {
      exec_req.proximity_response = body["proximity_response"].get<std::string>();
    }

    auto result = script_mgr_->start_execution(entity_id, script_id, exec_req);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    std::string entity_type_segment = (req.path.find("/components/") != std::string::npos) ? "components" : "apps";
    auto exec_path =
        "/api/v1/" + entity_type_segment + "/" + entity_id + "/scripts/" + script_id + "/executions/" + result->id;

    HandlerContext::send_json(res, execution_info_to_json(*result));
    res.status = 202;
    res.set_header("Location", exec_path);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_get_execution(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto script_id = req.matches[2].str();
    auto execution_id = req.matches[3].str();
    if (!is_valid_resource_id(script_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid script ID format");
      return;
    }
    if (!is_valid_resource_id(execution_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid execution ID format");
      return;
    }
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto result = script_mgr_->get_execution(entity_id, script_id, execution_id);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    HandlerContext::send_json(res, execution_info_to_json(*result));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_control_execution(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto script_id = req.matches[2].str();
    auto execution_id = req.matches[3].str();
    if (!is_valid_resource_id(script_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid script ID format");
      return;
    }
    if (!is_valid_resource_id(execution_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid execution ID format");
      return;
    }
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error &) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON body");
      return;
    }

    if (!body.contains("action") || !body["action"].is_string() || body["action"].get<std::string>().empty()) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Missing required field: action");
      return;
    }

    auto action = body["action"].get<std::string>();
    auto result = script_mgr_->control_execution(entity_id, script_id, execution_id, action);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    HandlerContext::send_json(res, execution_info_to_json(*result));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void ScriptHandlers::handle_delete_execution(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto entity_id = req.matches[1].str();
    auto script_id = req.matches[2].str();
    auto execution_id = req.matches[3].str();
    if (!is_valid_resource_id(script_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid script ID format");
      return;
    }
    if (!is_valid_resource_id(execution_id)) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid execution ID format");
      return;
    }
    auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity) {
      return;
    }

    auto result = script_mgr_->delete_execution(entity_id, script_id, execution_id);
    if (!result) {
      send_script_error(res, result.error());
      return;
    }

    res.status = 204;
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
