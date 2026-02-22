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

#include "ros2_medkit_gateway/http/handlers/update_handlers.hpp"

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

UpdateHandlers::UpdateHandlers(HandlerContext & ctx, UpdateManager * update_manager)
  : ctx_(ctx), update_mgr_(update_manager) {
}

bool UpdateHandlers::check_backend(httplib::Response & res) {
  if (!update_mgr_ || !update_mgr_->has_backend()) {
    HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Software updates backend not configured");
    return false;
  }
  return true;
}

json UpdateHandlers::status_to_json(const UpdateStatusInfo & status) {
  json j;
  switch (status.status) {
    case UpdateStatus::Pending:
      j["status"] = "pending";
      break;
    case UpdateStatus::InProgress:
      j["status"] = "inProgress";
      break;
    case UpdateStatus::Completed:
      j["status"] = "completed";
      break;
    case UpdateStatus::Failed:
      j["status"] = "failed";
      break;
  }
  if (status.progress.has_value()) {
    j["progress"] = *status.progress;
  }
  if (status.sub_progress.has_value()) {
    j["sub_progress"] = json::array();
    for (const auto & sp : *status.sub_progress) {
      j["sub_progress"].push_back({{"name", sp.name}, {"progress", sp.progress}});
    }
  }
  if (status.error_message.has_value()) {
    json err;
    err["error_code"] = ERR_INTERNAL_ERROR;
    err["message"] = *status.error_message;
    j["error"] = err;
  }
  return j;
}

void UpdateHandlers::handle_list_updates(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    UpdateFilter filter;
    if (req.has_param("origin")) {
      filter.origin = req.get_param_value("origin");
    }
    if (req.has_param("target-version")) {
      filter.target_version = req.get_param_value("target-version");
    }

    auto result = update_mgr_->list_updates(filter);
    if (!result) {
      HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, result.error());
      return;
    }

    json response;
    response["items"] = *result;
    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_get_update(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto id = req.matches[1].str();
    auto result = update_mgr_->get_update(id);
    if (!result) {
      HandlerContext::send_error(res, 404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error());
      return;
    }
    HandlerContext::send_json(res, *result);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_register_update(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error &) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON body");
      return;
    }

    auto result = update_mgr_->register_update(body);
    if (!result) {
      // Distinguish "already exists" from "missing fields"
      if (result.error().find("already exists") != std::string::npos) {
        HandlerContext::send_error(res, 400, ERR_X_MEDKIT_UPDATE_ALREADY_EXISTS, result.error());
      } else {
        HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, result.error());
      }
      return;
    }

    auto id = body.value("id", std::string{});
    res.status = 201;
    res.set_header("Location", api_path("/updates/" + id));
    res.set_header("Content-Type", "application/json");
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_delete_update(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto id = req.matches[1].str();
    auto result = update_mgr_->delete_update(id);
    if (!result) {
      if (result.error().find("in progress") != std::string::npos) {
        HandlerContext::send_error(res, 405, ERR_X_MEDKIT_UPDATE_IN_PROGRESS, result.error());
      } else if (result.error().find("not found") != std::string::npos) {
        HandlerContext::send_error(res, 404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error());
      } else {
        HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, result.error());
      }
      return;
    }
    res.status = 204;
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_prepare(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto id = req.matches[1].str();
    auto result = update_mgr_->start_prepare(id);
    if (!result) {
      if (result.error().find("not found") != std::string::npos) {
        HandlerContext::send_error(res, 404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error());
      } else {
        HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, result.error());
      }
      return;
    }
    res.status = 202;
    res.set_header("Location", api_path("/updates/" + id + "/status"));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_execute(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto id = req.matches[1].str();
    auto result = update_mgr_->start_execute(id);
    if (!result) {
      if (result.error().find("not found") != std::string::npos) {
        HandlerContext::send_error(res, 404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error());
      } else if (result.error().find("must be prepared") != std::string::npos) {
        HandlerContext::send_error(res, 400, ERR_X_MEDKIT_UPDATE_NOT_PREPARED, result.error());
      } else {
        HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, result.error());
      }
      return;
    }
    res.status = 202;
    res.set_header("Location", api_path("/updates/" + id + "/status"));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_automated(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto id = req.matches[1].str();
    auto result = update_mgr_->start_automated(id);
    if (!result) {
      if (result.error().find("not found") != std::string::npos) {
        HandlerContext::send_error(res, 404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error());
      } else if (result.error().find("does not support") != std::string::npos) {
        HandlerContext::send_error(res, 400, ERR_X_MEDKIT_UPDATE_NOT_AUTOMATED, result.error());
      } else {
        HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, result.error());
      }
      return;
    }
    res.status = 202;
    res.set_header("Location", api_path("/updates/" + id + "/status"));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

void UpdateHandlers::handle_get_status(const httplib::Request & req, httplib::Response & res) {
  if (!check_backend(res)) {
    return;
  }

  try {
    auto id = req.matches[1].str();
    auto result = update_mgr_->get_status(id);
    if (!result) {
      HandlerContext::send_error(res, 404, ERR_X_MEDKIT_UPDATE_NOT_FOUND, result.error());
      return;
    }
    HandlerContext::send_json(res, status_to_json(*result));
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
