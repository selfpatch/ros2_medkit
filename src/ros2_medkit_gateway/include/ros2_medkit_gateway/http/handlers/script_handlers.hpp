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

#pragma once

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/script_manager.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

class ScriptHandlers {
 public:
  ScriptHandlers(HandlerContext & ctx, ScriptManager * script_manager);

  void handle_upload_script(const httplib::Request & req, httplib::Response & res);
  void handle_list_scripts(const httplib::Request & req, httplib::Response & res);
  void handle_get_script(const httplib::Request & req, httplib::Response & res);
  void handle_delete_script(const httplib::Request & req, httplib::Response & res);

  void handle_start_execution(const httplib::Request & req, httplib::Response & res);
  void handle_get_execution(const httplib::Request & req, httplib::Response & res);
  void handle_control_execution(const httplib::Request & req, httplib::Response & res);
  void handle_delete_execution(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
  ScriptManager * script_mgr_;

  bool check_backend(httplib::Response & res);
  void send_script_error(httplib::Response & res, const ScriptBackendErrorInfo & err);
  static nlohmann::json script_info_to_json(const ScriptInfo & info, const std::string & base_path);
  static nlohmann::json execution_info_to_json(const ExecutionInfo & info);
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
