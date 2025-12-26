// Copyright 2025 bburda
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

#include "ros2_medkit_gateway/auth_models.hpp"

#include <sstream>

namespace ros2_medkit_gateway {

// URL-decode a string (handle %XX encoding)
static std::string url_decode(const std::string & encoded) {
  std::string decoded;
  decoded.reserve(encoded.size());

  for (size_t i = 0; i < encoded.size(); ++i) {
    if (encoded[i] == '%' && i + 2 < encoded.size()) {
      int hex_val = 0;
      std::istringstream hex_stream(encoded.substr(i + 1, 2));
      hex_stream >> std::hex >> hex_val;
      decoded += static_cast<char>(hex_val);
      i += 2;
    } else if (encoded[i] == '+') {
      decoded += ' ';
    } else {
      decoded += encoded[i];
    }
  }

  return decoded;
}

AuthorizeRequest AuthorizeRequest::from_form_data(const std::string & body) {
  AuthorizeRequest req;
  std::istringstream stream(body);
  std::string pair;

  while (std::getline(stream, pair, '&')) {
    size_t eq_pos = pair.find('=');
    if (eq_pos != std::string::npos) {
      std::string key = url_decode(pair.substr(0, eq_pos));
      std::string value = url_decode(pair.substr(eq_pos + 1));

      if (key == "grant_type") {
        req.grant_type = value;
      } else if (key == "client_id") {
        req.client_id = value;
      } else if (key == "client_secret") {
        req.client_secret = value;
      } else if (key == "refresh_token") {
        req.refresh_token = value;
      } else if (key == "scope") {
        req.scope = value;
      }
    }
  }

  return req;
}

}  // namespace ros2_medkit_gateway
