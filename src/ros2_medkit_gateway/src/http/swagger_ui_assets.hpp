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

#pragma once

#include <cstddef>

namespace ros2_medkit_gateway {
namespace swagger_ui {

/// Embedded swagger-ui-bundle.js (minified)
extern const unsigned char swagger_ui_bundle_js[];
extern const size_t swagger_ui_bundle_js_size;

/// Embedded swagger-ui.css
extern const unsigned char swagger_ui_css[];
extern const size_t swagger_ui_css_size;

/// Embedded swagger-ui-standalone-preset.js (minified)
extern const unsigned char swagger_ui_standalone_preset_js[];
extern const size_t swagger_ui_standalone_preset_js_size;

/// Whether assets were successfully embedded at build time.
/// When false, handlers should serve a CDN-based fallback page.
extern const bool assets_embedded;

}  // namespace swagger_ui
}  // namespace ros2_medkit_gateway
