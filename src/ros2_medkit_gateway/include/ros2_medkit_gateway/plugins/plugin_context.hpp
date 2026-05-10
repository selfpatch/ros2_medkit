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

// Backwards-compatibility shim. The neutral PluginContext now lives under
// core/plugins/, the ROS-aware RosPluginContext (with node() and the
// make_gateway_plugin_context factory) lives next to this shim. Existing
// plugin code that includes the legacy path continues to see both classes.
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
