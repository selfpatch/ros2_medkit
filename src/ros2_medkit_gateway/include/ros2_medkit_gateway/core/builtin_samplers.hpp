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

namespace ros2_medkit_gateway {

class GatewayServices;

/// Registers the built-in cyclic-subscription samplers on `services.sampler_registry()`:
/// `data`, `faults`, `configurations` and `logs`, plus `updates` when `services.update_manager()`
/// is set. Each sampler reads through the managers and helpers of the matching GET route, so a
/// resource the route serves can be subscribed to, whoever provides it. A collection the host has
/// no manager for answers with the error text of its GET route.
///
/// `services` must outlive every sampler: the samplers keep a reference and read the manager
/// getters on each call. Call it once, before the first subscription is created.
void register_builtin_samplers(GatewayServices & services);

}  // namespace ros2_medkit_gateway
