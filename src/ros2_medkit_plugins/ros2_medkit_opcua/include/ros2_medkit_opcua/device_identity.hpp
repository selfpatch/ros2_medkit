// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/opcua_client.hpp"

#include <ros2_medkit_gateway/core/discovery/models/asset_identity.hpp>

#include <string>

namespace ros2_medkit_gateway {

/// Map an OPC-UA server DeviceInfo (ServerStatus/BuildInfo + optional DI
/// nameplate) onto an AssetIdentity, stamping per-field provenance to "opcua".
///
/// DI nameplate values are device-specific, so they take precedence over the
/// server-level BuildInfo for manufacturer / model / software version.
/// BuildInfo.BuildNumber has no typed field and is carried as the ``extra``
/// entry ``buildNumber``. ``endpoint_url`` populates the network endpoint.
/// Empty inputs are skipped, so the result is ::AssetIdentity::empty() when the
/// server exposes no device-info at all.
AssetIdentity opcua_device_info_to_identity(const OpcuaClient::DeviceInfo & info, const std::string & endpoint_url);

/// Whether a live nameplate read over this connection may outrank the
/// operator-authored manifest in the identity merge.
///
/// True only when the server's identity is actually authenticated: a secured
/// SecureChannel (Sign / SignAndEncrypt) AND certificate validation against the
/// trust list (``reject_untrusted``). An unsecured channel, or a secured one
/// with ``reject_untrusted: false`` (accept-any cert), can be served by a rogue
/// endpoint, so its nameplate must not override the manifest - the plugin then
/// tags the component with the generic "plugin" source, which ranks below
/// "manifest" (it fills gaps but never overrides). Per-field provenance stays
/// "opcua" either way for transparency about where a value was read.
bool opcua_identity_trusted(const OpcuaClientConfig & config);

}  // namespace ros2_medkit_gateway
