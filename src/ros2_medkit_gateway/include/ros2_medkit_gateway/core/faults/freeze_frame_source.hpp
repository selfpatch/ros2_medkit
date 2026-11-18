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

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {

/// Read side of the entity freeze-frames served on fault detail and trigger
/// values. Implementations must be thread-safe.
class FreezeFrameSource {
 public:
  /// Capture-path identifiers stored in Frame::source and served as
  /// ``x-medkit.source``. The plugin's own DataProvider, and the in-process
  /// dispatch of the plugin's `x-plc-data` route for plugins that export no
  /// DataProvider.
  static constexpr const char * kSourceDataProvider = "plugin_data_provider";
  static constexpr const char * kSourceXPlcDataRoute = "plugin_x_plc_data_route";

  /// One captured frame: the entity's data values at fault-confirm time.
  /// captured_at_ns dates the capture, not the values - a disconnected entity
  /// serves its last known values, whose age is bounded only by the outage.
  struct Frame {
    std::string entity_id;
    nlohmann::json values;  ///< compact {resource_id: value} dict
    int64_t captured_at_ns{0};
    /// True when the frame comes from the startup catch-up: captured_at_ns is
    /// then the gateway's start, possibly long after the fault confirmed.
    bool startup_catchup{false};
    std::optional<bool> connected;    ///< payload's top-level link flag, when reported
    nlohmann::json source_timestamp;  ///< payload's own "timestamp" field verbatim (null when absent)
    /// Which capture path read the values (kSourceDataProvider /
    /// kSourceXPlcDataRoute), served as ``x-medkit.source``. These values are
    /// entity data, not a ROS message, so ``topic`` and ``message_type`` are
    /// empty on the wire and would otherwise leave a consumer with nothing at
    /// all saying where the numbers came from. Empty when the caller named no
    /// path.
    std::string source;
  };

  FreezeFrameSource() = default;
  virtual ~FreezeFrameSource() = default;
  FreezeFrameSource(const FreezeFrameSource &) = delete;
  FreezeFrameSource & operator=(const FreezeFrameSource &) = delete;
  FreezeFrameSource(FreezeFrameSource &&) = delete;
  FreezeFrameSource & operator=(FreezeFrameSource &&) = delete;

  /// Frames captured for a fault code (empty when none). Thread-safe.
  virtual std::vector<Frame> frames_for(const std::string & fault_code) const = 0;

  /// Build the compact {resource_id: value} dict from a DataProvider::list_data
  /// response. Items without a "value" field map to null; a response without an
  /// "items" array is kept verbatim (plugin-defined shape). Total: malformed
  /// items (non-string id/name) are skipped, never thrown on.
  static nlohmann::json values_from_list_content(const nlohmann::json & content);

  /// True when list-data-shaped content (DataProvider::list_data or x-plc-data
  /// route) carries values at all: a non-empty items array. The entity's
  /// `connected` flag is deliberately not consulted - a bridge serving its last
  /// known values while the link is down is the case a loss-of-comms fault most
  /// needs frozen. The row of nulls a cold cache yields is rejected by
  /// values_have_data() instead.
  static bool content_has_live_data(const nlohmann::json & content);

  /// True when the payload's top-level `connected` flag reports the link down.
  /// Consulted only by the gateway's fault-trigger value fetcher: a down link
  /// serves frozen last-known values, and a threshold rule must hold state on
  /// them (fetcher yields nullopt) instead of firing on a stale number for the
  /// whole outage. The freeze-frame paths deliberately ignore this flag.
  static bool content_reports_disconnected(const nlohmann::json & content);

  /// True when a compact values dict holds at least one non-null value.
  /// Rejects the {} / all-null rows a cold poll cache or dead link yields.
  static bool values_have_data(const nlohmann::json & values);
};

}  // namespace ros2_medkit_gateway
