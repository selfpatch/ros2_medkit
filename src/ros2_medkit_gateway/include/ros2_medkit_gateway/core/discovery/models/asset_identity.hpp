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

#include <nlohmann/json.hpp>

#include <map>
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief Asset-identity (nameplate) of a physical or logical asset behind a Component.
 *
 * Captures the stable identity of the thing a Component represents (an ECU, a PLC,
 * a drive, a sensor) as opposed to its runtime ROS 2 footprint (topics/services).
 * One source rarely knows everything: a manifest may carry manufacturer + role, a
 * protocol device-info read (OPC UA nameplate, S7 order code, EtherNet/IP Identity
 * Object) fills in serial and firmware. The fields below are merged across sources
 * by ::merge_identity, which records per-field provenance.
 *
 * Design: typed fields AND a generic `extra` map are intentional. Typed fields are
 * the common, queryable, AAS-mappable identity attributes; `extra` is the escape
 * hatch for vendor-specific keys we do not want to model up front (rack/slot,
 * MAC address, asset tag, ...). Both are merged with the same precedence rules.
 *
 * AAS alignment (IEC 63278) so the model maps cleanly to/from an Asset
 * Administration Shell without rework:
 *   - the asset itself      <-> the AAS / Submodel "shell" (Component.id == localId)
 *   - this identity block    <-> Nameplate submodel (IDTA 02006)
 *       manufacturer         <-> ManufacturerName
 *       model                <-> ManufacturerProductDesignation (a.k.a. order code)
 *       serial_number        <-> SerialNumber
 *       hardware_revision    <-> HardwareVersion
 *       firmware_version     <-> FirmwareVersion
 *       software_version     <-> SoftwareVersion
 *       network_endpoint     <-> Asset Interface Description (AID) endpoint
 *       role                 <-> functional role (BoM/usage context)
 *       extra                <-> additional Nameplate properties
 *   - Component hierarchy    <-> Bill of Material (BoM) submodel
 */
struct AssetIdentity {
  std::string manufacturer;       ///< Manufacturer / vendor name (AAS ManufacturerName)
  std::string model;              ///< Product designation / order code (AAS ManufacturerProductDesignation)
  std::string serial_number;      ///< Unit serial number (AAS SerialNumber)
  std::string hardware_revision;  ///< Hardware revision (AAS HardwareVersion)
  std::string firmware_version;   ///< Firmware version (AAS FirmwareVersion)
  std::string software_version;   ///< Software/application version (AAS SoftwareVersion)
  std::string network_endpoint;   ///< Network endpoint, e.g. "opc.tcp://host:4840" (AAS AID)
  std::string role;               ///< Functional role of the asset (e.g. "plc", "drive")

  std::map<std::string, std::string> extra;  ///< Vendor-specific extras (rack/slot, MAC, asset tag, ...)

  /// Per-field source provenance: field name -> source that set it.
  /// Typed fields use their snake_case names ("serial_number"); `extra` entries
  /// use the key prefixed with "extra." ("extra.slot").
  std::map<std::string, std::string> provenance;

  /// True when no identity information is present (typed fields + extras all empty).
  /// Provenance alone does not make an identity non-empty.
  bool empty() const {
    return manufacturer.empty() && model.empty() && serial_number.empty() && hardware_revision.empty() &&
           firmware_version.empty() && software_version.empty() && network_endpoint.empty() && role.empty() &&
           extra.empty();
  }

  /**
   * @brief Serialize to JSON (camelCase keys, only non-empty fields emitted).
   *
   * Provenance is emitted under "_provenance" when present so consumers can audit
   * which source set each field. Returns an empty object when ::empty().
   */
  json to_json() const {
    json j = json::object();
    if (!manufacturer.empty()) {
      j["manufacturer"] = manufacturer;
    }
    if (!model.empty()) {
      j["model"] = model;
    }
    if (!serial_number.empty()) {
      j["serialNumber"] = serial_number;
    }
    if (!hardware_revision.empty()) {
      j["hardwareRevision"] = hardware_revision;
    }
    if (!firmware_version.empty()) {
      j["firmwareVersion"] = firmware_version;
    }
    if (!software_version.empty()) {
      j["softwareVersion"] = software_version;
    }
    if (!network_endpoint.empty()) {
      j["networkEndpoint"] = network_endpoint;
    }
    if (!role.empty()) {
      j["role"] = role;
    }
    if (!extra.empty()) {
      j["extra"] = extra;
    }
    if (!provenance.empty()) {
      j["_provenance"] = provenance;
    }
    return j;
  }

  /**
   * @brief Parse from JSON produced by ::to_json (camelCase keys). Tolerant of
   * missing keys and of a null/empty object.
   */
  static AssetIdentity from_json(const json & j) {
    AssetIdentity id;
    if (!j.is_object()) {
      return id;
    }
    auto get_str = [&](const char * key, std::string & dst) {
      auto it = j.find(key);
      if (it != j.end() && it->is_string()) {
        dst = it->get<std::string>();
      }
    };
    get_str("manufacturer", id.manufacturer);
    get_str("model", id.model);
    get_str("serialNumber", id.serial_number);
    get_str("hardwareRevision", id.hardware_revision);
    get_str("firmwareVersion", id.firmware_version);
    get_str("softwareVersion", id.software_version);
    get_str("networkEndpoint", id.network_endpoint);
    get_str("role", id.role);
    if (auto it = j.find("extra"); it != j.end() && it->is_object()) {
      for (auto & [k, v] : it->items()) {
        if (v.is_string()) {
          id.extra[k] = v.get<std::string>();
        }
      }
    }
    if (auto it = j.find("_provenance"); it != j.end() && it->is_object()) {
      for (auto & [k, v] : it->items()) {
        if (v.is_string()) {
          id.provenance[k] = v.get<std::string>();
        }
      }
    }
    return id;
  }
};

inline bool operator==(const AssetIdentity & a, const AssetIdentity & b) {
  return a.manufacturer == b.manufacturer && a.model == b.model && a.serial_number == b.serial_number &&
         a.hardware_revision == b.hardware_revision && a.firmware_version == b.firmware_version &&
         a.software_version == b.software_version && a.network_endpoint == b.network_endpoint && a.role == b.role &&
         a.extra == b.extra && a.provenance == b.provenance;
}
inline bool operator!=(const AssetIdentity & a, const AssetIdentity & b) {
  return !(a == b);
}

}  // namespace ros2_medkit_gateway
