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

/// Shared fault-detection model for ros2_medkit protocol plugins.
///
/// A single, protocol-agnostic evaluator maps a raw value read from any
/// source (OPC UA, S7, Modbus, ADS, ...) into the set of faults it implies,
/// using one of three composable detection modes:
///
///   * threshold   - numeric above/below a setpoint.
///   * status word - decode named bits of an integer status register.
///   * fault enum  - map a fault-code register value to a fault code + text.
///
/// The evaluator (``evaluate``) is a pure function of ``(value, rule)`` and
/// has no ROS / protocol dependencies, so it is trivially unit-testable and
/// safe to compile into a dlopen-loaded plugin MODULE. Raise/clear edge
/// detection is layered on top via ``FaultTransitionTracker``.
///
/// Header-only on purpose: protocol plugins are built as MODULE libraries
/// that resolve gateway symbols from the host process at load time
/// (``--unresolved-symbols=ignore-all``); a separately linked shared library
/// would not be present in that host, so the detection logic is compiled
/// directly into each plugin instead.

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

namespace ros2_medkit::fault_detection {

/// A raw value read from a protocol source.
using Value = std::variant<bool, std::int64_t, double, std::string>;

/// The identity + presentation of a fault a rule can raise.
struct FaultSpec {
  std::string fault_code;
  std::string severity{"ERROR"};  // "INFO" / "WARNING" / "ERROR" / "CRITICAL"
  std::string message;            // human-readable text; defaults to fault_code
};

/// Numeric threshold: raise ``fault`` when the value crosses ``threshold``.
struct ThresholdRule {
  FaultSpec fault;
  double threshold{0.0};
  bool above{true};  // true: active when value > threshold; false: value < threshold
};

/// One named bit of a status word.
struct BitRule {
  unsigned bit{0};  // bit position, 0 = least significant
  FaultSpec fault;
};

/// Status-word decode: one integer register, many named-bit faults.
struct StatusWordRule {
  std::vector<BitRule> bits;
};

/// One entry of a fault-code enum register.
struct EnumRule {
  std::int64_t code{0};
  FaultSpec fault;
};

/// Fault-code enum decode: a register value selects at most one fault.
struct EnumMapRule {
  std::vector<EnumRule> codes;
  std::int64_t ok_value{0};  // value meaning "no fault" (no rule raised)
};

/// One of the composable detection modes for a single mapped point.
using DetectionRule = std::variant<ThresholdRule, StatusWordRule, EnumMapRule>;

/// A fault the evaluator says is currently active or inactive for a value.
/// The evaluator reports the full set of faults a rule governs (so a cleared
/// fault is reported as ``active == false``); edge detection is done by
/// ``FaultTransitionTracker``.
struct FaultSignal {
  std::string fault_code;
  std::string severity;
  std::string message;
  bool active{false};
};

namespace detail {

/// Coerce a value to double. bool -> 0/1, integers/doubles as-is. Strings are
/// not numerically comparable here and yield ``false`` (handled by caller).
inline bool as_double(const Value & v, double & out) {
  return std::visit(
      [&out](auto && x) -> bool {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, bool>) {
          out = x ? 1.0 : 0.0;
          return true;
        } else if constexpr (std::is_same_v<T, std::int64_t> || std::is_same_v<T, double>) {
          out = static_cast<double>(x);
          return true;
        } else {
          return false;  // string
        }
      },
      v);
}

/// Coerce a value to a 64-bit integer for bit / enum decode. bool -> 0/1,
/// int as-is, double rounded to nearest. Strings are not decodable.
inline bool as_int(const Value & v, std::int64_t & out) {
  return std::visit(
      [&out](auto && x) -> bool {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, bool>) {
          out = x ? 1 : 0;
          return true;
        } else if constexpr (std::is_same_v<T, std::int64_t>) {
          out = x;
          return true;
        } else if constexpr (std::is_same_v<T, double>) {
          out = static_cast<std::int64_t>(std::llround(x));
          return true;
        } else {
          return false;  // string
        }
      },
      v);
}

inline FaultSignal make_signal(const FaultSpec & spec, bool active) {
  FaultSignal s;
  s.fault_code = spec.fault_code;
  s.severity = spec.severity.empty() ? std::string{"ERROR"} : spec.severity;
  s.message = spec.message.empty() ? spec.fault_code : spec.message;
  s.active = active;
  return s;
}

}  // namespace detail

/// Evaluate a value against one detection rule.
///
/// Returns the full set of faults the rule governs, each flagged active or
/// inactive. A threshold rule yields one signal; a status-word rule yields one
/// per configured bit; an enum rule yields one per configured code (the
/// matching code active, the rest inactive) so a transition tracker can clear
/// the previous code and raise the new one.
inline std::vector<FaultSignal> evaluate(const Value & value, const DetectionRule & rule) {
  std::vector<FaultSignal> out;

  std::visit(
      [&](auto && r) {
        using T = std::decay_t<decltype(r)>;

        if constexpr (std::is_same_v<T, ThresholdRule>) {
          bool active = false;
          if (std::holds_alternative<bool>(value)) {
            // A boolean point is alarm-on-true, threshold ignored. Preserves
            // the original OPC UA threshold-path behaviour for Bool tags.
            active = std::get<bool>(value);
          } else {
            double d = 0.0;
            if (detail::as_double(value, d)) {
              active = r.above ? (d > r.threshold) : (d < r.threshold);
            }
          }
          out.push_back(detail::make_signal(r.fault, active));

        } else if constexpr (std::is_same_v<T, StatusWordRule>) {
          std::int64_t raw = 0;
          const bool ok = detail::as_int(value, raw);
          const auto mask = static_cast<std::uint64_t>(raw);
          for (const auto & b : r.bits) {
            bool active = ok && b.bit < 64 && ((mask >> b.bit) & 0x1ULL);
            out.push_back(detail::make_signal(b.fault, active));
          }

        } else if constexpr (std::is_same_v<T, EnumMapRule>) {
          std::int64_t code = 0;
          const bool ok = detail::as_int(value, code);
          const bool healthy = ok && code == r.ok_value;
          for (const auto & e : r.codes) {
            bool active = ok && !healthy && code == e.code;
            out.push_back(detail::make_signal(e.fault, active));
          }
        }
      },
      rule);

  return out;
}

/// Stateful raise/clear edge detector over a stream of evaluator outputs.
///
/// Feed each ``evaluate`` result through ``apply``; it returns only the
/// signals whose active state changed since the last call (the raise and
/// clear transitions), which a plugin forwards to the fault manager as
/// report / clear. Keyed by ``fault_code``, so it tracks threshold, bit and
/// enum faults uniformly.
///
/// Contract: a single tracker may be shared across many points only if every
/// ``fault_code`` is globally unique. Because the key is the code alone, two
/// points emitting the same code would alternately raise and clear it each
/// cycle. Consumers that share one tracker (e.g. the OPC UA poller across all
/// node-map entries) must enforce that uniqueness at config-load time.
class FaultTransitionTracker {
 public:
  std::vector<FaultSignal> apply(const std::vector<FaultSignal> & signals) {
    std::vector<FaultSignal> changes;
    for (const auto & s : signals) {
      auto it = last_active_.find(s.fault_code);
      const bool known = it != last_active_.end();
      if (!known || it->second != s.active) {
        last_active_[s.fault_code] = s.active;
        // A fault unknown-and-inactive is not a transition (nothing was ever
        // raised); only emit a clear for a fault we had previously raised.
        if (s.active || known) {
          changes.push_back(s);
        }
      }
    }
    return changes;
  }

  /// Current active state of a fault (false if never seen).
  bool active(const std::string & fault_code) const {
    auto it = last_active_.find(fault_code);
    return it != last_active_.end() && it->second;
  }

 private:
  std::unordered_map<std::string, bool> last_active_;
};

}  // namespace ros2_medkit::fault_detection
