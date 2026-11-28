// Copyright 2026 ahmedsleem109
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
#include <utility>
#include <vector>

namespace ros2_medkit_policy_bridge::core {

/// The state each fault code should have at the FaultManager, and the state
/// last sent for it. The node sends only the difference, and only while the
/// FaultManager service can be reached, so:
///
///   - at start nothing has been sent, so every code is sent once as PASSED:
///     a fault left CONFIRMED by an earlier run of the bridge heals;
///   - a fault raised while the FaultManager is missing is sent when it
///     appears;
///   - when the FaultManager appears again (it restarted), forget_sent() makes
///     every code due again, so a raise it lost is sent once more.
///
/// Only the latest state of a code is kept, so the memory is one entry per
/// code. A raise followed by a pass while the FaultManager cannot be reached
/// sends nothing: the fault came and went where nobody could record it.
class FaultSync {
 public:
  struct Report {
    std::string code;
    bool raise{false};  // false = PASSED
    uint8_t severity{0};
    std::string description;
  };

  explicit FaultSync(const std::vector<std::string> & codes) {
    for (const auto & code : codes) {
      states_.emplace_back(code, State{});
    }
  }

  /// Set the state a code should have. Unknown codes are ignored.
  void set(const std::string & code, bool raise, uint8_t severity, std::string description) {
    for (auto & [name, state] : states_) {
      if (name == code) {
        state.raise = raise;
        state.severity = severity;
        state.description = std::move(description);
        return;
      }
    }
  }

  /// Reports that would bring the FaultManager to the wanted state, in the
  /// order the codes were given.
  std::vector<Report> due() const {
    std::vector<Report> out;
    for (const auto & [name, state] : states_) {
      if (!state.sent || *state.sent != state.raise) {
        out.push_back({name, state.raise, state.severity, state.description});
      }
    }
    return out;
  }

  /// Record that a report from due() was handed to the FaultManager client.
  void mark_sent(const Report & report) {
    for (auto & [name, state] : states_) {
      if (name == report.code) {
        state.sent = report.raise;
        return;
      }
    }
  }

  /// The FaultManager may not hold what was sent (it restarted): send every
  /// code's state again.
  void forget_sent() {
    for (auto & entry : states_) {
      entry.second.sent.reset();
    }
  }

  bool raised(const std::string & code) const {
    for (const auto & [name, state] : states_) {
      if (name == code) {
        return state.raise;
      }
    }
    return false;
  }

 private:
  struct State {
    bool raise{false};
    uint8_t severity{0};
    std::string description;
    std::optional<bool> sent;  // empty: nothing sent since start or since forget_sent()
  };
  std::vector<std::pair<std::string, State>> states_;
};

}  // namespace ros2_medkit_policy_bridge::core
