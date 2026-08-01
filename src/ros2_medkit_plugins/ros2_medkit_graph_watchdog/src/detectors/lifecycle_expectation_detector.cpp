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
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_graph_watchdog/aggregated_fault.hpp"
#include "ros2_medkit_graph_watchdog/detector_config_keys.hpp"
#include "ros2_medkit_graph_watchdog/detector_registry.hpp"
#include "ros2_medkit_graph_watchdog/graph_fault_codes.hpp"
#include "ros2_medkit_graph_watchdog/lifecycle_expectation_tracker.hpp"
#include "ros2_medkit_graph_watchdog/reliability_gate.hpp"

namespace ros2_medkit_graph_watchdog {

namespace {
// Fault severity scale: SEVERITY_WARN=1, SEVERITY_ERROR=2 (ros2_medkit_msgs/msg/Fault.msg).
// A required-active node stuck inactive means the robot silently will not act to the
// outside world - as severe as node death.
constexpr std::uint8_t kLifecycleSeverity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
// Consecutive not-active ticks a required node tolerates before being reported inactive.
// Config-overridable via "grace" - gives a managed node bringup time to reach "active"
// (configure + activate) before the operator's expectation is enforced.
constexpr int kDefaultGrace = 5;
// Widest `grace` accepted, range-checked on the WIDE integer before narrowing for the same
// reason prune_grace is (see kMaxPruneGrace): get<int>() truncates first, so 4294967296
// would arrive as 0 and pass the >= 0 check - turning the documented "non-negative integer"
// contract into a hair-trigger that reports a node on its very first not-active tick, with
// no warning and no default. One below INT_MAX because the prune clamp computes grace + 1,
// which would otherwise overflow.
constexpr std::int64_t kMaxGrace = std::numeric_limits<int>::max() - 1;
/// Consecutive ticks an entry must match ONLY unmanaged nodes before the typo warning
/// fires. One transient tick is not evidence: discover_apps() wraps the per-node service
/// enumeration in a try/catch and pushes the app regardless, so a sweep that races service
/// discovery yields an app with no services and no tracked lifecycle state.
constexpr int kUnmanagedWarnTicks = 5;
// Fallback plugin-scope prune_grace (ticks) when the config doesn't carry one - mirrors
// GraphWatchdogPlugin's own default.
constexpr int kDefaultPruneGrace = 60;
// Widest prune_grace accepted, in ticks (one hour at the shipped 1 s cadence). The value is
// range-checked on the WIDE integer before narrowing: get<int>() truncates first, so
// 4294967296 would arrive as 0 and reclaim bookkeeping on the first absent tick, and 2^63-1
// would arrive as -1 and be dropped in silence.
constexpr std::int64_t kMaxPruneGrace = 3600;
/// Consecutive matched-but-never-read ticks a required node may hold back the clear (see
/// the withheld-clear guard in tick()). Bounded so an unmanaged node - or a typo that
/// happens to match a real, non-lifecycle node - cannot block healing forever. Mirrors
/// param_drift's kFrozenHoldTicks and the prune horizon (kDefaultPruneGrace): one minute
/// at the shipped 1 s cadence, far past the LifecycleWatcher seeding budget (8 GetState
/// seeds per tick) for any realistic require_active set.
constexpr int kUnmeasuredHoldTicks = 60;
/// Consecutive withheld ticks before the reason is put in the log. Withholding is right -
/// health that was never measured is not health - but from the outside it is
/// indistinguishable from a detector that is working and finding nothing, so once per
/// episode the detector says which it is. Ten ticks: label seeding normally completes
/// within a tick or two of the watcher seeing the node, so a hold that lives this long is
/// worth explaining.
constexpr int kWithheldClearReportTicks = 10;
}  // namespace

/// Watches operator-declared "must be active" nodes and raises GRAPH_NODE_INACTIVE for
/// one that is present in the graph (alive) but sitting in a non-active lifecycle state
/// for more than `grace` consecutive ticks. Distinct from the presence class
/// (GRAPH_NODE_DISAPPEARED, pure presence):
/// here the process is ALIVE and in the graph, but a node the operator declared
/// critical-active is stuck inactive/unconfigured/finalized - on a Nav2 stack a
/// controller_server stuck inactive means the robot silently will not act, no crash, no
/// log. Safe default: require_active is empty, so nothing is checked and there are zero
/// false positives until an operator opts specific nodes in (mirrors param_drift being
/// config-scoped). See lifecycle_expectation_tracker.hpp for the pure tracking core and
/// the design doc.
///
/// Design note (why grace, not the reliability gate): this detector cannot gate its
/// raise on reliability_allows() - it ANDs lifecycle_.node_ok(), which is FALSE for
/// exactly the inactive node this detector exists to catch, so gating on it would
/// suppress the signal forever. Instead the tracker counts consecutive not-active ticks
/// itself and raises only past its own grace, independent of the central gate (which
/// still applies to the aggregated RAISE itself via ctx.raise_fault - see
/// DetectorContext - for warmup/bringup-quiesce, just not for the lifecycle-inactive
/// state this detector is specifically built to report).
class LifecycleExpectationDetector : public Detector {
 public:
  std::string id() const override {
    return "lifecycle_expectation";
  }

  void configure(const nlohmann::json & config) override {
    std::vector<std::string> warnings;
    std::set<std::string> require_active;
    if (config.contains("require_active")) {
      if (config["require_active"].is_array()) {
        for (const auto & entry : config["require_active"]) {
          if (entry.is_string() && !entry.get<std::string>().empty()) {
            require_active.insert(entry.get<std::string>());
          } else {
            // Skip "" - it would match every unbound app's empty leaf. Silently dropping the
            // entry leaves the operator believing the node is covered, so it has to say so.
            warnings.push_back("'require_active' entries must be non-empty node names; skipping one");
          }
        }
      } else {
        warnings.push_back("'require_active' must be a string array of node names; ignoring it");
      }
    }
    int grace = kDefaultGrace;
    if (config.contains("grace")) {
      const auto & value = config["grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxGrace) {
        grace = static_cast<int>(wide);
      } else {
        warnings.push_back("'grace' must be a non-negative integer; keeping the default (" +
                           std::to_string(kDefaultGrace) + ")");
      }
    }
    // prune_grace is injected into every detector's config by the plugin and is exempt from the
    // unknown-key warning, so ignoring a bad value here would accept the key with no warning and
    // no effect - the exact silent failure the README says cannot happen. Range-check on the
    // WIDE value before narrowing (see kMaxPruneGrace).
    int prune_grace = kDefaultPruneGrace;
    if (config.contains("prune_grace")) {
      const auto & value = config["prune_grace"];
      const std::int64_t wide = value.is_number_integer() ? value.get<std::int64_t>() : -1;
      if (wide >= 0 && wide <= kMaxPruneGrace) {
        prune_grace = static_cast<int>(wide);
      } else {
        warnings.push_back("'prune_grace' must be an integer in 0.." + std::to_string(kMaxPruneGrace) + "; keeping " +
                           std::to_string(kDefaultPruneGrace));
      }
    }
    collect_unknown_detector_keys(config, known_keys(), warnings);
    // Clamp: an entry can only ever be reported once its miss count exceeds grace, so
    // prune_ticks must be at least grace + 1 or a currently-reported node could be pruned
    // out from under its own fault.
    const int prune_ticks = std::max(prune_grace, grace + 1);
    require_active_ = require_active;
    tracker_ = LifecycleExpectationTracker(require_active_, grace, kDefaultAbsenceGrace, kDefaultNoMatchWarnTicks,
                                           prune_ticks);
    unmanaged_streak_.clear();
    // The one-time warning is scoped to the CURRENT config. Without this, an entry that
    // already warned stays permanently silenced across a reconfigure - including after it
    // was removed and re-added, which is exactly when the operator wants to hear that it
    // still names nothing managed.
    warned_.clear();
    warnings_ = std::move(warnings);
    warnings_logged_ = false;
    // The withheld-clear guard restarts with the config: a reconfigure rebuilds the
    // tracker above, so both halves of the guard have to start over together or the new
    // tracker's fresh streaks would be judged against the old config's read latches.
    //
    // What the latch is scoped to, precisely: a configure() (so, in production, the
    // process). It is keyed by fqn and it does NOT reset when the NODE restarts - a
    // process that dies and respawns under the same name within the prune horizon
    // re-enters with `read` still true from the dead incarnation, and its brand-new,
    // never-measured labels therefore cannot withhold anything. That is deliberate rather
    // than merely tolerated: the pending half of the guard covers the respawn case
    // directly (a node that comes back stuck starts a streak, and a streak below grace
    // withholds the clear on its own), while making the latch per-incarnation would put a
    // fresh unread hold on every blink of a healthy node.
    prune_ticks_ = prune_ticks;
    guard_.clear();
    ever_matched_.clear();
    unmatched_ticks_ = 0;
    withheld_ticks_ = 0;
    withheld_reported_ = false;
  }

  std::size_t tracked_count_for_test() const override {
    return tracker_.tracked_count();
  }

  void tick(DetectorContext & ctx) override {
    // Before the zero-config early return, deliberately: the misspelt key most worth
    // reporting is `require_activ`, and that typo is exactly the config in which
    // require_active_ is empty.
    log_warnings_once(ctx);
    if (!ctx.snapshot || require_active_.empty()) {
      return;  // zero-config safe default: nothing configured, nothing to check
    }

    // Match each configured require_active entry against a live app by App::id OR its stable
    // effective_fqn() OR the bare leaf name of that fqn. App::id alone is unstable: it is
    // recomputed each sweep and gets a namespace prefix once a same-bare-name collision exists
    // anywhere in the graph, so a bare-name config would silently stop matching on a
    // multi-robot graph - the worst failure mode for a silent-fault detector. A bare name
    // therefore matches every namespace's node of that name; use a full FQN to pin one.
    std::vector<LifecycleMatch> matches;
    // Nodes matched by >=1 entry this tick -> whether ANY of those matches carried a real
    // label. Keyed by NODE, not by (entry, node): a node named by two entries - the
    // documented bare-name plus pinned-FQN mix - must not burn its guard budget twice a
    // tick, exactly as the tracker must not advance its streak twice (see the tracker's
    // "One node, one streak per tick").
    std::map<std::string, bool> matched_read;
    std::set<std::string> entry_has_managed_match;  // entries matching >=1 app with a tracked lifecycle state
    for (const auto & app : ctx.snapshot->apps) {
      const std::string fqn = app.effective_fqn();
      if (fqn.empty()) {
        continue;  // an unnamed entry can never be useful in a fault
      }
      const std::string leaf = fqn.substr(fqn.find_last_of('/') + 1);
      for (const auto & id : require_active_) {
        if (id != app.id && id != fqn && id != leaf) {
          continue;
        }
        const std::optional<std::string> state = ctx.gate ? ctx.gate->lifecycle_state_of(app.id) : std::nullopt;
        if (state.has_value()) {
          entry_has_managed_match.insert(id);
        }
        // One match per (entry, node): the tracker keys violations by NODE, so two
        // namesakes are both reported instead of one silently replacing the other.
        matches.push_back(LifecycleMatch{id, fqn, state});
        bool & read = matched_read[fqn];
        read = read || (state.has_value() && !state->empty());
      }
    }
    std::set<std::string> matched_entries;
    for (const auto & match : matches) {
      matched_entries.insert(match.entry);
    }
    // Third leg of the withheld-clear guard: an entry that has NEVER matched a node since
    // configure(). Both other legs are keyed by a matched node, so in the window before
    // the first match they are empty, the tracker reports nothing affected, and the clear
    // flows about a node the detector has not once looked at. That window is not exotic -
    // it is every restart: the plugin ticks as soon as it is loaded, while the entity
    // snapshot is still catching up with the graph. Deliberately NOT the same as an entry
    // whose node VANISHES after having matched: that is a presence problem and the clear
    // is correct there (the tracker makes the same handoff). Bounded by the same hold as
    // the unread leg, so a misspelt entry cannot block healing for the process lifetime.
    ever_matched_.insert(matched_entries.begin(), matched_entries.end());
    const bool any_never_matched = ever_matched_.size() < require_active_.size();
    if (!any_never_matched) {
      unmatched_ticks_ = 0;
    } else if (unmatched_ticks_ <= kUnmeasuredHoldTicks) {
      ++unmatched_ticks_;  // frozen one past the hold, like GuardEntry::unmeasured
    }
    const bool unmatched_blocking =
        any_never_matched && unmatched_ticks_ >= 1 && unmatched_ticks_ <= kUnmeasuredHoldTicks;

    // An entry that matches present nodes but NONE with a tracked lifecycle state is
    // probably a typo (or a plain, non-managed node). Requiring N CONSECUTIVE such ticks
    // before saying so is what keeps that from latching on a transient: LifecycleWatcher
    // only tracks a node whose get_state service was seen on THIS sweep, and
    // discover_apps() can yield an app with empty services when a sweep races service
    // enumeration - one such tick used to pin the accusation for the process lifetime,
    // since warned_ is only cleared by configure() and configure() runs once.
    //
    // Requires ctx.gate: with no gate wired, lifecycle_state_of() is never consulted and
    // EVERY entry looks unmanaged, so this would accuse every correct entry of being a typo.
    if (ctx.gateway_node && ctx.gate) {
      for (const auto & id : matched_entries) {
        if (entry_has_managed_match.count(id) != 0) {
          unmanaged_streak_.erase(id);  // resolved: it may warn again if it regresses
          warned_.erase(id);
          continue;
        }
        if (++unmanaged_streak_[id] > kUnmanagedWarnTicks && warned_.insert(id).second) {
          RCLCPP_WARN(ctx.gateway_node->get_logger(),
                      "graph_watchdog lifecycle_expectation: require_active entry '%s' has matched a present node "
                      "with no tracked lifecycle state for %d consecutive ticks (not a managed rclcpp_lifecycle "
                      "node?) - check for a typo",
                      id.c_str(), kUnmanagedWarnTicks + 1);
        }
      }
    }

    const auto report = tracker_.update(matches);
    // An entry that matches nothing at all is reported by the tracker, not here: the loop
    // above only sees entries that DID match a present node.
    if (ctx.gateway_node) {
      for (const auto & entry : report.entries_matching_nothing) {
        // The tracker counts CONSECUTIVE no-match ticks, so it also surfaces an entry whose
        // node matched and later left the graph. The warning below says the opposite in two
        // places ("no node at all since startup", and that the presence class cannot report
        // it), and for a node that WAS present both statements are false - a departed node
        // is precisely what GRAPH_NODE_DISAPPEARED owns. Same handoff the never-matched
        // hold makes above: an entry that has matched once is never treated as missing
        // again.
        if (ever_matched_.count(entry) != 0) {
          continue;
        }
        RCLCPP_WARN(ctx.gateway_node->get_logger(),
                    "graph_watchdog lifecycle_expectation: require_active entry '%s' has matched no node at all "
                    "since startup - a misspelt entry, or a required node that never came up (the presence class "
                    "GRAPH_NODE_DISAPPEARED cannot report it either: it only tracks nodes that were present at "
                    "least once)",
                    entry.c_str());
      }
    }

    // Withheld-clear guard. The emitter is level-triggered: an empty affected map is a
    // clear, and a clear asserts that every required node is healthy. THREE states produce
    // an empty affected map without that being true, and all three are the normal state of
    // a detector whose bookkeeping just started over - a restarted gateway, a reconfigure
    // (configure() rebuilds the tracker), or a node that respawned stuck:
    //
    //  1. NOT MATCHED YET. Before the entity snapshot catches up with the graph, a
    //     require_active entry matches nothing at all, so the other two legs - both keyed
    //     by a matched node - are empty and the clear is about a node the detector has
    //     never looked at. Computed above as `unmatched_blocking`.
    //  2. NOT MEASURED. An unread label (nullopt or "") is benign to the tracker, so a
    //     matched node nothing has answered for looks exactly like a healthy one.
    //  3. MEASURED VIOLATING, BELOW GRACE. The tracker only reports past `grace`
    //     consecutive not-active ticks, so for a whole grace-wide window the detector's
    //     own last read says "inactive" while its report says nothing is affected. Labels
    //     are seeded BEFORE detectors tick, so on a responsive stack the first tick after
    //     a restart already reads the real label and state 2 never happens - this is the
    //     one that fires instead.
    //
    // Any of them withholds the emission entirely - neither raise nor clear - so a
    // GRAPH_NODE_INACTIVE already in the store keeps its state instead of being healed by
    // a detector that has not yet re-established the node is fine.
    //
    // A raise is never withheld: a violation read from the nodes that DID answer is real
    // regardless of the unread ones. Every hold is bounded. The never-matched and unread
    // holds release after kUnmeasuredHoldTicks, so a misspelt entry - or a typo that
    // matches a real non-lifecycle node - cannot block healing forever; note the unread
    // one burns on every matched tick, including ticks a raise about a DIFFERENT node was
    // flowing, so a raise that outlives the hold leaves the clear free the moment it
    // heals. The pending hold releases as soon as the node reads active (streak zeroed) or
    // passes grace (raised). And a node absent past kDefaultAbsenceGrace stops blocking on
    // both node-keyed counts - sustained absence belongs to the presence class
    // (GRAPH_NODE_DISAPPEARED), the same handoff the tracker makes, which is also why an
    // entry that HAS matched before and stops matching does not re-enter state 1.
    for (const auto & [fqn, read] : matched_read) {
      auto & guard = guard_[fqn];
      guard.absent = 0;
      if (read) {
        guard.read = true;
        guard.unmeasured = 0;
      } else if (!guard.read && guard.unmeasured <= kUnmeasuredHoldTicks) {
        ++guard.unmeasured;  // frozen one past the hold; the blocking check below stops consulting it
      }
    }
    for (auto it = guard_.begin(); it != guard_.end();) {
      if (matched_read.count(it->first) != 0) {
        ++it;
        continue;
      }
      if (++it->second.absent > prune_ticks_) {
        it = guard_.erase(it);  // bookkeeping reclaimed, bounding the map under identity churn
      } else {
        ++it;
      }
    }
    std::size_t unread = 0;
    std::string unread_example;
    for (const auto & [fqn, guard] : guard_) {
      if (!guard.read && guard.unmeasured >= 1 && guard.unmeasured <= kUnmeasuredHoldTicks &&
          guard.absent <= kDefaultAbsenceGrace) {
        if (unread == 0) {
          unread_example = fqn;
        }
        ++unread;
      }
    }
    if (report.affected.empty() && (unread > 0 || !report.pending.empty() || unmatched_blocking)) {
      report_withheld_clear(ctx, unread, unread_example, report.pending, unmatched_blocking);
      return;  // nothing measured - and a young streak - is not the same as healthy
    }
    withheld_ticks_ = 0;
    withheld_reported_ = false;
    aggregated_.emit(ctx, report.affected);
  }

 private:
  static const std::set<std::string> & known_keys() {
    static const std::set<std::string> keys{"require_active", "grace"};
    return keys;
  }

  void log_warnings_once(const DetectorContext & ctx) {
    if (warnings_.empty() || warnings_logged_ || !ctx.gateway_node) {
      return;
    }
    for (const auto & warning : warnings_) {
      RCLCPP_WARN(ctx.gateway_node->get_logger(), "graph_watchdog lifecycle_expectation: %s", warning.c_str());
    }
    warnings_logged_ = true;
  }

  /// Say, once per withhold episode, that the clear is being withheld and why.
  /// Withholding is correct - health that was never measured is not health, and neither is
  /// a violation the counter has not finished counting - but from the outside it is
  /// indistinguishable from a detector that is working and finding nothing: no raise, no
  /// clear, and a GRAPH_NODE_INACTIVE already in the store simply never heals. Once per
  /// episode, past a horizon the normal label seeding and a normal grace both fit inside.
  /// The two reasons are named separately because they release on different conditions.
  void report_withheld_clear(const DetectorContext & ctx, std::size_t unread, const std::string & unread_example,
                             const std::set<std::string> & pending, bool unmatched) {
    ++withheld_ticks_;
    if (withheld_ticks_ <= kWithheldClearReportTicks || withheld_reported_ || !ctx.gateway_node) {
      return;
    }
    withheld_reported_ = true;
    std::string reason;
    const auto add = [&reason](const std::string & phrase) {
      if (!reason.empty()) {
        reason += " and ";
      }
      reason += phrase;
    };
    if (unmatched) {
      add("a require_active entry has not matched any node yet (that hold releases as soon as it matches, "
          "or by itself after " +
          std::to_string(kUnmeasuredHoldTicks) + " ticks)");
    }
    if (unread > 0) {
      add(std::to_string(unread) + " required node(s) have had no lifecycle label read this run (starting with '" +
          unread_example + "'; that hold releases by itself after " + std::to_string(kUnmeasuredHoldTicks) +
          " unread ticks per node)");
    }
    if (!pending.empty()) {
      add(std::to_string(pending.size()) + " required node(s) are measured not-active but still within grace " +
          "(starting with '" + *pending.begin() + "'; that hold releases as soon as the node reads active or " +
          "its streak passes grace)");
    }
    RCLCPP_WARN(ctx.gateway_node->get_logger(),
                "graph_watchdog lifecycle_expectation: nothing is reported inactive, but GRAPH_NODE_INACTIVE is "
                "withheld from clearing because %s",
                reason.c_str());
  }

  /// Per-required-node bookkeeping for the withheld-clear guard, keyed by fqn.
  struct GuardEntry {
    bool read = false;   ///< a lifecycle label was successfully read this run ("" is not a read)
    int unmeasured = 0;  ///< consecutive matched-and-never-read ticks (frozen one past the hold)
    int absent = 0;      ///< consecutive absent ticks (entry reclaimed past prune_ticks_)
  };

  std::set<std::string> require_active_;
  std::set<std::string> warned_;                 // require_active entries already warned about (no lifecycle state)
  std::map<std::string, int> unmanaged_streak_;  // entry -> consecutive ticks matching only unmanaged nodes
  std::vector<std::string> warnings_;            // configure()-time findings, surfaced once per config
  bool warnings_logged_ = false;
  std::map<std::string, GuardEntry> guard_;  // fqn -> withheld-clear guard state
  std::set<std::string> ever_matched_;       // require_active entries that have matched >=1 node this config
  int unmatched_ticks_ = 0;  // consecutive ticks with an entry that has never matched (frozen past the hold)
  int prune_ticks_ = std::max(kDefaultPruneGrace, kDefaultGrace + 1);
  int withheld_ticks_ = 0;          // consecutive ticks the clear has been withheld
  bool withheld_reported_ = false;  // withhold already explained this episode
  LifecycleExpectationTracker tracker_{{}, kDefaultGrace};
  AggregatedFault aggregated_{graph_fault_codes::kNodeInactive, kLifecycleSeverity};
};

REGISTER_DETECTOR(LifecycleExpectationDetector, "lifecycle_expectation")

}  // namespace ros2_medkit_graph_watchdog
