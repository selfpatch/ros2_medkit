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

#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_graph_watchdog {

/// Consecutive ticks a require_active entry may match NO node at all before it is worth
/// warning about. A required node that never came up is invisible to the presence class
/// (GRAPH_NODE_DISAPPEARED needs the node present and online first), so nothing else
/// reports it.
inline constexpr int kDefaultNoMatchWarnTicks = 10;
/// Consecutive ticks a matched node may be ABSENT from the snapshot - or present with an
/// unread label, which is the same event seen one tick later - before its violation streak
/// is discarded. Sustained absence belongs to the presence class
/// (GRAPH_NODE_DISAPPEARED), but zeroing the streak on the first blink means a node that
/// drops out of one snapshot in every few never accumulates enough consecutive
/// present-and-inactive ticks to be reported at all.
inline constexpr int kDefaultAbsenceGrace = 3;

/// One live node matched by a require_active entry, with its lifecycle state.
struct LifecycleMatch {
  std::string entry;  ///< the require_active entry that matched
  std::string fqn;    ///< the node's stable App::effective_fqn()
  std::optional<std::string> state;
};

/// What one update() sweep found.
struct LifecycleExpectationReport {
  /// fqn -> detail phrase. Keyed by the NODE, not by the config entry.
  std::map<std::string, std::string> affected;
  /// Nodes whose violation streak has started but has not yet passed grace - measured
  /// not-active, not yet worth a fault.
  ///
  /// Not a fault, and NOT health either, which is why it is reported separately: a caller
  /// that emits a level-triggered clear whenever `affected` is empty would otherwise
  /// assert "nothing is stuck" about a node its own last read found stuck, purely because
  /// the counter is young. That window is a whole `grace` wide every time the counters
  /// start over (a restarted gateway, a reconfigure, a node that respawns stuck), which is
  /// exactly when a fault raised earlier is still in the store waiting to be healed.
  std::set<std::string> pending;
  /// Entries that have matched nothing for more CONSECUTIVE ticks than the no-match
  /// threshold and have not been reported yet. Consecutive, not "never": an entry whose node
  /// matched and later left the graph surfaces here too, and for that one the caller's
  /// "never came up, and the presence class cannot see it either" framing is false - so the
  /// caller filters on whether the entry has ever matched before it says anything.
  std::vector<std::string> entries_matching_nothing;
};

/// Enforces "these nodes must be active". Pure. A required node that is present and
/// neither active nor unknown for more than grace consecutive ticks is reported. Unknown
/// state is never enforced (no lifecycle to be wrong about); grace absorbs the normal
/// bringup transitions.
///
/// **Keyed by node, not by config entry.** The bare-name form of require_active is
/// deliberately fleet-wide ("all controller_servers must be active"), so one entry
/// legitimately covers N nodes. Keying the violation on the ENTRY left the fault unable to
/// say which of them broke: on a dual-arm robot `require_active: ["controller_server"]`
/// produced `node controller_server expected active but is inactive` with nothing pointing
/// at /left or /right, and with two offenders only one survived into the report at all.
/// The source_id cannot disambiguate either - every GRAPH_* fault shares one - so the
/// description is the only carrier, and it has to name the node.
///
/// **Absence has its own grace.** A node dropping out of one snapshot used to zero its
/// violation streak outright, so a genuinely stuck node that blinks out of one snapshot in
/// every few could never accumulate grace+1 consecutive present-and-inactive ticks and was
/// never reported at all. That is reachable at the shipped cadence: the tick is 1s and the
/// entity cache is graph-event driven with a 1s debounce, so on a churning graph the
/// snapshot turns over about once per tick. An app leaving the snapshot also makes
/// LifecycleWatcher erase its tracked entry, so on return the state is re-seeded and reads
/// "" for a tick or two - the tail of the same event, and counted as absent here too: an
/// UNREAD label ages the absence budget instead of resetting the streak. A first-contact
/// "" has no streak to preserve, so it costs nothing and holds no bookkeeping. Sustained
/// unread spends the budget exactly as sustained absence does, and the streak is then
/// discarded. `std::nullopt` is a different fact - the node has no tracked lifecycle at
/// all, so there is nothing to enforce and nothing to preserve - and resets like a healthy
/// read.
///
/// **One node, one streak per tick.** The caller pushes one match per (entry, node) pair,
/// so a node named by two require_active entries - the documented bare-name (fleet-wide)
/// plus full-FQN (pinned) mix - arrives twice in the same sweep. Every counter here moves
/// at most once per node per update(); counting per match would advance the streak at 2x
/// and silently halve the grace the operator configured. Each matching entry is still
/// named in the report, as context.
class LifecycleExpectationTracker {
 public:
  /// kNoPrune keeps every entry forever - the default, so a short-lived unit test is
  /// unaffected. Production wiring passes an explicit, clamped value.
  static constexpr int kNoPrune = std::numeric_limits<int>::max();

  /// prune_ticks is LAST on purpose: every parameter here is an int, so inserting one in
  /// the middle keeps existing positional calls compiling while silently rebinding them.
  LifecycleExpectationTracker(std::set<std::string> require_active, int grace, int absence_grace = kDefaultAbsenceGrace,
                              int no_match_warn_ticks = kDefaultNoMatchWarnTicks, int prune_ticks = kNoPrune)
    : require_active_(std::move(require_active))
    , grace_(grace)
    , absence_grace_(absence_grace)
    , no_match_warn_ticks_(no_match_warn_ticks)
    , prune_ticks_(prune_ticks) {
  }

  /// Nodes whose bookkeeping is still held - for tests asserting the maps stay bounded.
  ///
  /// Counts DISTINCT fqns across both maps, not one map's size. Every map here is keyed by
  /// fqn and a healthy node sits in both, so on the ordinary path this is exactly
  /// `absent_.size()` and the number does not move. What it catches is the case where the
  /// two horizons disagree: reporting only the absence map made a retained violation streak
  /// invisible to every boundedness assertion in the suite, so a leak of one `misses_` entry
  /// per fqn ever seen read as a bounded map.
  std::size_t tracked_count() const {
    std::size_t held = absent_.size();
    for (const auto & miss : misses_) {
      if (absent_.count(miss.first) == 0) {
        ++held;
      }
    }
    return held;
  }

  LifecycleExpectationReport update(const std::vector<LifecycleMatch> & matches) {
    LifecycleExpectationReport report;

    // Collapse the per-(entry, node) matches to one record per NODE before any counter
    // moves - see "One node, one streak per tick" above.
    struct NodeTick {
      std::vector<std::string> entries;  ///< every entry that named this node, in match order
      std::optional<std::string> state;  ///< the label enforced for it this tick
    };
    std::set<std::string> matched_entries;
    std::map<std::string, NodeTick> nodes;
    for (const auto & match : matches) {
      matched_entries.insert(match.entry);
      const auto inserted = nodes.emplace(match.fqn, NodeTick{});
      NodeTick & node = inserted.first->second;
      if (std::find(node.entries.begin(), node.entries.end(), match.entry) == node.entries.end()) {
        node.entries.push_back(match.entry);
      }
      // Duplicate matches for one node carry the same label by construction (the caller
      // reads the label once per node), but if they ever disagree the violating read wins:
      // a detector for silent faults must not lose a measured violation to a tie-break.
      if (inserted.second || (is_violating(match.state) && !is_violating(node.state))) {
        node.state = match.state;
      }
    }

    std::set<std::string> seen_fqns;
    for (const auto & entry : nodes) {
      const std::string & fqn = entry.first;
      const NodeTick & node = entry.second;
      // Unread label ("" from a LifecycleWatcher seed that has not answered): counted as
      // absent, so the streak survives it and the absence loop below ages it. See
      // "Absence has its own grace".
      if (node.state.has_value() && node.state->empty()) {
        continue;
      }
      seen_fqns.insert(fqn);
      absent_[fqn] = 0;
      if (!is_violating(node.state)) {
        misses_[fqn] = 0;  // active, or no tracked lifecycle at all: nothing to enforce
        continue;
      }
      const int m = ++misses_[fqn];
      if (m > grace_) {
        report.affected[fqn] = "node " + fqn + " expected active but is " + *node.state + " (required by " +
                               join_quoted(node.entries) + ")";
      }
    }

    // A matched node that left the snapshot (or came back unread) keeps its streak for
    // absence_grace_ ticks; past that the streak is discarded, since absence itself
    // belongs to the presence class (GRAPH_NODE_DISAPPEARED).
    for (auto it = absent_.begin(); it != absent_.end();) {
      if (seen_fqns.count(it->first) != 0) {
        ++it;
        continue;
      }
      const int gone = ++it->second;
      // The streak also goes when the absence bookkeeping itself is reclaimed, not only at
      // the absence grace. prune_ticks is allowed to be SHORTER than absence_grace through
      // ordinary config (grace: 1 with prune_grace: 2 clamps to 2, one under the default
      // absence grace of 3), and this loop is the only place that can erase a streak: an
      // fqn pruned before it ever crossed the absence grace would keep its `misses_` entry
      // for the process lifetime - the streak resuming on return, and one permanent entry
      // per fqn ever seen under identity churn.
      if (gone > absence_grace_ || gone > prune_ticks_) {
        misses_.erase(it->first);  // streak discarded: sustained absence is the presence class's concern
      }
      if (gone > prune_ticks_) {
        it = absent_.erase(it);  // bookkeeping reclaimed, bounding the map under identity churn
      } else {
        ++it;
      }
    }

    // Pending: streaks that have started but not passed grace. Bounded by the SAME absence
    // budget the streak itself is, so a node the tracker has already handed to the presence
    // class holds nothing back - and a streak whose absence bookkeeping is gone entirely
    // cannot resurrect as a hold either.
    for (const auto & miss : misses_) {
      if (miss.second <= 0 || miss.second > grace_) {
        continue;  // 0 = healthy, past grace = already in `affected` (or already handed over)
      }
      const auto absent_it = absent_.find(miss.first);
      if (absent_it == absent_.end() || absent_it->second > absence_grace_) {
        continue;
      }
      report.pending.insert(miss.first);
    }

    // An entry matching nothing is silent in every other path: it never reaches the
    // violation branch above, and the presence class never starts tracking a node that was
    // never present, so a misspelt entry - or one whose node crashed during launch -
    // produces no fault and no log line anywhere.
    for (const auto & entry : require_active_) {
      if (matched_entries.count(entry) != 0) {
        no_match_[entry] = 0;
        continue;
      }
      if (++no_match_[entry] > no_match_warn_ticks_ && reported_no_match_.insert(entry).second) {
        report.entries_matching_nothing.push_back(entry);
      }
    }
    return report;
  }

 private:
  /// A label that contradicts "must be active". Empty and nullopt are NOT violations:
  /// empty means the seed has not answered yet (enforcing it would false-positive an
  /// actually-active node whose seed was lost) and nullopt means there is no tracked
  /// lifecycle to be wrong about.
  static bool is_violating(const std::optional<std::string> & state) {
    return state.has_value() && !state->empty() && *state != "active";
  }

  /// "'a', '/a'" - every entry that named the node, for the report's context phrase.
  static std::string join_quoted(const std::vector<std::string> & entries) {
    std::string joined;
    for (const auto & entry : entries) {
      if (!joined.empty()) {
        joined += ", ";
      }
      joined += "'" + entry + "'";
    }
    return joined;
  }

  std::set<std::string> require_active_;
  int grace_;
  int absence_grace_;
  int no_match_warn_ticks_;
  int prune_ticks_;
  std::map<std::string, int> misses_;        ///< fqn -> consecutive present-and-inactive ticks
  std::map<std::string, int> absent_;        ///< fqn -> consecutive absent ticks
  std::map<std::string, int> no_match_;      ///< entry -> consecutive ticks matching nothing
  std::set<std::string> reported_no_match_;  ///< entries already warned about
};

}  // namespace ros2_medkit_graph_watchdog
