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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/// ROS-free core of the policy bridge: signals computed from what a learned
/// policy publishes about itself, and the rules that turn them into fault
/// transitions. The node only moves messages in and reports out.
namespace ros2_medkit_policy_bridge::core {

/// Values in one chunk above which a sample is not used. A chunk is copied
/// into the history, so its size bounds the memory and the work per message.
inline constexpr size_t kMaxChunkValues = 65536;

/// Valid samples in a row without a chunk spread before the monitor says the
/// check cannot run (the predictions do not overlap), and spreads in a row
/// before it may say so again.
inline constexpr uint32_t kNoOverlapNoticeAfter = 10;

/// One inference of the policy, as published on the introspection topic.
struct PolicySample {
  uint32_t episode{0};
  uint32_t step{0};
  uint16_t chunk_length{0};
  uint16_t action_dim{0};
  std::vector<float> chunk;  // chunk_length x action_dim, row major
  float inference_ms{0.0f};  // 0, negative or non-finite: not measured
  // Set instead of `chunk` when the message held more than kMaxChunkValues
  // values: the caller does not copy a chunk that cannot be used.
  size_t dropped_chunk_values{0};
};

/// Why a sample was not used, by kind: a warning is given once per kind, so
/// the kind carries no numbers.
enum class Rejection { kNone, kBadShape, kTooLarge, kSizeMismatch, kNotFinite, kRepeatedStep };

/// True when the sample carries a measured inference time.
inline bool has_inference_time(const PolicySample & s) {
  return std::isfinite(s.inference_ms) && s.inference_ms > 0.0f;
}

/// A double as text for messages: "0.05", not "0.050000".
inline std::string number_text(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

/// How much the policy changes its mind. A policy that predicts chunks of
/// future actions predicts the same steps again on its next inference. When
/// it knows what to do, the new chunk agrees with the older ones on the steps
/// they share; when it does not, they drift apart. The value is the mean,
/// over the last `history` chunks, of the RMS difference on the overlapping
/// steps (at most `horizon` of them), each joint divided by its scale.
class ChunkSpread {
 public:
  ChunkSpread(size_t history, size_t horizon, std::vector<double> scale)
    : history_(std::max<size_t>(1, history)), horizon_(std::max<size_t>(1, horizon)), scale_(std::move(scale)) {
  }

  void reset() {
    chunks_.clear();
  }

  /// Add a chunk predicted at `step`. Returns the spread against the up to
  /// `history` chunks stored before it, or nothing when no stored chunk overlaps it (the first chunk of
  /// an episode, or chunks too far apart). The caller makes sure steps grow.
  std::optional<double> update(uint32_t step, const PolicySample & s) {
    std::optional<double> result;
    double sum = 0.0;
    size_t count = 0;
    for (const auto & old : chunks_) {
      const size_t age = step - old.step;
      if (age >= old.length) {
        continue;
      }
      const size_t n = std::min({horizon_, old.length - age, static_cast<size_t>(s.chunk_length)});
      double sq = 0.0;
      for (size_t t = 0; t < n; ++t) {
        for (size_t j = 0; j < s.action_dim; ++j) {
          const double d = (static_cast<double>(s.chunk[t * s.action_dim + j]) -
                            static_cast<double>(old.values[(age + t) * s.action_dim + j])) /
                           scale_for(j);
          sq += d * d;
        }
      }
      sum += std::sqrt(sq / static_cast<double>(n * s.action_dim));
      ++count;
    }
    if (count > 0) {
      result = sum / static_cast<double>(count);
    }
    chunks_.push_back({step, s.chunk_length, s.chunk});
    while (chunks_.size() > history_) {
      chunks_.pop_front();
    }
    return result;
  }

  size_t stored() const {
    return chunks_.size();
  }

 private:
  struct Stored {
    uint32_t step;
    size_t length;
    std::vector<float> values;
  };

  double scale_for(size_t joint) const {
    return joint < scale_.size() ? scale_[joint] : 1.0;
  }

  size_t history_;
  size_t horizon_;
  std::vector<double> scale_;
  std::deque<Stored> chunks_;
};

/// Raise after `raise_after` readings above the threshold in a row, clear
/// after `clear_after` readings at or below it (0 = never clear on readings).
/// A missing reading changes nothing: it neither counts nor resets a count.
class RaiseGate {
 public:
  enum class Edge { kNone, kRaise, kClear };

  RaiseGate(double threshold, uint32_t raise_after, uint32_t clear_after)
    : threshold_(threshold), raise_after_(std::max<uint32_t>(1, raise_after)), clear_after_(clear_after) {
  }

  Edge update(std::optional<double> value) {
    if (!value || !std::isfinite(*value)) {
      return Edge::kNone;
    }
    if (*value > threshold_) {
      ++above_;
      below_ = 0;
      if (!active_ && above_ >= raise_after_) {
        active_ = true;
        return Edge::kRaise;
      }
    } else {
      ++below_;
      above_ = 0;
      if (active_ && clear_after_ > 0 && below_ >= clear_after_) {
        active_ = false;
        return Edge::kClear;
      }
    }
    return Edge::kNone;
  }

  /// Forget the counts; returns true when the gate was active (the caller
  /// passes the fault).
  bool reset() {
    const bool was = active_;
    active_ = false;
    above_ = below_ = 0;
    return was;
  }

  bool active() const {
    return active_;
  }

 private:
  double threshold_;
  uint32_t raise_after_;
  uint32_t clear_after_;
  uint32_t above_{0};
  uint32_t below_{0};
  bool active_{false};
};

struct MonitorConfig {
  // Chunk spread above which a reading counts as disagreeing. <= 0 turns the
  // check off: the right value depends on the policy and must be measured
  // on its successful runs. The check needs predictions that overlap: the
  // policy must infer more often than every chunk_length steps.
  double spread_threshold{0.0};
  size_t spread_history{4};
  size_t spread_horizon{20};
  std::vector<double> joint_scale;  // empty = 1 for every joint
  // Inference time above which the policy counts as too slow. <= 0 = off.
  double max_inference_ms{0.0};
  uint32_t raise_after{3};
  uint32_t clear_after{0};  // readings; 0 = only a new episode clears
  bool clear_on_new_episode{true};
};

/// Parameter values as read, before any check. Integers are int64 because
/// that is what a ROS parameter holds; narrowing first would let a huge value
/// wrap around into a valid-looking one.
struct RawConfig {
  double spread_threshold{0.0};
  int64_t spread_history{4};
  int64_t spread_horizon{20};
  std::vector<double> joint_scale;
  double max_inference_ms{0.0};
  int64_t raise_after{3};
  int64_t clear_after{0};
  bool clear_on_new_episode{true};
};

/// Turn raw parameters into a config. Out-of-range values are clamped to the
/// nearest valid one and a warning says what was changed; nothing here throws.
inline MonitorConfig validate_config(const RawConfig & raw, std::vector<std::string> * warnings) {
  auto warn = [warnings](const std::string & w) {
    if (warnings) {
      warnings->push_back(w);
    }
  };
  auto clamp_int = [&warn](const char * name, int64_t value, int64_t lo, int64_t hi) {
    if (value < lo || value > hi) {
      const int64_t fixed = std::clamp(value, lo, hi);
      warn(std::string(name) + " " + std::to_string(value) + " is outside [" + std::to_string(lo) + ", " +
           std::to_string(hi) + "], using " + std::to_string(fixed));
      return fixed;
    }
    return value;
  };
  auto threshold = [&warn](const char * name, double value, const char * check) {
    if (!std::isfinite(value)) {
      warn(std::string(name) + " is not a finite number, the " + check + " check is off");
      return 0.0;
    }
    if (value < 0.0) {
      warn(std::string(name) + " " + number_text(value) + " is negative, the " + check + " check is off");
      return 0.0;
    }
    return value;
  };
  MonitorConfig c;
  c.spread_threshold = threshold("spread_threshold", raw.spread_threshold, "chunk disagreement");
  c.max_inference_ms = threshold("max_inference_ms", raw.max_inference_ms, "slow inference");
  c.spread_history = static_cast<size_t>(clamp_int("spread_history", raw.spread_history, 1, 64));
  c.spread_horizon = static_cast<size_t>(clamp_int("spread_horizon", raw.spread_horizon, 1, 10000));
  c.raise_after = static_cast<uint32_t>(clamp_int("raise_after", raw.raise_after, 1, 100000));
  c.clear_after = static_cast<uint32_t>(clamp_int("clear_after", raw.clear_after, 0, 100000));
  c.clear_on_new_episode = raw.clear_on_new_episode;
  if (c.clear_after == 0 && !c.clear_on_new_episode && (c.spread_threshold > 0.0 || c.max_inference_ms > 0.0)) {
    warn("clear_after is 0 and clear_on_new_episode is false: a raised policy fault is never passed");
  }
  for (double v : raw.joint_scale) {
    if (!std::isfinite(v) || v <= 0.0) {
      warn("joint_scale holds a value that is not a positive number, using 1 for every joint");
      return c;
    }
  }
  c.joint_scale = raw.joint_scale;
  return c;
}

/// What the node has to tell the fault manager after one sample.
struct MonitorResult {
  enum class Signal { kChunkDisagreement, kSlowInference };
  struct Transition {
    Signal signal;
    bool raise;  // false = passed
    double value;
  };
  std::vector<Transition> transitions;
  std::optional<double> spread;
  // Set when the sample was not used, with the reason (for a warning).
  Rejection rejection{Rejection::kNone};
  std::string rejected;
  // Things the operator should know, each given once until it changes: the
  // spread check cannot run, joint_scale does not fit the action size.
  std::vector<std::string> notices;
};

/// Per-policy state: one instance per introspection topic.
class PolicyMonitor {
 public:
  explicit PolicyMonitor(const MonitorConfig & config)
    : config_(config)
    , spread_(config.spread_history, config.spread_horizon, config.joint_scale)
    , spread_gate_(config.spread_threshold, config.raise_after, config.clear_after)
    , latency_gate_(config.max_inference_ms, config.raise_after, config.clear_after) {
  }

  /// Why a sample cannot be used (kNone when it can), and the reason in words.
  static Rejection validate(const PolicySample & s, std::string * why) {
    auto reject = [why](Rejection kind, std::string text) {
      if (why) {
        *why = std::move(text);
      }
      return kind;
    };
    if (s.dropped_chunk_values > 0) {
      return reject(Rejection::kTooLarge, "chunk has " + std::to_string(s.dropped_chunk_values) +
                                              " values, more than the " + std::to_string(kMaxChunkValues) +
                                              " the bridge reads");
    }
    if (s.chunk_length == 0 || s.action_dim == 0) {
      return reject(Rejection::kBadShape, "chunk_length and action_dim must be > 0");
    }
    const size_t expected = static_cast<size_t>(s.chunk_length) * s.action_dim;
    if (expected > kMaxChunkValues) {
      return reject(Rejection::kTooLarge, "chunk_length x action_dim = " + std::to_string(expected) +
                                              " values, more than the " + std::to_string(kMaxChunkValues) +
                                              " the bridge reads");
    }
    if (s.chunk.size() != expected) {
      return reject(Rejection::kSizeMismatch,
                    "chunk has " + std::to_string(s.chunk.size()) +
                        " values, expected chunk_length x action_dim = " + std::to_string(expected));
    }
    for (float v : s.chunk) {
      if (!std::isfinite(v)) {
        return reject(Rejection::kNotFinite, "chunk holds NaN or inf");
      }
    }
    return Rejection::kNone;
  }

  MonitorResult update(const PolicySample & s) {
    MonitorResult out;
    if (started_ && s.episode != episode_) {
      new_episode(out);
    }
    if (started_ && s.episode == episode_ && s.step <= last_step_) {
      if (s.step == last_step_) {
        // The same inference again (a re-sent message) is not a new reading.
        out.rejection = Rejection::kRepeatedStep;
        out.rejected = "step " + std::to_string(s.step) + " repeated";
        return out;
      }
      // Steps went back without a new episode: the policy restarted. Its old
      // chunks say nothing about the new ones; keep the alarms as they are.
      spread_.reset();
    }
    // Steps apart from the last inference; 0 when they went back (a restart).
    const uint32_t gap = (started_ && s.episode == episode_ && s.step > last_step_) ? s.step - last_step_ : 0;
    started_ = true;
    episode_ = s.episode;
    last_step_ = s.step;

    // The latency of a run says nothing about its chunk, and the other way
    // round: check it even when the chunk itself cannot be used.
    if (config_.max_inference_ms > 0.0 && has_inference_time(s)) {
      apply(latency_gate_.update(static_cast<double>(s.inference_ms)), MonitorResult::Signal::kSlowInference,
            s.inference_ms, out);
    }

    out.rejection = validate(s, &out.rejected);
    if (out.rejection != Rejection::kNone) {
      return out;
    }
    if (dim_ != 0 && dim_ != s.action_dim) {
      // A policy with another action size is another policy.
      spread_.reset();
    }
    dim_ = s.action_dim;
    if (config_.spread_threshold > 0.0) {
      check_joint_scale(s.action_dim, out);
      out.spread = spread_.update(s.step, s);
      if (out.spread) {
        no_overlap_run_ = 0;
        // Said again only after the spread was computed for a while: a gap
        // that jitters around chunk_length would otherwise repeat it.
        if (no_overlap_told_ && ++spread_run_ >= kNoOverlapNoticeAfter) {
          no_overlap_told_ = false;
        }
        apply(spread_gate_.update(out.spread), MonitorResult::Signal::kChunkDisagreement, *out.spread, out);
      } else {
        spread_run_ = 0;
        if (++no_overlap_run_ >= kNoOverlapNoticeAfter && !no_overlap_told_) {
          no_overlap_told_ = true;
          out.notices.push_back("chunk spread cannot be computed: the last " + std::to_string(no_overlap_run_) +
                                " inferences shared no predicted step (latest " + std::to_string(gap) +
                                " steps apart, chunk_length " + std::to_string(s.chunk_length) +
                                "); the chunk disagreement check needs inferences closer than chunk_length steps");
        }
      }
    }
    return out;
  }

  bool chunk_disagreement_active() const {
    return spread_gate_.active();
  }
  bool slow_inference_active() const {
    return latency_gate_.active();
  }

 private:
  void new_episode(MonitorResult & out) {
    // Chunks of the last attempt say nothing about this one.
    spread_.reset();
    if (!config_.clear_on_new_episode) {
      // Alarms stay up until readings clear them (clear_after).
      return;
    }
    const bool spread_was = spread_gate_.reset();
    const bool latency_was = latency_gate_.reset();
    if (spread_was) {
      out.transitions.push_back({MonitorResult::Signal::kChunkDisagreement, false, 0.0});
    }
    if (latency_was) {
      out.transitions.push_back({MonitorResult::Signal::kSlowInference, false, 0.0});
    }
  }

  void check_joint_scale(uint16_t action_dim, MonitorResult & out) {
    const auto & scale = config_.joint_scale;
    if (scale.empty() || scale.size() == action_dim || scale_told_dim_ == action_dim) {
      return;
    }
    scale_told_dim_ = action_dim;
    out.notices.push_back("joint_scale has " + std::to_string(scale.size()) + " values but the policy sends " +
                          std::to_string(action_dim) + " joints: " +
                          (scale.size() < action_dim ? "the joints past them use 1" : "the extra values are unused"));
  }

  static void apply(RaiseGate::Edge edge, MonitorResult::Signal signal, double value, MonitorResult & out) {
    if (edge == RaiseGate::Edge::kRaise) {
      out.transitions.push_back({signal, true, value});
    } else if (edge == RaiseGate::Edge::kClear) {
      out.transitions.push_back({signal, false, value});
    }
  }

  MonitorConfig config_;
  ChunkSpread spread_;
  RaiseGate spread_gate_;
  RaiseGate latency_gate_;
  bool started_{false};
  uint32_t episode_{0};
  uint32_t last_step_{0};
  uint16_t dim_{0};
  uint32_t no_overlap_run_{0};
  uint32_t spread_run_{0};
  bool no_overlap_told_{false};
  uint16_t scale_told_dim_{0};
};

}  // namespace ros2_medkit_policy_bridge::core
