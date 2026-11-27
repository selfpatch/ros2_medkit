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

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

/// The host and port of a peer URL as cpp-httplib reads them: an IPv6 literal
/// without its brackets, and the scheme's default port when none is given.
struct PeerHostPort {
  std::string host;
  int port{0};
};

/// The host and port of `url` (`http://host:port...`), std::nullopt when it
/// names no host.
std::optional<PeerHostPort> peer_host_port(const std::string & url);

/**
 * @brief Resolves a peer's host name, waiting for it no longer than the
 * caller allows.
 *
 * cpp-httplib resolves the host itself, with getaddrinfo, inside the call and
 * under the lock its stop() takes. A resolver that does not answer holds the
 * call there for as long as the system resolver retries - ten seconds and
 * more with the default resolv.conf - and with it everything that waits on
 * the call or tries to stop it, which no budget of this gateway bounds. Here
 * the lookup runs on a thread of its own and a caller waits for it at most
 * the time it is given; the call is then made to the numeric address, which
 * cpp-httplib converts without asking anyone.
 *
 * At most one lookup is in flight per resolver: a caller that finds one
 * running waits for it rather than starting another, so a resolver that has
 * gone silent costs one thread per peer, not one per call. A lookup no caller
 * waits for any more finishes on its own thread, which owns what it writes.
 * A host that is an IP address, or a URL this cannot read a host from, is not
 * looked up at all.
 */
class PeerResolver {
 public:
  enum class Outcome : uint8_t {
    kResolved,  ///< `addresses` holds the host's addresses; empty when no lookup was needed.
    kFailed,    ///< The resolver answered that the name does not resolve.
    kTimedOut,  ///< The resolver did not answer in the time the caller allowed.
    kCanceled,  ///< cancel() was called; no lookup is waited for any more.
  };

  struct Result {
    Outcome outcome{Outcome::kResolved};
    /// Numeric addresses, in the order the system resolver returned them.
    std::vector<std::string> addresses;
  };

  /// For the host `url` names.
  explicit PeerResolver(const std::string & url);
  ~PeerResolver() = default;
  PeerResolver(const PeerResolver &) = delete;
  PeerResolver & operator=(const PeerResolver &) = delete;
  PeerResolver(PeerResolver &&) = delete;
  PeerResolver & operator=(PeerResolver &&) = delete;

  /// The host, as cpp-httplib keys its address map by it. Empty when the URL
  /// names none.
  const std::string & host() const;

  /// The host's addresses, waiting for them at most `timeout`.
  Result resolve(std::chrono::milliseconds timeout);

  /// Wake every caller waiting in resolve() and make later calls return
  /// kCanceled at once. For shutdown; it cannot be undone.
  void cancel();

 private:
  struct Lookup;

  std::string host_;
  std::string port_;
  /// The host is an IP address, or there is none: nothing to look up.
  bool needs_no_lookup_{true};

  std::mutex mutex_;
  std::shared_ptr<Lookup> pending_;
  bool canceled_{false};
};

}  // namespace ros2_medkit_gateway
