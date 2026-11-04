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

#include <httplib.h>

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <shared_mutex>
#include <string>
#include <thread>
#include <tl/expected.hpp>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/peer_json.hpp"
#include "ros2_medkit_gateway/core/aggregation/peer_resolver.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Collection of entities fetched from a peer gateway
 */
struct PeerEntities {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;

  /// Nested collection routes this peer does not offer, as route templates
  /// (e.g. ``/components/{id}/subcomponents``), each recorded once.
  ///
  /// A peer running an older gateway answers 404 for a route that did not
  /// exist yet. That is a version boundary, not a read failure, so the fetch
  /// carries on without those members and names the routes here for the
  /// caller to report.
  std::vector<std::string> absent_routes;
};

/**
 * @brief Why a call to a peer produced no usable answer.
 *
 * `httplib::Result::operator bool()` is a null check, so a branch on the result
 * alone renders a peer that ran out of time and a peer that refused the
 * connection identically. They are not the same event: the first says the peer
 * is working and this gateway stopped waiting, the second says nothing
 * answered, and an operator reading "unavailable" about a peer that is serving
 * the same request in five seconds is being told something false.
 */
enum class PeerFailureKind : uint8_t {
  kNone,             ///< No failure.
  kTimeout,          ///< A connect, read or write budget ran out. The peer may still be working.
  kUnreachable,      ///< Nothing answered: refused, no route, DNS or TLS failure.
  kCanceled,         ///< This gateway stopped the call, which happens during its own shutdown.
  kErrorStatus,      ///< The peer answered, with a status outside 2xx.
  kTooLarge,         ///< The body passed the peer-response size limit.
  kInvalidResponse,  ///< The peer answered with something that is not the JSON the route promises.
  kEntityNotFound,   ///< The peer answered 404 `entity-not-found`: it holds nothing by the id asked about.
};

/// A peer call that failed, and why. Kept together because every caller that
/// reports the message also has to decide a status from the kind, and the two
/// were previously separated by the error being a bare string.
struct PeerError {
  PeerFailureKind kind{PeerFailureKind::kUnreachable};
  std::string message;
};

/// Classify a cpp-httplib transport error.
///
/// cpp-httplib reports both "the read budget ran out" and "the connection
/// broke while the answer was arriving" as `Error::Read`, so the error alone
/// cannot tell a peer that is still working from one that died mid-answer.
/// `elapsed` against `budget` is what separates them: a call that came back
/// inside its budget did not run out of it, whatever else went wrong.
/// @param read_budget Budget for the read this call was making.
/// @param write_budget Budget for pushing the request body.
/// A write that ran out compares against the WRITE budget, which is its own and
/// is smaller than the read budget by default; comparing it against the read
/// budget classifies every write timeout as unreachable.
PeerFailureKind classify_peer_error(httplib::Error error, std::chrono::milliseconds elapsed,
                                    std::chrono::milliseconds read_budget, std::chrono::milliseconds write_budget);

/// Which budget a failure was measured against, so a message can name the one
/// that actually expired. A connect timeout is bounded by the connect budget,
/// not by the read budget the call was hoping to use.
enum class PeerBudget : uint8_t { kNone, kConnect, kRead, kWrite };

/// The budget `classify_peer_error` measured this error against.
PeerBudget peer_budget_for(httplib::Error error);

/// Stable wire token for a failure kind, as carried in
/// ``x-medkit.peer_failures[].reason``.
const char * peer_failure_reason(PeerFailureKind kind);

/**
 * @brief The budgets one peer request runs under.
 *
 * Four numbers rather than one, because they bound different things and a
 * single value can only be right for one of them at a time. A metadata read
 * that is given a real operation's budget stalls a discovery pass behind a
 * dead peer; a real operation that is given a metadata read's budget is cut
 * off while the peer is still working on it.
 */
struct PeerTimeouts {
  /// Connect budget for every call. A peer that will not complete a TCP
  /// handshake inside this is out of reach whatever the request asks for, so
  /// this stays short even where the read budget is long.
  int connect_ms{2000};

  /// Read budget for the peer's description of itself: discovery, health, and
  /// the collection fan-out a client is waiting on across every peer at once.
  int metadata_read_ms{2000};

  /// Read budget for a forwarded request, which carries the peer's own work.
  /// A synchronous service call on the far side runs against that gateway's
  /// `service_call_timeout_sec`, so a budget below it cannot ever
  /// see the answer; a large resource needs the transfer on top of that.
  int forward_read_ms{15000};

  /// Write budget for every call. cpp-httplib defaults this to 5 s.
  int write_ms{5000};

  /// The longest one request may run, from its connect to the last byte of
  /// the answer, however steadily the peer keeps sending: each read budget
  /// above bounds one read, not the answer. Never less, for a request, than
  /// its connect, write and read budgets added up, so no request is cut before
  /// its own budgets are spent; the health check keeps its own, the sum of its
  /// capped connect and read budgets.
  int request_deadline_ms{30000};

  /// Every read, connect and write budget set to the same value - the
  /// pre-split shape, for a caller that is not testing the split. The request
  /// deadline keeps its default.
  static PeerTimeouts uniform(int ms) {
    return PeerTimeouts{ms, ms, ms, ms};
  }
};

/**
 * @brief The per-call clients in flight against one peer, each with the
 * deadline of the request it is making.
 *
 * cpp-httplib bounds each read, not an answer, so a peer that sends a byte
 * at a time inside the read budget holds a request for as long as it keeps
 * sending. Every request is armed with a deadline as it starts and disarmed
 * when it returns; a reaper thread stops the client of a request still
 * running at its deadline, as stop_all() stops every client at shutdown, and
 * the request fails.
 *
 * stop() takes the client's socket lock, which the request holds while it
 * connects and, over TLS, through the whole handshake - a handshake whose
 * timeout restarts on every fragment the peer sends. A stop() alone would
 * wait as long as such a peer kept sending, so a request is cancelled through
 * its socket first: the registry holds a duplicate of the socket each
 * request creates (attach_socket), and cancelling shuts that down, which ends
 * the connect, handshake or read the request is blocked in without taking
 * any of the client's locks. stop() follows as before.
 *
 * Each stop() runs on a thread of its own with the registry mutex released,
 * and one that waits holds up no other request: none is kept from arming,
 * disarming or being stopped at its own deadline. The client is pinned while
 * a stop() of it runs - remove() and disarm() wait for it - so it is never
 * freed under the stop(), and a late stop() never lands on the client's next
 * request.
 */
class PeerCallRegistry {
 public:
  PeerCallRegistry();
  /// Stops the reaper and waits for it.
  ~PeerCallRegistry();
  PeerCallRegistry(const PeerCallRegistry &) = delete;
  PeerCallRegistry & operator=(const PeerCallRegistry &) = delete;
  PeerCallRegistry(PeerCallRegistry &&) = delete;
  PeerCallRegistry & operator=(PeerCallRegistry &&) = delete;

  /// `cli` is in flight; it is stopped by stop_all() until removed.
  void add(httplib::Client * cli);
  /// `cli` is done. Waits for a stop() of it in progress; `cli` may be freed
  /// once this returns.
  void remove(httplib::Client * cli);
  /// The request `cli` starts now runs for at most `deadline`.
  void arm(httplib::Client * cli, std::chrono::milliseconds deadline);
  /// The request `cli` is making has created `sock`. Called from the client's
  /// socket-options callback, before the connect. A duplicate is kept until
  /// the request ends, so the number stays reserved and a cancel never shuts
  /// a socket the process reused it for; a request already past its deadline
  /// has it shut at once.
  void attach_socket(httplib::Client * cli, int sock);
  /// The request `cli` was making has returned. Waits for a stop() of it in
  /// progress, so none lands on the next request, and releases the socket
  /// duplicate. True when the request ran into its deadline and was stopped.
  bool disarm(httplib::Client * cli);
  /// True once the request `cli` is making has run into its deadline.
  bool expired(httplib::Client * cli);
  /// Shut the socket of every request in flight, then stop() every client,
  /// each from a thread of its own, and return once every one of those stop()
  /// calls has.
  void stop_all();

 private:
  struct Call {
    std::chrono::steady_clock::time_point deadline{std::chrono::steady_clock::time_point::max()};
    bool expired{false};
    /// stop() calls of this client running now; it is pinned while any is.
    int stops{0};
    /// A duplicate of the socket of the request in progress, -1 when it has
    /// none (attach_socket).
    int socket{-1};
  };

  void reap();
  /// stop() `cli` from a thread of its own, `call` pinned until it returns.
  /// Called with mutex_ held; the stop() runs without it.
  std::future<void> stop_async(httplib::Client * cli, Call & call);
  /// Wait, with `lock` on mutex_, until no stop() of `call` is running.
  void wait_unpinned(std::unique_lock<std::mutex> & lock, const Call & call);
  /// Shut the socket of the request `call` is making, if it has one. A
  /// syscall, called with mutex_ held; it takes no lock of the client's.
  static void shut_socket(const Call & call);
  /// Close the socket duplicate `call` holds. Called with mutex_ held.
  static void release_socket(Call & call);

  std::mutex mutex_;
  /// The reaper's: a deadline changed, or the registry is going away.
  std::condition_variable wake_;
  /// A stop() returned.
  std::condition_variable stopped_;
  std::unordered_map<httplib::Client *, Call> calls_;
  bool stopping_{false};
  /// Last, so it starts once everything it reads exists.
  std::thread reaper_;
};

/**
 * @brief HTTP client for communicating with a peer gateway instance
 *
 * PeerClient wraps cpp-httplib to provide typed access to a peer gateway's
 * REST API. It supports health checking, entity fetching, transparent request
 * forwarding (proxy), and JSON fan-out queries.
 *
 * Thread safety: The healthy_ flag is atomic, and every call makes its own
 * HTTP client. All public methods are safe to call from any thread.
 */
class PeerClient {
 public:
  /**
   * @brief Construct a PeerClient for a peer gateway
   * @param url Base URL of the peer (e.g., "http://localhost:8081")
   * @param name Human-readable peer name (e.g., "subsystem_b")
   * @param timeouts Per-kind budgets; see PeerTimeouts
   * @param forward_auth Whether to forward Authorization headers to this peer
   */
  PeerClient(const std::string & url, const std::string & name, PeerTimeouts timeouts, bool forward_auth = false,
             std::string peer_auth_header = "");

  /// Convenience overload giving every budget the same value.
  PeerClient(const std::string & url, const std::string & name, int timeout_ms, bool forward_auth = false,
             std::string peer_auth_header = "");

  /// Get the peer base URL
  const std::string & url() const;

  /// Get the peer name
  const std::string & name() const;

  /// Check if the peer was healthy at last health check
  bool is_healthy() const;

  /// The peer's `x-medkit-instance-id` from its last health check, empty if it
  /// sent none. Lets a fan-out skip a peer already on the visited path.
  std::string instance_id() const;

  /**
   * @brief Perform a health check against the peer
   *
   * GETs /api/v1/health on the peer. Sets the internal healthy_ flag
   * based on whether a 200 response was received, and records the peer's
   * instance id. The call is made on its own client, so shutdown() stops it
   * like any other, and it ends at its deadline, its capped connect and read
   * budgets added up, however steadily the peer keeps sending.
   */
  void check_health();

  /**
   * @brief Fetch all entity collections from the peer
   *
   * GETs /api/v1/areas, /api/v1/components, /api/v1/apps, /api/v1/functions
   * and parses the items[] arrays, then the nested routes that carry structure
   * the lists omit: subareas, subcomponents, per-entity detail (the only source
   * of a Component's relationships and a Function's hosts) and per-app
   * operations. Each entity's source is set to "peer:<name>".
   *
   * The result describes the peer or it does not. Any sub-request that could
   * not be read - a dead connection, a non-200 the route has no other meaning
   * for, an oversized body, unparsable JSON, a field of the wrong type - fails
   * the whole fetch, because a picture missing a branch is indistinguishable on
   * the wire from a peer that does not have that branch. So does a refresh that
   * would keep more of the peer than its bounds allow: more than 1000
   * operations or data items for one App, more than 250,000 records or 16 MiB
   * of text in all. Two statuses do carry a meaning of their own and are read
   * rather than failed:
   *
   * - 404 on a nested collection route: the peer predates the route. Those
   *   members are omitted and the route is named in
   *   PeerEntities::absent_routes.
   * - `504 not-responding` on a Component detail: the peer holds that id and
   *   says whoever contributes it has gone quiet, which is what an aggregating
   *   peer answers for a declaration it is retaining. The Component is kept as
   *   its list named it, marked unavailable.
   *
   * @return PeerEntities on success, error message on failure
   */
  tl::expected<PeerEntities, std::string> fetch_entities();

  /**
   * @brief Forward an HTTP request transparently to the peer (proxy)
   *
   * Copies method, path, body, and Content-Type from the incoming request.
   * Forwards the Authorization header only if forward_auth is enabled.
   * Copies the peer's response status, headers, and body back to the
   * outgoing response.
   *
   * On connection failure, returns 502 with x-medkit-peer-unavailable error.
   * Once shutdown has begun, whether before the call or while it is being
   * admitted or is in flight, returns 503 with the same vendor code.
   *
   * @param req Incoming HTTP request to forward
   * @param res Outgoing HTTP response to populate
   */
  void forward_request(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Forward a request and parse the JSON response
   *
   * Used for fan-out merge scenarios where the aggregator needs to
   * combine JSON results from multiple peers.
   *
   * @param method HTTP method (e.g., "GET")
   * @param path Request path (e.g., "/api/v1/components/abc/data")
   * @param auth_header Authorization header value (empty to omit)
   * @param extra_headers Additional headers to include in the request
   *   (e.g., X-Medkit-No-Fan-Out to prevent recursive fan-out loops)
   * @return Parsed JSON on success, a classified PeerError on failure. The
   *   kind is what lets a fanned-out collection say WHY a peer contributed
   *   nothing; the caller used to receive a bare string and drop it.
   */
  tl::expected<nlohmann::json, PeerError> forward_and_get_json(const std::string & method, const std::string & path,
                                                               const std::string & auth_header = "",
                                                               const httplib::Headers & extra_headers = {});

  /**
   * @brief Cancel every in-flight HTTP call against this peer.
   *
   * Sets a shutdown flag, after which no call against the peer starts: a
   * forward answers 503 and every other call fails as canceled without
   * dialing. Then invokes ``stop()`` on every active per-call
   * ``httplib::Client`` (health check, fetch, forward), which unblocks worker
   * threads sitting in a call so gateway shutdown does not have to wait out
   * the full peer read timeout. A call that read the flag before it was set has its
   * socket by the time ``stop()`` runs, so ``stop()`` shuts it (see
   * admission_mutex_). Idempotent. Safe to call from any thread.
   */
  void shutdown();

 private:
  /// RAII helper for per-call clients: adds itself to calls_ on construction
  /// and removes itself on destruction. shutdown() stops every client in
  /// calls_ so blocked I/O unwinds, and calls_ stops one whose request runs
  /// past its deadline; removing waits for a stop() in progress rather than
  /// freeing the client under it. Whether a call made on it may start at all
  /// is decided per call, by admission_mutex_.
  class ScopedClient {
   public:
    /// @param read_timeout_ms Read budget for this call. Write comes from the
    /// owner's PeerTimeouts, which does not vary per call kind.
    /// @param connect_ms Connect budget for this call, the owner's when
    /// negative. The host name is resolved within it, before any request.
    ScopedClient(PeerClient & owner, const std::string & url, int read_timeout_ms, int connect_ms = -1);
    ~ScopedClient();
    ScopedClient(const ScopedClient &) = delete;
    ScopedClient & operator=(const ScopedClient &) = delete;
    ScopedClient(ScopedClient &&) = delete;
    ScopedClient & operator=(ScopedClient &&) = delete;

    httplib::Client & operator*() {
      return cli_;
    }
    httplib::Client * operator->() {
      return &cli_;
    }

    /// Why no request can be made on this client - the host name did not
    /// resolve in time, or does not resolve - or empty when one can.
    const std::string & unresolved() const {
      return unresolved_;
    }

    /// Point the client at the next address the host name resolved to. False
    /// when there is none left: the host is an IP address, or every address
    /// has been tried.
    bool next_address();

   private:
    PeerClient & owner_;
    httplib::Client cli_;
    std::vector<std::string> addresses_;
    size_t next_address_{0};
    std::string unresolved_;
  };

  /// The deadline of one request whose read budget is `read_ms`:
  /// PeerTimeouts::request_deadline_ms, or the request's connect, write and
  /// read budgets added up when that is longer.
  std::chrono::milliseconds request_deadline(int read_ms) const;

  std::string url_;
  std::string name_;
  PeerTimeouts timeouts_;
  bool forward_auth_;
  /// What this gateway presents as Authorization when it is acting for itself
  /// rather than relaying a client's request: the health check, and any
  /// forward whose caller sent no credential to pass on. Empty for none.
  std::string peer_auth_header_;
  /// The peer's host name, resolved with a bound before each call
  /// (ScopedClient), never by cpp-httplib inside one.
  PeerResolver resolver_;
  std::atomic<bool> healthy_{false};
  std::atomic<bool> shutdown_requested_{false};

  mutable std::mutex instance_id_mutex_;
  std::string instance_id_;

  PeerCallRegistry calls_;

  /// What makes starting a call and shutdown() exclusive. A call holds it
  /// shared from the moment it reads shutdown_requested_ until its socket
  /// exists (released from the client's socket-options callback, which
  /// cpp-httplib runs under the lock its stop() takes); shutdown() holds it
  /// exclusively while it stops the active clients. So a call either sees the
  /// flag and never dials, or is in flight when stop() runs and is shut down
  /// by it. Without it, a call that read the flag just before shutdown() set
  /// it dialed after stop() had found no socket to shut, and stop() on a
  /// client between two calls did not keep the next call from dialing.
  std::shared_mutex admission_mutex_;
};

}  // namespace ros2_medkit_gateway
