// Copyright 2026 Selfpatch GmbH
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

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace ros2_medkit_gateway {

/**
 * @brief mDNS-based discovery for auto-discovering peer gateways
 *
 * Uses the mjansson/mdns header-only C library to announce this gateway's
 * presence on the local network and browse for other gateways. Peers are
 * discovered via the configured service type (default: _medkit._tcp.local).
 *
 * Thread safety: start()/stop() are not thread-safe with respect to each other.
 * The running_ flag is atomic and used for clean shutdown of background threads.
 */
class MdnsDiscovery {
 public:
  /**
   * @brief Configuration for mDNS discovery
   */
  /// Callback invoked when an mDNS operation encounters an error (e.g., socket failure)
  using ErrorCallback = std::function<void(const std::string & message)>;

  struct Config {
    bool announce{false};                       ///< Broadcast presence via mDNS
    bool discover{false};                       ///< Browse for peers via mDNS
    std::string service{"_medkit._tcp.local"};  ///< Service type to announce/browse
    int port{8080};                             ///< Port this gateway listens on
    std::string name;                           ///< Instance name for announcement
    ErrorCallback on_error;                     ///< Optional error reporting callback
  };

  /// Callback invoked when a peer gateway is discovered
  using PeerFoundCallback = std::function<void(const std::string & url, const std::string & name)>;

  /// Callback invoked when a peer gateway is removed (goodbye received)
  using PeerRemovedCallback = std::function<void(const std::string & name)>;

  /**
   * @brief Construct an MdnsDiscovery instance
   * @param config Configuration controlling announce/discover behavior
   */
  explicit MdnsDiscovery(const Config & config);

  /// Destructor calls stop() to ensure clean thread shutdown
  ~MdnsDiscovery();

  // Non-copyable, non-movable (owns threads)
  MdnsDiscovery(const MdnsDiscovery &) = delete;
  MdnsDiscovery & operator=(const MdnsDiscovery &) = delete;
  MdnsDiscovery(MdnsDiscovery &&) = delete;
  MdnsDiscovery & operator=(MdnsDiscovery &&) = delete;

  /**
   * @brief Start mDNS announce and/or browse threads
   *
   * Starts background threads based on config flags:
   * - If announce is true, starts a thread that responds to mDNS queries
   * - If discover is true, starts a thread that sends mDNS queries and
   *   invokes callbacks when peers are found or removed
   *
   * @param on_found Callback for when a peer is discovered
   * @param on_removed Callback for when a peer sends a goodbye
   */
  void start(PeerFoundCallback on_found, PeerRemovedCallback on_removed);

  /**
   * @brief Stop all mDNS threads
   *
   * Sets running_ to false and joins all background threads. Safe to call
   * multiple times (idempotent).
   */
  void stop();

  /// Check if the announce thread is running
  bool is_announcing() const;

  /// Check if the discover/browse thread is running
  bool is_discovering() const;

 private:
  /// Main loop for the announce thread (listens for queries and responds)
  void announce_loop();

  /// Main loop for the browse thread (sends queries and processes responses)
  void browse_loop();

  Config config_;
  std::atomic<bool> running_{false};
  std::atomic<bool> announcing_{false};
  std::atomic<bool> discovering_{false};
  PeerFoundCallback on_found_;
  PeerRemovedCallback on_removed_;
  std::thread announce_thread_;
  std::thread browse_thread_;
};

}  // namespace ros2_medkit_gateway
