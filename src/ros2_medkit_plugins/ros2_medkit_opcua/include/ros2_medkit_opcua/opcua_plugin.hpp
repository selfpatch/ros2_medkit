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

#include "ros2_medkit_opcua/address_space_browser.hpp"
#include "ros2_medkit_opcua/device_identity.hpp"
#include "ros2_medkit_opcua/network_discovery.hpp"
#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"
#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/get_fault.hpp>

#include <ros2_medkit_gateway/core/discovery/models/asset_identity.hpp>
#include <ros2_medkit_gateway/core/plugins/gateway_plugin.hpp>
#include <ros2_medkit_gateway/core/plugins/plugin_http_types.hpp>
#include <ros2_medkit_gateway/core/providers/data_provider.hpp>
#include <ros2_medkit_gateway/core/providers/fault_provider.hpp>
#include <ros2_medkit_gateway/core/providers/introspection_provider.hpp>
#include <ros2_medkit_gateway/core/providers/operation_provider.hpp>
#include <ros2_medkit_gateway/dto/faults.hpp>
#include <ros2_medkit_gateway/plugins/ros_plugin_context.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace ros2_medkit_gateway {

/// OPC-UA Gateway Plugin - bridges OPC-UA PLCs into the SOVD entity tree
///
/// Implements GatewayPlugin (lifecycle, routes), IntrospectionProvider
/// (entity discovery from OPC-UA address space), and the typed provider
/// interfaces (DataProvider, OperationProvider, FaultProvider) so that
/// standard SOVD endpoints (/data, /operations, /faults) work for PLC
/// entities alongside the vendor x-plc-* extensions.
///
/// Standard SOVD endpoints (via provider interfaces):
///   GET  /{type}/{id}/data                    - DataProvider::list_data
///   GET  /{type}/{id}/data/{name}             - DataProvider::read_data
///   PUT  /{type}/{id}/data/{name}             - DataProvider::write_data
///   GET  /{type}/{id}/operations              - OperationProvider::list_operations
///   POST /{type}/{id}/operations/{name}       - OperationProvider::execute_operation
///   GET  /{type}/{id}/faults                  - FaultProvider::list_faults
///   GET  /{type}/{id}/faults/{code}           - FaultProvider::get_fault
///   DELETE /{type}/{id}/faults/{code}         - FaultProvider::clear_fault
///
/// Vendor REST extensions (via get_routes):
///   GET  /apps/{id}/x-plc-data          - All OPC-UA values for entity
///   GET  /apps/{id}/x-plc-data/{node}   - Single node value
///   POST /apps/{id}/x-plc-operations/{op} - Write value to PLC
///   GET  /components/{id}/x-plc-status  - Connection state and stats
///
/// Whether a value can be written at all is a property of the build, not of a
/// setting. ``MEDKIT_OPCUA_READ_ONLY`` defaults to ON: the OPC-UA write path is
/// then absent from the object, no data point is ever marked writable, neither
/// the x-plc-operations capability nor its POST route is registered, and
/// write_data / the value-write half of execute_operation refuse with 501
/// before reaching the client. ``-DMEDKIT_OPCUA_READ_ONLY=OFF`` builds the
/// write-capable plugin.
class OpcuaPlugin : public ros2_medkit_gateway::GatewayPlugin,
                    public ros2_medkit_gateway::IntrospectionProvider,
                    public ros2_medkit_gateway::DataProvider,
                    public ros2_medkit_gateway::OperationProvider,
                    public ros2_medkit_gateway::FaultProvider {
 public:
  OpcuaPlugin();
  ~OpcuaPlugin() override;

  OpcuaPlugin(const OpcuaPlugin &) = delete;
  OpcuaPlugin & operator=(const OpcuaPlugin &) = delete;
  OpcuaPlugin(OpcuaPlugin &&) = delete;
  OpcuaPlugin & operator=(OpcuaPlugin &&) = delete;

  // -- GatewayPlugin interface --
  std::string name() const override {
    return "opcua";
  }
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  std::vector<PluginRoute> get_routes() override;
  void shutdown() override;

  // -- IntrospectionProvider interface --
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  // -- DataProvider interface --
  tl::expected<dto::DataListResult, DataProviderErrorInfo> list_data(const std::string & entity_id) override;
  tl::expected<dto::DataValue, DataProviderErrorInfo> read_data(const std::string & entity_id,
                                                                const std::string & resource_name) override;
  tl::expected<dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & entity_id, const std::string & resource_name, const nlohmann::json & value) override;
  // Component and alarms-fallback entities own no NodeMap entries; false here
  // keeps the gateway from advertising a `data` capability list_data would 404.
  bool has_data(const std::string & entity_id) const override;

  // -- OperationProvider interface --
  tl::expected<dto::Collection<dto::OperationItem>, OperationProviderErrorInfo>
  list_operations(const std::string & entity_id) override;
  tl::expected<dto::OperationExecutionResult, OperationProviderErrorInfo>
  execute_operation(const std::string & entity_id, const std::string & operation_name,
                    const nlohmann::json & parameters) override;
  // True for any entity_defs() entry (mirrors list_operations' own lookup),
  // false for the Component itself, which has no entity_defs entry.
  bool has_operations(const std::string & entity_id) const override;

  /// How many times the fault store has ANSWERED the link-state clear gate.
  /// Read by tests that have to tell "the store said this fault belongs to
  /// another bridge" apart from "the store was never asked" - an absent clear
  /// looks identical either way, and no log or REST response separates them.
  uint64_t comms_lost_probe_count_for_test() const {
    std::lock_guard<std::mutex> lock(comms_lost_probe_state_->mutex);
    return comms_lost_probe_state_->answers;
  }

  /// Substitute the discovery sweep and identify before configure().
  /// configure() keeps whatever is set here and installs the real POSIX /
  /// open62541pp probes otherwise, so a test can drive the auto-endpoint and
  /// rescan paths without a network while the endpoints it names are real. Not
  /// part of the plugin's supported surface.
  void set_discovery_io_for_test(PortScanFn scan, IdentifyFn identify) {
    discovery_scan_fn_ = std::move(scan);
    discovery_identify_fn_ = std::move(identify);
  }

  /// How many sweeps found no hit carrying the binding while one was held (see
  /// select_auto_endpoint). Read by tests that have to tell "the sweep saw a
  /// foreign server and selected nothing" apart from "the sweep found nothing
  /// at all", which an unchanged endpoint looks identical for.
  uint64_t rescan_refused_count_for_test() const {
    return rescan_refusals_.load();
  }

  /// How many sessions have been dropped for reaching a server other than the
  /// one this bridge is bound to. Read by tests that have to tell that drop
  /// apart from a connect that simply failed.
  uint64_t binding_mismatch_count_for_test() const {
    return binding_mismatch_disconnects_.load();
  }

  /// The address-space walk configuration after configure() has merged the
  /// node map and the ROS parameters. Read by tests that need to see which
  /// source supplied a setting, which no REST response exposes.
  const AutoBrowseConfig & auto_browse_config_for_test() const {
    return node_map_.auto_browse_config();
  }

  // -- FaultProvider interface --
  tl::expected<dto::FaultListResult, FaultProviderErrorInfo> list_faults(const std::string & entity_id) override;
  tl::expected<dto::FaultDetailResult, FaultProviderErrorInfo> get_fault(const std::string & entity_id,
                                                                         const std::string & fault_code) override;
  tl::expected<dto::FaultClearResult, FaultProviderErrorInfo> clear_fault(const std::string & entity_id,
                                                                          const std::string & fault_code) override;

  // Resolve the SOVD severity bucket for an event alarm. An explicit configured
  // override wins; with none configured the raw OPC-UA event Severity (1-1000)
  // is mapped to a bucket by band (>=801 CRITICAL, >=501 ERROR, >=201 WARNING,
  // else INFO). Pure + static so it is unit-testable without a live server.
  static std::string map_severity(uint16_t live_severity, const std::string & severity_override);

  // Overlay a ``plugins.opcua.auto_alarms`` JSON/ROS param onto ``cfg``. Accepts
  // the bare-boolean shorthand or the full map form (same fields as the node-map
  // YAML ``auto_alarms:`` loader: source_node_id, entity_id, auto_clear,
  // severity_bands, include/exclude). Only keys actually present overwrite
  // ``cfg``, so this composes on top of whatever the YAML block set - the JSON
  // param wins, mirroring how ``plugins.opcua.auto_browse`` overlays its config
  // and how env vars override the rest of the plugin config. Unknown keys warn.
  // Callers run ``NodeMap::finalize_auto_alarms_overlay`` afterwards to re-derive
  // the default entity and parsed source NodeId. Static + injected ``warn`` so
  // the parse is unit-testable without a plugin instance.
  static void apply_auto_alarms_param(const nlohmann::json & value, AutoAlarmsConfig & cfg,
                                      const std::function<void(const std::string &)> & warn);

  // Where one discovery pass reports to, plus the memory that keeps a repeated
  // identical pass quiet. A rescan runs every ``interval_s`` for the life of a
  // disconnected process, so re-emitting the same per-server lines, summary and
  // "no auto-connectable server" WARN each time buries every other message in
  // the log. ``previous_outcome`` is owned by the caller (the plugin keeps one
  // across rescans): when it is non-null and the pass reaches the same outcome
  // as the pass before it, the whole report goes to ``debug`` instead. The first
  // pass, and every pass whose outcome changed, is always reported at info/warn.
  // A null ``previous_outcome`` (tests that do not care) reports every pass.
  //
  // The "scanning [subnets]" announcement is NOT part of that report. It is sent
  // before the sweep runs, because a wide subnet takes minutes and an operator
  // watching start-up has to see the gateway working. It says what the pass is
  // about to do, which is known before the outcome is, so it carries the same
  // first-pass / rescan levelling on its own.
  struct DiscoveryReporter {
    std::function<void(const std::string &)> info;
    std::function<void(const std::string &)> warn;
    std::function<void(const std::string &)> debug;
    std::string * previous_outcome{nullptr};
  };

  // Run one read-only discovery pass and return the endpoint URL to adopt.
  //
  // Returns nullopt - meaning "keep the endpoint you have" - when discovery is
  // disabled, when an endpoint was configured explicitly, when no subnet could
  // be resolved, or when the pass found no auto-connectable None/Anonymous data
  // server. Both the startup scan and the reconnect rescan go through here, so
  // the two cannot drift apart. Static with injected probes and log sinks so
  // both are unit-testable without a network.
  //
  // @param config discovery configuration (subnets, ports, timeouts, ...)
  // @param endpoint_configured true when the operator pinned endpoint_url, so
  //        discovery then selects nothing and can neither override the
  //        operator's target nor open a second session on an already polled PLC
  // @param scan injected TCP port probe
  // @param identify injected OPC-UA GetEndpoints identify
  // @param reporter operator-visible log sinks + repeat-suppression memory
  // @param cancelled abort predicate handed to NetworkDiscovery::run, so a
  //        shutdown does not have to wait out a full sweep
  // @param application_uri when non-null, receives the selected server's
  //        ApplicationUri - the identity the binding is made of
  // @param bound_application_uri the identity of the server the caller already
  //        holds a session with. Set, it is what selection looks for, at any
  //        address; empty, selection falls back to the lowest ip:port. A sweep
  //        that finds no hit carrying it selects nothing: the server this
  //        bridge is bound to is not on the network right now.
  static std::optional<std::string> discover_endpoint(const OpcuaDiscoveryConfig & config, bool endpoint_configured,
                                                      const PortScanFn & scan, const IdentifyFn & identify,
                                                      const DiscoveryReporter & reporter,
                                                      const std::function<bool()> & cancelled = {},
                                                      std::string * application_uri = nullptr,
                                                      const std::string & bound_application_uri = {});

  // Seconds between reconnect rescans, or 0 when the reconnect loop must never
  // rescan. That is the answer when discovery is disabled, when an endpoint was
  // configured explicitly, and when the operator set ``interval_s: 0``, which
  // means "keep discovery on but leave the startup scan one-shot". An UNSET
  // interval is the config-less case - it cannot name a cadence and is the one
  // that most needs its PLC adopted once it finishes booting - so it takes the
  // built-in default and keeps rescanning.
  static int effective_rescan_interval_s(const OpcuaDiscoveryConfig & config, bool endpoint_configured);

  // Default reconnect rescan cadence, in seconds, when discovery is enabled with
  // no explicit ``interval_s``. Long enough that a bounded subnet sweep stays a
  // background cost on the poll thread, short enough that a PLC finishing its
  // boot is picked up in well under a minute.
  static constexpr int kDefaultRescanIntervalS = 30;

  // One rate-limited rescan step: run ``sweep`` when the cadence is due,
  // otherwise do nothing.
  //
  // The cadence is measured from the END of the previous sweep, which
  // ``*last_scan_end`` stores. A sweep of a legal /16 takes minutes, so
  // stamping its start would make the next one due the moment it returned: the
  // poll thread would sweep back to back and the reconnect attempt would drop to
  // one per sweep. ``now`` is injected so the spacing is unit-testable without
  // sleeping.
  //
  // @return whatever ``sweep`` returned, or nullopt when it was not due yet.
  static std::optional<std::string> rescan_step(int interval_s,
                                                const std::function<std::chrono::steady_clock::time_point()> & now,
                                                std::chrono::steady_clock::time_point * last_scan_end,
                                                const std::function<std::optional<std::string>()> & sweep);

  // ``rescan_step`` with nothing escaping it.
  //
  // The sweep runs on the poll thread, which has no try of its own and is not
  // the thread that owns the process's error handling: ``std::system_error``
  // from a thread the parallel scan could not create, or ``bad_alloc`` from a
  // wide target list, leaves the whole gateway to ``std::terminate``. A sweep
  // that failed is a sweep that found nothing, so anything thrown is logged at
  // WARN and answered with nullopt; the cadence stamp ``rescan_step`` takes
  // still holds, so the next sweep is an interval away. The handlers allocate
  // nothing, because a bad_alloc is one of the throws they answer. This is the
  // callable the plugin's rescan hook runs, so a test that drives it drives
  // what the poll thread drives.
  static std::optional<std::string> rescan_guarded(int interval_s,
                                                   const std::function<std::chrono::steady_clock::time_point()> & now,
                                                   std::chrono::steady_clock::time_point * last_scan_end,
                                                   const std::function<std::optional<std::string>()> & sweep);

  // Ceiling for the poller's exponential reconnect backoff.
  //
  // Without discovery this is ``default_ceiling`` (60 s). While the reconnect
  // loop is rescanning, the rescan is only consulted once per reconnect attempt,
  // so the real adoption cadence is max(interval_s, backoff) - a documented
  // "every 30 s" would silently become every 60 s. Capping the backoff at the
  // rescan interval makes the documented cadence the true one. Never shorter
  // than ``base`` (the configured reconnect interval), so a tiny interval cannot
  // turn the backoff into a hot retry loop.
  static std::chrono::milliseconds effective_max_reconnect_wait(std::chrono::milliseconds base,
                                                                std::chrono::milliseconds default_ceiling,
                                                                int rescan_interval_s);

  // The component identity a config-less deployment should serve after a
  // connect, or nullopt when the identity it already has still holds.
  //
  // A gateway that starts before its PLC connects to nothing, so the identity
  // derived at start-up comes from an empty DeviceInfo and the fallback
  // endpoint: the neutral ``opcua-<host>`` placeholder. Once discovery adopts
  // the real server, the device can finally name itself, and the SOVD component
  // must stop serving the placeholder. Pure / static so the rule is testable
  // without a server.
  // What the caller knows about the identity it is serving, so the rule below
  // can be pure. The plugin owns the stand-in list these fields summarise.
  struct ComponentIdentityState {
    std::string current_id;              ///< the id being served
    bool current_is_placeholder{false};  ///< current_id came from a device read with no nameplate
  };

  static std::optional<ComponentIdentity> rederived_component_identity(const ComponentIdentityState & state,
                                                                       const OpcuaClient::DeviceInfo & info,
                                                                       const std::string & endpoint_url);

  // Extra reads the connect hook spends before it lets the session subscribe,
  // and the pause between them. The address space of a PLC that has just
  // accepted a session fills in over the following second, and this is the only
  // window in which a rename still precedes the conditions the server replays.
  // Plugin start-up pays this burst once, and only when the device answers the
  // first read without a nameplate.
  static constexpr int kIdentityConnectRetries = 3;
  static constexpr std::chrono::milliseconds kIdentityConnectRetryPause{250};

  // Device-info reads the POLL path may spend on one session, counted apart
  // from the connect burst above. A server that names itself settles on its
  // first read; one that has still not named itself after the connect read, the
  // burst and these keeps its stand-in for the session, so a nameplate-less
  // device costs a bounded number of reads per session.
  static constexpr int kMaxIdentityPollReads = 5;

  // What one call of the read loop did.
  struct IdentityReadOutcome {
    OpcuaClient::DeviceInfo info;  ///< the last read; empty when no read was made
    int reads_spent{0};            ///< the caller's budget after this call
    bool read_made{false};         ///< false when the budget was already spent
    bool interrupted{false};       ///< a pause asked the loop to stop
  };

  // Read device info until the device names itself, the budget runs out, or a
  // pause is refused.
  //
  // One read is made per attempt, up to ``extra_reads`` further attempts, each
  // preceded by ``pause_fn``. ``pause_fn`` returns false to stop the loop, which
  // is how a shutdown ends the connect burst without waiting it out; its
  // argument is the pause the caller asked for. ``reads_spent`` in and out is
  // the caller's own budget against ``max_reads``, so the connect burst and the
  // poll path can hold separate ones. Injected read and pause so the bound and
  // the cadence are testable without a server and without sleeping.
  static IdentityReadOutcome identity_reads(int extra_reads, int reads_spent, int max_reads,
                                            const std::function<OpcuaClient::DeviceInfo()> & read_fn,
                                            const std::function<bool(std::chrono::milliseconds)> & pause_fn);

  // Why a ClearFault is being sent. Two properties follow from it and nothing
  // else does, so the origin is what travels and both are derived from it:
  //   - whether the correlation cascade must be skipped
  //     (``clear_skips_correlation``), which goes on the wire, and
  //   - whether the clear is re-derivable (``clear_is_link_state``), which is
  //     what the pending buffer may give up first under pressure.
  enum class ClearOrigin {
    /// The device reported the condition inactive (an ``event_alarms`` /
    /// ``auto_alarms`` condition, or a threshold rule going false). A one-shot
    /// edge nothing will re-send, and a real resolution, so the cascade stands.
    DeviceAlarm,
    /// The OPC-UA session came back, so ``PLC_COMMS_LOST`` no longer holds.
    /// Re-derived on the next reconnect if it is lost, and not an operator
    /// resolving a root cause, so it must not cascade.
    LinkState,
    /// The SOVD per-entity ``DELETE /{entity}/faults/{code}`` route reached
    /// FaultProvider::clear_fault. An operator scoped to one entity must not
    /// cascade-clear symptoms reported by apps in other entities, which is the
    /// same rule the gateway applies on its own (non-plugin) branch of that
    /// route. One-shot: nothing re-derives an operator's decision.
    ScopedOperator
  };

  // Whether this clear must leave the correlation engine's auto_clear_with_root
  // cascade alone. True for everything except a device-reported clear.
  static bool clear_skips_correlation(ClearOrigin origin) {
    return origin != ClearOrigin::DeviceAlarm;
  }

  // Whether this clear will be re-derived if it is dropped. Only the link-state
  // clear will: the next reconnect sends it again.
  static bool clear_is_link_state(ClearOrigin origin) {
    return origin == ClearOrigin::LinkState;
  }

  // Whether a link-state clear may be sent, given what the fault store answered
  // when asked for that fault code.
  //
  // ClearFault carries no source: both storages clear the whole row. One
  // gateway may load two field-bus bridges (``plugins: ["opcua",
  // "beckhoff_ads"]``) that both raise ``PLC_COMMS_LOST``, and the fault
  // manager keys faults by code alone, so a fault whose reporting sources
  // include one this process never reported under is a fault another bridge
  // still holds. The clear goes out only when EVERY reporting source is one of
  // ``my_ids``: the component id being served, every placeholder id this
  // process assigned, and the NodeMap default. A fault that is absent, carries
  // no sources, or names any foreign source is left standing. A shared
  // ``PLC_COMMS_LOST`` is healed by an operator clear; the fault manager has no
  // per-source de-assert. Pure + static so the rule is testable without a fault
  // manager.
  static bool link_state_clear_permitted(bool fault_found, const std::vector<std::string> & reporting_sources,
                                         const std::unordered_set<std::string> & my_ids);

  // Whether a parked answer belongs to the probe the poll thread is waiting for.
  //
  // Each probe carries a number, stamped when the request goes out. The timeout
  // branch moves that number on before it re-owes the decision, because rclcpp
  // takes a pending entry out of the client before it invokes the callback and
  // outside its own mutex: a callback that won that race parks its answer while
  // the poll thread has already given the probe up. Pure + static so the rule is
  // testable without a store.
  static bool comms_lost_generation_current(uint64_t answered_generation, uint64_t current_generation) {
    return answered_generation == current_generation;
  }

  // Whether a link-state decision may be probed for on this poll.
  //
  // A probe costs a service round trip and its answer decides whether a fault
  // is cleared, so it is sent only when all four hold: a decision is owed, the
  // session is up (a clear while the link is down would clear a fault that is
  // genuinely standing), no probe is outstanding, and the store is reachable.
  // Pure + static so the conjunction is testable without a session.
  static bool comms_lost_probe_due(bool owed, bool connected, bool probe_in_flight, bool store_ready) {
    return owed && connected && !probe_in_flight && store_ready;
  }

  // Whether an answer the store parked may be acted on.
  //
  // The answer describes the store at the moment the probe was served, and the
  // link can die between the probe and the answer. "The link is up now" is not
  // enough to tell that it never went down: the decision is driven from
  // publish_values, which the poll loop reaches only while connected, so an
  // answer parked just before a drop is next looked at on the first tick AFTER
  // the reconnect - when the link is up again and the store's answer describes
  // an outage that has since been reported. The session the probe was sent on
  // is what settles it: ``probe_session`` is the connection generation stamped
  // when the request went out and ``live_session`` the one in force now.
  // Clearing on an answer from an earlier session would clear a fault that is
  // standing for a reason. Pure + static so the conjunction is testable without
  // a session.
  static bool comms_lost_answer_applicable(bool connected, bool has_answer, uint64_t probe_session,
                                           uint64_t live_session) {
    return connected && has_answer && probe_session == live_session;
  }

  // How long a fault-store probe may stay outstanding before it is dropped and
  // the decision is owed again. A fire-and-forget request whose answer never
  // comes would otherwise hold the decision for the life of the process.
  static constexpr std::chrono::milliseconds kDefaultFaultServiceTimeout{5000};

  // Whether a discovery sweep must stop now, given the two independent stop
  // signals. Static and pure so both inputs are testable: the member
  // ``discovery_cancelled()`` only reads them off the process and hands them
  // here, so this is the whole rule.
  //
  //   - ``shutdown_requested`` is set by shutdown(), which the gateway calls
  //     after its executor returns. That ends a RESCAN sweep, which runs on the
  //     poll thread long after start-up.
  //   - ``rclcpp_ok`` is false once rclcpp's own SIGINT / SIGTERM handler has
  //     run. The START-UP sweep runs inside set_context(), during node
  //     construction and before the executor spins, so shutdown() cannot be
  //     reached while it is in progress and the signal is the only thing that
  //     can end it.
  static bool discovery_cancelled_for(bool shutdown_requested, bool rclcpp_ok) {
    return shutdown_requested || !rclcpp_ok;
  }

  // ``discovery_cancelled_for`` reading the second signal off the node's OWN
  // context. A host that builds its GatewayNode on a private context
  // (``NodeOptions().context(...)``) never initialises the default one, and
  // ``rclcpp::ok()`` with no argument is then false for the life of the
  // process: every sweep cancels before it starts and the PLC is never found. A
  // null ``context`` - the plugin before set_context() has handed it one -
  // reads the default context, which is what a gateway built the ordinary way
  // runs on.
  static bool discovery_cancelled_for_context(bool shutdown_requested, const rclcpp::Context::SharedPtr & context) {
    return discovery_cancelled_for(shutdown_requested, context ? rclcpp::ok(context) : rclcpp::ok());
  }

  // Which kind of clear a fault-detection signal going inactive is. The poller
  // emits the component-scoped ``PLC_COMMS_LOST`` clear through the same
  // callback as every device alarm, and only that one is a link-state event.
  static ClearOrigin clear_origin_for_signal(const std::string & fault_code) {
    return fault_code == kCommsLostFaultCode ? ClearOrigin::LinkState : ClearOrigin::DeviceAlarm;
  }

  // Build the ClearFault request for one fault code.
  // ``skip_correlation_auto_clear`` goes on the wire verbatim (see ClearOrigin
  // for who sets it and why). Static so the wire field is assertable without a
  // fault manager.
  static ros2_medkit_msgs::srv::ClearFault::Request make_clear_fault_request(const std::string & fault_code,
                                                                             bool skip_correlation_auto_clear);

  // One entry in the bounded buffer of fault dispatches held while the
  // fault_manager service is unmatched.
  struct PendingFaultDispatch {
    enum class Kind { Report, Clear };
    Kind kind{Kind::Report};
    std::string fault_code;  ///< dedup key for a Clear, diagnostic for a Report
    /// Clear only: this dispatch is re-derivable (ClearOrigin::LinkState), so
    /// the buffer may drop it before anything that is not.
    bool link_state{false};
    std::function<void()> dispatch;
  };

  // What ``enqueue_pending_dispatch`` did, so the caller can log it.
  enum class PendingEnqueueOutcome {
    Buffered,               ///< appended, nothing lost
    ReplacedClear,          ///< superseded the pending clear for the same fault code
    EvictedLinkStateClear,  ///< buffer was full: dropped a re-derivable clear to make room
    EvictedOldest,          ///< buffer was full with nothing re-derivable in it: dropped the oldest entry
    Refused                 ///< buffer was full with nothing re-derivable in it and the incoming
                            ///< dispatch was itself a re-derivable clear, so it was dropped instead
  };

  // Enqueue policy for the bounded pending-dispatch buffer.
  //
  // Only a link-state clear is re-derivable: the next reconnect sends it again.
  // Everything else in the buffer is a one-shot edge nothing will re-send - a
  // report, a device alarm going inactive, an operator's scoped clear - so those
  // rank together and age out oldest-first, exactly as the buffer behaved before
  // any of this. A link-state clear is what a full buffer gives up first, and an
  // incoming one is refused so that every one-shot dispatch in the buffer
  // survives. At most ONE clear per fault code is pending at a time (a newer one
  // moves to the back, so an interleaved report-then-clear still flushes in that
  // order).
  //
  // Without the link-state ranking a flapping link enqueued one connect-time
  // clear per reconnect attempt and pushed real alarm reports out of the buffer.
  // Without the "only link-state" part, a device alarm's inactive edge was
  // evicted ahead of an older report and the flush replayed the raise with no
  // clear behind it, leaving the fault standing while the device said inactive.
  static PendingEnqueueOutcome enqueue_pending_dispatch(std::vector<PendingFaultDispatch> & buffer, size_t max_size,
                                                        PendingFaultDispatch entry);

  // Bound on the pending-dispatch buffer, so a deployment with no fault_manager
  // cannot grow it without limit.
  static constexpr size_t kMaxPendingDispatches = 256;

 private:
  // Route handlers
  void handle_plc_data(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);
  void handle_plc_data_single(const ros2_medkit_gateway::PluginRequest & req,
                              ros2_medkit_gateway::PluginResponse & res);
#if !MEDKIT_OPCUA_READ_ONLY
  /// Route handler for the vendor write endpoint. Declared and registered only
  /// in a write-capable build; a read-only build serves no such route.
  void handle_plc_operations(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);
#endif
  void handle_plc_status(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);

  // Fault-detection signal (threshold / status-bit / enum) -> Fault bridge
  void on_alarm_change(const std::string & entity_id, const ros2_medkit::fault_detection::FaultSignal & signal);

  // Issue #386: native AlarmConditionType event lifecycle bridge.
  void on_event_alarm(const AlarmEventDelivery & delivery);

  // Report/clear fault via ROS 2 service (private helpers, not the FaultProvider overrides)
  void send_report_fault(const std::string & entity_id, const std::string & fault_code,
                         const std::string & severity_str, const std::string & message);
  // ``origin`` says why the clear is being sent, which decides both the wire
  // flag and how the pending buffer ranks it. See ClearOrigin.
  void send_clear_fault(const std::string & fault_code, ClearOrigin origin = ClearOrigin::DeviceAlarm);

  // Record that a link-state decision about PLC_COMMS_LOST is due.
  //
  // Called from every point that learns the link state changed or that a
  // decision was lost: a successful connect (set_context and the poller's
  // reconnect arm, the latter through on_alarm_change), a link-state clear the
  // bounded buffer evicted or refused, and a probe that timed out. Setting a
  // flag is all it does; the decision itself belongs to the poll thread.
  void owe_comms_lost_decision();

  // Drive the owed link-state decision one step. Poll thread only (called from
  // publish_values), which is the thread that performs the config-less rename,
  // so component ids are read here without a lock.
  //
  // One step is: consume an answer the executor thread parked, or drop a probe
  // that outlived the store timeout, or send a probe when one is due
  // (comms_lost_probe_due). The store is asked asynchronously because the first
  // decision is owed from set_context, where the gateway's executor is not
  // spinning yet and a blocking wait would never be answered.
  void drive_comms_lost_decision();

  // Apply the gate to an answer the store gave and send the clear it permits.
  // Poll thread only.
  void apply_comms_lost_answer(bool found, const std::vector<std::string> & reporting_sources);

  // The ids a fault reported by THIS process can carry: the component id being
  // served, every placeholder this process assigned, and the NodeMap default.
  // Poll thread only (see drive_comms_lost_decision).
  std::unordered_set<std::string> reporting_ids_of_this_process() const;

  // Check the session that just came up against the binding, and establish the
  // binding when there is none. Returns false when the session reached a
  // different server, in which case it has been dropped and the caller must
  // treat the connect as not having happened. Called from every path that
  // observes a successful connect.
  bool bind_or_drop_session();

  // Record an ApplicationUri this bridge has reported as not its own, keeping
  // the last kMaxRefusedApplicationUris. Returns true the first time a given
  // URI is seen since the last adoption, which is what makes the report once
  // per outage per server.
  bool note_refused_application_uri(const std::string & uri);

  // Record an id assigned from a read with no nameplate, keeping the last
  // kMaxPlaceholderIds. Poll thread / set_context thread.
  void remember_placeholder_id(const std::string & id);

  // Dispatch now if the fault_manager service is matched, else buffer the
  // dispatch (bounded, order-preserving) to be flushed once it appears.
  void send_or_buffer(PendingFaultDispatch entry);
  // Flush buffered fault dispatches when the fault_manager service is ready.
  void flush_pending_reports();

  // Publish PLC values to ROS 2 topics (called after each poll)
  void publish_values(const PollSnapshot & snap);

  // Create the per-entry ROS 2 Float32 publishers for numeric node map
  // entries. Called from set_context() AFTER connect()+run_auto_browse() so
  // that auto-discovered entries also get a publisher (moved out of its
  // original inline position, which ran before the node map could contain
  // anything auto_browse added).
  void create_value_publishers();

  // Run the recursive OPC-UA address-space walk (auto_browse) against the
  // now-connected client and merge the discovered entries into node_map_, then
  // (re)create value publishers for any new entries. Requires a live session.
  // Called from set_context() right after the initial connect(), and again from
  // the poll thread on every reconnect via maybe_rebrowse_on_reconnect(). The
  // merge into node_map_ is serialized against the REST read paths by
  // node_map_mutex_. No-op (never called) when auto_browse is disabled.
  void run_auto_browse();

  // Re-derive the SOVD component identity from the device once a NEW session is
  // up, in config-less mode only. Called from PollerConfig::on_connected, which
  // the poller fires right after a session comes up and BEFORE it
  // (re)subscribes, and again from publish_values on every poll until the
  // identity is settled (a first device-info read that came back empty leaves
  // it unsettled, and the poll retries within the same session)
  // (an explicit node map owns the name). This is what stops a gateway that
  // started before its PLC from serving the ``opcua-<fallback host>``
  // placeholder for the life of the process after discovery adopted the real
  // server. Logs the change at INFO and rebuilds every derived reference (the
  // ``<component_id>_alarms`` entity, entity_defs) under the node-map lock.
  // No-op when the identity is unchanged.
  //
  // Running before the (re)subscribe is what keeps the rename ahead of the
  // conditions the server replays on the adopted session: apply_condition_state
  // pins an entity id at the FIRST observation of a ConditionId, so a
  // ConditionRefresh burst delivered before the rename would pin every replayed
  // fault under the old ``<old-id>_alarms`` entity, which the rename then drops.
  // ``extra_reads`` is how many further device-info reads this call may spend,
  // kIdentityConnectRetryPause apart, when the first comes back without a
  // nameplate. The connect hook spends kIdentityConnectRetries; the poll-thread
  // retry spends none and draws on kMaxIdentityPollReads for the session.
  void maybe_rederive_component_identity(int extra_reads);

  // Poll-thread hook (from publish_values): re-run auto_browse when the client
  // has established a new session since the last walk. Covers the field case
  // where the gateway starts before the PLC is reachable (initial connect
  // fails, so the initial walk is skipped) and the PLC only comes up later, as
  // well as a PLC restart. Idempotent: merge drops already-mapped node ids and
  // create_value_publishers() skips existing topics.
  void maybe_rebrowse_on_reconnect();

  // Log the effective OPC-UA security profile (policy / mode / user auth) at
  // startup; warns when running unsecured.
  void log_security_profile() const;

  // Read-only active-scan discovery. When discovery is enabled and no endpoint
  // was explicitly configured, run one pass, log a summary, and (if a suitable
  // None/Anonymous OPC-UA data server is found) set client_config_.endpoint_url
  // to the discovered ip:port so the existing connect + introspect path adopts
  // it. Injected scan/identify default to the real POSIX + open62541pp
  // implementations; tests override them. No-op when discovery is disabled or
  // an endpoint is already configured.
  void run_startup_discovery();

  // Log sinks for a discovery pass: the plugin's operator-visible info/warn
  // plus the named ``opcua.plugin`` debug logger a repeated identical pass falls
  // back to. ``previous_outcome`` is the caller's repeat-suppression memory
  // (null to report every pass in full).
  DiscoveryReporter discovery_reporter(std::string * previous_outcome) const;

  // Abort predicate handed to a discovery sweep: reads the two stop signals off
  // the process and applies ``discovery_cancelled_for_context``, which holds the
  // rule and the reasoning behind it.
  bool discovery_cancelled() const;

  // Poll-thread hook bound into PollerConfig::rediscover_endpoint whenever
  // discovery runs without a configured endpoint. Called from the poller's
  // reconnect arm, so only while no session is up, and rate-limited to one scan
  // per effective_rescan_interval_s(). Returns the newly selected endpoint when
  // a rescan found a different server (and logs the swap at INFO), nullopt when
  // the rescan is not due yet or changed nothing.
  std::optional<std::string> rescan_endpoint_for_reconnect();

  // Build JSON response for data endpoint
  nlohmann::json build_data_response(const std::string & entity_id) const;

  std::atomic<bool> shutdown_requested_{false};
  ros2_medkit_gateway::RosPluginContext * ctx_{nullptr};
  // The context the gateway node was built on. A discovery sweep reads the
  // shutdown signal of THAT context. Null until set_context() (the plugin is
  // constructed before it has a node), which discovery_cancelled_for_context
  // treats as "use the default".
  rclcpp::Context::SharedPtr context_;
  OpcuaClientConfig client_config_;
  PollerConfig poller_config_;
  std::string node_map_path_;

  // Read-only PLC/OPC-UA network discovery (opt-in, default disabled).
  OpcuaDiscoveryConfig discovery_config_;
  // True when the operator pinned endpoint_url via config or OPCUA_ENDPOINT_URL.
  // Discovery only auto-selects an endpoint when this is false, so it never
  // overrides an explicit target or opens a second session on a polled PLC.
  bool endpoint_configured_{false};
  // True when the operator supplied a ``plugins.opcua.auto_alarms`` block.
  // When false AND no node_map_path is set, config-less discovery enables
  // native A&C by itself (default source Server object i=2253) so discovered
  // Alarms & Conditions surface on /api/v1/faults with no per-alarm mapping.
  bool auto_alarms_configured_{false};
  // Injected discovery I/O; default to the real POSIX / open62541pp probes.
  // Tests substitute in-memory fakes to exercise the auto-endpoint path without
  // a network. Set in configure(), consumed in run_startup_discovery().
  PortScanFn discovery_scan_fn_;
  IdentifyFn discovery_identify_fn_;
  // When the last discovery pass FINISHED, so the reconnect rescan honours the
  // cadence and sweeps the subnet at most once per interval, whatever the
  // reconnect attempts do. Stamped by the startup scan, then only ever
  // read/written on the poll thread. See rescan_step for why the end of the
  // sweep is the reference point.
  std::chrono::steady_clock::time_point last_discovery_scan_end_{};
  // The ApplicationUri of the server this plugin is bound to: the identity of
  // the server a session was actually established with, read off that session
  // (OpcuaClient::read_server_application_uri). It is what discovery looks for
  // on every later sweep and what a fresh session is checked against.
  //
  // Empty until a session has been held: a process that has never connected is
  // bound to nothing, so its first adoption is unconstrained. It is also empty
  // for a server that publishes no ApplicationUri, which therefore cannot be
  // bound to. A restart clears it, because it is never persisted. Written on
  // the set_context thread, then the poll thread.
  std::string bound_application_uri_;
  // ApplicationUris already reported as not this bridge's, so one outage does
  // not log the same foreign server every interval_s. Bounded, and cleared on
  // every successful adoption so a later outage reports again. Poll thread only.
  std::vector<std::string> refused_application_uris_;
  static constexpr size_t kMaxRefusedApplicationUris = 8;
  // Sweeps that found no hit carrying the bound identity, and sessions dropped
  // for reaching a different server (see the *_for_test accessors).
  std::atomic<uint64_t> rescan_refusals_{0};
  std::atomic<uint64_t> binding_mismatch_disconnects_{0};

  // Outcome digest of the previous discovery pass, so an unchanged rescan
  // reports at DEBUG and the whole report goes out once per change, not once
  // per interval_s. Poll thread only (the startup scan runs before the poller
  // exists).
  std::string last_discovery_outcome_;

  std::unique_ptr<OpcuaClient> client_;
  NodeMap node_map_;

  // Guards the mutable parts of node_map_ (entries / indices / entity_defs)
  // that auto_browse rewrites on reconnect against the REST read paths. The
  // re-walk runs on the poll thread and takes the unique lock only for the
  // merge; the poller's own node_map_ reads share that thread, so they need no
  // lock. Read handlers on the HTTP thread take a shared lock for as long as
  // they hold pointers/refs into node_map_.
  mutable std::shared_mutex node_map_mutex_;

  // OpcuaClient::connection_generation the last auto_browse walk ran against (0
  // = never walked). The poll thread re-walks when the live generation differs,
  // mirroring device_identity_generation_. Written on the set_context thread
  // (initial walk, happens-before the poller starts) then only on the poll
  // thread, so no atomic is needed.
  uint64_t auto_browse_generation_{0};

  // INV2: asset-identity nameplate read once per session from the server's
  // device-info (ServerStatus/BuildInfo + optional OPC-UA DI nameplate) on the
  // first connected introspect, then reused until the client reconnects.
  // device_identity_generation_ stores the OpcuaClient::connection_generation
  // the nameplate was read at (0 = never read), so a poller reconnect triggers
  // a fresh read on the next introspect without hammering the server.
  AssetIdentity device_identity_;
  uint64_t device_identity_generation_{0};

  // OpcuaClient::connection_generation the config-less component identity was
  // derived at (0 = derived with no session, i.e. from the fallback endpoint and
  // an empty DeviceInfo). The poll thread re-derives when the live generation
  // differs, mirroring device_identity_generation_. Written on the set_context
  // thread (happens-before the poller starts) then only on the poll thread.
  uint64_t component_identity_generation_{0};
  // Session the poll-read counter below belongs to, so a new session gets its
  // full kMaxIdentityPollReads budget.
  uint64_t component_identity_read_generation_{0};

  // ROS 2 service clients for fault reporting
  struct FaultClients;
  std::unique_ptr<FaultClients> fault_clients_;

  // Ordered buffer of pending fault report/clear dispatches. ReportFault /
  // ClearFault are fire-and-forget, so a report sent before the fault_manager
  // service is DDS-matched is dropped. Instead of blocking startup for the sink,
  // buffer the dispatch (preserving report-then-clear order) and flush it on the
  // next poll once the service is ready.
  //
  // Accessed from TWO threads: the poll thread (on_alarm_change / on_event_alarm
  // -> send_*_fault -> send_or_buffer, and publish_values -> flush_pending_reports)
  // AND the REST thread (the FaultProvider::clear_fault route override ->
  // send_clear_fault -> send_or_buffer). A push_back that reallocates the vector
  // while the other thread iterates/reads it is a use-after-free that corrupts the
  // heap, so every access is serialised by pending_reports_mutex_. The mutex is
  // never held across the actual dispatch (async_send_request) to keep ROS I/O out
  // of the critical section.
  std::mutex pending_reports_mutex_;
  std::vector<PendingFaultDispatch> pending_reports_;

  // Where the fault store's answer is parked between the executor thread that
  // receives it and the poll thread that acts on it.
  //
  // It is a separately owned block, not plugin members, because the GetFault
  // callback outlives nothing it can check: rclcpp hands a callback to an
  // executor thread that may already be inside it when the plugin is destroyed,
  // and a mutex the plugin owns cannot make its own destruction safe. The
  // callback holds a weak_ptr and does nothing when the lock fails.
  //
  // ``generation`` counts probes. The poll thread stamps it when it sends one;
  // the callback carries the stamp it was created with and stores an answer
  // only while it still matches, so an answer to a probe that already timed out
  // cannot be read as the answer to the one that replaced it.
  struct CommsLostProbeState {
    std::mutex mutex;
    uint64_t generation{0};           ///< the probe the poll thread is waiting for
    uint64_t answered_generation{0};  ///< the probe the parked answer belongs to
    bool answered{false};
    bool found{false};
    std::vector<std::string> sources;
    uint64_t answers{0};  ///< answers ever parked (comms_lost_probe_count_for_test)
  };

  // Link-state decision state. ``owed`` is set by whoever learns a decision is
  // due (set_context, the poll thread via on_alarm_change, the REST thread via
  // the pending buffer) and taken by the poll thread with exchange(false)
  // BEFORE it sends a probe, so an owe raised while the send is in progress
  // survives it. Everything else belongs to the poll thread, except the answer
  // slot inside the state block.
  //
  // The buffer gives a link-state clear up first because the next reconnect
  // re-derives it, which holds for the poller's reconnect clear and not for the
  // one a successful connect queued: a gateway restarting with a persisted
  // PLC_COMMS_LOST behind 256 alarm edges has no reconnect coming, so the debt
  // is recorded here instead.
  std::atomic<bool> comms_lost_decision_owed_{false};
  bool comms_lost_probe_in_flight_{false};
  std::chrono::steady_clock::time_point comms_lost_probe_sent_{};
  int64_t comms_lost_probe_request_id_{0};
  // OpcuaClient::connection_generation at the moment the probe went out. An
  // answer is acted on only while that session is still the live one.
  uint64_t comms_lost_probe_session_{0};
  std::shared_ptr<CommsLostProbeState> comms_lost_probe_state_{std::make_shared<CommsLostProbeState>()};
  // A PLC_COMMS_LOST raise this process sent since the last decision. The store
  // can answer a probe before it has processed a ReportFault that was already
  // on the wire, so one further decision follows any that had a raise behind
  // it. Set on the poll thread, taken on the poll thread.
  std::atomic<bool> comms_lost_raise_since_decision_{false};
  // How long a probe may stay outstanding (kDefaultFaultServiceTimeout unless
  // ``fault_service_timeout_ms`` says otherwise).
  std::chrono::milliseconds fault_service_timeout_{kDefaultFaultServiceTimeout};

  // Component ids this process assigned from a device read with no nameplate,
  // i.e. ``opcua-<host>`` for some endpoint it targeted, oldest first. A fault
  // reported while one of these was being served carries it as its source, and
  // healing that fault after the device names itself is the whole point of
  // keeping them. One entry per adopted endpoint: the FIRST kMaxPlaceholderIds
  // distinct stand-ins of a process are kept, and a re-adopted one keeps its
  // place, so the id set the gate builds stays small on a link that is
  // re-discovered many times. Written on the set_context thread then the poll
  // thread.
  std::vector<std::string> placeholder_ids_;
  static constexpr size_t kMaxPlaceholderIds = 8;
  // Poll-path device-info reads spent on this session's identity, bounded by
  // kMaxIdentityPollReads. The connect burst carries a budget of its own and
  // does not draw on this one.
  int component_identity_poll_reads_{0};

  // Tracks which non-numeric nodes have already been warned about (avoids log spam).
  // Instance member instead of static to survive plugin reload (dlclose/dlopen).
  std::unordered_set<std::string> warned_non_numeric_;

  // ROS 2 publishers for PLC value bridging (node_id_str -> publisher).
  // Declared before poller_ so that C++ reverse-destruction-order guarantees
  // the poller thread is joined before publishers are destroyed.
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_;

  // Must be declared last among resources used by the poll thread.
  // ~OpcuaPoller() calls stop() which joins the thread.
  std::unique_ptr<OpcuaPoller> poller_;
};

}  // namespace ros2_medkit_gateway
