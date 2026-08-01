graph_watchdog design
=====================

Role
----
Detects silent faults in the ROS 2 graph. A ``GatewayPlugin``
hosting a fleet of detectors; each detector observes the ROS 2 graph and raises
faults via a ``ReportFault`` service client on the gateway node. Faults reach the
SOVD ``/faults`` API via FaultManager - the plugin injects no entities. Its only
HTTP surface is a read-only reliability status route (see `Reliability core`_
below).

Structure
---------
- **Plugin shell** (``GraphWatchdogPlugin``): loads via the gateway plugin ABI
  (v7). In ``set_context`` it casts the context with ``as_ros_plugin_context``,
  creates one ``rclcpp::Client<ReportFault>`` on the gateway node, and runs the
  detector tick loop on a dedicated thread the plugin owns - not a gateway wall
  timer, because detectors do blocking service reads and the gateway's small
  executor is only safe while blocking work never runs on an executor thread.
  No child node - the gateway spins its own node.
- **Detector pattern**: ``Detector`` + ``DetectorContext``; each detector is one
  self-registering file under ``src/detectors/`` (``REGISTER_DETECTOR``), globbed
  by CMake so parallel authors never edit a shared file. A detector's source
  needs no CMake or registry edit; adding a per-detector unit test is the one
  shared touch - it adds an ``ament_add_gtest`` entry to the test block.
- **Fault-code contract**: the frozen ``GRAPH_*`` namespace.
- **mode seam**: ``raise`` / ``advisory`` / ``off`` per detector.
- **Reliability core**: see the dedicated section below - central warmup and
  lifecycle gating enforced inside ``raise_fault``, detector-consulted clock
  validity, and the ``x-medkit-watchdog`` status route.

Reliability core
-----------------
- **ReliabilityGate** composes two independent trackers and is the single entry
  point both the tick loop and the HTTP route go through:

  - ``WarmupTracker`` (pure, no ROS): an entity arms once continuously present
    for ``warmup_cycles`` ticks. A disappearance longer than a short forget
    grace followed by a reappearance (a real mid-run restart) re-warms it from
    scratch; a transient one-tick discovery gap is absorbed and does not
    re-warm, so recurring DDS churn cannot re-arm (and thus permanently
    suppress) a still-running entity. Unknown ``source_id``\ s (e.g. a topic,
    not a tracked app) fall back to a global bringup grace window keyed off the
    first tick the graph was seen non-empty, which re-arms whenever the graph
    empties out so a full-stack restart gets the grace again.
  - ``LifecycleWatcher`` (event-driven): discovers managed ``rclcpp_lifecycle``
    nodes from the introspection snapshot, seeds their state via a
    ``GetState`` service call, then keeps it fresh by subscribing to each
    node's ``~/transition_event`` topic (reliable + volatile, matching the
    ``rcl_lifecycle`` publisher). A node still cached non-active shortly after
    discovery is briefly re-seeded via ``GetState`` so an ``active`` transition
    lost during the subscription's DDS endpoint-matching window self-heals
    rather than suppressing the node for the process lifetime; the blocking
    seeds are bounded per tick so a batch bringup cannot stall the tick loop.
    A re-seed reads with no lock held and blocks for as long as the remote
    takes, so its answer is applied only when no ``~/transition_event`` for that
    entry landed in the meantime - a per-entry ``label_epoch`` is recorded when
    the job is selected and re-compared when it is applied, or the mechanism
    that recovers a lost activation could undo an observed one, permanently
    when it happens on the last attempt of the bounded budget.
    The subscription callback holds only a ``weak_ptr`` to the watcher's shared
    state, so a callback in flight during teardown bails instead of touching
    freed memory - and that ``weak_ptr``, not the ``node_mutex_`` the drop path
    takes, is what makes teardown safe. The lock's scope is the case where the
    erase itself runs ``~Subscription``; when the executor still holds its own
    strong ref to an already-dispatched subscription, the destructor runs later
    on the executor thread with no lock of ours held, which no mutex taken on
    the tick thread can close. Non-managed nodes are never gated - ``node_ok``
    returns true for them unconditionally.
- **Central enforcement.** ``DetectorContext::raise_fault`` runs every raise
  through ``reliability_allows(gate, source_id)`` before the fault client
  sends anything; a detector raising about a still-warming-up entity or a
  lifecycle-inactive node is silently suppressed, transparent to the
  detector. ``clear_fault`` is never gated - a fault can always be cleared. A
  null gate (not yet wired) always allows, so unit tests that construct a
  bare ``DetectorContext`` are unaffected.
- **Clock validity is detector-consulted, not central.**
  ``WatchdogClock::time_is_valid()`` flags a paused or absent ``/clock`` under
  ``use_sim_time`` by comparing wall-clock advancement against sim-time
  advancement on each ``mark_tick()``. Unlike warmup/lifecycle, this is not
  enforced inside ``raise_fault`` - a time-based detector must check it
  itself and skip its own age/grace-period math for the tick when it is
  false.
- **tf_static QoS helper.** ``tf_static_qos()`` returns a depth-1
  ``transient_local`` ``rclcpp::QoS``, available to detectors alongside
  ``time_is_valid()``. ``/tf_static`` publishers latch with transient-local
  durability, so a late subscriber must match it; the actual ``/tf_static``
  subscription belongs to a later detector.
- **Status endpoint.** ``GraphWatchdogPlugin::get_routes()`` registers
  ``GET x-medkit-watchdog`` (mounted at ``/api/v1/x-medkit-watchdog`` by the
  gateway), returning ``ReliabilityGate::status_json()``: schema version,
  ``warmup_cycles``, a ``global_state`` (``armed``/``warming_up``), and one
  entry per known entity (``id``, ``first_seen_tick``, ``armed``, ``state``,
  ``lifecycle``). Returns 503 with ``ERR_SERVICE_UNAVAILABLE`` if the gate has
  not been constructed yet or has already been torn down by ``shutdown()``.
- **Build note.** ``LifecycleWatcher`` reuses the gateway's own
  lifecycle-state helpers (``lifecycle_status_helpers.cpp``,
  ``ros2_lifecycle_state_reader.cpp``) compiled in via ``GATEWAY_SRC_DIR`` -
  the same non-header-only reuse pattern other gateway plugins use - rather than reimplementing lifecycle-state
  parsing.

Detectors
---------
``qos_mismatch``, ``orphan``, ``param_drift`` and ``lifecycle_expectation`` are the
detectors this package ships so far. The remaining silent-fault classes land in
follow-up changes, each against its own issue.

``qos_mismatch`` raises ``GRAPH_QOS_MISMATCH``. It
watches every topic's publisher/subscriber QoS pairs rather than parameter values: each
tick it enumerates every topic via ``get_topic_names_and_types()`` and, for each one,
every currently-connected publisher and subscriber (``get_publishers_info_by_topic`` /
``get_subscriptions_info_by_topic``, which report the RESOLVED live profile - never a
hand-built one), checking each pub x sub pair for incompatibility.

**Two levels behind one fault code.** Per-pair incompatibility is not reported uniformly.
A subscriber incompatible with EVERY publisher on its topic receives nothing and raises at
``SEVERITY_ERROR``. A subscriber incompatible with some publisher but not all raises at
``SEVERITY_WARN``: DDS refuses that one pair, so that producer's data is silently
discarded while the topic keeps looking alive - a real silent fault, since an
RxO-incompatible pair is a match DDS has already computed, not a heuristic. One fault code
carries one severity, so the emitted fault reflects the worst finding in the sweep. Before
this split the partial case was indistinguishable from healthy, which meant e.g. a second
``/tf`` broadcaster or a hand-rolled ``BEST_EFFORT`` publisher on ``/diagnostics`` went
unreported forever.

**RxO rule.** ``qos_policy.hpp``'s ``qos_incompatibility(pub, sub)`` implements RxO
(Request <= Offered) compatibility for the QoS policies that silently starve a
subscriber with no error surfaced anywhere in the graph: reliability, durability,
liveliness kind, deadline, and liveliness lease duration. Publisher = offered,
subscriber = requested; a pair is incompatible only in the one strict direction per
policy - a ``BEST_EFFORT`` publisher against a ``RELIABLE`` subscriber, a ``VOLATILE``
publisher against a ``TRANSIENT_LOCAL`` subscriber, an ``AUTOMATIC``-liveliness
publisher against a ``MANUAL_BY_TOPIC`` subscriber, or an offered deadline / lease
duration greater than the requested one. The reverse direction (e.g. a ``RELIABLE``
publisher feeding a ``BEST_EFFORT`` subscriber) is compatible by design - the subscriber
asked for no more than it is offered - even though the QoS profiles differ, which is
exactly the discriminator the integration and e2e tests each pin with a positive control
(see "Test tiers" below). For deadline and lease, an unspecified/infinite offer is
unbounded (fails any finite request), while an unspecified subscriber value does not
constrain the policy (always compatible); history and depth are not RxO dimensions and
are not checked. The checks match only concrete incompatible enum pairs, so
``SYSTEM_DEFAULT``/``UNKNOWN`` (never
reported by a live endpoint, which always carries the resolved profile) never raise.

**Aggregation via the shared helper.** ``qos_mismatch`` was the first detector to use the
``AggregatedFault`` helper (``aggregated_fault.hpp``); ``orphan`` and ``param_drift`` go
through the same one, so no detector reimplements the level-triggered raise/clear
pattern. The rationale: the fault_manager identifies a fault by
``fault_code`` alone, so one ``GRAPH_QOS_MISMATCH`` per mismatched topic would collide
into a single record under the shared code. The detector keeps one ``AggregatedFault``
instance for the whole graph and, each tick, hands it every currently-mismatched
topic's description (keyed by topic name so a repeat mismatch on the same topic
overwrites rather than duplicates); an empty map on a clean tick clears
(``EVENT_PASSED``), level-triggered semantics (see
"Closing the loop" in the README for the ``healing_enabled`` requirement to actually
reach HEALED). ``graph_source_id()``, also in ``aggregated_fault.hpp``, returns the
constant ``kGraphWatchdogEntityId`` (``"graph_watchdog"``) and nothing else, so every
detector's aggregated faults land under the same ``source_id``. It deliberately does NOT
fall back to the host Component id: a runtime host Component built by ``HostInfoProvider``
never sets ``external``, ``collect_component_app_fqns`` only puts a Component's bare id in
the fault scope set when it does, and a fault raised under it is therefore listed by no
entity endpoint at all. The ``ctx`` argument is unused and kept only because every call
site already has it in hand.

**Coverage is exhaustive, not budgeted.** Unlike a budgeted
round-robin sweep (bounded by a parameter-service round-trip budget),
``qos_mismatch`` reads no external service - ``get_topic_names_and_types()`` and the
two ``get_*_info_by_topic()`` calls are local graph-cache queries, so every topic is
checked every tick with no coverage-latency trade-off to configure or budget knob to
tune. ``/rosout`` and ``/parameter_events`` are skipped - ROS 2 system topics with
their own well-known QoS conventions, not useful signal for this detector. The sweep
polls ``ctx.cancelled`` between topics so a shutdown mid-sweep bails promptly, the same
shutdown-responsiveness contract every detector honors.

**Test tiers.** Three tiers each prove a different layer, deliberately not
overlapping:

1. **Unit** (``test_qos_policy.cpp``): pure ``qos_incompatibility()`` logic against
   hand-built ``rmw_qos_profile_t`` values - the RxO trio (reliability, durability,
   liveliness), the reverse-direction compatible case, identical-profile compatibility,
   and the never-raise guarantee for ``SYSTEM_DEFAULT``/``UNKNOWN`` enums a live
   endpoint never actually reports.
2. **Integration** (``test_qos_mismatch_integration.cpp``): a real ``rclcpp::Node``
   publisher/subscriber pair over real DDS, read through the actual
   ``get_publishers_info_by_topic``/``get_subscriptions_info_by_topic`` API - proving
   the detector reads the RESOLVED live profile correctly, not just that the pure
   comparison function is correct - against a fake ``ReportFault`` service. A second,
   concurrent RxO-compatible-but-different-QoS topic pair proves the detector does not
   over-fire on "QoS differs somewhere in the graph".
3. **E2e** (``test/e2e/test_qos_e2e.test.py``): the Acceptance gate - a
   real gateway process with the plugin ``.so`` loaded, a real fault_manager, and the
   operator-visible ``GET /api/v1/faults`` surface, proving the whole raise/clear story
   reaches a SOVD fault through the real tick timer, not just the detector's own
   ``tick()`` called directly.



``orphan`` raises ``GRAPH_ORPHAN``. It looks for the mistake no ROS 2 tool
reports: a topic that exists but has no counterpart, because one side was spelled
slightly differently. Each tick it counts publishers and subscribers per topic, keeps
the one-sided ones, and pairs a publisher-only topic with a subscriber-only topic when
their names are within ``max_edit_distance`` Levenshtein edits of each other. Only a
pair is reported, never a lone one-sided topic: a publisher with no subscriber is
normal on almost every robot, so it carries no signal on its own.

**Three guards keep this from crying wolf.**

- Names must sit in the same parent namespace. ``/a/scan`` and ``/b/scan`` differ by one
  character but belong to two different robots, and pairing them would be nonsense. The
  leaf and the namespace carry separate edit budgets and the namespace one defaults to
  zero, so this is an exact match unless an operator raises
  ``namespace_edit_distance``. Raising it is what makes a misspelled namespace
  (``/robott/scan`` against ``/robot/scan``) visible at all, at the price of reporting
  every robot of a fleet whose namespaces are themselves near-misses, which is why it is
  opt-in rather than tuned.
- A pair that matches once every run of digits collapses to one placeholder is an
  enumeration, not a typo. ``/lidar_1`` next to ``/lidar_2``, each one-sided, is the
  ordinary state of a multi-sensor machine, and reporting it would fire on every such
  machine. Collapsing runs rather than comparing character positions also covers
  ``/lidar_9`` next to ``/lidar_10``, where the index changes length. The price is a real
  typo in a digit going unreported, which is a deliberate trade.
- ``grace`` consecutive sweeps must agree before anything is raised. During bringup one
  side of a healthy pair is routinely missing for a moment, which looks exactly like a
  typo until the other side appears.

An operator who still gets a false alarm puts the topic in ``allowlist``. What this
detector cannot see at all is a namespace added or dropped by accident: ``/scan``
against ``/robot/scan`` is far past any sensible edit distance.

**Test tiers.** ``test_orphan_policy.cpp`` pins the pure matching rules, including each
guard and the cases it deliberately lets through. ``test_orphan_integration.cpp`` drives
the detector over a real ``rclcpp`` graph and a fake ``ReportFault`` service, covering
the grace counter, the clear on repair, and the allowlist. ``test/e2e/test_orphan_e2e.test.py``
is the acceptance gate: a real gateway with the plugin loaded, a real fault_manager, and
the fault read back from ``GET /api/v1/faults``. It carries three pairs at once - one
reported, one allowlisted, one visible only under a raised ``namespace_edit_distance`` -
so it also proves a string array and an integer survive the parameter path into
``configure()``, which no C++ tier can show.


``param_drift`` raises ``GRAPH_PARAM_DRIFT``. It reads node parameters over the
parameter service and reports a value that no longer matches its reference. The
reference is either self-captured, the value the parameter had when the node armed, or
pinned in config through ``expect``. A node absent for longer than the window the
reliability gate absorbs loses its captured baseline, so a restart - the documented way to apply
a new configuration - re-captures instead of reporting the change as drift. A shorter gap keeps
the baseline on purpose: DDS discovery drops a node from a single poll routinely, and re-capturing
on that would take the node's current, possibly already drifted, value as the new reference and
heal a real fault that could then never fire again. Only the pinned form can catch a value that was
already wrong at startup; self-capture by construction treats whatever it first sees as
correct.

**A budgeted sweep, unlike every other detector here.** ``qos_mismatch`` and ``orphan``
read the local graph cache and are therefore exhaustive every tick. This one makes real
service calls to other nodes, so it is bounded by ``max_reads_per_tick`` and sweeps the
graph round-robin. The budget is spent in service ROUND TRIPS at the watched node rather
than in transport calls, because one call is several requests there: a self-capture read
is a list plus a batched get, and an ``expect`` pin the node declares is a list, a get and a
descriptor read. A pin the node does NOT declare stops at the list, and is charged two rather
than the one it costs, because a second and rarer NOT_FOUND path does cost two and the error
code cannot tell them apart - a load bound may overstate, never understate. That buys a bound
on the load the sweep puts on watched nodes and pays with
coverage latency: a drift is invisible until its node's turn comes, and so is a repair.
The reads run on a thread the plugin owns rather than the gateway executor, so a node
that stops answering stalls this sweep only.

**Three horizons for a stale entry.** A node missing from one sweep is not forgotten
immediately. Its captured BASELINE is dropped on the third consecutive sweep without it - the
two the warmup tracker's forget grace absorbs, plus one - so that a one-tick discovery gap does
not silently reset the baseline and hide a real drift behind a fresh capture. Its FINDING
survives ``prune_grace`` consecutive absences and is dropped on the next one, so that the same
gap does not clear the aggregated fault either. Only the finding horizon is configurable:
``prune_grace`` is a per-detector config key, while the baseline horizon is the compile-time
``kBaselineForgetGrace``. A ``prune_grace`` below 2 makes the finding the shorter of the two and
the baseline then goes on the same sweep, because the absence counter it rides on is dropped
with the finding.

The third horizon is not about absence at all. A node that stays PRESENT but leaves the
reliability gate's ``active`` state is never re-evaluated (it is not armed) and never pruned (it
is still there), so its finding would otherwise be re-raised for ever - an operator who fixes
the parameter and leaves the node deactivated would watch ``GRAPH_PARAM_DRIFT`` stay CONFIRMED
naming a value that is no longer true. ``kFrozenHoldTicks`` (60 consecutive gate-denied ticks,
one minute at the default tick period) bounds that: past it the finding is released and the app
also stops holding back a clear, and both re-derive from a fresh read if it ever arms again.
Because it is a compile-time constant and counted in ticks rather than in rotations, neither
``max_reads_per_tick`` nor ``tick_interval_ms`` shortens it.

**Severity is WARN, deliberately.** The detector knows a value moved; it does not know
whether that breaks the robot. Grading the consequence needs knowledge of the machine
that the graph does not carry.

**Nothing is cleared that was not measured.** A clear asserts that nothing is drifting anywhere,
which is only true if every app was actually looked at, so the aggregate emits nothing at all while
any app still has no usable read. That covers three states which are all "nothing is known about
this app": the round-robin has not reached it, its reads keep failing, or the reliability gate has
not armed it - and the last of those is the ordinary state of every app for the first
``warmup_cycles`` ticks of a run, so without it a restart clears before issuing a single round trip.
A cached read counts as evidence only until a read of that app FAILS, after which the app is
unmeasured again. Each hold is bounded from the other side: an app is given up on after both sixty
failed attempts and sixty ticks unread, and a gate-denied one after the frozen hold. The bound on
attempts alone would not do, because an attempt is not a duration - the reader paces itself off
``max_reads_per_tick``, so at the accepted ceiling sixty-one attempts are spent in milliseconds and
a node still being discovered would be written off. Finally, a clear withheld for a minute of ticks
is logged once with the count of apps behind it, because from outside the process a correctly
withheld clear and a detector that has nothing to report look exactly the same.

**Test tiers.** ``test_param_drift_policy.cpp`` pins the pure comparison, rendering and
config rules. ``test_param_drift_integration.cpp`` drives real nodes with real parameters over
the real parameter service against a fake ``ReportFault`` service, including a node that
never answers, which must not hang the tick, and a node that answers past the per-read bound,
which must not be read at all.
``test/e2e/test_param_drift_e2e.test.py`` is the acceptance gate: one gateway launch carrying
the whole raise-and-clear story for the detector's default self-capturing mode, with a real
fault_manager and both ends read back from ``GET /api/v1/faults``.
``test/e2e/test_param_roundtrip_e2e.test.py`` measures what one parameter read really costs the
node that serves it, against a node that answers the parameter services by hand and counts every
request - so a round trip added to or removed from the TRANSPORT is caught, instead of quietly
invalidating the round-trip figures the budget is built on. Nothing else in the package can
notice that: every other test proves the arithmetic GIVEN those figures, and the stand-in
transport they use counts calls. It does not, however, read the detector's own charge constants
at all - it runs with ``param_drift`` switched off - so what holds those to the measurement is
the timing suite in ``test_param_drift_integration.cpp``, which times the reader against a
configured budget and therefore moves when a charge does. Cost and charge are pinned separately
and on purpose.
``test/e2e/test_config_plumbing_e2e.test.py`` drives three further gateway launches covering the
nested ``expect`` sub-object reader, ``mode: "off"``, and the bare ``mode: off`` that the
ROS parser types as a YAML 1.1 boolean; those three prove configuration delivery and
suppression only, and assert no clear.


``lifecycle_expectation`` raises ``GRAPH_NODE_INACTIVE``. It watches an
operator-declared set of ``require_active`` node names rather than topics, parameters,
or presence. Each tick it matches every configured entry against the live apps by
``App::id`` OR the stable ``effective_fqn()`` OR that fqn's bare leaf name -
``App::id`` alone is recomputed each sweep and gets namespaced on a same-bare-name
collision, so an id-only match would silently stop covering a multi-robot graph; a
bare name therefore matches that node in every namespace (use a full FQN to pin one).
For each matched present node it reads the node's raw lifecycle label via
``ReliabilityGate::lifecycle_state_of()`` (an additive, const accessor on the gate,
delegating to the internal ``LifecycleWatcher::state_of`` under the gate's own mutex)
and hands the matches - a ``std::vector<LifecycleMatch>`` of (entry, node fqn, state) -
to ``lifecycle_expectation_tracker.hpp``'s ``LifecycleExpectationTracker``, the
detector's pure, ROS-free core (mirroring ``qos_policy.hpp``/``orphan_policy.hpp``'s
role for their detectors), independently unit-tested before the detector ever touches
a live gate.

**Presence is not enough here - and absence is someone else's fault.** A
present-but-non-active managed node (``inactive``, ``unconfigured``, ``finalized``) is
exactly the case this detector exists to catch: on a Nav2 stack, a
``controller_server`` stuck ``inactive`` means the robot silently will not act, with
no crash and no signal on ``/diagnostics`` or ``/rosout``. A node that VANISHES from
the snapshot is the presence fault class (``GRAPH_NODE_DISAPPEARED``, reserved in the
frozen namespace; its detector lands in a follow-up change), so if a node previously
reported ``GRAPH_NODE_INACTIVE`` then vanishes entirely, the aggregate CLEARS rather
than staying raised - the integration test's
``AbsenceAfterRaiseClearsNotNodeDeathsDomain`` guards exactly this handoff. **Safe
default: off.** ``require_active`` is empty by default, so the detector emits nothing
at all - not even clears - until an operator opts specific nodes in (zero false
positives, the same config-scoped posture ``param_drift``'s ``expect`` uses).

**Keyed by node, not by config entry.** The bare-name form of ``require_active`` is
deliberately fleet-wide ("all ``controller_server``\ s must be active"), so one entry
legitimately covers N nodes. The tracker therefore keys violations by the node's
stable fqn: two namesakes are two report entries (one cannot silently replace the
other, and a healthy one cannot mask a broken one), and the fault description names
each offending node's FQN with the entry that demanded it as context - the
description is the only carrier, since every ``GRAPH_*`` fault shares one
``source_id``.

**Why grace, not the reliability gate (the design decision this detector turns on).**
Every detector's raise is subject to the central ``reliability_allows(gate,
source_id)`` gate inside ``ctx.raise_fault()``, which ANDs the entity's warmup state
with ``LifecycleWatcher::node_ok()`` - true only when a managed node's lifecycle is
``active`` (or it is not tracked at all). Gating THIS detector's raise the same way on
the required node's OWN lifecycle would be self-defeating: ``node_ok()`` is exactly
FALSE for the inactive node this detector exists to report, so the central gate would
suppress the signal forever. Instead ``LifecycleExpectationTracker::update`` counts
consecutive not-active ticks itself, per node, and returns a node in the affected map
only once the count exceeds its own ``grace`` - entirely independent of
``reliability_allows``. This does not bypass bringup-quiesce entirely: the aggregated
fault's own outer ``ctx.raise_fault`` call is still subject to the central gate for
the aggregate's own ``source_id`` (``graph_watchdog``) - only the per-node lifecycle
check bypasses it, because that check IS what this detector reports on.

**Empty label is never enforced - a load-bearing correctness rule - but it is not a
healthy read either.** ``lifecycle_state_of()`` can return ``optional("")`` for a node
that IS tracked (managed) but whose one-shot ``GetState`` seed failed and no
``~/transition_event`` has arrived yet - ``node_ok()`` already treats an empty label as
"do not gate" for exactly this reason (an unconfirmed label is not proof of anything,
and ``transition_event`` is volatile so it never backfills a genuinely-active node's
missed seed). If the tracker ENFORCED an empty string - e.g. as "unknown non-active" -
it would false-positive an actually-active node whose seed transiently failed. A node
is therefore reported only when its label was read, is non-empty, and is not
``active``, unit-tested explicitly (``EmptyLabelIsBenignNotInactive``) as its own case
distinct from the ``nullopt`` case.

Not enforcing it is not the same as treating it as health, though, and that is the
second half of the rule: an empty label arriving AFTER the node was read once is the
tail of an absence (see "Absence has its own grace" below), so it ages the absence
budget instead of zeroing the violation streak. ``nullopt`` is a different fact - no
tracked lifecycle at all, nothing to preserve - and resets the streak the way a healthy
read does. The two are pinned separately
(``UnreadLabelAfterAStreakCountsAsAbsentNotAsAReset``,
``FirstContactUnreadLabelHoldsNoBookkeeping``).

**Unknown (``nullopt``) state is never enforced, for a different reason than empty: no
lifecycle exists to be wrong about.** A ``nullopt`` from ``lifecycle_state_of()``
means the node is not a tracked managed lifecycle node at all (no
``GetState``/``ChangeState`` service pair was found for it). Two warnings keep that
from hiding a misconfiguration, neither of them a fault: an entry that matches NO node
at all for more than 10 consecutive ticks is surfaced by the tracker once per entry
(nothing else can report it - a presence detector only tracks nodes that were present
at least once), and an entry that matches present nodes but none with a tracked
lifecycle state for more than 5 consecutive ticks is warned about as a probable typo.
The tracker counts CONSECUTIVE no-match ticks, so an entry whose node matched and later
left the graph surfaces there too - and for that one the warning's own sentence ("never
came up, and the presence class cannot see it either") is false in both halves, so the
detector says it only about entries that have never matched anything since
``configure()``; a departed node belongs to ``GRAPH_NODE_DISAPPEARED``. Same
distinction the withheld-clear guard's never-matched leg makes below.

The consecutive-ticks requirement is what keeps the typo warning from latching on a
transient: ``discover_apps()`` wraps per-node service enumeration in a try/catch and
pushes the app regardless, so a sweep that races service discovery yields an app with
no services and no tracked state for a tick or two. Both warn-once latches are scoped
to the CURRENT config (cleared on every ``configure()``), so an entry removed and
re-added warns again; the typo warning also clears when the entry resolves, and is
skipped entirely when ``ctx.gate`` is null - with no gate, every entry looks unmanaged
and the warning would accuse correct config.

**Absence has its own grace; sustained absence discards the streak.** A matched node
that leaves the snapshot keeps its violation streak for ``absence_grace`` (3) ticks -
zeroing it on the first blink would mean a stuck node that drops out of one snapshot
in every few (routine on a churning graph at the shipped 1s tick against the entity
cache's 1s debounce) never accumulates ``grace + 1`` consecutive
present-and-inactive ticks. The node's RETURN spends the same budget rather than
resetting it: leaving the snapshot also makes ``LifecycleWatcher`` erase the node's
tracked entry, so on return the state is re-seeded and reads ``""`` until the fresh
``GetState`` answers - the tail of the same event, one tick later, and counted as
absence here. Past the absence grace the streak is ERASED - it does not survive to be
resumed - since sustained absence is the presence class's concern. The tracker keeps
the two counters per node fqn (``misses_`` and ``absent_``), and every counter moves at
most once per NODE per ``update()``: the caller pushes one match per (entry, node)
pair, so a node named by both a bare-name and a full-FQN entry would otherwise advance
at 2x and silently halve the operator's grace.

**Bounded tracking, by absence - deliberately.** The map is NOT simply bounded by the
operator-declared ``require_active`` set: keys are node fqns, and one bare-name entry
matches a node of that name in every namespace, so identity churn (nodes reappearing
under ever-new fqns) would grow it without a prune. A node absent past ``prune_ticks``
has its bookkeeping reclaimed - both tracker counters and the detector's own
withheld-clear guard entry; ``configure()`` clamps
``prune_ticks = max(prune_grace, grace + 1)``. The horizons can be ordered either way
through ordinary config (``grace: 1`` with ``prune_grace: 2`` clamps to 2, one under
the default absence grace of 3), so the absence loop discards the streak at whichever
of the two comes first: an fqn pruned before it ever crossed the absence grace would
otherwise keep its ``misses_`` entry for the process lifetime, resuming the streak on
return and leaking one entry per fqn ever seen. ``tracked_count()`` counts DISTINCT
fqns across both maps for that reason - reporting only the absence map made exactly
that leak invisible to every boundedness assertion in the suite.

Pruning by absence is safe here in a
way it is not for a presence-style tracker, because "reported" and "absent" are
mutually exclusive by construction: a node lands in the affected map only while
present, an absent node is never this detector's own reported fault, so ageing out an
absent node's bookkeeping can never silently heal a violation this detector raised -
and the ``grace + 1`` floor makes a currently-reported node structurally
prune-exempt.

**The clear is withheld until the required set has actually been measured.** A clear
asserts that every required node is healthy. That is the restart-heal hazard: a
gateway restart brings every detector counter back to zero while the
``GRAPH_NODE_INACTIVE`` raised before it is still in the fault_manager's store (a
separate process), so a clear emitted on the strength of an empty affected map heals a
fault that is still real. THREE states produce that empty map without the assertion
being true, and all three are the ordinary state of bookkeeping that just started over
(a restarted gateway, a reconfigure - ``configure()`` rebuilds the tracker - or a node
that respawned stuck):

- **Not matched yet.** Before the entity snapshot catches up with the graph a
  ``require_active`` entry matches nothing at all, so both other legs (each keyed by a
  MATCHED node) are empty and the clear is about a node the detector has never once
  looked at. The plugin ticks as soon as it is loaded, so every restart passes through
  this window.
- **Not measured.** An unread label (``nullopt`` or ``""``) is never enforced, so a
  matched node nothing has answered for looks exactly like a healthy one.
- **Measured not-active, below grace.** The tracker reports only past ``grace``
  consecutive not-active ticks, so for a whole grace-wide window the detector's own
  last read says ``inactive`` while its report says nothing is affected. The tracker
  surfaces those nodes separately (``LifecycleExpectationReport::pending``). Labels are
  seeded BEFORE the detectors tick, so on a responsive stack this - not the unread
  state - is what a restart actually lands in.

Any of the three withholds the emission entirely, neither raise nor clear. A raise is
never withheld: a violation read from the nodes that DID answer is real regardless of
the unread ones. Every hold is bounded. The never-matched and never-read legs release
after 60 consecutive ticks (a minute at the shipped cadence, mirroring
``param_drift``'s frozen hold - an unmanaged or typo'd entry must not block healing
forever), and the never-read one burns on every matched tick, including ticks on which
a raise about a DIFFERENT node was flowing, so a raise that outlives the hold leaves
the clear free the moment it heals. The pending leg releases as soon as the node reads
``active`` or its streak passes grace and the fault is raised again. Both node-keyed
legs stop at the absence grace when a node leaves the snapshot (the same presence-class
handoff the tracker makes, and the reason an entry that HAS matched and then stops
matching does not re-enter the never-matched state), and guard bookkeeping for a node
absent past ``prune_ticks`` is reclaimed. All three counters move once per NODE per
tick, never once per matching entry. Because a correctly withheld clear and a detector
with nothing to report look identical from outside, a hold that lives past 10
consecutive ticks is explained in the log once per episode, naming every reason in
force and, for the two node-keyed ones, the count behind each and one node by name.

The guard restarts with ``configure()``, and that is the scope of its "ever read"
latch: it is keyed by fqn and it does NOT reset when the NODE restarts. A process that
dies and respawns under the same name re-enters with ``read`` still true from the dead
incarnation, so its brand-new labels cannot withhold anything on the unread leg. That
is deliberate rather than tolerated - the pending leg covers the respawn case directly
(a node that comes back stuck starts a streak, and a streak below grace withholds on
its own), while a per-incarnation latch would put a fresh 60-tick hold on every blink
of a healthy node.

**A re-bind is a fresh binding.** ``LifecycleWatcher`` keys its tracked map by
``App::id``, and an id can survive a graph sweep while pointing at a DIFFERENT node
(id assignment shifts under bare-name collisions) - an entry kept across such a move
would keep enforcing the old node's label, and its old ``~/transition_event``
subscription, against the new binding. An entry's binding identity is its fqn plus
its ``GetState`` service path, both captured at first sighting, and ``update()``
re-checks that identity every tick. A moved binding is two events at once: the OLD
binding departed (recorded under ITS OWN fqn in ``recently_departed_``, same record
and retention as a vanish), and the id is new again (erased and re-seeded through the
ordinary new-node path: fresh ``GetState``, fresh subscription, fresh self-heal
budget). A straggler callback from the old subscription - the executor keeps its own
strong ref to an already-dispatched subscription, so a message the old binding
published can still arrive after the drop - matches on the captured ``GetState`` path
and cannot write the old node's label into the new binding's entry. For this
detector the consequence is that a re-bind is never enforced with the departed node's
label: the new binding starts unknown (benign) until its own label is read.

What that match discriminates is BINDINGS, not incarnations, and that is its one
permanent leg. A node that blinks out of one snapshot and returns, or is respawned
under the same name, produces a fresh entry with an identical (fqn, ``GetState`` path)
pair, so path equality passes and a straggler from the dead incarnation would land in
the fresh entry. Reaching that needs the executor to have collected the message before
the erase and to run it after the re-create - at least one full tick in flight - and
the fresh entry's own ``GetState`` re-seed corrects any non-active label it leaves
behind. A stale ``active`` is the one that sticks, because the re-seed only selects
entries cached non-active. No further mechanism is shipped for it: no seam in
``rclcpp`` forces a dispatched-but-not-executed subscription across two ``update()``
calls, so the case is not reachable from any harness we can write, and from outside the
process it is indistinguishable from a legitimate late delivery.

**Test tiers.** Three tiers each prove a different layer, deliberately not
overlapping - plus the shared-watcher seam the re-bind behaviour lives in:

1. **Unit** (``test_lifecycle_expectation_tracker.cpp``, 23 cases): pure
   ``LifecycleExpectationTracker`` logic - a required node stuck inactive past grace
   raising, keyed by the node and naming the entry as context; an active required
   node never raising; reaching active within grace resetting the streak; an absent
   node never being enforced; an unknown (``nullopt``) lifecycle state never being
   enforced; an empty label never being enforced either; two entries naming one node
   neither halving its grace nor losing a violating read to a benign duplicate; both
   namesakes being reported separately and a healthy namesake not masking a broken
   one; the absence budget - a single-snapshot blink not resetting the violation
   streak, an unread label after a streak spending the budget rather than resetting
   it, a first-contact unread label holding no bookkeeping at all, and both sustained
   absence and sustained unread discarding the streak; the pending set a streak below
   grace produces and its end once absence outlives the absence grace; the no-match
   report firing exactly once past its threshold and never for a matching entry; and
   the prune bound - a currently-reported entry is never pruned, a node absent past
   ``prune_ticks`` is reclaimed and the map shrinks, the streak goes even when the
   absence bookkeeping is reclaimed first, and identities churning under a prune
   horizon shorter than the absence grace stay bounded. The re-bind seam is shared
   infrastructure and is pinned separately in ``test_lifecycle_watcher.cpp`` (6 of its
   14 cases): a re-bind to a live node drops the old label and seeds the new binding,
   a re-bind to a dead binding reads unknown rather than the old label, a move of the
   ``GetState`` path alone is still a re-bind, an unchanged binding is never treated
   as one, the old binding is recorded as departed under its own fqn, and a re-seed
   never overwrites a fresher ``~/transition_event``.
2. **Integration** (``test_lifecycle_expectation_integration.cpp``, 45 cases): the
   detector driven against a fake ``ReportFault`` service, with a REAL
   ``ReliabilityGate`` arming the global bringup grace the aggregated
   ``graph_watchdog`` source needs and feeding the detector its labels through
   ``lifecycle_state_of()`` - injected via ``set_lifecycle_state_for_test()``
   strictly AFTER the last ``gate.update()`` call: ``LifecycleWatcher::update()``
   rebuilds its tracked set from the snapshot's app SERVICES on every call and erases
   any id it cannot find a ``GetState`` path for, and the fixture's apps carry no
   services, so any later ``gate.update()`` would silently wipe the injected label
   back to ``nullopt`` (the re-bind case is the one deliberate exception - it uses
   snapshots WITH lifecycle services, because the re-bind is expressed through
   ``gate.update()`` itself). Thirty fixture cases cover the raise/clear round trip
   naming the stuck node; the active-from-arming positive control; the
   absence-after-raise clear; bare-name and full-FQN matching against a namespaced
   app; the zero-config default measured as zero fault_manager requests of any kind,
   for both the missing key and an explicit ``require_active: []``; the
   unmanaged-entry warning firing once per configuration and again after a
   remove-and-re-add reconfigure; the no-match warning fired for an entry that never
   matched and withheld from one whose node departed; all three withheld-clear states
   with their releases (a restart stand-in emits nothing until a label is read; the
   same stand-in withholds while the node reads not-active below grace; the clear is
   withheld until a required entry has matched at least once; each of the two 60-tick
   bounds releases; an unread node that VANISHES stops blocking at the absence grace
   rather than at the prune horizon; a raise is never withheld); the guard's per-node
   counting, with two entries naming one node halving neither the configured grace nor
   the unread hold; the blink-plus-unread-re-seed sequence that must not restart the
   violation count, asserted tick-exact; the mixed set where an unread sibling holds
   the clear inside its hold and stops holding it past it; the read latch surviving a
   respawn under the same fqn, together with the streak that guards that case; the
   withhold explained once per episode and again, with a different reason, for a later
   episode; a ``grace`` past the int range rejected rather than truncated into a
   hair-trigger; 25 stuck nodes aggregating into exactly one FAILED whose description
   honours the 480-char cap and ends in the truncation marker; a required node
   appearing mid-run and then sticking inactive; and a re-bind under the same
   ``App::id`` never enforcing the old node's label. Fifteen ``configure()``-level
   cases pin the config contract: the unknown-key warning for ``require_activ`` (the
   worst-case typo - it also leaves the detector unconfigured, so the warning must
   precede the zero-config early return), a fully-valid config producing zero
   warnings, negative, non-integer and past-the-int-range ``grace``, ``prune_grace``
   out of 0..3600 rejected on the WIDE integer (never truncated into a hair-trigger
   prune) plus both range endpoints accepted, non-array ``require_active`` and
   empty-string and non-string entries each warning, the
   ``prune_ticks = max(prune_grace, grace + 1)`` clamp exercised at both endpoints
   and on both winning branches, and boundedness under identity churn when
   ``prune_grace`` undercuts the absence grace.
3. **E2e** (``test/e2e/test_lifecycle_expectation_e2e.test.py``): the acceptance
   gate - one source file, three CTest targets (the config-plumbing pattern: the
   plugin reads its config once at ``set_context()``, so different configs need
   different gateway launches), each bringing up a real gateway with the plugin
   ``.so``, a real fault_manager, and the ``managed_lifecycle`` demo node, asserting
   on the operator-visible ``GET /api/v1/faults`` surface. The main scenario proves
   raise-survives-CONFIGURE-survives-RESTART-heals-on-ACTIVATE through real
   ``lifecycle_msgs/srv/ChangeState`` transitions, including the entity-scoped
   ``/apps/graph_watchdog/faults`` surface. The restart leg is the withheld-clear
   guard at the only tier that can reach it: the gateway is SIGTERMed, its port is
   waited down, the relaunched process is gated on being armed again, and the fault
   about the still-inactive node must come back with ``last_passed`` unset - the
   fault_manager sets that field on the FIRST PASSED a fault ever receives and never
   unsets it, so it catches a single spurious clear even though the re-raise follows
   within a few ticks and a status sample taken afterwards would show CONFIRMED
   again. Nothing below this tier reaches that state: the C++ integration test
   rebuilds the detector against a fake sink, and the other two scenarios run one
   gateway process from start to finish. Its pre-raise gate is deliberately the
   GLOBAL armed state, not the target's per-entity state: ``status_json()`` reports
   a tracked node as ``armed`` only when warmup has elapsed AND ``node_ok()`` holds,
   and ``node_ok()`` is false for exactly the inactive node under test, so a
   per-entity gate on the target would wait for the fault to be impossible - what
   the raise needs is the SOURCE entity's arming (the aggregate goes out under
   ``graph_watchdog``), for which global armed is the precise precondition; the heal
   leg then DOES gate per-entity, which becomes reachable exactly when the watcher
   has read ``active``. The default-config scenario launches with no
   ``lifecycle_expectation`` config at all against the same inactive node and holds
   a sustained silence window - the only falsification of the zero-false-positive
   default against live lifecycle machinery - and the negative control does the same
   with the self-activating variant of the same executable, under the same grace and
   cadence and with ``require_active`` naming that active variant; the discriminating
   variable is the node's actual lifecycle state. Both silence scenarios gate on three
   facts before asserting absence, because absence is also what a stack that never
   came up produces: the plugin is armed; ``GET /faults`` answers 200 in THIS launch
   (a dead or unmatched fault_manager answers 503, which the poll helper swallows into
   the same ``None`` the assertion wants); and the target's label was actually READ,
   pinned through the plugin's own ``GET /x-medkit-watchdog`` route (``unconfigured``
   and ``active`` respectively) - the gate and the tracker both treat an unread label
   as benign, so silence with an unread label would be vacuous. The label pin is
   re-checked after the window as well, since a trigger that exits mid-window would
   leave most of the window measuring an empty graph.


Status
---------------
The plugin loads, ticks the graph, and shuts down cleanly. The reliability core is real
and already ticking. Four silent-fault detector classes raise through it today,
``qos_mismatch``, ``orphan``, ``param_drift`` and ``lifecycle_expectation``. The
remaining classes land in follow-up changes, each against its own issue.
