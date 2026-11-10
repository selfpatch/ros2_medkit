^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* A closed rosbag recording carries a ``manifest.json`` next to its storage files. It lists the files in the order ``metadata.yaml`` names them, each with its size and sha256, and their total size, so a recording split across several files past ``snapshots.rosbag.max_bag_size_mb`` can be fetched and checked as a whole. The manifest is written once, through a temporary file and a rename, after the writer is closed and before the recording is listed, and it is never rewritten. A recording whose manifest could not be written is still listed, without one, and recordings made before this change get none. The stored ``size_bytes`` of a recording includes the manifest
* A recording larger than ``snapshots.rosbag.max_total_storage_mb`` is no longer deleted by its own quota sweep right after it is written. The sweep evicts oldest first and stopped only under the quota, so a recording that alone exceeded it, the newest row, went the same way as everything before it and the fault kept no black box at all. The recording just stored is now never evicted: every older recording may go, it stays, and a warning says the storage is over the limit. A warning is also logged at startup when ``max_bag_size_mb`` is larger than ``max_total_storage_mb``
* A recording never opens inside an existing recording's directory. The directory name carries the millisecond the path was generated in and nothing made it unique, so a capture landing on a taken name failed to open, and its cleanup then removed the earlier recording. A taken name now moves the timestamp on by one millisecond until a free one is found, the name is reserved while the writer opens, and a failed open removes only the directory it created itself. The id format is unchanged

0.7.0 (2026-08-27)
------------------
* Rosbag black-box recordings are no longer limited to one per fault code. A fault that re-confirms keeps a bounded history of recordings instead of overwriting the previous one, controlled by the new ``snapshots.rosbag.max_bags_per_fault`` (default ``1``, which reproduces the previous behaviour exactly; ``0`` = unlimited). Retention is keep-newest and the bag is unlinked only when no fault still references it, so a burst that shares one recording behaves as before. Internally the ``rosbag_files`` grain changed from "one row per fault" to "one row per (fault, recording) link": ``recording_id`` is now a stored, indexed column, and the legacy column-level ``UNIQUE(fault_code)`` is replaced by a ``UNIQUE INDEX`` on ``(fault_code, file_path)`` through an automatic, idempotent table rebuild on first open. Four latent defects are fixed on the way: quota eviction deleted by fault code rather than by recording, ``get_rosbag_file`` had no ``ORDER BY`` and would have served an arbitrary recording, the stale-row self-heals deleted a fault's entire history because one bag had vanished from disk, and both ``delete_rosbag_file`` / ``delete_rosbag_files`` read only the first ``file_path`` of a fault, so deleting a fault with several recordings removed every row but left all but one bag on disk - unreachable and still charged against the quota (`#623 <https://github.com/selfpatch/ros2_medkit/pull/623>`_, `#620 <https://github.com/selfpatch/ros2_medkit/issues/620>`_)
* Optional append-only, hash-chained audit log of fault state transitions: each transition appends one immutable row (``record_hash = sha256(prev_hash + canonical(event))`` via OpenSSL EVP SHA-256) with a persisted chain head, a ``verify`` routine, a read API, and retention that seals a segment anchor before pruning. Time-based (PREFAILED->CONFIRMED) auto-confirmations are also audited. ``verify`` reads the chain head directly from the database, so deleting the newest row together with the head row is reported as tampering instead of silently recovering. ``BEFORE UPDATE`` / ``BEFORE DELETE`` triggers reject out-of-band edits as defense-in-depth. The chain is unkeyed and stored in a single writable file, so ``verify`` detects edits/deletions that did not recompute the chain (casual or accidental tampering); it is not a defence against an attacker who can rewrite the whole file. Off by default (`#487 <https://github.com/selfpatch/ros2_medkit/pull/487>`_, `#483 <https://github.com/selfpatch/ros2_medkit/issues/483>`_)
* **Breaking:** the default rosbag storage format is ``mcap`` again. ``snapshots.rosbag.format`` now defaults to ``"mcap"``, so black-box recordings land as ``.mcap`` files instead of ``.db3``, and the filename a bulk-data download serves changes with them. The storage plugins are declared explicitly and the plugin loader is serialised, which is what makes the format selectable reliably rather than dependent on load order. Set ``snapshots.rosbag.format: sqlite3`` to keep the previous on-disk format (`#610 <https://github.com/selfpatch/ros2_medkit/pull/610>`_)
* Freeze-frame: a compact JSON snapshot of the entity's data is persisted when a fault is confirmed, so the state at the moment of confirmation survives the fault being cleared (`#491 <https://github.com/selfpatch/ros2_medkit/pull/491>`_)
* A fault that confirms inside an active post-roll window keeps its black-box recording instead of finding the buffer already finalised (`#561 <https://github.com/selfpatch/ros2_medkit/pull/561>`_), and a fault landing on a recording-window boundary gets its own bag rather than none (`#594 <https://github.com/selfpatch/ros2_medkit/pull/594>`_)
* The debounce counter is clamped and the confirmed / healed status is latched, so a counter cannot run past its threshold and a status cannot silently regress (`#484 <https://github.com/selfpatch/ros2_medkit/pull/484>`_)
* A PASSED event no longer re-dates a fault - ``first_occurred`` keeps marking the start of the current occurrence - and genuine SSE loss is counted rather than absorbed (`#573 <https://github.com/selfpatch/ros2_medkit/pull/573>`_)
* The near-miss series survives a fault being cleared, so acknowledging a fault no longer discards the evidence gathered around it (`#629 <https://github.com/selfpatch/ros2_medkit/pull/629>`_)
* The fault manager's YAML parameter file supports launch substitutions, so a path can be composed at launch time instead of being fixed in the file (`#634 <https://github.com/selfpatch/ros2_medkit/pull/634>`_)
* Build and test only: the package is instrumented for coverage (`#582 <https://github.com/selfpatch/ros2_medkit/pull/582>`_), every launch test lives under ``test/integration`` and takes a DDS domain at run time (`#628 <https://github.com/selfpatch/ros2_medkit/pull/628>`_, `#551 <https://github.com/selfpatch/ros2_medkit/pull/551>`_, `#597 <https://github.com/selfpatch/ros2_medkit/pull/597>`_)
* A ``fault_code`` up to the advertised 256 characters is accepted, where the manager previously stopped at 128, and a long code no longer costs the recording: the rosbag filename is truncated with a digest instead of exceeding the filesystem's component limit (`#591 <https://github.com/selfpatch/ros2_medkit/pull/591>`_)
* Contributors: @bburda, @mfaferek93, @nnarain

0.6.0 (2026-06-22)
------------------
* Bounded concurrent snapshot capture under fault storms with a ``CaptureThreadPool`` and configurable capture pool / queue / overflow-policy parameters. The rosbag leg is serialized and the cooldown map is bounded, so a burst of simultaneous faults can no longer exhaust capture threads or grow memory without limit (`#456 <https://github.com/selfpatch/ros2_medkit/pull/456>`_)
* Entity-scoped rosbag capture by default (`#431 <https://github.com/selfpatch/ros2_medkit/pull/431>`_)
* Made rosbag capture enablement crash-safe (`#430 <https://github.com/selfpatch/ros2_medkit/pull/430>`_)
* The default rosbag storage format moved back from ``mcap`` to ``sqlite3``, which ships with rosbag2 and needs no extra package; ``mcap`` stayed selectable through ``snapshots.rosbag.format``. This came in with the crash-safety fix above and was not recorded at the time (`#430 <https://github.com/selfpatch/ros2_medkit/pull/430>`_)
* Contributors: @bburda, @mfaferek93

0.5.0 (2026-06-08)
------------------
* ``ClearFault`` honors the new ``skip_correlation_auto_clear`` request flag so per-entity fault clears can opt out of cascade-clearing correlated symptom fault codes (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* Three-layer protection against unbounded snapshot growth (bounded buffers plus pruning)
* Concurrency and lifetime hardening: serialize concurrent subscription creation in ``SnapshotCapture``, join capture threads in the ``FaultManagerNode`` destructor, and defense-in-depth shutdown guards to prevent teardown crashes across distros
* Aggregation security hardening and improved test coverage
* Build: adopt the centralized ``ROS2MedkitWarnings`` and ``ROS2MedkitSanitizers`` cmake modules and ``bugprone`` / ``special-member-functions`` clang-tidy checks
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Per-entity confirmation and healing thresholds via manifest configuration (`#269 <https://github.com/selfpatch/ros2_medkit/pull/269>`_)
* Default rosbag storage format changed from ``sqlite3`` to ``mcap``
* Support for namespaced fault manager nodes - gateway resolves service/topic names when the fault manager runs in a custom namespace
* Build: use shared cmake modules from ``ros2_medkit_cmake`` package
* Build: centralized clang-tidy configuration
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------
* Accurate HIGHEST_SEVERITY reassignment and stale ``fault_to_cluster_`` cleanup (`#221 <https://github.com/selfpatch/ros2_medkit/pull/221>`_)
* Clean up ``pending_clusters_`` when fault cleared before ``min_count`` (`#211 <https://github.com/selfpatch/ros2_medkit/pull/211>`_)
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda, @eclipse0922

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* Central fault management node with ROS 2 services:

  * ReportFault - report FAILED/PASSED events with debounce filtering
  * GetFaults - query faults with filtering by severity, status, correlation
  * ClearFault - clear/acknowledge faults

* Debounce filtering with configurable thresholds:

  * FAILED events decrement counter, PASSED events increment
  * Configurable confirmation_threshold (default: -1, immediate)
  * Optional healing support (healing_enabled, healing_threshold)
  * Time-based auto-confirmation (auto_confirm_after_sec)
  * CRITICAL severity bypasses debounce

* Dual storage backends:

  * SQLite persistent storage with WAL mode (default)
  * In-memory storage for testing/lightweight deployments

* Snapshot capture on fault confirmation:

  * Topic data captured as JSON with configurable topic resolution
  * Priority: fault_specific > patterns > default_topics
  * Stored in SQLite with indexed fault_code lookup
  * Auto-cleanup on fault clear

* Rosbag capture with ring buffer:

  * Configurable duration, post-fault recording, topic selection
  * Lazy start mode (start on PREFAILED) or immediate
  * Auto-cleanup of bag files, storage limits (max_bag_size_mb)
  * GetRosbag service for bag file metadata

* Fault correlation engine:

  * Hierarchical mode: root cause to symptom relationships
  * Auto-cluster mode: group similar faults within time window
  * YAML-based configuration with pattern wildcards
  * Muted faults tracking, auto-clear on root cause resolution

* FaultEvent publishing on ~/events topic for SSE streaming
* Wall clock timestamps (compatible with use_sim_time)
* Contributors: Bartosz Burda, Michal Faferek
