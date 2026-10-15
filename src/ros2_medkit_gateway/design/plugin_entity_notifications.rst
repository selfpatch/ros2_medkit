Plugin-Driven Entity Surface Notifications (Plugin API v7)
===========================================================

.. contents:: On this page
   :local:

Motivation
----------

A plugin that implements :cpp:class:`UpdateProvider` (or any other plugin
that mutates the entity surface at runtime) has two mechanisms available
in v6:

* *declare* the entity in the base manifest file (static, known at
  gateway startup).
* *rely on runtime discovery* to notice a new ROS 2 node appearing on
  the graph.

Neither covers the OTA install case. The installer drops a new app on
disk, starts it, and wants the operator's client (Web UI, MCP tool,
Foxglove panel) to see the change immediately. Runtime discovery
catches the new node within a few seconds but shows it as
``source: orphan`` - it is not attached to any component so the entity
tree looks inconsistent. On plugin rollback the opposite problem
appears: the manifest entry (if it existed) lingers even though the
app is gone.

v7 adds two pieces that together close the gap:

#. ``PluginContext::notify_entities_changed(EntityChangeScope)`` - a
   plugin-driven refresh trigger.
#. ``discovery.manifest.fragments_dir`` - a directory of drop-in
   manifest yaml chunks that the gateway scans on every manifest load
   / reload.

Neither is OTA-specific. The same pair is usable by rosbag injectors,
dynamic config deployers, hot-reloadable adapters - any plugin whose
correct behaviour depends on adding or removing entities at runtime.

Lifecycle
---------

.. code-block:: text

   (1) Plugin installs new app on disk
   (2) Plugin writes /path/to/fragments/<deploy-id>.yaml declaring
       the app (is_located_on, ros_binding, ...)
   (3) Plugin starts the app process
   (4) Plugin calls
         ctx.notify_entities_changed(
             EntityChangeScope::for_component("my-ecu"));
   (5) Gateway re-parses base manifest and re-scans fragments_dir,
       rebuilds the entity cache, runs a full discovery cycle
   (6) Client's next GET /apps / GET /components/{id}/apps reflects
       the new app

Rollback reverses the sequence:

.. code-block:: text

   (7)  Plugin stops the app process
   (8)  Plugin deletes /path/to/fragments/<deploy-id>.yaml
   (9)  Plugin calls
          ctx.notify_entities_changed(
              EntityChangeScope::for_component("my-ecu"));
   (10) Gateway re-loads, app is no longer in the merged manifest,
        cache drops it, clients refresh to find it gone

The scope hint in steps (4) / (9) is informational in v7: the gateway
always performs a full ``refresh_cache()``. A future optimisation may
restrict the pass to the named area or component subtree; the plugin
API does not change when that lands.

Fragment merge rules
--------------------

Files in ``discovery.manifest.fragments_dir`` are parsed with
``ManifestParser::parse_fragment_file`` and merged on top of the
loaded base manifest before validation runs.

*Allowed* in a fragment:

* ``apps`` - appended to ``Manifest::apps``
* ``components`` - appended to ``Manifest::components``
* ``functions`` - appended to ``Manifest::functions``

*Forbidden* in a fragment (owned by the base manifest):

* ``areas``
* ``metadata`` (any field - ``name``, ``description``, ``version``,
  ``created_at``)
* ``discovery``
* ``scripts``
* ``capabilities`` (vendor extensions)
* ``lock_overrides``

A fragment that declares any forbidden top-level field fails the load
with a ``FRAGMENT_FORBIDDEN_FIELD`` validation error. Each forbidden
field in a fragment is reported separately so a single load reports
every violation, not just the first. ``manifest_version`` is optional
in fragments - when omitted a synthetic ``"1.0"`` is injected before
parsing.

File ordering is deterministic: fragment files in the directory are
sorted by full path before being merged. Duplicate IDs across the
combined manifest (base + every fragment) are caught by the normal
validator run and cause the load to fail with the same error the user
would see from a single file, plus the offending fragment path in the
error message.

A missing fragments directory is *not* an error - the plugin can
create the directory lazily on first install.

All-or-nothing fragment contract
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Fragment loading is intentionally atomic: if ANY fragment in
``fragments_dir`` fails to parse, exceeds the size limit, declares a
forbidden top-level field, or causes the merged manifest to fail
validation, the entire ``load_manifest`` / ``reload_manifest`` call
returns failure. The previously-loaded manifest stays active until the
next successful reload. One broken fragment therefore blocks every
fragment in that directory (including valid ones) from taking effect.

The contract is deliberate - a partially-applied fragment set is worse
than the pre-existing manifest, because it produces a client-visible
entity tree that neither matches the old deployment nor the intended
new one. Plugins that want independent failure domains should use
separate fragment directories per domain.

Plugin write contract (TOCTOU and size safety)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To keep the all-or-nothing semantics from being tripped by a racing
reader, plugins that write into ``fragments_dir`` MUST publish each
fragment atomically:

#. write the final content to ``fragments_dir/.tmp-<id>.yaml`` and
   ``fsync`` it;
#. ``rename()`` to ``fragments_dir/<id>.yaml`` (POSIX ``rename`` is
   atomic within one filesystem).

The gateway scans the directory on every reload and calls
``notify_entities_changed`` runs the reload synchronously on the caller's
thread, so a partial write observed between ``open()`` and ``close()``
can fail the entire merge if the plugin publishes in-place.

Two other reader-side safeguards protect the gateway from malformed
inputs:

* **Size cap.** Each fragment file is rejected before it is read into
  memory if it exceeds
  ``ManifestParser::kMaxFragmentBytes`` (1 MiB). This protects against
  a misconfigured ``fragments_dir`` pointing at a log or data file.
* **Symlink scoping.** Symlinks ARE followed (k8s ConfigMap mounts
  rely on that), but any directory entry whose resolved real path is
  not a descendant of the canonical ``fragments_dir`` is skipped with
  a warning. This stops a ``evil.yaml -> /etc/shadow`` escape while
  keeping ConfigMap use cases working.

Compatibility
-------------

* ``PLUGIN_API_VERSION`` bumped from v6 to v7.
* ``PluginContext::notify_entities_changed`` has a default no-op
  implementation, so plugin **source code** written against v6
  compiles unchanged against v7 headers. No code changes are needed.
* Binary compatibility is NOT provided: the plugin loader compares
  the exported ``plugin_api_version()`` against the gateway's
  ``PLUGIN_API_VERSION`` with strict equality, so a ``.so``
  pre-compiled against v6 IS rejected. In-tree plugins pick up the
  bump automatically because they ``return PLUGIN_API_VERSION`` from
  the shared header; out-of-tree plugins must be recompiled.
* A v7 gateway loading a recompiled v6 plugin is fully functional;
  the new lifecycle hooks are off by default until the plugin opts
  in by calling ``notify_entities_changed``.

Related APIs
------------

* :cpp:class:`ResourceChangeNotifier` covers *resource-item* changes
  (faults appearing, data values updating, configuration mutations).
  It is a push-notification hub for individual items; it does not
  trigger discovery rework. Use it for streaming updates to subscribers
  via SSE / triggers.
* ``notify_entities_changed`` covers *entity-surface* changes - the
  structural tree that hosts those resource items. Use it when the
  set of apps / components / functions itself changes.

The two are intentionally separate. Plugins that mutate the surface AND
stream new resource-item events should call both: one for each concern.
