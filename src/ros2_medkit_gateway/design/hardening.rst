Gateway hardening (secure field profile)
========================================

The gateway ships every transport and access control needed for a hardened
deployment - JWT authentication with RBAC, TLS/HTTPS, restricted CORS, and
token-bucket rate limiting - but they are **disabled by default** so local
development works out of the box. A gateway exposed on a plant network with the
defaults is wide open: unauthenticated reads and writes over cleartext HTTP.

Protected mode limits that exposure: a gateway without authentication binds only a
loopback address. With authentication off and a ``server.host`` that is not a
loopback address, the gateway exits with status 1 and a message that names the
address. The check runs after the environment is applied, so ``MEDKIT_JWT_SECRET``
counts as authentication on. The opt-out is ``server.allow_unauthenticated_remote:
true`` (launch argument ``allow_unauthenticated_remote``), which ``MEDKIT_AUTH_DISABLED=1``
implies; the gateway then warns at start that it is reachable without authentication.

For any deployment reachable from an untrusted network, start from the secure
field profile preset ``config/gateway_params.secure.yaml`` instead of
``config/gateway_params.yaml``.

Supply the secrets in the environment and start the gateway with the file:

.. code-block:: bash

   export MEDKIT_JWT_SECRET="$(openssl rand -base64 32)"
   export MEDKIT_CLIENTS="operator:${OP_SECRET}:operator,viewer:${VIEW_SECRET}:viewer"
   ros2 run ros2_medkit_gateway gateway_node \
     --ros-args --params-file gateway_params.secure.yaml

The file names ``/etc/ros2_medkit/certs/cert.pem`` and ``key.pem``. The gateway
does not start without a readable pair there, and it does not start without
``MEDKIT_JWT_SECRET`` (the file leaves ``auth.jwt_secret`` empty). The file serves
HTTPS on port 8443 with TLS 1.3 as the floor, answers ``401`` to every request
without a token except ``/api/v1/auth/*``, where clients log in (``/api/v1/health``
included; see ``auth.public_routes`` below), and allows browser origin
``https://medkit-ui.local`` only.

The Docker image takes the same file as its params layer, the certificates at the
paths the file names, and the secrets as ``-e`` variables; see
:doc:`/tutorials/docker`:

.. code-block:: bash

   IMAGE=ghcr.io/selfpatch/ros2_medkit-jazzy:latest
   docker create --name medkit -p 8443:8443 \
     -e MEDKIT_JWT_SECRET -e MEDKIT_CLIENTS "$IMAGE"
   docker cp gateway_params.secure.yaml medkit:/etc/ros2_medkit/params.yaml
   tar --owner="$(docker run --rm --entrypoint id "$IMAGE" -u)" \
     --group="$(docker run --rm --entrypoint id "$IMAGE" -g)" \
     -cf - certs/cert.pem certs/key.pem | docker cp - medkit:/etc/ros2_medkit
   docker start medkit

Ship only ``cert.pem`` and ``key.pem``, plus ``ca.pem`` when mutual TLS is on: never
``ca_key.pem`` or ``client-key.pem``. The gateway runs as user ``medkit`` in the image.
``docker cp`` of a plain directory gives the files the owner they have on the host, so the
key stays unreadable to the gateway; a tar stream keeps the owner it names. The values of
``-e`` variables are visible to anyone who can use the Docker API on the host
(``docker inspect``). A liveness probe needs a token, or an entry in
``auth.public_routes``.

What the secure profile turns on
--------------------------------

================================ ============== ===========================================
Control                          Default        Secure profile
================================ ============== ===========================================
``auth.enabled``                 false          true
``auth.require_auth_for``        write          all (auth on reads + writes)
``server.tls.enabled``           false          true (HTTPS, min TLS 1.3)
``cors.allowed_origins``         ``[]``         explicit origin list (no wildcard)
``rate_limiting.enabled``        false          true (global + per-client + per-endpoint)
``scripts.allow_uploads``        true           false (manifest-defined scripts only)
``docs.enabled``                 true           false (reduced surface)
``bulk_data.max_upload_size``    100 MiB        25 MiB
``locking`` on operations        none           lock required for component and app operations
================================ ============== ===========================================

``rate_limiting`` is 600 requests per minute for the gateway, 120 per client and 30 per
client on the path ``/api/v1/*/*/operations/*/executions``, outside the 120. That path is the
start (``POST``) and the list (``GET``) of executions; the limiter does not see the method.
Status polls and cancels (``.../executions/{id}``) stay under the 120. ``*`` is one path
segment; an operation id with ``/`` is held only by the per-client limit.
``auth.public_routes`` is empty. To open one route,
for example a liveness probe, write ``public_routes: ["GET /api/v1/health"]``; a caller
without a token then gets the reduced health body.

Credential and certificate provisioning
----------------------------------------

1. **TLS certificate.** Provision a real server certificate + private key and
   point ``server.tls.cert_file`` / ``server.tls.key_file`` at them. The key
   file must be ``chmod 600`` and owned by the gateway service user. For a
   dev/test box only, ``scripts/generate_dev_certs.sh`` emits a self-signed
   ``cert.pem`` / ``key.pem`` / ``ca.pem`` (never use these in production).

2. **JWT secret.** Generate a high-entropy secret of at least 32 characters
   (HS256) or provision an RS256 key pair. Inject it at deploy time from a
   secret store or environment variable - do not commit it to source control.

   The gateway node reads ``MEDKIT_JWT_SECRET``, ``MEDKIT_CLIENTS`` and
   ``MEDKIT_AUTH_DISABLED`` itself, so the same rule holds on every way of
   starting it, and the environment wins over the params file. A non-empty
   ``MEDKIT_JWT_SECRET`` closes the gateway (``require_auth_for`` ``all``); set
   but empty, the gateway refuses to start.

   .. note::

      The gateway does not publish the secret as a parameter value. It declares
      ``auth.jwt_secret`` with a placeholder that names the source
      (``<from MEDKIT_JWT_SECRET>`` or ``<set at start>``), so ``ros2 param get``
      and ``/parameter_events`` do not carry it. ``auth.clients`` and
      ``aggregation.peer_auth_header`` get the same treatment, and none of them
      can be changed at runtime. Plugin config (``plugins.<name>.<key>``) is read
      from the node's parameter overrides and handed to the plugin; it is never
      declared as a parameter, so it is not served by the parameter services and
      does not appear on ``/parameter_events``.

   .. warning::

      Do not pass the secret as ``-p auth.jwt_secret:=...``: it appears in
      ``ps`` and ``/proc/<pid>/cmdline``. Use ``MEDKIT_JWT_SECRET``. A secret
      written into a params file sits there in clear text; the file must be
      readable by the gateway's user only.

3. **Role-scoped clients.** Create the minimum set of clients in
   ``MEDKIT_CLIENTS`` or ``auth.clients`` (``client_id:client_secret:role``,
   comma-separated in the variable, which replaces ``auth.clients`` when
   ``MEDKIT_JWT_SECRET`` is set). Roles, least to most
   privileged: ``viewer`` (read), ``operator`` (+ trigger ops / ack faults /
   publish), ``configurator`` (+ modify configs), ``admin`` (+ auth
   management). Rotate secrets periodically. The role a token grants is the
   one this list gives its ``sub``, also for a token another gateway minted.

4. **Obtain a token** and call the API over HTTPS:

   .. code-block:: bash

      curl -sk -X POST https://gateway:8443/api/v1/auth/authorize \
        -H 'Content-Type: application/json' \
        -d '{"client_id":"operator","client_secret":"...","grant_type":"client_credentials"}'
      # use the returned access_token:
      curl -sk https://gateway:8443/api/v1/faults -H "Authorization: Bearer $TOKEN"

Hardening checklist
-------------------

Before exposing a gateway on a shared / plant network, confirm:

- [ ] ``auth.enabled: true`` and ``auth.require_auth_for`` is ``all`` (or
  ``write`` only if unauthenticated reads are explicitly acceptable).
- [ ] ``auth.jwt_secret`` is set to a >= 32-char secret injected from a secret
  store (not the placeholder, not in version control).
- [ ] ``auth.clients`` lists only the role-scoped clients you need; default /
  example credentials removed; secrets rotated.
- [ ] ``server.tls.enabled: true`` with a real certificate; private key is
  ``chmod 600``; ``min_version`` is ``1.3`` (or ``1.2`` only for legacy
  clients). The gateway enforces it as the protocol floor. Set
  ``server.tls.require_client_cert: true`` with ``server.tls.ca_file`` to
  require client certificates.
- [ ] ``cors.allowed_origins`` is an explicit list (no ``*``); ``*`` is never
  combined with ``allow_credentials: true``.
- [ ] ``rate_limiting.enabled: true`` with per-client and mutating-endpoint
  limits tuned to the deployment.
- [ ] ``scripts.allow_uploads: false`` unless remote script upload is a
  required, reviewed capability.
- [ ] ``bulk_data.max_upload_size`` bounded to what the deployment needs.
- [ ] If peer aggregation is used: ``aggregation.require_tls: true`` and
  ``forward_auth`` only enabled when every peer is trusted. Revocation is per
  gateway, and ``forward_auth`` forwards the access token: revoke the refresh
  token it was minted from on every gateway, or disable the client.
- [ ] Bind ``server.host`` to a management interface where the network layout
  allows, and place the gateway behind the plant firewall / segmentation.
- [ ] Back the gateway with persistent storage on a volume with restricted
  permissions (faults DB, triggers DB, rosbag snapshots).

OPC-UA plugin (southbound) hardening
------------------------------------

The gateway controls the northbound REST surface; the OPC-UA plugin controls
the southbound connection to the PLC. Harden both. The plugin supports
SecurityPolicy (Basic256Sha256 / Aes128 / Aes256), MessageSecurityMode
(Sign / SignAndEncrypt), a client application-instance certificate, a server
trust store with reject-untrusted, and user identity (anonymous /
username-password / X.509). See ``ros2_medkit_opcua`` README, section
"OPC-UA client security".
