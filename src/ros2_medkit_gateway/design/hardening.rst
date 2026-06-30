Gateway hardening (secure field profile)
========================================

The gateway ships every transport and access control needed for a hardened
deployment - JWT authentication with RBAC, TLS/HTTPS, restricted CORS, and
token-bucket rate limiting - but they are **disabled by default** so local
development works out of the box. A gateway exposed on a plant network with the
defaults is wide open: unauthenticated reads and writes over cleartext HTTP.

For any deployment reachable from an untrusted network, start from the secure
field profile preset ``config/gateway_params.secure.yaml`` instead of
``config/gateway_params.yaml``:

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node \
     --ros-args --params-file gateway_params.secure.yaml \
     -p auth.jwt_secret:="$MEDKIT_JWT_SECRET" \
     -p 'auth.clients:=["operator:'"$OP_SECRET"':operator"]'

What the secure profile turns on
--------------------------------

================================ ============== ===========================================
Control                          Default        Secure profile
================================ ============== ===========================================
``auth.enabled``                 false          true
``auth.require_auth_for``        write          all (auth on reads + writes)
``server.tls.enabled``           false          true (HTTPS, min TLS 1.3)
``cors.allowed_origins``         ``[""]``       explicit origin list (no wildcard)
``rate_limiting.enabled``        false          true (global + per-client + per-endpoint)
``scripts.allow_uploads``        true           false (manifest-defined scripts only)
``docs.enabled``                 true           false (reduced surface)
``bulk_data.max_upload_size``    100 MiB        25 MiB
``locking`` on operations        none           lock required before mutation
================================ ============== ===========================================

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

3. **Role-scoped clients.** Create the minimum set of clients in
   ``auth.clients`` (``client_id:client_secret:role``). Roles, least to most
   privileged: ``viewer`` (read), ``operator`` (+ trigger ops / ack faults /
   publish), ``configurator`` (+ modify configs), ``admin`` (+ auth
   management). Rotate secrets periodically.

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
  clients).
- [ ] ``cors.allowed_origins`` is an explicit list (no ``*``); ``*`` is never
  combined with ``allow_credentials: true``.
- [ ] ``rate_limiting.enabled: true`` with per-client and mutating-endpoint
  limits tuned to the deployment.
- [ ] ``scripts.allow_uploads: false`` unless remote script upload is a
  required, reviewed capability.
- [ ] ``bulk_data.max_upload_size`` bounded to what the deployment needs.
- [ ] If peer aggregation is used: ``aggregation.require_tls: true`` and
  ``forward_auth`` only enabled when every peer is trusted.
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
