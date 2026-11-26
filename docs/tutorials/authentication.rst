Configuring Authentication
==========================

This tutorial shows how to enable JWT-based authentication with
Role-Based Access Control (RBAC) in ros2_medkit_gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

By default, the gateway runs without authentication for easy development.
For production deployments, you should enable authentication to:

- Control who can access the API
- Limit write operations to authorized users
- Audit API access

Environment Variables
---------------------

The gateway node reads three environment variables when it loads its
parameters, so the rule is the same however it is started (``ros2 run``, any
launch file, a container). The environment wins over the params file and over
``-p`` arguments.
A wrongly typed ``auth.*`` value in a params file still stops the gateway at
start, even when an environment variable would replace it.

.. list-table::
   :widths: 28 72
   :header-rows: 1

   * - Variable
     - Effect
   * - ``MEDKIT_JWT_SECRET``
     - A non-empty value turns authentication on, sets ``require_auth_for`` to
       ``all`` and is the signing secret. Set but empty, the gateway refuses to
       start (exit code 1): a blank value is usually a failed interpolation, and
       starting open under it is the outcome the variable exists to prevent.
       With ``auth.jwt_algorithm`` ``RS256`` it is refused at startup, because
       the variable carries a shared secret and RS256 wants a key file path.
   * - ``MEDKIT_CLIENTS``
     - Replaces ``auth.clients``. Comma-separated ``id:secret:role`` entries;
       surrounding spaces and tabs are trimmed. The id ends at the first colon
       and the role starts after the last, so a secret may contain colons. A
       repeated id keeps the first entry and logs a warning. A malformed entry
       is dropped and reported by its position, never by its text. An empty
       value means no clients and logs a warning; a value of which every entry
       was refused stops the gateway. Ignored with a warning unless
       ``MEDKIT_JWT_SECRET`` is set and non-empty.
   * - ``MEDKIT_AUTH_DISABLED``
     - Exactly ``1`` turns authentication off, over the params file and over
       ``MEDKIT_JWT_SECRET``. It also opts out of protected mode (below), so a
       container can be opened with this one variable. Any other value has no
       effect.

``ros2 param get`` reports what the gateway enforces: ``auth.enabled`` and
``auth.require_auth_for`` hold the effective values (``none`` while
authentication is off). No secret is readable: ``auth.jwt_secret`` reads
``<from MEDKIT_JWT_SECRET>``, ``<set at start>`` or empty, each
``auth.clients`` entry reads ``<id>:<sentinel>:<role>``, and
``aggregation.peer_auth_header`` reads ``<set at start>`` or empty. With
authentication on, a client secret equal to one of these placeholders stops the
gateway, so a ``ros2 param dump`` reused as a params file never registers a
known secret; with it off no client is registered. The
auth configuration is built once at start, so ``ros2 param set``, ``ros2 param load``
and atomic batches touching ``auth.*`` or ``aggregation.peer_auth_header`` are
refused; restart the gateway with the new configuration.

Plugin config (``plugins.<name>.<key>``) is read at start and is not served as
parameters, so a plugin password never reaches ``ros2 param get`` or
``/parameter_events``.

Where a secret is safe:

- The environment variable (``MEDKIT_JWT_SECRET``, ``MEDKIT_CLIENTS``) keeps the
  secret out of params files and out of the process list. The same user and
  root can still read it from ``/proc/<pid>/environ``, and child processes
  inherit it.
- A params file that holds a secret must be readable only by the gateway's user.
- A value given with ``-p`` on the command line is visible to every local user
  in the process list.

Protected Mode
--------------

A gateway that answers every request anonymously (authentication off, or on with
``auth.require_auth_for`` ``none``) serves only the local machine. With such a
``server.host`` that is not a loopback address (``0.0.0.0``, ``::``, a
concrete address or a host name other than ``localhost``), the gateway exits
with status 1 and a message that names the address. ``localhost`` counts as
loopback by its name, as RFC 6761 reserves it; the gateway binds what the
resolver returns for it. The decision uses the authentication in force after the
environment is applied: ``MEDKIT_JWT_SECRET`` counts as on (and forces
``require_auth_for`` ``all``), whatever ``auth.enabled`` says. ``write`` and
``all`` count as authentication, in any letter case; ``write`` binds the
network and leaves reads anonymous, ``none`` leaves everything open and does
not count. The ways out are

- turn authentication on (``MEDKIT_JWT_SECRET`` and ``MEDKIT_CLIENTS``, or the
  ``auth.*`` parameters with ``require_auth_for`` ``write`` or ``all``);
- bind a loopback address (``server.host: 127.0.0.1``);
- opt out with ``server.allow_unauthenticated_remote: true``, the launch
  argument ``allow_unauthenticated_remote:=true``, or ``MEDKIT_AUTH_DISABLED=1``.
  The gateway then logs a warning that it is reachable without authentication.
  A launch argument that is set wins over the config file, ``false`` included: with
  ``allow_unauthenticated_remote:=false`` a file that says ``true`` no longer opens it.

A gateway that cannot bind its address (the address:port is held by a live
process, an address the machine does not own, a name that does not resolve) also
exits with status 1, naming the host, the port and the reason. An address the
machine does not own is refused unless the kernel allows non-local binds
(``net.ipv4.ip_nonlocal_bind``, ``net.ipv6.ip_nonlocal_bind``).

See :ref:`protected-mode` for the parameter.

Authentication Modes
--------------------

The gateway supports three authentication modes via the ``require_auth_for`` parameter:

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Mode
     - Description
   * - ``none``
     - No authentication required. Auth endpoints still available for token management.
   * - ``write``
     - Authentication required for POST, PUT, DELETE operations. GET is open. **(Recommended for development)**
   * - ``all``
     - All endpoints require authentication. **(Recommended for production)**

Roles and Permissions
---------------------

Roles are cumulative: each one may do everything the row above it may do, plus
its own column. There is no inheritance in the stored table - the gateway
expands each route's declared role upward when it builds the table - but the
effect for a caller is the ladder below.

.. list-table::
   :widths: 18 14 14 14 20 20
   :header-rows: 1

   * - Role
     - Read (GET)
     - Data (PUT)
     - Operations, faults, locks, subscriptions, triggers, bulk-data,
       script runs, start/restart
     - Configurations, log configuration, script upload/delete, updates,
       shutdown
     - Plugin-served routes
   * - ``viewer``
     - ✅
     - ❌
     - ❌
     - ❌
     - ❌
   * - ``operator``
     - ✅
     - ✅
     - ✅
     - ❌
     - ❌
   * - ``configurator``
     - ✅
     - ✅
     - ✅
     - ✅
     - ❌
   * - ``admin``
     - ✅
     - ✅
     - ✅
     - ✅
     - ✅

Clearing faults is ``operator``, not ``admin`` - it is a runtime action, not a
change to how the system is configured. Tearing an entity down
(``PUT /{entity}/status/shutdown`` and ``force-shutdown``) is ``configurator``,
while bringing one up or restarting it is ``operator``.

Where the table comes from
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each route declares its own weakest caller at the point it is registered, and
that single declaration produces both halves of the contract:

* the permission entries ``AuthManager::check_authorization`` matches against,
  and
* the ``security`` requirement the served OpenAPI document publishes for that
  operation - ``GET /api/v1/docs`` names the role each endpoint needs.

Enforcement fails closed: a path no entry matches is refused, so an endpoint's
published role is the role the gateway actually demands rather than a separate
claim about it.

Two consequences worth knowing:

* **Plugin-served routes are ``admin``-only.** A plugin mounts its routes
  directly on the HTTP server, outside the route registry, so no per-route
  declaration describes them. They are covered only by the admin wildcards, and
  the document publishes ``admin`` for them.
* **The document reflects this deployment, not the product.** With
  ``auth.enabled`` false the gateway serves every endpoint unauthenticated and
  the document publishes no roles at all. With it true, ``require_auth_for``
  still decides how much is checked - under ``write`` a GET is served without a
  token even though its operation names the role the table would grant.

Basic Setup
-----------

1. **Create a configuration file** (``auth_params.yaml``):

   .. code-block:: yaml

      ros2_medkit_gateway:
        ros__parameters:
          auth:
            enabled: true
            jwt_secret: "CHANGE-ME-use-openssl-rand-base64-32"
            jwt_algorithm: "HS256"
            token_expiry_seconds: 3600
            refresh_token_expiry_seconds: 86400
            require_auth_for: "write"
            issuer: "ros2_medkit_gateway"
            clients:
              - "admin:REPLACE_WITH_STRONG_SECRET:admin"
              - "operator:REPLACE_WITH_STRONG_SECRET:operator"
              - "viewer:REPLACE_WITH_STRONG_SECRET:viewer"

   .. danger::

      **Never use example credentials in production.** Generate strong secrets with
      ``openssl rand -base64 32`` and keep them out of version control.

2. **Launch with authentication:**

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py \
        extra_params_file:=auth_params.yaml

Using Authentication
--------------------

**Step 1: Get an access token**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/auth/authorize \
     -H "Content-Type: application/json" \
     -d '{
       "grant_type": "client_credentials",
       "client_id": "admin",
       "client_secret": "YOUR_ADMIN_SECRET"
     }'

Response:

.. code-block:: json

   {
     "access_token": "eyJhbGciOiJIUzI1NiIs...",
     "token_type": "Bearer",
     "expires_in": 3600,
     "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
     "scope": "admin"
   }

**Step 2: Use the token**

.. code-block:: bash

   TOKEN="eyJhbGciOiJIUzI1NiIs..."

   # Protected endpoint (POST requires auth in "write" mode)
   curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
     -H "Authorization: Bearer $TOKEN" \
     -H "Content-Type: application/json" \
     -d '{}'

**Step 3: Refresh the token**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/auth/token \
     -H "Content-Type: application/json" \
     -d '{
       "grant_type": "refresh_token",
       "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g..."
     }'

**Step 4: Revoke a token**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/auth/revoke \
     -H "Content-Type: application/json" \
     -d '{"token": "dGhpcyBpcyBhIHJlZnJlc2g..."}'

Production Recommendations
--------------------------

1. **Use a strong, random secret:**

   .. code-block:: bash

      # Generate a secure secret
      openssl rand -base64 32

2. **Store secrets securely:**

   - Use environment variables or secret management systems
   - Never commit secrets to version control

3. **Use short token expiry:**

   - Access tokens: 15-60 minutes
   - Refresh tokens: 8-24 hours

4. **Consider RS256 for distributed systems:**

   RS256 uses asymmetric keys, allowing verification without sharing the signing key.

   .. code-block:: yaml

      auth:
        enabled: true
        jwt_algorithm: "RS256"
        jwt_secret: "/path/to/private_key.pem"
        jwt_public_key: "/path/to/public_key.pem"

5. **Always use HTTPS in production:**

   See :doc:`https` for TLS configuration.

Troubleshooting
---------------

**"Invalid token" error**

- Check that the token hasn't expired
- Verify the ``Authorization`` header format: ``Bearer <token>``
- Ensure the JWT secret matches between token generation and verification

**"Insufficient permissions" error**

- Check that your client has the required role
- Verify the operation matches the role's permissions

**Token refresh fails**

- Refresh tokens are single-use; get a new one after refresh
- Check that the refresh token hasn't been revoked

See Also
--------

- :doc:`https` - Enable TLS/HTTPS
- `JWT.io <https://jwt.io/>`_ - Debug and verify JWT tokens
