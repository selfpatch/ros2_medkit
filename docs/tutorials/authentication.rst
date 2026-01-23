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

.. list-table::
   :widths: 20 15 15 20 15 15
   :header-rows: 1

   * - Role
     - Read (GET)
     - Data (PUT)
     - Operations (POST)
     - Config (PUT/DEL)
     - Faults (DEL)
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

Basic Setup
-----------

1. **Create a configuration file** (``auth_params.yaml``):

   .. code-block:: yaml

      ros2_medkit_gateway:
        ros__parameters:
          auth:
            enabled: true
            jwt_secret: "your-secret-key-at-least-32-characters-long"
            jwt_algorithm: "HS256"
            token_expiry_seconds: 3600
            refresh_token_expiry_seconds: 86400
            require_auth_for: "write"
            issuer: "ros2_medkit_gateway"
            clients:
              - "admin:admin_secret_key:admin"
              - "operator:operator_secret:operator"
              - "viewer:viewer_pass:viewer"

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
       "client_secret": "admin_secret_key"
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
