Configuring HTTPS/TLS
=====================

This tutorial shows how to enable TLS (Transport Layer Security) for
encrypted HTTPS communication with the gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

By default, the gateway uses plain HTTP. For production deployments,
you should enable TLS to:

- Encrypt all traffic between clients and the gateway
- Prevent eavesdropping on sensitive data
- Authenticate the server to clients

Quick Start (Development)
-------------------------

For development and testing, use the provided launch file that
auto-generates self-signed certificates:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway_https.launch.py

The gateway will start on ``https://localhost:8443``.

Test with curl (skip certificate verification for self-signed):

.. code-block:: bash

   curl -k https://localhost:8443/api/v1/health

Generating Development Certificates
-----------------------------------

The package includes a helper script:

.. code-block:: bash

   cd ~/ros2_medkit_ws/src/ros2_medkit/src/ros2_medkit_gateway
   ./scripts/generate_dev_certs.sh ./certs

This creates:

- ``ca.crt`` - Certificate Authority certificate
- ``server.crt`` - Server certificate
- ``server.key`` - Server private key
- ``client.crt`` - Client certificate (for mutual TLS)
- ``client.key`` - Client private key

Manual Configuration
--------------------

1. **Create a configuration file** (``tls_params.yaml``):

   .. code-block:: yaml

      ros2_medkit_gateway:
        ros__parameters:
          server:
            host: "0.0.0.0"
            port: 8443
            tls:
              enabled: true
              cert_file: "/path/to/server.crt"
              key_file: "/path/to/server.key"
              min_version: "1.2"

2. **Launch with TLS:**

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py \
        extra_params_file:=tls_params.yaml

Configuration Options
---------------------

.. list-table::
   :widths: 30 15 55
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``server.tls.enabled``
     - ``false``
     - Enable/disable TLS
   * - ``server.tls.cert_file``
     - (required)
     - Path to PEM-encoded certificate
   * - ``server.tls.key_file``
     - (required)
     - Path to PEM-encoded private key
   * - ``server.tls.ca_file``
     - ``""``
     - CA certificate (for future mutual TLS)
   * - ``server.tls.min_version``
     - ``"1.2"``
     - Minimum TLS version: ``"1.2"`` or ``"1.3"``

Using with curl
---------------

**With CA verification (recommended):**

.. code-block:: bash

   curl --cacert ./certs/ca.crt https://localhost:8443/api/v1/areas

**Skip verification (development only):**

.. code-block:: bash

   curl -k https://localhost:8443/api/v1/areas

Using with Postman
------------------

**Option A: Disable SSL verification (development)**

1. Go to Settings → Settings
2. Find "SSL certificate verification"
3. Toggle OFF

**Option B: Add CA certificate (recommended)**

1. Go to Settings → Certificates
2. Under "CA Certificates", click "Select File"
3. Select your ``ca.crt`` file

Production Certificates
-----------------------

For production, obtain certificates from a trusted Certificate Authority:

**Using Let's Encrypt (free):**

1. Install certbot:

   .. code-block:: bash

      sudo apt install certbot

2. Obtain certificates:

   .. code-block:: bash

      sudo certbot certonly --standalone -d your-domain.com

3. Configure the gateway:

   .. code-block:: yaml

      server:
        tls:
          enabled: true
          cert_file: "/etc/letsencrypt/live/your-domain.com/fullchain.pem"
          key_file: "/etc/letsencrypt/live/your-domain.com/privkey.pem"

Security Best Practices
-----------------------

1. **Protect private keys:**

   .. code-block:: bash

      chmod 600 server.key

2. **Use TLS 1.3 when possible:**

   .. code-block:: yaml

      server:
        tls:
          min_version: "1.3"

3. **Never use self-signed certificates in production**

4. **Rotate certificates before expiry**

5. **Combine with authentication:**

   See :doc:`authentication` for JWT configuration.

Troubleshooting
---------------

**"SSL certificate problem" with curl**

- Use ``-k`` flag for self-signed certs, or
- Provide CA certificate with ``--cacert``

**"Key does not match certificate"**

- Regenerate both certificate and key together

**Connection refused**

- Check the port (8443 for HTTPS, not 8080)
- Verify ``tls.enabled`` is ``true``

See Also
--------

- :doc:`authentication` - JWT authentication
- `Let's Encrypt <https://letsencrypt.org/>`_ - Free TLS certificates
