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

For a hardened deployment use ``config/gateway_params.secure.yaml``, which turns on TLS
1.3 together with authentication and rate limiting; see ``design/hardening.rst``.

Generating Development Certificates
-----------------------------------

The package includes a helper script:

.. code-block:: bash

   cd ~/ros2_medkit_ws/src/ros2_medkit/src/ros2_medkit_gateway
   ./scripts/generate_dev_certs.sh ./certs

This creates:

- ``ca.pem`` - Certificate Authority certificate
- ``ca_key.pem`` - Certificate Authority private key
- ``cert.pem`` - Server certificate
- ``key.pem`` - Server private key
- ``client.pem`` - Client certificate (for mutual TLS)
- ``client-key.pem`` - Client private key

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

   TLS encrypts the traffic but does not authenticate clients. With
   ``server.host`` ``"0.0.0.0"`` and authentication off, the gateway refuses to
   start (protected mode): set ``MEDKIT_JWT_SECRET`` and ``MEDKIT_CLIENTS`` (see
   :doc:`authentication`), or add ``allow_unauthenticated_remote: true`` under
   ``server``.

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
     - CA bundle. When set, the file must exist; it is checked against client certificates only with ``require_client_cert``
   * - ``server.tls.require_client_cert``
     - ``false``
     - Require a client certificate signed by ``ca_file`` (mutual TLS)
   * - ``server.tls.min_version``
     - ``"1.2"``
     - Protocol floor of the server: ``"1.2"`` or ``"1.3"``; with TLS on, any other value stops the start

Mutual TLS
----------

Set ``require_client_cert`` and name the CA that signed the client certificates:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         tls:
           enabled: true
           cert_file: "/path/to/server.crt"
           key_file: "/path/to/server.key"
           ca_file: "/path/to/ca.crt"
           require_client_cert: true

A client presents its certificate with curl:

.. code-block:: bash

   curl --cacert ./certs/ca.pem \
        --cert ./certs/client.pem \
        --key ./certs/client-key.pem \
        https://localhost:8443/api/v1/areas

Without ``--cert``, or with a certificate from another CA, the handshake fails.
The certificate does not replace a token when authentication is on.

``gateway_https.launch.py`` turns mutual TLS on with ``require_client_cert:=true``
or with a ``config_file`` that sets ``server.tls.require_client_cert: true``.
Without a ``server.tls.ca_file`` in that file, the launch's own CA in
``cert_dir`` signs the client certificate: the launch writes ``client.pem`` and
``client-key.pem``, makes them again unless that CA signed them and the key is
the certificate's, and prints a curl line with them. A ``ca_file`` the file sets
is kept, and the client certificate must come from that CA. The launch never
writes over it and never signs with it, even when it is ``cert_dir/ca.pem``; if
``cert_dir`` then has no ``cert.pem`` and ``key.pem`` to reuse, the launch stops
with an error naming the file.

Using with curl
---------------

**With CA verification (recommended):**

.. code-block:: bash

   curl --cacert ./certs/ca.pem https://localhost:8443/api/v1/areas

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
3. Select your ``ca.pem`` file

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

      chmod 600 key.pem

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
