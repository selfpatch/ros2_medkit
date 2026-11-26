Docker Deployment
=================

This tutorial shows how to deploy ros2_medkit using Docker containers.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Docker deployment is useful for:

- Reproducible environments
- CI/CD pipelines
- Production deployments
- Testing with simulation (e.g., Nav2, Isaac Sim)

Pre-built Images
----------------

Pre-built images are published to GitHub Container Registry on every push to main
and on every release tag. Each image is multi-arch (``linux/amd64`` and ``linux/arm64``).

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Trigger
     - Image Tags
   * - Push/merge to ``main``
     - ``latest``, ``sha-<commit>``
   * - Release tag ``1.2.3``
     - ``1.2.3``, ``1.2``, ``sha-<commit>``

Images are available for all supported ROS 2 distributions:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Distribution
     - Image
   * - Jazzy (recommended)
     - ``ghcr.io/selfpatch/ros2_medkit-jazzy:latest``
   * - Humble
     - ``ghcr.io/selfpatch/ros2_medkit-humble:latest``
   * - Lyrical
     - ``ghcr.io/selfpatch/ros2_medkit-lyrical:latest``

Each image includes the gateway and all open-core packages:

- ``ros2_medkit_gateway`` - HTTP REST server
- ``ros2_medkit_fault_manager`` - Fault aggregation and management
- ``ros2_medkit_fault_reporter`` - Client library for fault reporting
- ``ros2_medkit_diagnostic_bridge`` - Bridges ``/diagnostics`` to fault manager
- ``ros2_medkit_serialization`` - Runtime JSON/ROS 2 serialization
- ``ros2_medkit_graph_provider`` - ROS 2 graph introspection plugin
- ``ros2_medkit_topic_beacon``, ``ros2_medkit_param_beacon`` - Discovery plugins
- ``ros2_medkit_linux_introspection`` - Linux system introspection plugin

Quick Start
-----------

.. code-block:: bash

   docker run -p 127.0.0.1:8080:8080 -e MEDKIT_AUTH_DISABLED=1 ghcr.io/selfpatch/ros2_medkit-jazzy:latest

The image binds ``0.0.0.0``. A gateway without authentication refuses to bind a
non-loopback address (protected mode), so a container started with no
``MEDKIT_*`` variable exits with an error that names the ways out.
``MEDKIT_AUTH_DISABLED=1`` opts out, as above; ``MEDKIT_JWT_SECRET`` closes the
container (next section).

Test the gateway:

.. code-block:: bash

   curl http://localhost:8080/api/v1/health
   # {"status":"healthy","timestamp":...}

   curl http://localhost:8080/api/v1/version-info
   # {"items":[{"version":"<gateway-version>","vendor_info":{"name":"ros2_medkit",...}}]}

With ``MEDKIT_AUTH_DISABLED=1`` the container answers requests without a
credential. Publish the port only where that is acceptable, or close the
container as described next.

Running the container closed
----------------------------

Set ``MEDKIT_JWT_SECRET`` and ``MEDKIT_CLIENTS`` and the container requires a
token on every route except ``/api/v1/auth/*``, which answers without one so a
client can obtain its token. The secret signs the tokens. Each client is written
``id:secret:role`` and exchanges its credential for a token at
``POST /api/v1/auth/authorize``:

.. code-block:: bash

   CLIENT_SECRET="$(head -c 24 /dev/urandom | base64 | tr -d '=+/')"
   echo "client secret: ${CLIENT_SECRET}"
   docker run -p 8080:8080 \
     -e MEDKIT_JWT_SECRET="$(head -c 32 /dev/urandom | base64)" \
     -e MEDKIT_CLIENTS="medkit:${CLIENT_SECRET}:admin" \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

The client secret is generated and printed first, so whoever has to log in knows
it. The signing secret is needed by nothing outside the container.

The three variables are read by the gateway itself, so they apply to every way
of starting it in the image: the default command, an arguments-only override,
``docker run <image> ros2 launch ...`` and a shell. They take effect over any
params file, including a mounted one that says ``auth.enabled: false``:

``MEDKIT_JWT_SECRET``
  Authentication on, ``auth.require_auth_for`` ``all``, this signing secret.

``MEDKIT_CLIENTS``
  The clients that can obtain a token. It replaces ``auth.clients`` when
  ``MEDKIT_JWT_SECRET`` is set.

``MEDKIT_AUTH_DISABLED``
  Set to exactly ``1``, it turns authentication off even when
  ``MEDKIT_JWT_SECRET`` is set, and it opts out of protected mode, so the
  gateway may bind ``0.0.0.0`` without authentication. Anyone who can set it on
  a gateway's environment can open that gateway.

See :doc:`authentication` for the entry syntax, the refusal rules and the
startup warnings.

A closed container answers ``401`` to an unauthenticated ``/api/v1/health`` probe, so use a
probe that accepts it, or list the route in ``auth.public_routes`` (see `Health Checks`_).

Custom Configuration
--------------------

The image applies its configuration in layers, each one overriding the one
before it:

1. ``/etc/ros2_medkit/base.yaml`` - the packaged ``gateway_params.yaml``, the
   same file a source install gets
2. ``/etc/ros2_medkit/container.yaml`` - the container settings:
   ``server.host: "0.0.0.0"``, ``refresh_interval_ms: 2000`` and the CORS origins
   ``http://localhost:3000`` and ``http://localhost:5173``
3. ``/etc/ros2_medkit/params.yaml`` - the mount point
4. the arguments given to ``docker run``

The layers apply to the default command, and to a command that starts with a
ROS argument flag such as ``--ros-args``. A command that starts with anything
else (``ros2 launch ...``, ``bash -c '...'``) runs exactly as given and takes its
configuration from its own arguments.

The entrypoint merges the three files into one before it starts the gateway.
Each file may hold a ``/**`` section and sections keyed by node full name, such
as ``/ros2_medkit_gateway``. Within one file a node section wins over ``/**``;
across files a later layer wins, parameter by parameter. A list, such as
``cors.allowed_origins``, is replaced as a whole. A node renamed with
``-r __node:=...`` or moved with ``-r __ns:=...`` takes the section keyed by its
new full name (``/renamed``, ``/ns/renamed``) and the ``/**`` sections. The
image's own base and container layers are keyed ``/ros2_medkit_gateway`` and do
not apply to it, so give the renamed node every parameter it needs.

The merged file is written to the directory ROS writes its logs to
(``$ROS_LOG_DIR``, else ``$ROS_HOME/log``, else ``/home/medkit/.ros/log``). With
``docker run --read-only`` mount a writable volume or tmpfs there, for example
``--tmpfs /home/medkit/.ros/log:uid=999``; nothing else of the image is written.

A mounted file names only the keys it changes and keeps every other layer's
value. It can set ``server.host`` and ``refresh_interval_ms`` again. A file that
sets ``cors.allowed_origins`` replaces the list. To turn CORS off from a mounted
file, set ``allowed_origins: [""]``: the parameter is a typed string array and
``[]`` stops the gateway at start. A wildcard origin is not used.

.. note::

   Upgrading: a file mounted at ``/etc/ros2_medkit/params.yaml`` used to replace
   the image's own file, so a file without ``server.host`` gave the compiled
   default ``127.0.0.1``. It now inherits ``server.host: "0.0.0.0"`` and the two
   CORS origins from the container layer. A container with such a file and
   ``--network host`` now listens on every interface, and without
   ``MEDKIT_JWT_SECRET`` or ``MEDKIT_AUTH_DISABLED=1`` it refuses to start. To
   keep the old behaviour, put ``server.host: "127.0.0.1"`` in the mounted file.

.. note::

   A launch file run in the container binds ``127.0.0.1`` unless told
   otherwise. Without ``--network host`` that is the container's own loopback,
   and the published port answers with ``Connection reset by peer`` or an empty
   reply. Pass ``server_host:=0.0.0.0``. Without authentication the gateway
   then refuses to start unless it is told to accept that, with
   ``allow_unauthenticated_remote:=true`` or ``MEDKIT_AUTH_DISABLED=1``:

   .. code-block:: bash

      docker run -p 127.0.0.1:8080:8080 -e MEDKIT_AUTH_DISABLED=1 \
        ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
        ros2 launch ros2_medkit_gateway bringup.launch.py server_host:=0.0.0.0

   ``127.0.0.1:8080:8080`` publishes the port on the host's loopback only, which
   keeps the API (no authentication) off the LAN.

To use a custom configuration, mount a params file:

.. code-block:: bash

   docker run -p 127.0.0.1:8080:8080 \
     -e MEDKIT_AUTH_DISABLED=1 \
     -v ./my_params.yaml:/etc/ros2_medkit/params.yaml \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

Example ``my_params.yaml`` (``MEDKIT_AUTH_DISABLED=1`` above, or
``MEDKIT_JWT_SECRET``, is what lets the ``0.0.0.0`` bind start):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         host: "0.0.0.0"
         port: 8080
       refresh_interval_ms: 2000
       cors:
         allowed_origins: ["http://localhost:5173", "http://localhost:3000"]
       discovery:
         mode: "runtime_only"

You can also pass ROS arguments directly. The three layers stay in place, so an
override changes the key it names and nothing else:

.. code-block:: bash

   docker run -p 127.0.0.1:9090:9090 -e MEDKIT_AUTH_DISABLED=1 ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
     --ros-args -p server.port:=9090

If the node is renamed or moved to another namespace, none of the image's files
apply (see above).

External Plugins
----------------

External plugins (e.g., custom providers) can be mounted into the container at
``/opt/ros2_medkit/plugins/`` and referenced in the params file:

.. code-block:: bash

   docker run -p 127.0.0.1:8080:8080 \
     -e MEDKIT_AUTH_DISABLED=1 \
     -v ./my_plugin.so:/opt/ros2_medkit/plugins/my_plugin.so \
     -v ./my_params.yaml:/etc/ros2_medkit/params.yaml \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

With ``my_params.yaml`` referencing the plugin:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_plugin"]
       plugins.my_plugin.path: "/opt/ros2_medkit/plugins/my_plugin.so"
       # plugin-specific config
       plugins.my_plugin.some_key: "some_value"

Building from Source
--------------------

To build the image locally (e.g., for development or custom modifications):

.. code-block:: bash

   git clone https://github.com/selfpatch/ros2_medkit.git
   cd ros2_medkit
   docker build -t ros2_medkit .

Build for a specific ROS 2 distribution:

.. code-block:: bash

   docker build --build-arg ROS_DISTRO=humble -t ros2_medkit-humble .

Docker Compose
--------------

Example ``docker-compose.yml`` with the gateway and web UI:

.. code-block:: yaml

   services:
     gateway:
       image: ghcr.io/selfpatch/ros2_medkit-jazzy:latest
       ports:
         - "127.0.0.1:8080:8080"
       environment:
         - ROS_DOMAIN_ID=42
         # The image binds 0.0.0.0; without authentication it needs this opt-out.
         # Set MEDKIT_JWT_SECRET and MEDKIT_CLIENTS instead to require a login.
         - MEDKIT_AUTH_DISABLED=1
       healthcheck:
         # A closed container answers 401, which still shows the process is up.
         test:
           - CMD-SHELL
           - >-
             code=$$(curl -s -o /dev/null -w '%{http_code}'
             http://localhost:8080/api/v1/health) && case "$$code" in
             200|401|403) exit 0 ;; *) exit 1 ;; esac
         interval: 10s
         timeout: 5s
         retries: 3
         start_period: 15s
       networks:
         - ros2_net

     web-ui:
       image: ghcr.io/selfpatch/ros2_medkit_web_ui:latest
       ports:
         - "3000:80"
       depends_on:
         gateway:
           condition: service_healthy
       networks:
         - ros2_net

   networks:
     ros2_net:
       driver: bridge

Quick Start with Demos
----------------------

The `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_ repository
includes complete Docker Compose setups with simulation:

.. code-block:: bash

   git clone https://github.com/selfpatch/selfpatch_demos.git
   cd selfpatch_demos/demos/turtlebot3_integration
   docker compose up

This starts TurtleBot3 with Nav2, the gateway, and the web UI.

Network Configuration
---------------------

**ROS 2 Discovery:**

For containers to discover each other's ROS 2 nodes, use the same ``ROS_DOMAIN_ID``:

.. code-block:: yaml

   environment:
     - ROS_DOMAIN_ID=42

**Host network mode** (simplest for development):

.. code-block:: yaml

   network_mode: host

CORS for Web UI
---------------

The image enables CORS for the default web UI origins (``http://localhost:3000``
and ``http://localhost:5173``). A wildcard is deliberately not used: with auth
disabled and write methods enabled it would let any site drive cross-origin
writes. A mounted file that sets ``cors.allowed_origins`` replaces the whole
list, so name every origin you need (``allowed_origins: [""]`` turns CORS off;
``[]`` stops the gateway at start):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       cors:
         allowed_origins:
           - "http://localhost:3000"
           - "https://my-dashboard.example.com"

The origin must match what the browser sends exactly: ``http://127.0.0.1:3000``
is not ``http://localhost:3000``. When the container runs a launch file, pass
the origins as ``cors_allowed_origins:=http://localhost:3000,http://127.0.0.1:3000``;
the argument replaces the defaults.

Health Checks
-------------

The gateway exposes a health endpoint at ``/api/v1/health``. A container at the
image default answers it without a credential; a closed one answers ``401``.
A probe that works for both accepts any of these HTTP answers as proof that the
process is up. With ``GET /api/v1/health`` in ``auth.public_routes`` a closed
container answers ``200`` with a body that says only that the gateway is alive.
Pass the list in a mounted params file:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       auth:
         public_routes: ["GET /api/v1/health"]

or as an argument to ``docker run``, after the image name:
``--ros-args -p auth.public_routes:="['GET /api/v1/health']"``. The probe below
works with or without it:

.. code-block:: yaml

   healthcheck:
     test:
       - CMD-SHELL
       - >-
         code=$$(curl -s -o /dev/null -w '%{http_code}'
         http://localhost:8080/api/v1/health) && case "$$code" in
         200|401|403) exit 0 ;; *) exit 1 ;; esac
     interval: 10s
     timeout: 5s
     retries: 3
     start_period: 15s

Production Considerations
-------------------------

1. **Set resource limits:**

   .. code-block:: yaml

      deploy:
        resources:
          limits:
            cpus: '1.0'
            memory: 512M

2. **Enable TLS for production:**

   See :doc:`https` for certificate configuration.

3. **Configure logging:**

   .. code-block:: yaml

      logging:
        driver: "json-file"
        options:
          max-size: "10m"
          max-file: "3"

Troubleshooting
---------------

**Container can't connect to ROS 2 network**

- Ensure all containers use the same network and ``ROS_DOMAIN_ID``
- Try ``network_mode: host`` for development

**Gateway returns empty areas/components**

- Wait for ROS 2 discovery (can take a few seconds)
- Check that other ROS 2 nodes are running and visible

**Web UI can't connect to gateway**

- Verify CORS is configured correctly
- Check the gateway host is ``0.0.0.0``, not ``127.0.0.1``

See Also
--------

- :doc:`https` - TLS configuration
- :doc:`authentication` - JWT authentication
- `Docker Compose documentation <https://docs.docker.com/compose/>`_
