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

Pre-built images are published to GitHub Container Registry on every push to main.
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

Every push to ``main`` moves ``:latest`` and also publishes
``:main-<sha7>`` - the same image under the short commit hash it was built
from, which is what to pin when ``:latest`` moving underneath a deployment is
not acceptable. Release tags carry the semver tags and ``:sha-<sha7>``, which
name a multi-architecture manifest list; this one is amd64-only.

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

   docker run -p 8080:8080 ghcr.io/selfpatch/ros2_medkit-jazzy:latest

Test the gateway:

.. code-block:: bash

   curl http://localhost:8080/api/v1/health
   # {"status":"healthy","timestamp":...}

   curl http://localhost:8080/api/v1/version-info
   # {"items":[{"version":"<gateway-version>","vendor_info":{"name":"ros2_medkit",...}}]}

The image carries ``config/gateway_params.yaml``, the same file a source
install gets, so it answers without a credential like a source install does.
Publish the port only where that is acceptable.

Running the container closed
----------------------------

Set ``MEDKIT_JWT_SECRET`` and the container runs with authentication on,
``require_auth_for`` ``all``, and the secret you gave it. ``MEDKIT_CLIENTS``
carries the credentials a client exchanges for a token:

.. code-block:: bash

   docker run -p 8080:8080 \
     -e MEDKIT_JWT_SECRET="$(head -c 32 /dev/urandom | base64)" \
     -e MEDKIT_CLIENTS="medkit:$(head -c 24 /dev/urandom | base64):admin" \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

The environment contract
^^^^^^^^^^^^^^^^^^^^^^^^

Three variables, read by the gateway itself when it reads its parameters. That
is why they hold on every way of starting it - the default ``docker run``, an
arguments-only override, ``docker run <image> ros2 launch ros2_medkit_gateway
bringup.launch.py``, a source install started with ``ros2 run`` - and every
other path, including those a container entrypoint never sees.

``MEDKIT_AUTH_DISABLED``
  Set to exactly ``1``, authentication is off and every route is readable by
  anyone who can reach the port. It wins over everything: the params file, a
  launch argument, and ``MEDKIT_JWT_SECRET``. No other value means anything -
  ``true``, ``yes`` and ``0`` all leave it unset in effect.

  This is not a container-only switch. The gateway reads it wherever it runs, so
  the variable **opens any gateway**, including a source install started from
  ``gateway_params.secure.yaml``. Treat the ability to set it on a gateway's
  environment as equivalent to the ability to turn its authentication off,
  because it is.

``MEDKIT_JWT_SECRET``
  Non-empty, and with ``MEDKIT_AUTH_DISABLED`` not set to ``1``: authentication
  is **on**, ``auth.require_auth_for`` is **``all``**, and this is the signing
  secret. It overrides ``auth.enabled``, ``auth.require_auth_for`` and
  ``auth.jwt_secret`` from any params file. ``"write"`` would leave every read
  open, and the entity tree, the fault history and the operation list are the
  disclosure.

``MEDKIT_CLIENTS``
  The credentials that can be exchanged for a token, read only in the case
  above. Entries are separated by **commas**; each is written
  ``id:secret:role``. The id is everything before the first colon and the role
  everything after the last, so **a secret may contain colons while an id and a
  role may not, and no field may contain a comma**. The role is one of
  ``viewer``, ``operator``, ``configurator``, ``admin``. Surrounding spaces and
  tabs are dropped from each entry, so ``a:b:admin, c:d:viewer`` works; a space
  inside a field belongs to that field.

  A comma in a secret ends the entry there, so the rest of that secret is read
  as the start of the next entry and both are refused - generate secrets from an
  alphabet without commas, or the credential silently is not the one you set.

  An entry that does not parse, or that repeats an id an earlier entry claimed,
  is dropped and named by its position in a ``WARN`` line; the rest are still
  registered, and for a duplicated id the first entry stands. If **every** entry
  is refused the gateway does not start, because a gateway closed to its own
  operator is a misconfiguration, and never a posture somebody chose.

  When ``MEDKIT_CLIENTS`` is set it replaces ``auth.clients`` entirely - the
  empty string included, which leaves no client able to obtain a token and warns
  that it has. Unset it to keep the file's clients.

Whatever the environment overrides, the gateway logs at ``WARN`` on startup, so
the posture a container is running is in its first few lines of output.

The image also carries the closed profile - TLS, rate limiting and the rest -
which you can point ``--params-file`` at. It binds **8443**, not 8080:

.. code-block:: bash

   CLIENT_SECRET="$(head -c 24 /dev/urandom | base64)"
   docker run -p 8443:8443 \
     -v ./certs:/etc/ros2_medkit/certs:ro \
     -e MEDKIT_JWT_SECRET="$(head -c 32 /dev/urandom | base64)" \
     -e MEDKIT_CLIENTS="medkit:${CLIENT_SECRET}:admin" \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
     --ros-args --params-file \
     /home/medkit/ws/install/ros2_medkit_gateway/share/ros2_medkit_gateway/config/gateway_params.secure.yaml \
     -p server.tls.cert_file:=/etc/ros2_medkit/certs/cert.pem \
     -p server.tls.key_file:=/etc/ros2_medkit/certs/key.pem

That profile enables TLS, so the container needs a certificate and key or it
refuses to start. The client logs in with ``CLIENT_SECRET``; the signing secret
is needed by nothing outside the container.

It also needs a secret and a client, and **the mount point cannot supply them**:
the entrypoint puts ``/etc/ros2_medkit/params.yaml`` in front of your
arguments, so the secure profile named after it wins and its empty
``jwt_secret`` stops the gateway. Supply them from the environment as above, or
name a second ``--params-file`` **after** the secure one on the command line.

Custom Configuration
--------------------

The container listens on ``0.0.0.0:8080`` and refreshes discovery every 2 s -
the two values the image sets on top of the packaged config. CORS is off, so
a browser UI on another origin needs its origin named (see `CORS for Web UI`_
below). To use a custom configuration, mount a params file:

.. code-block:: bash

   docker run -p 8080:8080 \
     -v ./my_params.yaml:/etc/ros2_medkit/params.yaml \
     ghcr.io/selfpatch/ros2_medkit-jazzy:latest

Example ``my_params.yaml``:

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

The mounted file is the **last** of three layers and wins over both below it:

1. ``/etc/ros2_medkit/base.yaml`` - the packaged ``gateway_params.yaml``, the
   same file a source install gets
2. ``/etc/ros2_medkit/container.yaml`` - ``server.host: "0.0.0.0"`` and
   ``refresh_interval_ms: 2000``
3. ``/etc/ros2_medkit/params.yaml`` - the mount point

So a file naming only the keys you care about keeps everything else, and the
keys it does name take effect - ``server.host`` and ``refresh_interval_ms``
included. The image passes neither key as a ``-p`` argument. In this container
the entrypoint's three files come first and your arguments after them, and
rclcpp applies the merged node entries in order of first appearance, so a
``-p`` you pass wins over the files; had the image put those two keys in front
of the files as ``-p`` arguments, a mounted file could set neither.

You can also pass ROS arguments directly. The entrypoint puts the three layers
in front of whatever you pass, so an override changes the key it names and
nothing else - the container still binds ``0.0.0.0`` here:

.. code-block:: bash

   docker run -p 9090:9090 ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
     --ros-args -p server.port:=9090

The three layers are keyed on the node's default name, ``ros2_medkit_gateway``,
and apply to that name only. A container that renames the node
(``--ros-args -r __node:=other``) gets none of them: it binds the packaged
``127.0.0.1``, refreshes at the packaged cadence, and a mounted file keyed on
the old name is inert. A renamed node needs a params file keyed on the new
name for every key it relies on.

External Plugins
----------------

External plugins (e.g., custom providers) can be mounted into the container at
``/opt/ros2_medkit/plugins/`` and referenced in the params file:

.. code-block:: bash

   docker run -p 8080:8080 \
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

Example ``docker-compose.yml`` with the gateway and web UI. The gateway mounts a
params file for one reason: the image names no CORS origin, and the browser
loads the UI from ``http://localhost:3000`` while it calls the gateway on
``http://localhost:8080``. Those are different origins, so without the gateway
naming the UI's origin the browser blocks every call and the UI shows an empty
tree with no error a user can act on.

``gateway-cors.yaml``, next to the compose file:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       cors:
         allowed_origins: ["http://localhost:3000"]

.. code-block:: yaml

   services:
     gateway:
       image: ghcr.io/selfpatch/ros2_medkit-jazzy:latest
       ports:
         - "8080:8080"
       volumes:
         - ./gateway-cors.yaml:/etc/ros2_medkit/params.yaml:ro
       environment:
         - ROS_DOMAIN_ID=42
       healthcheck:
         # Any HTTP answer proves the process is up, 401 included. `curl -f`
         # exits non-zero on the refusal a closed container gives an
         # uncredentialed probe, and reports a healthy container as sick.
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

The image names no CORS origin. A published image that allowed
``http://localhost:3000`` would be making a development machine's choice for
every deployment, so the origin a browser UI is served from is named by the
deployment that runs it. A wildcard is the wrong answer here: with auth off and
write methods enabled it would let any site drive cross-origin writes. Add your
own UI origin(s):

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       cors:
         allowed_origins:
           - "http://localhost:3000"
           - "https://my-dashboard.example.com"

Health Checks
-------------

The gateway exposes a health endpoint at ``/api/v1/health``. A container left
at the image default answers it without a credential; one running closed
refuses it, so a probe that has to work in both cases reads the status code
while accepting any HTTP answer as proof the process is up:

.. code-block:: yaml

   healthcheck:
     # 401 means the gateway is up and refused an uncredentialed probe, which
     # is exactly what a liveness check wants to know.
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

If a closed container has to answer a probe that cannot be changed - a load
balancer that only accepts 200, say - open the route explicitly instead:

.. code-block:: yaml

   auth:
     public_routes: ["GET /api/v1/health"]

An anonymous caller then gets liveness only, marked ``x-medkit-reduced``. See
:doc:`/config/server` for what that setting does and does not open.

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
