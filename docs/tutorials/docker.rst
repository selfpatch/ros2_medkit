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
name a multi-architecture manifest list rather than this amd64-only image.

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

The precedence is one rule: ``MEDKIT_AUTH_DISABLED=1`` wins over everything and
forces authentication off; otherwise a set ``MEDKIT_JWT_SECRET`` closes the
container whatever any params file says, because those parameters are passed
after the file; otherwise the file decides. The image also carries
``config/gateway_params.secure.yaml`` - TLS, rate limiting and the rest - which
you can point ``--params-file`` at once the container has a certificate.

Custom Configuration
--------------------

The container listens on ``0.0.0.0:8080`` and refreshes discovery every 2 s -
the two values the image passes on top of the packaged config. CORS is off, so
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

You can also pass ROS arguments directly:

.. code-block:: bash

   docker run -p 9090:9090 ghcr.io/selfpatch/ros2_medkit-jazzy:latest \
     --ros-args --params-file /etc/ros2_medkit/params.yaml -p server.port:=9090

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

Example ``docker-compose.yml`` with the gateway and web UI:

.. code-block:: yaml

   services:
     gateway:
       image: ghcr.io/selfpatch/ros2_medkit-jazzy:latest
       ports:
         - "8080:8080"
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
rather than insisting on success:

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
