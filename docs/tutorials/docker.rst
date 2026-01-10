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

Quick Start with Demo
---------------------

The `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_ repository
includes a complete Docker setup with TurtleBot3 and Nav2:

.. code-block:: bash

   git clone https://github.com/selfpatch/selfpatch_demos.git
   cd selfpatch_demos/demos/turtlebot3_integration
   docker-compose up

This starts:

- TurtleBot3 simulation with Nav2
- ros2_medkit_gateway
- `sovd_web_ui <https://github.com/selfpatch/sovd_web_ui>`_ (Web UI)

Access the UI at http://localhost:5173

Building the Gateway Image
--------------------------

**Using the provided Dockerfile:**

.. code-block:: bash

   cd ros2_medkit
   docker build -t ros2_medkit_gateway:latest \
     -f src/ros2_medkit_gateway/Dockerfile .

**Multi-stage build for smaller images:**

.. code-block:: dockerfile

   # Build stage
   FROM ros:jazzy AS builder
   WORKDIR /ws
   COPY . src/ros2_medkit
   RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y
   RUN colcon build --packages-select ros2_medkit_gateway

   # Runtime stage
   FROM ros:jazzy
   COPY --from=builder /ws/install /opt/ros2_medkit
   ENV ROS_PACKAGE_PATH=/opt/ros2_medkit
   ENTRYPOINT ["/opt/ros2_medkit/ros2_medkit_gateway/lib/ros2_medkit_gateway/gateway_node"]

Docker Compose Configuration
----------------------------

Example ``docker-compose.yml`` for your own setup:

.. code-block:: yaml

   version: '3.8'

   services:
     gateway:
       image: ros2_medkit_gateway:latest
       ports:
         - "8080:8080"
       environment:
         - ROS_DOMAIN_ID=42
       command: >
         ros2 launch ros2_medkit_gateway gateway.launch.py
         server_host:=0.0.0.0
       networks:
         - ros2_net

   networks:
     ros2_net:
       driver: bridge

Network Configuration
---------------------

**Important:** When running in Docker, configure the gateway to listen on all interfaces:

.. code-block:: yaml

   server:
     host: "0.0.0.0"  # Listen on all interfaces, not just localhost

**ROS 2 Discovery:**

For containers to discover each other, use the same ``ROS_DOMAIN_ID``:

.. code-block:: yaml

   environment:
     - ROS_DOMAIN_ID=42

Or use the DDS discovery configuration for specific peers.

CORS for Web UI
---------------

When the Web UI runs in a separate container or host, enable CORS:

.. code-block:: yaml

   cors:
     allowed_origins:
       - "http://localhost:5173"
       - "http://web_ui:80"
     allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
     allowed_headers: ["Content-Type", "Accept", "Authorization"]

Volume Mounts
-------------

**For development (live code changes):**

.. code-block:: yaml

   volumes:
     - ./src:/ws/src:ro
     - ./config:/ws/config:ro

**For certificates:**

.. code-block:: yaml

   volumes:
     - ./certs:/etc/ros2_medkit/certs:ro

Health Checks
-------------

Add health checks to ensure the gateway is ready:

.. code-block:: yaml

   services:
     gateway:
       # ...
       healthcheck:
         test: ["CMD", "curl", "-f", "http://localhost:8080/api/v1/health"]
         interval: 10s
         timeout: 5s
         retries: 3
         start_period: 30s

Production Considerations
-------------------------

1. **Use specific image tags** (not ``latest``):

   .. code-block:: yaml

      image: ros2_medkit_gateway:<version-tag>

2. **Set resource limits:**

   .. code-block:: yaml

      deploy:
        resources:
          limits:
            cpus: '1.0'
            memory: 512M

3. **Enable TLS for production:**

   See :doc:`https` for certificate configuration.

4. **Use secrets for credentials:**

   .. code-block:: yaml

      secrets:
        jwt_secret:
          file: ./secrets/jwt_secret.txt

      services:
        gateway:
          secrets:
            - jwt_secret

5. **Configure logging:**

   .. code-block:: yaml

      logging:
        driver: "json-file"
        options:
          max-size: "10m"
          max-file: "3"

Running with Simulation
-----------------------

**Nav2 + TurtleBot3:**

See the `TurtleBot3 integration demo <https://github.com/selfpatch/selfpatch_demos/tree/main/demos/turtlebot3_integration>`_
for a complete example.

**Isaac Sim:**

The gateway's topic-based discovery works with Isaac Sim which publishes
topics without creating ROS 2 nodes:

.. code-block:: bash

   docker-compose up gateway
   # Start Isaac Sim separately
   # Gateway will discover /carter1, /carter2, etc. from topics

Troubleshooting
---------------

**Container can't connect to ROS 2 network**

- Ensure all containers use the same network
- Check ``ROS_DOMAIN_ID`` is consistent
- Try ``network_mode: host`` for development

**Gateway returns empty areas/components**

- Wait for ROS 2 discovery (can take a few seconds)
- Check that other nodes are running and visible

**Web UI can't connect to gateway**

- Verify CORS is configured correctly
- Check the gateway host is ``0.0.0.0``, not ``127.0.0.1``

See Also
--------

- :doc:`https` - TLS configuration
- :doc:`authentication` - JWT authentication
- `Docker Compose documentation <https://docs.docker.com/compose/>`_
