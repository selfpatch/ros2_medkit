Troubleshooting & FAQ
=====================

Common issues and solutions when using ros2_medkit.

.. contents:: Table of Contents
   :local:
   :depth: 2

Installation Issues
-------------------

**rosdep fails to find packages**

.. code-block:: bash

   ERROR: the following packages/stacks could not have their rosdep keys resolved

Solution: Initialize and update rosdep:

.. code-block:: bash

   sudo rosdep init  # Only needed once
   rosdep update

**Cannot find ros2_medkit packages**

.. code-block:: bash

   Package 'ros2_medkit_gateway' not found

Solution: Source the workspace:

.. code-block:: bash

   source ~/ros2_medkit_ws/install/setup.bash

Gateway Issues
--------------

**Gateway shows no areas or components**

Possible causes:

1. **No other ROS 2 nodes running** - Start some nodes first
2. **Different ROS_DOMAIN_ID** - Ensure all nodes use the same domain
3. **Discovery not complete** - Wait a few seconds and refresh

Check with:

.. code-block:: bash

   ros2 node list
   ros2 topic list

**"Connection refused" when accessing API**

Possible causes:

1. **Gateway not running** - Check process is active
2. **Wrong port** - Default is 8080 (HTTP) or 8443 (HTTPS)
3. **Localhost binding** - Gateway binds to 127.0.0.1 by default

For network access, set host to 0.0.0.0:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0

**Topic data returns empty or timeout**

Possible causes:

1. **Topic has no publishers** - Check with ``ros2 topic info``
2. **Publisher is slow** - Increase timeout in config
3. **Topic type not supported** - Check for custom message types

For idle topics, the gateway returns metadata instantly. If a topic has
publishers but no data, check if messages are actually being published:

.. code-block:: bash

   ros2 topic hz /your/topic

**CORS errors in browser**

.. code-block:: text

   Access to fetch blocked by CORS policy

Solution: Configure CORS in gateway params:

.. code-block:: yaml

   cors:
     allowed_origins: ["http://localhost:5173"]
     allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]

Authentication Issues
---------------------

**"Invalid token" error**

Possible causes:

1. **Token expired** - Get a new token or use refresh token
2. **Wrong secret** - Ensure JWT secret matches
3. **Malformed header** - Format must be ``Authorization: Bearer <token>``

**"Insufficient permissions" error**

Check your role has the required permission:

- ``viewer`` - Read only
- ``operator`` - Read + operations + data publish
- ``configurator`` - Operator + configurations
- ``admin`` - Full access

**Token refresh fails**

Refresh tokens are single-use. After refreshing, use the new refresh token
from the response.

TLS/HTTPS Issues
----------------

**"SSL certificate problem"**

For development with self-signed certificates:

.. code-block:: bash

   curl -k https://localhost:8443/api/v1/health

For production, use certificates from a trusted CA.

**"Key does not match certificate"**

Regenerate both certificate and key together:

.. code-block:: bash

   ./scripts/generate_dev_certs.sh ./certs

Docker Issues
-------------

**Container can't discover ROS 2 nodes**

Ensure all containers use the same:

1. Docker network
2. ``ROS_DOMAIN_ID``
3. DDS configuration

For development, try ``network_mode: host``.

**Web UI can't connect to gateway in container**

1. Gateway must listen on ``0.0.0.0``, not ``127.0.0.1``
2. CORS must allow the UI origin
3. Port must be exposed in docker-compose

Fault Manager Issues
--------------------

**Faults API returns 503 or empty**

The Faults API requires ``ros2_medkit_fault_manager`` node:

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node

**Faults not appearing for my component**

Components must use ``ros2_medkit_fault_reporter`` to report faults.
The REST API is read-only for fault status.

Performance Issues
------------------

**Slow response for component data**

Possible causes:

1. **Many topics** - Response time scales with topic count
2. **Slow publishers** - 3-second timeout per topic by default
3. **Low parallelism** - Increase ``max_parallel_topic_samples``

Optimize with:

.. code-block:: yaml

   max_parallel_topic_samples: 20  # Increase from default 10
   topic_sample_timeout_sec: 0.5   # Decrease from default 1.0

**High CPU usage**

Reduce cache refresh rate:

.. code-block:: yaml

   refresh_interval_ms: 30000  # 30 seconds instead of default 10s

FAQ
---

**Q: Does ros2_medkit modify my ROS 2 nodes?**

No. ros2_medkit is read-only by default. It discovers existing nodes and
exposes them via REST API. Write operations (publishing, service calls,
parameter changes) only happen when explicitly requested via the API.

**Q: Can I use ros2_medkit with ROS 1?**

No. ros2_medkit requires ROS 2 Jazzy. For ROS 1 systems, consider using
the ``ros1_bridge`` and running ros2_medkit on the ROS 2 side.

**Q: Is ros2_medkit production-ready?**

ros2_medkit is currently suitable for development and testing. For production:

- Enable TLS encryption
- Configure JWT authentication
- Test thoroughly with your specific use case
- Monitor performance under expected load

**Q: How does ros2_medkit compare to rosbridge?**

.. list-table::
   :widths: 30 35 35
   :header-rows: 1

   * - Feature
     - ros2_medkit
     - rosbridge
   * - Protocol
     - REST/HTTP
     - WebSocket/JSON
   * - Discovery
     - Automatic entity hierarchy
     - Topic/service access
   * - Authentication
     - JWT with RBAC
     - None built-in
   * - Fault management
     - Yes
     - No
   * - SOVD compatible
     - Yes
     - No

**Q: Can I extend ros2_medkit with custom endpoints?**

Not directly at this time. Future versions may support plugins.
For now, you can fork the gateway and add custom routes in ``rest_server.cpp``.

**Q: How do I report a bug or request a feature?**

- **Bug reports**: https://github.com/selfpatch/ros2_medkit/issues
- **Feature requests**: https://github.com/selfpatch/ros2_medkit/discussions
- **Discord**: https://discord.gg/6CXPMApAyq

Getting Help
------------

If your issue isn't covered here:

1. **Search existing issues**: https://github.com/selfpatch/ros2_medkit/issues
2. **Ask on Discord**: https://discord.gg/6CXPMApAyq
3. **Open a new issue** with:
   - ros2_medkit version
   - ROS 2 distribution
   - Steps to reproduce
   - Expected vs actual behavior
   - Relevant logs
