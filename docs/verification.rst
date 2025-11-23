Verification
============

This section documents the test cases and their traceability to requirements.

.. test:: Health Endpoint Check
   :id: TEST_001
   :status: verified
   :verifies: REQ_SOVD_080

   Verifies that the gateway exposes a ``/health`` endpoint that returns the node status and timestamp.

   **Implementation:** ``src/ros2_medkit_gateway/test/test_gateway_node.cpp`` (Test: ``test_health_endpoint``)

.. test:: Root Endpoint Check
   :id: TEST_002
   :status: verified
   :verifies: REQ_SOVD_006, REQ_SOVD_015

   Verifies that the root endpoint ``/`` returns service information, version, and a list of available endpoints.

   **Implementation:** ``src/ros2_medkit_gateway/test/test_gateway_node.cpp`` (Test: ``test_root_endpoint``)

.. needtable::
   :columns: id, title, status, verifies
   :style: table

