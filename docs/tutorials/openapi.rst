OpenAPI / Swagger
=================

The gateway self-describes its REST API using the
`OpenAPI 3.1.0 <https://spec.openapis.org/oas/v3.1.0>`_ specification.
Every level of the URL hierarchy exposes a ``/docs`` endpoint that returns
a context-scoped OpenAPI spec - so tools and developers always have an
accurate, up-to-date description of available operations.

Accessing the Spec
------------------

Append ``/docs`` to any valid API path:

.. code-block:: bash

   # Full gateway spec (all endpoints)
   curl http://localhost:8080/api/v1/docs | jq .

   # Spec scoped to the components collection
   curl http://localhost:8080/api/v1/components/docs

   # Spec for a specific component and its resource collections
   curl http://localhost:8080/api/v1/components/my_sensor/docs

   # Spec for one resource collection (e.g. data)
   curl http://localhost:8080/api/v1/components/my_sensor/data/docs

Entity-level specs reflect the actual capabilities of each entity at
runtime. Plugin-registered vendor routes also appear when the requested
path matches a plugin route prefix.

Swagger UI
----------

For an interactive API browser, build the gateway with Swagger UI support:

.. code-block:: bash

   colcon build --cmake-args -DENABLE_SWAGGER_UI=ON

Then open ``http://localhost:8080/api/v1/swagger-ui`` in your browser.
The UI assets are embedded in the binary - no CDN dependency at runtime.

.. note::

   The CMake configure step downloads assets from unpkg.com, so network
   access is required during the build.

Using with External Tools
-------------------------

The spec returned by ``/docs`` is standard OpenAPI and works with any
compatible tooling:

- **Postman** - Import ``http://localhost:8080/api/v1/docs`` as a URL to
  auto-generate a request collection.
- **Client generators** - Feed the spec to
  `openapi-generator <https://openapi-generator.tech/>`_ to produce typed
  clients in Python, TypeScript, Go, and many other languages.
- **Documentation** - Render with `Redoc <https://redocly.com/redoc>`_ or
  any OpenAPI-compatible doc renderer.

Configuration
-------------

The ``/docs`` endpoints are enabled by default. To disable them:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       docs:
         enabled: false

When disabled, all ``/docs`` endpoints return HTTP 501.

See :doc:`/config/server` for the full parameter reference.

See Also
--------

- :doc:`/api/rest` - Complete REST API reference
- :doc:`/config/server` - Server configuration options
