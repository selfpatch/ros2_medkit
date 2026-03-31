#!/usr/bin/env python3
# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""OpenAPI callability test - verifies the spec can be used to call every endpoint.

Fetches the OpenAPI spec from GET /docs, then for every declared endpoint
constructs a request exactly as a generated client would (using only information
from the spec) and sends it to the live gateway. A 400 response means the spec
led a client to construct an invalid request - that is a spec bug.

Any other status code (200, 201, 204, 404, 409, 501 ...) is acceptable because
it means the handler understood the request format; it may have rejected it for
business-logic reasons (entity not found, resource conflict, etc.) but NOT
because the payload or parameters were structurally wrong.

"""

import re
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor', 'calibration'],
        gateway_params={
            'locking.enabled': True,
            'locking.default_max_expiration': 3600,
            'locking.cleanup_interval': 1,
            'triggers.enabled': True,
            'scripts.enabled': True,
        },
        fault_manager=True,
    )


# ---------------------------------------------------------------------------
# Schema helpers
# ---------------------------------------------------------------------------

def resolve_ref(schema, component_schemas):
    """Follow a single $ref to the component schema."""
    if not isinstance(schema, dict):
        return schema
    if '$ref' in schema:
        ref_name = schema['$ref'].rsplit('/', 1)[-1]
        return component_schemas.get(ref_name, schema)
    return schema


def generate_value(schema, component_schemas, field_name=''):
    """Generate a minimal plausible value from a JSON Schema node."""
    schema = resolve_ref(schema, component_schemas)
    if not isinstance(schema, dict):
        return 'test'

    schema_type = schema.get('type', 'string')

    # OpenAPI 3.1 nullable: type can be a list like ["object", "null"]
    if isinstance(schema_type, list):
        schema_type = next((t for t in schema_type if t != 'null'), 'string')

    if 'enum' in schema:
        return schema['enum'][0]

    if schema_type == 'string':
        fmt = schema.get('format', '')
        if fmt == 'date-time':
            return '2026-01-01T00:00:00Z'
        if fmt == 'binary':
            return 'dGVzdA=='  # base64 "test"
        # Context-aware string generation
        if field_name == 'type':
            return 'std_msgs/msg/String'
        return 'test_value'
    if schema_type == 'integer':
        return max(int(schema.get('minimum', 1)), 1)
    if schema_type == 'number':
        return max(float(schema.get('minimum', 1.0)), 1.0)
    if schema_type == 'boolean':
        return True
    if schema_type == 'array':
        items_schema = schema.get('items', {})
        return [generate_value(items_schema, component_schemas)]
    if schema_type == 'object':
        return generate_payload(schema, component_schemas)

    return 'test'


def generate_payload(schema, component_schemas, context=None):
    """Build a dict with all required fields set to plausible values.

    context is an optional dict with 'entity_map', 'path', 'data_topics' keys
    for generating context-aware values (e.g. valid resource URIs).
    """
    schema = resolve_ref(schema, component_schemas)
    if not isinstance(schema, dict) or schema.get('type') != 'object':
        return {}

    payload = {}
    properties = schema.get('properties', {})
    required = set(schema.get('required', []))

    for field_name in required:
        if field_name in properties:
            field_schema = properties[field_name]

            # Context-aware: 'resource' fields need valid SOVD resource URIs
            if field_name == 'resource' and context:
                payload[field_name] = _generate_resource_uri(context)
            else:
                payload[field_name] = generate_value(
                    field_schema, component_schemas, field_name=field_name
                )

    return payload


def _generate_resource_uri(context):
    """Generate a valid SOVD resource URI for trigger/subscription creation.

    The URI must reference the same entity as the current route path.
    Format: /api/v1/{entity_type}/{entity_id}/{collection}/{topic}
    """
    path = context.get('path', '')
    entity_map = context.get('entity_map', {})
    data_topics = context.get('data_topics', {})

    # Extract entity type and ID from the current path
    for param, etype in _ENTITY_PARAM_MAP.items():
        prefix = f'/{etype}/{{{param}}}'
        if prefix in path:
            eid = entity_map.get(etype, 'test_entity')
            topic = data_topics.get(eid, 'test_topic')
            return f'/api/v1/{etype}/{eid}/data/{topic}'

    # Fallback
    if 'apps' in entity_map:
        eid = entity_map['apps']
        topic = data_topics.get(eid, 'test_topic')
        return f'/api/v1/apps/{eid}/data/{topic}'
    return '/api/v1/apps/test_entity/data/test_topic'


def generate_query_params(parameters, component_schemas):
    """Generate query param values from the operation's parameter list."""
    params = {}
    for p in parameters:
        if p.get('in') != 'query':
            continue
        p_schema = p.get('schema', {})
        p_schema = resolve_ref(p_schema, component_schemas)
        params[p['name']] = generate_value(p_schema, component_schemas)
    return params


def generate_headers(parameters, component_schemas):
    """Generate header values from the operation's parameter list."""
    headers = {}
    for p in parameters:
        if p.get('in') != 'header':
            continue
        p_schema = p.get('schema', {})
        p_schema = resolve_ref(p_schema, component_schemas)
        headers[p['name']] = str(generate_value(p_schema, component_schemas))
    return headers


# Path parameter patterns -> entity types
_ENTITY_PARAM_MAP = {
    'area_id': 'areas',
    'component_id': 'components',
    'app_id': 'apps',
    'function_id': 'functions',
}

# Business-logic 400s that are NOT spec bugs. These occur when the handler
# rejects a request for reasons that cannot be expressed in the OpenAPI schema
# (e.g. cross-resource validation, runtime state dependencies).
_KNOWN_BUSINESS_400_PATTERNS = [
    # Aggregated config requires entity-qualified param names (e.g. app_id:param)
    'Aggregated configuration requires app_id prefix',
    # Bulk data category validation returns 400 instead of 404 for unknown categories
    'Unknown bulk-data category',
    # Entity type mismatch when test uses a dummy/wrong ID for a route type
    'Invalid entity type for route',
    # Trigger condition_type is runtime-validated against registered providers
    'Unknown condition type',
    # Data collection requires a specific topic in the resource URI
    'Data collection requires a resource path',
    # Resource URI must match the route's entity (cross-entity validation)
    'Resource URI must reference the same entity',
    # Configuration value type mismatch - generated value doesn't match actual ROS 2 param type
    'Failed to set parameter',
]


def substitute_path_params(path, entity_map):
    """Replace {param} placeholders with real or dummy values.

    entity_map maps entity types (areas, components, apps, functions)
    to a real entity ID so the request gets past entity validation.
    Resource-level params get a dummy 'test_id'.
    """
    def replacer(match):
        param = match.group(1)
        if param in _ENTITY_PARAM_MAP:
            entity_type = _ENTITY_PARAM_MAP[param]
            return entity_map.get(entity_type, 'test_entity')
        # Resource-level params (lock_id, trigger_id, etc.)
        return 'test_id'

    return re.sub(r'\{([^}]+)\}', replacer, path)


def is_sse_endpoint(operation):
    """Detect SSE streaming endpoints that would hang on GET."""
    summary = operation.get('summary', '') + operation.get('description', '')
    return 'SSE' in summary or 'stream' in summary.lower()


def is_known_business_400(message):
    """Check if a 400 error matches a known business-logic rejection."""
    return any(pattern in message for pattern in _KNOWN_BUSINESS_400_PATTERNS)


class TestOpenApiCallability(GatewayTestCase):
    """Verify that every endpoint in the OpenAPI spec is callable.

    For each operation declared in GET /docs, construct a request using only
    information from the spec (path params, query params, headers, request
    body) and send it to the live gateway. Any 400 response is a test
    failure - it means the spec misled a client.
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        # Discover real entity IDs to use in path substitution
        cls._entity_map = {}
        for entity_type in ('areas', 'components', 'apps', 'functions'):
            try:
                data = cls._get_json_cls(f'/{entity_type}')
                items = data.get('items', [])
                if items:
                    cls._entity_map[entity_type] = items[0].get('id', 'test_entity')
            except Exception:
                pass

        # Discover a data topic for each known app (for resource URIs)
        cls._data_topics = {}
        for etype in ('apps', 'components'):
            eid = cls._entity_map.get(etype)
            if not eid:
                continue
            try:
                data = cls._get_json_cls(f'/{etype}/{eid}/data')
                items = data.get('items', [])
                if items:
                    cls._data_topics[eid] = items[0].get('id', 'test_topic')
            except Exception:
                pass

    @classmethod
    def _get_json_cls(cls, endpoint):
        """Class-level GET helper for setUpClass."""
        resp = requests.get(f'{cls.BASE_URL}{endpoint}', timeout=10)
        return resp.json()

    def _fetch_spec(self):
        """Fetch and return the full OpenAPI spec."""
        return self.poll_endpoint_until(
            '/docs',
            lambda d: d if d.get('paths') else None,
        )

    def test_all_endpoints_accept_spec_requests(self):
        """Every endpoint in the spec must accept a spec-conformant request.

        A 400 response means the spec is wrong - a generated client would
        build a request that the handler rejects.

        @verifies REQ_INTEROP_002
        """
        spec = self._fetch_spec()
        schemas = spec.get('components', {}).get('schemas', {})
        failures = []

        for path, path_item in spec.get('paths', {}).items():
            for method in ('get', 'post', 'put', 'delete'):
                if method not in path_item:
                    continue

                operation = path_item[method]

                # Skip SSE streaming endpoints - they hang on GET
                if is_sse_endpoint(operation):
                    continue

                url = self.BASE_URL + substitute_path_params(
                    path, self._entity_map
                )
                op_id = operation.get(
                    'operationId',
                    f'{method.upper()} {path}',
                )

                # Per-path context for smart payload generation
                context = {
                    'entity_map': self._entity_map,
                    'data_topics': self._data_topics,
                    'path': path,
                }

                # Build request kwargs from spec
                kwargs = {'timeout': 10}

                # Query params
                params = generate_query_params(
                    operation.get('parameters', []), schemas
                )
                if params:
                    kwargs['params'] = params

                # Headers from spec
                headers = generate_headers(
                    operation.get('parameters', []), schemas
                )
                if headers:
                    kwargs['headers'] = headers

                # Request body (POST/PUT)
                req_body = operation.get('requestBody', {})
                body_schema = (
                    req_body
                    .get('content', {})
                    .get('application/json', {})
                    .get('schema', {})
                )
                if body_schema:
                    resolved = resolve_ref(body_schema, schemas)
                    kwargs['json'] = generate_payload(
                        resolved, schemas, context=context
                    )

                # Send
                try:
                    resp = requests.request(method.upper(), url, **kwargs)
                except requests.exceptions.RequestException as exc:
                    failures.append(f'{op_id}: connection error - {exc}')
                    continue

                if resp.status_code == 400:
                    try:
                        err = resp.json()
                        msg = err.get('message', resp.text[:200])
                    except Exception:
                        msg = resp.text[:200]

                    # Skip known business-logic rejections
                    if is_known_business_400(msg):
                        continue

                    failures.append(
                        f'{op_id}: 400 Bad Request - {msg}'
                    )

        if failures:
            detail = '\n  '.join(failures)
            self.fail(
                f'{len(failures)} endpoint(s) rejected spec-conformant '
                f'requests:\n  {detail}'
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with code '
                f'{info.returncode}',
            )
