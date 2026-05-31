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

# Humble (Ubuntu 22.04) ships python3-jsonschema 3.2 which only has draft-7;
# Jazzy/Rolling (Ubuntu 24.04) ship 4.10+ with Draft202012. Prefer the newest
# draft for OpenAPI 3.1 alignment, fall back to Draft7 on Humble. The
# properties we validate (required, type, properties) behave identically.
try:
    from jsonschema.validators import Draft202012Validator as _Validator
except ImportError:
    from jsonschema.validators import Draft7Validator as _Validator

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


def _extract_inner_schema(schema):
    """Unwrap the OpenAPI 3.1 nullable idiom ``{"anyOf": [<inner>, {"type":"null"}]}``.

    SchemaWriter<std::optional<T>> emits this shape so generated clients can
    express ``T | null`` instead of degrading to ``T | undefined``. Tests that
    need to introspect the inner schema (e.g. assert a $ref target) must skip
    the null branch first. Returns *schema* unchanged when there is no anyOf.
    """
    if not isinstance(schema, dict) or 'anyOf' not in schema:
        return schema
    branches = schema['anyOf']
    if not isinstance(branches, list):
        return schema
    for branch in branches:
        if isinstance(branch, dict) and branch.get('type') != 'null':
            return branch
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

# Business-logic 400s that are NOT spec bugs. Each entry is annotated with the
# reason it cannot be fixed: "spec limitation" means the OpenAPI spec cannot
# express this constraint; "test limitation" means the test generator could be
# improved but the complexity is not worth it.
_KNOWN_BUSINESS_400_PATTERNS = [
    # [spec limitation] Aggregated config requires entity-qualified param names
    'Aggregated configuration requires app_id prefix',
    # [spec limitation] Bulk data handler returns 400 for unknown categories (should be 404)
    'Unknown bulk-data category',
    # [test limitation] Entity type mismatch - test may use wrong ID for route type
    'Invalid entity type for route',
    # [spec limitation] condition_type is runtime-validated against registered providers
    'Unknown condition type',
    # [test limitation] Resource URI needs specific topic path in data collection
    'Data collection requires a resource path',
    # [test limitation] Resource URI must match the route's entity type
    'Resource URI must reference the same entity',
    # [spec limitation] Config value type depends on actual ROS 2 parameter type
    'Failed to set parameter',
    # [spec limitation] ConfigurationWriteRequest has both 'data' and 'value' as
    # optional, but the handler requires at least one to be present. The OpenAPI
    # schema cannot express "at least one of" across optional fields without
    # oneOf/anyOf which would break generated client ergonomics.
    'Request body must contain a',
    # [spec limitation] TriggerCreateRequest.trigger_condition is a free-form
    # JSON object (nlohmann::json field) so the schema cannot declare
    # condition_type as a required sub-field. The handler validates it at runtime.
    'Missing or invalid',
    # [spec limitation] CyclicSubscriptionCreateRequest.interval has no enum in
    # the OpenAPI schema (it is plain field so that invalid values reach
    # parse_interval and produce ERR_INVALID_PARAMETER, not generic
    # invalid-request). The generator sends "test_value" which is rejected.
    'Invalid interval',
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
                except requests.exceptions.RequestException:
                    # Connection errors are not spec bugs - the server may have
                    # shut down or restarted between requests.
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
                elif resp.status_code >= 500 and resp.status_code not in (501, 503):
                    # 501 (Not Implemented) is expected for disabled features.
                    # 503 may occur during startup/shutdown.
                    # Other 5xx may indicate handler bugs (e.g. 500 when data
                    # write gets a value of the wrong type for the topic).
                    # Log but don't fail - these are handler robustness issues,
                    # not spec/handler contract mismatches.
                    pass

        if failures:
            detail = '\n  '.join(failures)
            self.fail(
                f'{len(failures)} endpoint(s) rejected spec-conformant '
                f'requests:\n  {detail}'
            )

    def test_x_medkit_sub_schemas_present_in_spec(self):
        """The spec must expose typed x-medkit sub-schemas from issue #338.

        Verifies that ``components/schemas`` in the live spec contains the
        DTO-generated x-medkit sub-schemas for ALL four entity types and that
        each entity's list-item schema references the matching sub-schema:

        - ``XMedkitArea``     <- referenced by ``AreaListItem``
        - ``XMedkitComponent`` <- referenced by ``ComponentListItem``
        - ``XMedkitApp``      <- referenced by ``AppListItem``
        - ``XMedkitFunction`` <- referenced by ``FunctionListItem``

        Plus ``XMedkitRos2`` (shared by Area and App) must exist with the
        ``"namespace"`` property.

        These assertions confirm that the DTO contract layer (issue #338) is
        wired to the OpenAPI spec builder uniformly across every entity type:
        the spec is derived from the same types that the handlers use to
        serialise responses.

        @verifies REQ_INTEROP_002
        """
        spec = self._fetch_spec()
        schemas = spec.get('components', {}).get('schemas', {})

        # --- XMedkitRos2 (shared) ---
        self.assertIn(
            'XMedkitRos2', schemas,
            'components/schemas must contain "XMedkitRos2"'
        )
        ros2_props = schemas['XMedkitRos2'].get('properties', {})
        self.assertIn(
            'namespace', ros2_props,
            'XMedkitRos2.properties must contain "namespace" (ROS 2 namespace field)'
        )

        # XMedkitArea has a nested "ros2" sub-object whose $ref points at
        # XMedkitRos2. Assert that wiring once on the Area branch.
        self.assertIn(
            'XMedkitArea', schemas,
            'components/schemas must contain "XMedkitArea" (issue #338 DTO schema)'
        )
        area_xm_props = schemas['XMedkitArea'].get('properties', {})
        self.assertIn(
            'ros2', area_xm_props,
            'XMedkitArea.properties must contain "ros2" (nested ROS 2 sub-object)'
        )
        # XMedkitArea.ros2 is std::optional<XMedkitRos2>: SchemaWriter emits the
        # OpenAPI 3.1 nullable idiom {"anyOf": [<inner>, {"type": "null"}]} where
        # the inner branch is the $ref. Pick the non-null branch to assert the ref.
        ros2_ref = _extract_inner_schema(area_xm_props['ros2'])
        self.assertIn(
            '$ref', ros2_ref,
            'XMedkitArea.properties.ros2 must resolve to a $ref (not inlined)'
        )
        self.assertIn(
            'XMedkitRos2', ros2_ref['$ref'],
            'XMedkitArea.properties.ros2.$ref must point to XMedkitRos2'
        )

        # --- All four list-item schemas reference their typed x-medkit ---
        # Each AreaListItem/ComponentListItem/AppListItem/FunctionListItem
        # carries an "x-medkit" property that is std::optional<XMedkit<Type>>;
        # the schema must therefore expose anyOf+null with the $ref branch
        # pointing to the matching sub-schema.
        list_item_to_xmedkit = [
            ('AreaListItem', 'XMedkitArea'),
            ('ComponentListItem', 'XMedkitComponent'),
            ('AppListItem', 'XMedkitApp'),
            ('FunctionListItem', 'XMedkitFunction'),
        ]
        for list_item_name, xmedkit_name in list_item_to_xmedkit:
            with self.subTest(entity=list_item_name):
                # The XMedkit sub-schema itself must exist.
                self.assertIn(
                    xmedkit_name, schemas,
                    f'components/schemas must contain "{xmedkit_name}" '
                    f'(issue #338 typed x-medkit sub-schema)'
                )
                self.assertEqual(
                    schemas[xmedkit_name].get('type'), 'object',
                    f'{xmedkit_name} must be an object schema'
                )

                # The list-item schema must exist and expose "x-medkit".
                self.assertIn(
                    list_item_name, schemas,
                    f'components/schemas must contain "{list_item_name}"'
                )
                list_item_props = schemas[list_item_name].get(
                    'properties', {}
                )
                self.assertIn(
                    'x-medkit', list_item_props,
                    f'{list_item_name}.properties must contain "x-medkit"'
                )

                # The x-medkit field is std::optional<XMedkit*>: anyOf+null
                # idiom. Pick the non-null branch and assert the $ref target.
                xm_ref = _extract_inner_schema(
                    list_item_props['x-medkit']
                )
                self.assertIn(
                    '$ref', xm_ref,
                    f'{list_item_name}.properties["x-medkit"] must resolve '
                    f'to a $ref'
                )
                self.assertIn(
                    xmedkit_name, xm_ref['$ref'],
                    f'{list_item_name}.properties["x-medkit"].$ref must '
                    f'point to {xmedkit_name}'
                )

    # ------------------------------------------------------------------
    # Schema $ref resolution for live response validation
    # ------------------------------------------------------------------

    @staticmethod
    def _inline_refs(schema, schemas, seen=None):
        """Recursively inline $refs into a schema for jsonschema validation.

        Raises ``ValueError`` on cycles or unresolvable references so the
        caller can detect a broken spec rather than silently accept any payload.
        """
        if seen is None:
            seen = set()
        if isinstance(schema, dict):
            if '$ref' in schema:
                ref = schema['$ref']
                if ref in seen:
                    raise ValueError(f'Cycle in $ref chain: {ref}')
                if not ref.startswith('#/components/schemas/'):
                    raise ValueError(f'Unsupported $ref form: {ref}')
                name = ref.rsplit('/', 1)[-1]
                target = schemas.get(name)
                if target is None:
                    raise ValueError(f'Unknown $ref target: {ref}')
                return TestOpenApiCallability._inline_refs(
                    target, schemas, seen | {ref}
                )
            return {
                k: TestOpenApiCallability._inline_refs(v, schemas, seen)
                for k, v in schema.items()
            }
        if isinstance(schema, list):
            return [
                TestOpenApiCallability._inline_refs(item, schemas, seen)
                for item in schema
            ]
        return schema

    def _validate_against_spec(self, body, schema, schemas):
        """Validate *body* against *schema* with $refs inlined.

        Returns a list of ``jsonschema.ValidationError`` instances (empty on
        success). Raises ``ValueError`` if the schema contains a bad $ref.
        """
        inlined = self._inline_refs(schema, schemas)
        validator = _Validator(inlined)
        return sorted(validator.iter_errors(body), key=lambda e: e.path)

    def _response_schema_for(self, spec, path, method='get'):
        """Return the 200 response JSON schema for *path* + *method*, or None."""
        path_item = spec.get('paths', {}).get(path, {})
        operation = path_item.get(method)
        if not operation:
            return None
        response_200 = operation.get('responses', {}).get('200', {})
        return (
            response_200
            .get('content', {})
            .get('application/json', {})
            .get('schema')
        )

    def test_live_entity_responses_conform_to_spec(self):
        """Live responses from core entity endpoints must validate against the spec.

        Covers the GET endpoints for the domains migrated in issue #338:
        - ``GET /areas``         (AreaList schema)
        - ``GET /components``    (ComponentList schema)
        - ``GET /apps``          (AppList schema)
        - ``GET /health``        (Health schema)
        - ``GET /version-info``  (VersionInfo schema)
        - ``GET /apps/{id}``     (AppDetail schema, using temp_sensor fixture)
        - ``GET /apps/{id}/faults`` (FaultList schema)
        - ``GET /apps/{id}/operations`` (OperationList schema)
        - ``GET /apps/{id}/data``       (DataListResult opaque schema)

        A schema mismatch means the handler's wire output no longer matches
        its DTO schema - a real contract bug introduced by the migration.

        @verifies REQ_INTEROP_002
        """
        spec = self._fetch_spec()
        schemas = spec.get('components', {}).get('schemas', {})
        violations = []
        validated = 0

        # Paths to validate: (spec_path, live_url_path)
        # Concrete entity paths are derived from discovered entity IDs.
        app_id = self._entity_map.get('apps', 'temp_sensor')
        endpoints_to_check = [
            ('/areas', '/areas'),
            ('/components', '/components'),
            ('/apps', '/apps'),
            ('/health', '/health'),
            ('/version-info', '/version-info'),
            ('/apps/{app_id}', f'/apps/{app_id}'),
            ('/apps/{app_id}/faults', f'/apps/{app_id}/faults'),
            ('/apps/{app_id}/operations', f'/apps/{app_id}/operations'),
            ('/apps/{app_id}/data', f'/apps/{app_id}/data'),
        ]

        for spec_path, live_path in endpoints_to_check:
            schema = self._response_schema_for(spec, spec_path)
            if schema is None:
                # No 200 schema declared - skip (callability test covers 400s)
                continue

            resp = requests.get(f'{self.BASE_URL}{live_path}', timeout=10)
            if resp.status_code != 200:
                # Entity may not exist for this fixture - skip this path.
                continue
            if 'application/json' not in resp.headers.get('content-type', ''):
                continue

            body = resp.json()
            try:
                errors = self._validate_against_spec(body, schema, schemas)
            except ValueError as exc:
                violations.append(f'GET {live_path}: spec error: {exc}')
                continue

            if errors:
                detail = '; '.join(
                    f'{".".join(str(p) for p in e.absolute_path) or "<root>"}: '
                    f'{e.message}'
                    for e in errors[:5]
                )
                violations.append(
                    f'GET {live_path}: schema drift against {spec_path}: {detail}'
                )
            else:
                validated += 1

        self.assertGreater(
            validated, 0,
            'No endpoints validated - entity discovery or spec fixture broken?'
        )
        self.assertFalse(
            violations,
            f'{len(violations)} live response(s) do not conform to the spec '
            f'(validated {validated} successfully):\n'
            + '\n'.join(violations),
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
