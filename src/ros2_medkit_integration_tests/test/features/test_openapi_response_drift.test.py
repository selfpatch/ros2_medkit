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

"""OpenAPI response-schema drift test.

For every GET endpoint declared in the runtime OpenAPI spec served at
GET /docs, this test fetches a real response from the live gateway and
validates it against the response schema declared in the same spec for
status 200. The validator runs jsonschema in its default mode, so what
this catches is the *subset* of drift that breaks the schema's positive
constraints:

- ``required`` fields the handler does not emit
- type mismatches (string vs number, etc.)
- ``enum`` values the handler emits but the schema does not list
- malformed nested objects against declared sub-schemas

What this does **not** catch (because most schemas in
``schema_builder.cpp`` do not set ``additionalProperties: false``):

- handler emits an extra top-level field the schema never declares
  (this was exactly the failure mode that allowed flat
  ``x-medkit-phase`` to slip through before issue #385 was filed)

A closed-object check would surface dozens of pre-existing schema gaps
that are out of scope for #385; the structural fix (compile-time link
from each emitter to its schema) lives under issue #338. Until then,
the explicit ``test_update_status_payload_uses_nested_x_medkit`` guards
the specific regression #385 closes.

If this test fails, fix the schema or the handler; do not silence it.
"""

import os
import re
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing
import requests

# Humble (Ubuntu 22.04) ships python3-jsonschema 3.2 which only has draft-7;
# Jazzy/Rolling (Ubuntu 24.04) ship 4.10+ with Draft202012. Prefer the newest
# draft for OpenAPI 3.1 alignment, fall back to Draft7 on Humble. The properties
# we validate (required, type, properties) behave identically across drafts.
try:
    from jsonschema.validators import Draft202012Validator as _Validator
except ImportError:
    from jsonschema.validators import Draft7Validator as _Validator

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _test_update_backend_path():
    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_gateway', 'libtest_update_backend.so'
    )


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor', 'calibration'],
        gateway_params={
            'updates.enabled': True,
            'plugins': ['test_update_backend'],
            'plugins.test_update_backend.path': _test_update_backend_path(),
        },
        fault_manager=True,
    )


# Path placeholders -> entity collection used for substitution.
_ENTITY_PARAM_MAP = {
    'area_id': 'areas',
    'component_id': 'components',
    'app_id': 'apps',
    'function_id': 'functions',
}


class _RefResolutionError(Exception):
    """Raised when a $ref in the spec cannot be resolved or forms a cycle.

    Returning an empty schema on these conditions would let jsonschema
    validate any payload, masking the drift the test is meant to detect.
    """


def _inline_refs(schema, schemas, seen=None):
    """Recursively inline $refs into a schema so jsonschema can validate it.

    The runtime spec uses $refs that point at #/components/schemas/, but
    Draft202012Validator constructed without a registry will not follow
    them. Cycles and unresolved refs raise ``_RefResolutionError`` rather
    than silently degrading the validator to "anything goes".
    """
    if seen is None:
        seen = set()
    if isinstance(schema, dict):
        if '$ref' in schema:
            ref = schema['$ref']
            if ref in seen:
                raise _RefResolutionError(
                    f'Cycle in $ref chain: {" -> ".join(sorted(seen))} -> {ref}'
                )
            if not ref.startswith('#/components/schemas/'):
                raise _RefResolutionError(f'Unsupported $ref form: {ref}')
            name = ref.rsplit('/', 1)[-1]
            target = schemas.get(name)
            if target is None:
                raise _RefResolutionError(f'Unknown $ref target: {ref}')
            return _inline_refs(target, schemas, seen | {ref})
        return {k: _inline_refs(v, schemas, seen) for k, v in schema.items()}
    if isinstance(schema, list):
        return [_inline_refs(item, schemas, seen) for item in schema]
    return schema


def _is_sse_endpoint(operation):
    """Detect SSE streaming endpoints that would hang on a plain GET."""
    summary = (operation.get('summary') or '') + (operation.get('description') or '')
    return 'SSE' in summary or 'stream' in summary.lower()


def _substitute_path_params(path, entity_map, resource_id='test_id'):
    def replacer(match):
        param = match.group(1)
        if param in _ENTITY_PARAM_MAP:
            return entity_map.get(_ENTITY_PARAM_MAP[param], 'test_entity')
        return resource_id

    return re.sub(r'\{([^}]+)\}', replacer, path)


class TestOpenApiResponseDrift(GatewayTestCase):
    """Catches handler-vs-spec drift on GET response bodies."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'calibration'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()

        # Discover real entity IDs to use in path substitution.
        cls._entity_map = {}
        for etype in ('areas', 'components', 'apps', 'functions'):
            resp = requests.get(f'{cls.BASE_URL}/{etype}', timeout=5)
            resp.raise_for_status()
            items = resp.json().get('items', [])
            if items:
                cls._entity_map[etype] = items[0].get('id')

        # Register and prepare a test update package so /updates/{id}/status
        # returns an interesting payload (status + x-medkit.phase).
        cls._update_pkg_id = 'drift-test-pkg'
        # Best-effort cleanup of any leftover from a previous interrupted run.
        requests.delete(
            f'{cls.BASE_URL}/updates/{cls._update_pkg_id}', timeout=5
        )
        post = requests.post(
            f'{cls.BASE_URL}/updates',
            json={
                'id': cls._update_pkg_id,
                'update_name': 'Drift test package',
                'automated': False,
                'origins': ['proximity'],
            },
            timeout=5,
        )
        post.raise_for_status()
        prepare = requests.put(
            f'{cls.BASE_URL}/updates/{cls._update_pkg_id}/prepare', timeout=5
        )
        prepare.raise_for_status()

    @classmethod
    def tearDownClass(cls):
        requests.delete(
            f'{cls.BASE_URL}/updates/{cls._update_pkg_id}', timeout=5
        )
        super().tearDownClass()

    def _fetch_spec(self):
        return self.poll_endpoint_until(
            '/docs', lambda d: d if d.get('paths') else None
        )

    def _validate_response(self, schema, body, schemas):
        """Validate body against schema with $refs inlined."""
        inlined = _inline_refs(schema, schemas)
        validator = _Validator(inlined)
        return sorted(validator.iter_errors(body), key=lambda e: e.path)

    def _resource_for_path(self, path):
        """Pick a resource_id that makes a given path return 200 when possible.

        Used for paths like /updates/{id}/status where 'test_id' would 404.
        """
        if path.startswith('/updates/{'):
            return self._update_pkg_id
        return 'test_id'

    def test_get_responses_match_declared_schema(self):
        """Every 200 response must validate against its declared schema.

        Iterates GET endpoints from /docs, fetches each, and validates
        bodies against the response schema declared for status 200. Schema
        drift (handler emits a field schema does not declare as required,
        or vice versa) raises an error.

        @verifies REQ_INTEROP_002
        """
        spec = self._fetch_spec()
        schemas = spec.get('components', {}).get('schemas', {})
        violations = []
        validated = 0

        for path, path_item in spec.get('paths', {}).items():
            operation = path_item.get('get')
            if not operation or _is_sse_endpoint(operation):
                continue

            response_def = operation.get('responses', {}).get('200')
            if not response_def:
                continue
            schema = (
                response_def.get('content', {})
                .get('application/json', {})
                .get('schema')
            )
            if not schema:
                continue

            uri = _substitute_path_params(
                path, self._entity_map, self._resource_for_path(path)
            )
            resp = requests.get(f'{self.BASE_URL}{uri}', timeout=10)

            # Non-200 responses fall outside this drift check (handler may
            # legitimately return 404/501 for the chosen path params); the
            # callability test covers status code shape separately.
            if resp.status_code != 200:
                continue
            if 'application/json' not in resp.headers.get('content-type', ''):
                continue

            body = resp.json()
            try:
                errors = self._validate_response(schema, body, schemas)
            except _RefResolutionError as exc:
                violations.append(f'GET {uri}: spec error: {exc}')
                continue
            if errors:
                detail = '; '.join(
                    f'{".".join(str(p) for p in e.absolute_path) or "<root>"}: {e.message}'
                    for e in errors[:5]
                )
                violations.append(f'GET {uri}: schema drift: {detail}')
            else:
                validated += 1

        self.assertGreater(
            validated, 0, 'No endpoints validated - test fixture broken?'
        )
        self.assertFalse(
            violations,
            f'{len(violations)} drift violation(s); validated {validated}:\n'
            + '\n'.join(violations),
        )

    def test_update_status_payload_uses_nested_x_medkit(self):
        """Specific guard for issue #385: /updates/{id}/status payload.

        The handler must emit ``x-medkit: {phase: ...}`` (nested object), not
        the legacy flat ``x-medkit-phase`` key. The drift test above already
        catches this via the schema declared in update_status_schema(); this
        explicit assertion makes the regression intent obvious.

        @verifies REQ_INTEROP_002
        """
        resp = requests.get(
            f'{self.BASE_URL}/updates/{self._update_pkg_id}/status', timeout=5
        )
        self.assertEqual(resp.status_code, 200)
        body = resp.json()
        self.assertNotIn('x-medkit-phase', body, 'Legacy flat key still emitted')
        self.assertIn('x-medkit', body)
        self.assertIn('phase', body['x-medkit'])

    def test_configurations_payload_uses_nested_x_medkit(self):
        """Specific guard for issue #385: configurations vendor fields nested.

        Items at the top level (only in aggregated mode) and parameters
        inside ``x-medkit.parameters[]`` (always emitted) must carry vendor
        source attribution under a nested ``x-medkit`` object, never as flat
        ``x-medkit-source`` / ``x-medkit-node`` keys. The drift test cannot
        catch reintroduction of the legacy flat keys because
        ``ConfigurationMetaData`` does not set ``additionalProperties: false``.

        ``temp_sensor`` declares four ROS 2 parameters in the test fixture,
        which exercises the per-parameter emit path that #385 migrated.

        @verifies REQ_INTEROP_002
        """
        resp = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations', timeout=10
        )
        self.assertEqual(resp.status_code, 200)
        body = resp.json()

        items = body.get('items', [])
        self.assertGreater(
            len(items), 0,
            'Fixture broken: temp_sensor should declare ROS 2 parameters'
        )
        for item in items:
            self.assertNotIn(
                'x-medkit-source', item, f'Legacy flat key on item: {item}'
            )
            self.assertNotIn(
                'x-medkit-node', item, f'Legacy flat key on item: {item}'
            )

        parameters = body.get('x-medkit', {}).get('parameters', [])
        self.assertGreater(len(parameters), 0)
        for param in parameters:
            self.assertNotIn(
                'x-medkit-source', param, f'Legacy flat key on parameter: {param}'
            )
            self.assertNotIn(
                'x-medkit-node', param, f'Legacy flat key on parameter: {param}'
            )
            self.assertIn('x-medkit', param)
            self.assertEqual(param['x-medkit'].get('source'), 'temp_sensor')
            self.assertIn('node', param['x-medkit'])


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with code {info.returncode}',
            )
