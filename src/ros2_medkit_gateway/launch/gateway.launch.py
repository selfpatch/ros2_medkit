# Copyright 2025 mfaferek93
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

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Default web UI origins enabled when the user does not override CORS, so the
# bundled web UI works out of the box. A wildcard is deliberately not used.
CORS_DEFAULT = 'http://localhost:3000,http://localhost:5173'


def parse_security_flag(name, raw):
    """
    Return True/False for a security launch argument, or raise on anything else.

    An allowlist for true with everything else meaning false is the wrong shape
    here: ``tls_enabled:=on`` and ``tls_enabled:=ture`` would both mean "serve
    plain HTTP", and the override is still written, so the typo beats a config
    file that had TLS on. For a flag whose two values are "encrypted" and "not",
    an unrecognised spelling has to stop the launch, because picking one for
    the operator is how a gateway ends up open.
    """
    value = raw.strip().lower()
    if value in ('true', '1', 'yes', 'on'):
        return True
    if value in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError(
        f'{name}:={raw!r} is not a boolean. Use true or false. '
        f'Leaving {name} unset lets the config file decide.'
    )


def config_says(config_file, *path, default=False):
    """
    Return a boolean from the config file at ``path``, or ``default``.

    Which profile is in force is a property of the file, not of this launch
    file: ``config/gateway_params.yaml`` leaves auth and TLS off and
    ``config/gateway_params.secure.yaml`` turns both on, and either can be
    named through ``config_file``. Assuming one of them here is how a warning
    ends up firing on a launch that is about to work perfectly, or staying
    silent on one that is about to be refused.

    Anything this cannot read - a missing file, invalid YAML, or a document
    that is not a mapping of node names - reads as ``default`` and says so,
    because "cannot tell" is what it is. This decides whether to PRINT a
    warning; the gateway parses the same file a moment later and is the
    authority on what it contains.
    """
    try:
        with open(config_file, encoding='utf-8') as handle:
            document = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return default
    if not isinstance(document, dict):
        print(f'[gateway.launch.py] {config_file} is not a mapping of node names, so '
              f'the auth and TLS settings in it cannot be read here. The gateway still '
              f'reads the file itself; only the advice printed below is affected.')
        return default
    for node in document.values():
        section = node.get('ros__parameters') if isinstance(node, dict) else None
        for key in path:
            if not isinstance(section, dict):
                section = None
                break
            section = section.get(key)
        if isinstance(section, bool):
            return section
    return default


def config_text(config_file, *path):
    """
    Return a string value from the config file at ``path``, or ``''``.

    The string twin of :func:`config_says`, with the same "cannot tell reads as
    the default" rule. Used to notice that a config file carries a
    ``jwt_secret``, so the advice below stays quiet on a launch that is about to
    work.
    """
    try:
        with open(config_file, encoding='utf-8') as handle:
            document = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError):
        return ''
    if not isinstance(document, dict):
        return ''
    for node in document.values():
        section = node.get('ros__parameters') if isinstance(node, dict) else None
        for key in path:
            if not isinstance(section, dict):
                section = None
                break
            section = section.get(key)
        if isinstance(section, str) and section:
            return section
    return ''


def cors_override(cors_arg, config_file, default_config):
    """
    Return the ``cors.allowed_origins`` entry for the final overrides, or {}.

    The final overrides dict is applied after the config file, so anything in it
    wins per key. To avoid silently overriding a user's ``config_file``:

    - an explicit ``cors_allowed_origins`` arg always wins (empty -> [''], which
      the gateway reads as CORS off);
    - with no arg and a *custom* config file, inject nothing so the file's
      ``cors.allowed_origins`` is respected;
    - with no arg and the default config, apply the web UI origins so the bundled
      UI works out of the box.

    An empty list cannot be passed as a launch parameter (and is the untyped-empty
    shape that aborts startup), so the off case uses [''] - the same placeholder
    the gateway config ships; config.cpp filters the empty string.
    """
    if cors_arg.strip() != CORS_DEFAULT:
        origins = [o.strip() for o in cors_arg.split(',') if o.strip()] or ['']
        return {'cors.allowed_origins': origins}
    if not config_file or config_file == default_config:
        return {'cors.allowed_origins': CORS_DEFAULT.split(',')}
    return {}


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_medkit_gateway')
    default_config = os.path.join(pkg_dir, 'config', 'gateway_params.yaml')

    # Resolve graph provider plugin path (optional - gateway starts without it)
    graph_provider_path = ''
    try:
        graph_provider_prefix = get_package_prefix('ros2_medkit_graph_provider')
        graph_provider_path = os.path.join(
            graph_provider_prefix, 'lib', 'ros2_medkit_graph_provider',
            'libros2_medkit_graph_provider_plugin.so')
    except PackageNotFoundError:
        print('[gateway.launch.py] ros2_medkit_graph_provider not installed '
              '- graph endpoints will not be available')

    if graph_provider_path and not os.path.isfile(graph_provider_path):
        print('[gateway.launch.py] WARNING: graph provider .so not found at '
              f'{graph_provider_path} - plugin will not load')
        graph_provider_path = ''

    declare_override_config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to YAML config file to override gateway parameters. Default config '
                    'is the ros2_medkit_gateway/config/gateway_params.yaml, which leaves auth '
                    'and TLS off. Point this at config/gateway_params.secure.yaml in the same '
                    'directory for the closed profile: auth on, require_auth_for "all", TLS on, '
                    'rate limiting on.')

    declare_host_arg = DeclareLaunchArgument(
        'server_host', default_value='127.0.0.1',
        description='Host to bind REST server (127.0.0.1 or 0.0.0.0)')

    declare_port_arg = DeclareLaunchArgument(
        'server_port', default_value='8080',
        description='Port for REST API')

    declare_refresh_arg = DeclareLaunchArgument(
        'refresh_interval_ms', default_value='30000',
        description=(
            'Safety-backstop refresh interval in milliseconds. Primary '
            'refresh is graph-event driven (~100 ms latency); this only '
            'controls the periodic forced refresh. Must match the default '
            'in config/gateway_params.yaml.'))

    declare_jwt_secret_arg = DeclareLaunchArgument(
        'jwt_secret', default_value='',
        description=(
            'HS256 signing secret, at least 32 characters. Required whenever '
            'auth is on - the default config leaves it off, the secure '
            'profile turns it on, and the gateway refuses to start with auth '
            'on and no secret. Pass one here, or point config_file at a file '
            'that sets auth.jwt_secret and auth.clients.'))

    declare_auth_enabled_arg = DeclareLaunchArgument(
        'auth_enabled', default_value='',
        description=(
            'Require a credential. Unset means the config_file decides: the '
            'default config leaves auth off, config/gateway_params.secure.yaml '
            'turns it on. With auth off the entity tree, the fault history '
            'and every operation are readable by anyone who can reach the '
            'port.'))

    declare_clients_arg = DeclareLaunchArgument(
        'auth_clients', default_value='',
        description=(
            'Comma-separated "client_id:client_secret:role" triples '
            '(roles: viewer, operator, configurator, admin). Needed to obtain '
            'a token from /auth/token.'))

    declare_tls_enabled_arg = DeclareLaunchArgument(
        'tls_enabled', default_value='',
        description=(
            'Serve HTTPS. Unset means the config_file decides: the default '
            'config leaves TLS off, config/gateway_params.secure.yaml turns '
            'it on. With TLS on, cert_file and key_file are required - the '
            'gateway refuses to start without a certificate, because it will not fall '
            'back to plaintext.'))

    declare_cert_file_arg = DeclareLaunchArgument(
        'cert_file', default_value='',
        description=(
            'PEM certificate (or full chain) for HTTPS. REQUIRED while '
            'tls_enabled is true. For a first run, generate a self-signed '
            'pair with scripts/generate_dev_certs.sh - browsers will warn, '
            'which is correct for a certificate nothing has vouched for.'))

    declare_key_file_arg = DeclareLaunchArgument(
        'key_file', default_value='',
        description=(
            'PEM private key matching cert_file. REQUIRED while tls_enabled '
            'is true. Keep it chmod 600 and owned by the gateway user.'))

    declare_cors_arg = DeclareLaunchArgument(
        'cors_allowed_origins',
        default_value=CORS_DEFAULT,
        description='Comma-separated CORS origins allowed to call the REST API from a browser, so '
                    'the bundled web UI (a different origin) works out of the box. Pass an '
                    'explicit value to override (empty disables CORS); when left at the default, '
                    'a config_file that sets cors.allowed_origins is respected. A wildcard is '
                    'intentionally not the default: with auth off and write methods enabled it '
                    'would let any site drive cross-origin writes.')

    # The cors arg is resolved at launch time (a comma-separated LaunchConfiguration
    # cannot be passed straight through as a string-array parameter) and folded in
    # by cors_override, which keeps a config_file's CORS from being silently
    # overridden by the launch default.
    def _launch_setup(context, *_args, **_kwargs):
        param_overrides = {
            'server.host': LaunchConfiguration('server_host'),
            'server.port': LaunchConfiguration('server_port'),
            'refresh_interval_ms': LaunchConfiguration('refresh_interval_ms'),
        }
        if graph_provider_path:
            param_overrides['plugins'] = ['graph_provider']
            param_overrides['plugins.graph_provider.path'] = graph_provider_path
        param_overrides.update(cors_override(
            LaunchConfiguration('cors_allowed_origins').perform(context),
            LaunchConfiguration('config_file').perform(context), default_config))

        # Precedence: an explicit launch argument, then whatever the config
        # file says. Unset means "do not touch it", which matters because this
        # launch file is included by others and is used with config_file:
        # re-asserting a default here would silently override a value someone
        # put in their own file on purpose.
        tls_arg = LaunchConfiguration('tls_enabled').perform(context).strip()
        if tls_arg:
            tls_enabled = parse_security_flag('tls_enabled', tls_arg)
            param_overrides['server.tls.enabled'] = tls_enabled
        else:
            # Nothing said otherwise, so the config file decides - read it
            # and make no assumption about which profile is in force.
            tls_enabled = config_says(
                LaunchConfiguration('config_file').perform(context),
                'server', 'tls', 'enabled')
        cert_file = LaunchConfiguration('cert_file').perform(context)
        key_file = LaunchConfiguration('key_file').perform(context)
        if cert_file:
            param_overrides['server.tls.cert_file'] = cert_file
        if key_file:
            param_overrides['server.tls.key_file'] = key_file
        if tls_enabled and not (cert_file and key_file):
            # The gateway would refuse to start a moment from now, naming the
            # config file. Name the launch arguments instead, here, where they
            # are the thing the reader can actually change.
            print('[gateway.launch.py] TLS is enabled and cert_file/key_file were not both '
                  'given. Pass cert_file:=<path> key_file:=<path>, set them in a config_file, '
                  'or pass tls_enabled:=false to serve plain HTTP. '
                  'scripts/generate_dev_certs.sh makes a self-signed pair for a first run.')

        # Same precedence as TLS above: explicit argument, then the config file.
        #
        # MEDKIT_JWT_SECRET, MEDKIT_CLIENTS and MEDKIT_AUTH_DISABLED are NOT
        # read here. The gateway node reads them itself when it reads its
        # parameters, and that is the only copy of the rule - see
        # resolve_auth_environment. Applying it in a launch file as well would
        # cover only the launches that include this file, while leaving `ros2
        # run ros2_medkit_gateway gateway_node` and every other exec path on a
        # different rule; two copies of a posture rule is how they drift.
        # The variables reach the node through its environment, which it
        # inherits.
        config_file = LaunchConfiguration('config_file').perform(context)
        auth_arg = LaunchConfiguration('auth_enabled').perform(context).strip()
        jwt_secret = LaunchConfiguration('jwt_secret').perform(context)
        clients = LaunchConfiguration('auth_clients').perform(context)
        auth_disabled_env = os.environ.get('MEDKIT_AUTH_DISABLED') == '1'
        if auth_arg:
            auth_enabled = parse_security_flag('auth_enabled', auth_arg)
            param_overrides['auth.enabled'] = auth_enabled
            asked_for_auth_by = 'auth_enabled:=true was given'
        else:
            auth_enabled = config_says(config_file, 'auth', 'enabled')
            asked_for_auth_by = f'{config_file} turns authentication on'
        if auth_enabled and auth_disabled_env:
            # The node will refuse in a moment whichever way authentication was
            # asked for - the argument and the config file lose alike. Say so
            # here, where what the operator typed is on screen, so they see it
            # there and do not have to find one WARN among the startup log.
            print(f'[gateway.launch.py] {asked_for_auth_by} and MEDKIT_AUTH_DISABLED=1 is '
                  'in the environment, which wins: the gateway will start WITHOUT '
                  'authentication. Unset the variable to honour the configuration.')
        if jwt_secret:
            param_overrides['auth.jwt_secret'] = jwt_secret
        if clients:
            # Trimmed per entry, the same rule the gateway applies to
            # MEDKIT_CLIENTS: the separator is a comma and the space a person
            # writes after it belongs to the separator, while a space inside a
            # field belongs to that field.
            param_overrides['auth.clients'] = [
                entry for entry in (c.strip() for c in clients.split(',')) if entry]
        secret_available = (jwt_secret or os.environ.get('MEDKIT_JWT_SECRET')
                            or config_text(config_file, 'auth', 'jwt_secret'))
        if auth_enabled and not secret_available and not auth_disabled_env:
            # The gateway would refuse to start a moment from now with a
            # message about the config file. Say the actionable thing instead,
            # here, where the launch argument that fixes it is in scope.
            #
            # Silent in the two cases where nothing is wrong: the secret is
            # coming from somewhere this cannot see as an argument (the
            # environment, or the config file itself), or MEDKIT_AUTH_DISABLED=1
            # means authentication is not starting at all.
            print('[gateway.launch.py] auth is enabled and no jwt_secret was given. '
                  'Pass jwt_secret:=<at least 32 chars> and '
                  'auth_clients:=<id>:<secret>:admin, set them in a config_file, '
                  'or pass auth_enabled:=false to run without authentication.')
        return [Node(
            package='ros2_medkit_gateway',
            executable='gateway_node',
            name='ros2_medkit_gateway',
            output='screen',
            parameters=[default_config, LaunchConfiguration('config_file'), param_overrides],
            arguments=['--ros-args', '--log-level', 'info'])]

    return LaunchDescription([
        declare_override_config_arg,
        declare_host_arg,
        declare_port_arg,
        declare_refresh_arg,
        declare_auth_enabled_arg,
        declare_jwt_secret_arg,
        declare_clients_arg,
        declare_tls_enabled_arg,
        declare_cert_file_arg,
        declare_key_file_arg,
        declare_cors_arg,
        OpaqueFunction(function=_launch_setup),
    ])
