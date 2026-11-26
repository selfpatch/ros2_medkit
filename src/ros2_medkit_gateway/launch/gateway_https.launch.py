# Copyright 2025 bburda
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

"""
Launch file for ROS 2 Medkit Gateway with HTTPS/TLS enabled.

This launch file automatically generates self-signed development certificates
and starts the gateway with TLS enabled. For production deployments, use
proper CA-signed certificates and the standard gateway.launch.py.

Usage:
    ros2 launch ros2_medkit_gateway gateway_https.launch.py

    # With custom port:
    ros2 launch ros2_medkit_gateway gateway_https.launch.py server_port:=8443

    # Persist certificates to custom directory:
    ros2 launch ros2_medkit_gateway gateway_https.launch.py cert_dir:=/path/to/certs

WARNING: Generated certificates are for DEVELOPMENT ONLY.
         Do NOT use in production environments.
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

NODE_NAME = 'ros2_medkit_gateway'

# Default web UI origins enabled when the user does not override CORS, so the
# bundled web UI works over HTTPS out of the box (kept in sync with
# gateway.launch.py; a wildcard is deliberately not used).
CORS_DEFAULT = 'http://localhost:3000,http://localhost:5173'


def cors_override(cors_arg, config_file, default_config):
    """
    Return the ``cors.allowed_origins`` entry for the final overrides, or {}.

    The final overrides dict wins per key, so to avoid silently overriding a
    user's ``config_file``: an explicit ``cors_allowed_origins`` arg always wins
    (empty -> [''], CORS off); with no arg and a custom config file, inject
    nothing so the file's value is respected; with no arg and the default config,
    apply the web UI origins so the bundled UI works out of the box.
    """
    if cors_arg.strip() != CORS_DEFAULT:
        origins = [o.strip() for o in cors_arg.split(',') if o.strip()] or ['']
        return {'cors.allowed_origins': origins}
    if not config_file or config_file == default_config:
        return {'cors.allowed_origins': CORS_DEFAULT.split(',')}
    return {}


def _flatten(params, prefix=''):
    """Return nested parameters as {dotted name: value}."""
    flat = {}
    for key, value in params.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(_flatten(value, f'{name}.'))
        else:
            flat[name] = value
    return flat


def config_value(config_files, dotted_key):
    """
    Return the value the params files give ``dotted_key`` on this node, or None.

    As rcl does: one table of node sections in first-seen order, a later file updating the
    section it names, and the sections that match the node applied in that order.
    """
    sections = {}
    for config_file in config_files:
        if not config_file or not os.path.isfile(config_file):
            continue
        with open(config_file, encoding='utf-8') as handle:
            loaded = yaml.safe_load(handle) or {}
        for node, section in loaded.items():
            if isinstance(section, dict) and isinstance(section.get('ros__parameters'), dict):
                name = '/' + str(node).lstrip('/')
                sections.setdefault(name, {}).update(_flatten(section['ros__parameters']))
    value = None
    for name, params in sections.items():
        if name in ('/**', '/*', f'/{NODE_NAME}') and dotted_key in params:
            value = params[dotted_key]
    return value


def _same_file(path, other):
    """Return True when both name one file: one inode where both exist, else one real path."""
    if os.path.exists(path) and os.path.exists(other):
        return os.path.samefile(path, other)
    return os.path.realpath(path) == os.path.realpath(other)


def generate_certificates(cert_dir: str, configured_ca: str = '') -> dict:
    """
    Generate self-signed certificates for TLS/HTTPS.

    Parameters
    ----------
    cert_dir : str
        Directory to store generated certificates
    configured_ca : str
        The ``server.tls.ca_file`` the params files name, never written over

    Returns
    -------
    dict
        Paths to the certificate files, and ``ca_created``: whether the CA was made now

    """
    cert_file = os.path.join(cert_dir, 'cert.pem')
    key_file = os.path.join(cert_dir, 'key.pem')
    ca_file = os.path.join(cert_dir, 'ca.pem')

    # Skip if certificates already exist
    if os.path.exists(cert_file) and os.path.exists(key_file):
        return {
            'cert_file': cert_file,
            'key_file': key_file,
            'ca_file': ca_file if os.path.exists(ca_file) else '',
            'ca_created': False,
        }

    ca_key_file = os.path.join(cert_dir, 'ca_key.pem')
    csr_file = os.path.join(cert_dir, 'server.csr')
    ext_file = os.path.join(cert_dir, 'ext.cnf')
    serial_file = os.path.join(cert_dir, 'ca.srl')

    # Making the server pair writes or removes each of these.
    made = (ca_key_file, ca_file, key_file, csr_file, ext_file, cert_file, serial_file)
    hit = next((path for path in made if configured_ca and _same_file(configured_ca, path)), None)
    if hit:
        raise RuntimeError(
            f'server.tls.ca_file {configured_ca} is the file {hit}, and cert_dir {cert_dir} '
            'has no cert.pem and key.pem; making them would write over it. Give cert_dir a '
            'server pair, or name a CA file the launch does not make.')

    os.makedirs(cert_dir, exist_ok=True)

    # Generate CA key and certificate
    subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', ca_key_file,
        '-out', ca_file,
        '-days', '30', '-nodes',
        '-subj', '/C=US/ST=Dev/L=Local/O=ROS2Medkit/CN=DevCA'
    ], check=True, capture_output=True)

    # Generate server key
    subprocess.run([
        'openssl', 'genrsa', '-out', key_file, '2048'
    ], check=True, capture_output=True)

    # Generate CSR
    subprocess.run([
        'openssl', 'req', '-new', '-key', key_file, '-out', csr_file,
        '-subj', '/C=US/ST=Dev/L=Local/O=ROS2Medkit/CN=localhost'
    ], check=True, capture_output=True)

    # Create extension file for SAN
    with open(ext_file, 'w') as f:
        f.write("""basicConstraints=CA:FALSE
keyUsage = digitalSignature, keyEncipherment
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
IP.2 = ::1
""")

    # Sign certificate with CA
    subprocess.run([
        'openssl', 'x509', '-req', '-in', csr_file,
        '-CA', ca_file,
        '-CAkey', ca_key_file,
        '-CAcreateserial',
        '-out', cert_file,
        '-days', '30',
        '-extfile', ext_file
    ], check=True, capture_output=True)

    # Cleanup temporary files
    for tmp_file in [csr_file, ext_file, serial_file]:
        if os.path.exists(tmp_file):
            os.remove(tmp_file)

    # Set secure permissions on private keys
    os.chmod(key_file, 0o600)
    os.chmod(ca_key_file, 0o600)

    return {
        'cert_file': cert_file,
        'key_file': key_file,
        'ca_file': ca_file,
        'ca_created': True,
    }


def signed_by(cert_file, ca_file):
    """Return True when ``openssl verify`` accepts cert_file against ca_file."""
    result = subprocess.run(['openssl', 'verify', '-CAfile', ca_file, cert_file],
                            capture_output=True, check=False)
    return result.returncode == 0


def key_matches(cert_file, key_file):
    """Return True when key_file is the private key of cert_file."""
    cert_key = subprocess.run(['openssl', 'x509', '-in', cert_file, '-noout', '-pubkey'],
                              capture_output=True, check=False)
    key = subprocess.run(['openssl', 'pkey', '-in', key_file, '-pubout'],
                         capture_output=True, check=False)
    return cert_key.returncode == 0 and key.returncode == 0 and cert_key.stdout == key.stdout


def ensure_client_certificate(cert_dir, ca_created):
    """Return client.pem and client-key.pem in cert_dir, made again unless a pair its CA signed."""
    client_cert = os.path.join(cert_dir, 'client.pem')
    client_key = os.path.join(cert_dir, 'client-key.pem')
    ca_file = os.path.join(cert_dir, 'ca.pem')
    if (not ca_created and os.path.exists(client_cert) and os.path.exists(client_key)
            and signed_by(client_cert, ca_file) and key_matches(client_cert, client_key)):
        return client_cert, client_key
    ca_key_file = os.path.join(cert_dir, 'ca_key.pem')
    if not os.path.exists(ca_key_file):
        raise RuntimeError(
            f'require_client_cert needs the CA key {ca_key_file} to sign a client certificate')
    csr_file = os.path.join(cert_dir, 'client.csr')
    ext_file = os.path.join(cert_dir, 'client.ext')
    with open(ext_file, 'w') as f:
        f.write('basicConstraints=CA:FALSE\n'
                'keyUsage=digitalSignature\n'
                'extendedKeyUsage=clientAuth\n')
    subprocess.run([
        'openssl', 'req', '-newkey', 'rsa:2048', '-nodes',
        '-keyout', client_key, '-out', csr_file,
        '-subj', '/C=US/ST=Dev/L=Local/O=ROS2Medkit/CN=dev-client'
    ], check=True, capture_output=True)
    subprocess.run([
        'openssl', 'x509', '-req', '-in', csr_file,
        '-CA', ca_file, '-CAkey', ca_key_file, '-CAcreateserial',
        '-out', client_cert, '-days', '30', '-extfile', ext_file
    ], check=True, capture_output=True)
    for tmp_file in [csr_file, ext_file, os.path.join(cert_dir, 'ca.srl')]:
        if os.path.exists(tmp_file):
            os.remove(tmp_file)
    os.chmod(client_key, 0o600)
    return client_cert, client_key


def curl_hints(cert_paths, client_paths, client_ca, server_host, server_port):
    """Return the LogInfo lines that show how to call the gateway."""
    url = f'https://{server_host}:{server_port}/api/v1/health'
    if client_paths:
        return [
            LogInfo(msg=['Client certificates are required. Test with curl:']),
            LogInfo(msg=[f'  curl --cacert {cert_paths["ca_file"]} '
                         f'--cert {client_paths[0]} --key {client_paths[1]} {url}']),
        ]
    if client_ca:
        return [
            LogInfo(msg=[f'Client certificates are required, signed by {client_ca}. '
                         'Test with curl and such a certificate:']),
            LogInfo(msg=[f'  curl --cacert {cert_paths["ca_file"]} '
                         f'--cert <client.pem> --key <client-key.pem> {url}']),
        ]
    return [
        LogInfo(msg=['Test with curl:']),
        LogInfo(msg=[f'  curl -k {url}']),
        LogInfo(msg=['']),
        LogInfo(msg=['Test with CA verification:']),
        LogInfo(msg=[f'  curl --cacert {cert_paths["ca_file"]} {url}']),
    ]


def launch_setup(context):
    """Set up launch with generated certificates."""
    # Get launch configuration values
    cert_dir = LaunchConfiguration('cert_dir').perform(context)
    server_host = LaunchConfiguration('server_host').perform(context)
    server_port = LaunchConfiguration('server_port').perform(context)
    refresh_interval = LaunchConfiguration('refresh_interval_ms').perform(context)
    min_tls_version = LaunchConfiguration('min_tls_version').perform(context)
    require_client_cert = LaunchConfiguration('require_client_cert').perform(context)
    override_config = LaunchConfiguration('config_file').perform(context)
    cors_arg = LaunchConfiguration('cors_allowed_origins').perform(context)
    allow_remote = LaunchConfiguration('allow_unauthenticated_remote').perform(context)
    # Empty leaves the config_file in charge; true and false both override it.
    allow_remote = allow_remote.strip().lower()
    remote_override = ({'server.allow_unauthenticated_remote': allow_remote == 'true'}
                       if allow_remote in ('true', 'false') else {})

    # Only a `true` is passed on: a `false` default must not override a config_file.
    mtls_override = ({'server.tls.require_client_cert': True}
                     if require_client_cert.strip().lower() == 'true' else {})

    # Use temp directory if not specified
    if not cert_dir:
        cert_dir = os.path.join(tempfile.gettempdir(), 'ros2_medkit_certs')

    # Get package directory for base config
    pkg_dir = get_package_share_directory('ros2_medkit_gateway')
    default_config = os.path.join(pkg_dir, 'config', 'gateway_params.yaml')

    # The params files can turn mutual TLS on and name the CA; the launch reads both.
    config_files = [default_config, override_config]
    mtls = bool(mtls_override) or config_value(
        config_files, 'server.tls.require_client_cert') is True
    config_ca = config_value(config_files, 'server.tls.ca_file') or ''

    # Generate certificates
    cert_paths = generate_certificates(cert_dir, config_ca)
    # A CA the params files name is external, wherever it is: the dev CA is used only without one.
    uses_dev_ca = not config_ca
    client_paths = (ensure_client_certificate(cert_dir, cert_paths['ca_created'])
                    if mtls and uses_dev_ca else None)
    client_ca = config_ca if mtls and not uses_dev_ca else ''
    ca_override = ({'server.tls.ca_file': cert_paths['ca_file']}
                   if mtls and not config_ca else {})

    # Protocol info for logging
    protocol = 'HTTPS' if cert_paths['cert_file'] else 'HTTP'

    gateway = Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[
            default_config, override_config,
            {
                'server.host': server_host,
                'server.port': int(server_port),
                'server.tls.enabled': True,
                'server.tls.cert_file': cert_paths['cert_file'],
                'server.tls.key_file': cert_paths['key_file'],
                'server.tls.min_version': min_tls_version,
                **mtls_override,
                **ca_override,
                'refresh_interval_ms': int(refresh_interval),
                **remote_override,
                **cors_override(cors_arg, override_config, default_config),
            }
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    def _gateway_exited(event, context):
        # A stack whose gateway is gone serves nothing: end the launch, and fail
        # it when the gateway failed (a refused start included). An exit during
        # the launch's own shutdown is not a failure.
        if context.is_shutdown:
            return None
        if event.returncode:
            raise RuntimeError(f'gateway_node exited with code {event.returncode}')
        return [Shutdown(reason='gateway_node exited')]

    return [
        LogInfo(msg=['='*60]),
        LogInfo(msg=['ROS 2 Medkit Gateway - HTTPS/TLS Mode']),
        LogInfo(msg=['='*60]),
        LogInfo(msg=['WARNING: Using self-signed development certificates!']),
        LogInfo(msg=['         Do NOT use in production.']),
        LogInfo(msg=['']),
        LogInfo(msg=['Certificate directory: ', cert_dir]),
        LogInfo(msg=['Server URL: ', f'{protocol.lower()}://{server_host}:{server_port}']),
        LogInfo(msg=['']),
        *curl_hints(cert_paths, client_paths, client_ca, server_host, server_port),
        LogInfo(msg=['='*60]),

        gateway,
        RegisterEventHandler(OnProcessExit(target_action=gateway, on_exit=_gateway_exited)),
    ]


def generate_launch_description():
    """Generate launch description for HTTPS gateway."""
    return LaunchDescription([
        # Certificate directory (empty = use temp)
        DeclareLaunchArgument(
            'cert_dir',
            default_value='',
            description='Directory for certificates (empty = temp dir)'
        ),

        # Server configuration
        DeclareLaunchArgument(
            'server_host',
            default_value='127.0.0.1',
            description='Host to bind HTTPS server (127.0.0.1 or 0.0.0.0)'
        ),

        DeclareLaunchArgument(
            'allow_unauthenticated_remote',
            default_value='',
            description=(
                'Let the gateway bind a non-loopback server_host with authentication off. '
                'Without it such a gateway refuses to start. When set, true or false '
                'overrides the config file; empty leaves the file in charge.'
            )
        ),

        DeclareLaunchArgument(
            'server_port',
            default_value='8443',
            description='Port for HTTPS API (default 8443 to avoid conflict with HTTP)'
        ),

        DeclareLaunchArgument(
            'refresh_interval_ms',
            default_value='30000',
            description=(
                'Safety-backstop refresh interval in milliseconds. Primary '
                'refresh is graph-event driven; this only controls the '
                'periodic forced refresh. Must match the default in '
                'config/gateway_params.yaml.'
            )
        ),

        DeclareLaunchArgument(
            'min_tls_version',
            default_value='1.2',
            description='Minimum TLS version (1.2 or 1.3)'
        ),

        DeclareLaunchArgument(
            'require_client_cert',
            default_value='false',
            description=(
                'Require a client certificate (mutual TLS). A config_file can turn it on too. '
                'Without a server.tls.ca_file there, the generated CA signs the client '
                'certificates; the launch makes client.pem and client-key.pem in cert_dir.'
            )
        ),

        DeclareLaunchArgument(
            'config_file', default_value=os.path.join(
                get_package_share_directory('ros2_medkit_gateway'),
                'config', 'gateway_params.yaml'),
            description='Path to YAML config file to override gateway parameters. Default config '
                        'is the ros2_medkit_gateway/config/gateway_params.yaml.'
        ),

        DeclareLaunchArgument(
            'cors_allowed_origins',
            default_value=CORS_DEFAULT,
            description='Comma-separated CORS origins allowed to call the gateway from a browser, '
                        'so the web UI works over HTTPS out of the box. Pass an explicit value to '
                        'override (empty disables CORS); when left at the default, a config_file '
                        'that sets cors.allowed_origins is respected.'
        ),

        # Use OpaqueFunction to generate certs and set up node
        OpaqueFunction(function=launch_setup),
    ])
