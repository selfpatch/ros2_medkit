#!/usr/bin/env bash
# Copyright 2026 mfaferek93
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
#
# Generate the throwaway OPC-UA application-instance certificates the secured
# A&C integration test (issue #477) needs. NOTHING is committed - the test
# regenerates these into a temp dir on every run.
#
# Produces, in the output directory:
#   server_key.pem  server_cert.der   URI SAN urn:test:alarms:server
#   client_key.pem  client_cert.der   URI SAN urn:selfpatch:medkit:opcua-client
#
# Certificates are DER (open62541 + the medkit client both auto-detect DER);
# keys are unencrypted RSA-2048 PEM. The server trust list gets the client
# cert; the gateway trust list gets the server cert. The URI SAN MUST equal the
# applicationUri each side advertises or open62541 rejects the channel with
# BadCertificateUriInvalid.
#
# Usage: gen_test_certs.sh <output-dir> [server-uri] [client-uri]

set -euo pipefail

OUT="${1:?usage: gen_test_certs.sh <output-dir> [server-uri] [client-uri]}"
SERVER_URI="${2:-urn:test:alarms:server}"
CLIENT_URI="${3:-urn:selfpatch:medkit:opcua-client}"

mkdir -p "${OUT}"

# Emit one self-signed application-instance cert (DER) + key (PEM).
#   $1 basename   $2 CN   $3 URI SAN
gen_cert() {
  local base="$1" cn="$2" uri="$3"
  local cnf="${OUT}/${base}.cnf"
  cat >"${cnf}" <<EOF
[req]
distinguished_name = dn
req_extensions = v3
x509_extensions = v3
prompt = no
[dn]
CN = ${cn}
O = selfpatch-test
[v3]
basicConstraints = CA:FALSE
# keyCertSign is REQUIRED: open62541's PKI builds a chain via X509_check_issued,
# which only accepts a self-signed app-instance cert as its own trust anchor
# when it carries keyCertSign (matches open62541's own create_self-signed
# v3_ca). Without it the server rejects the channel with
# BadCertificateChainIncomplete.
keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment, keyCertSign
extendedKeyUsage = serverAuth, clientAuth
subjectAltName = URI:${uri}, DNS:localhost, IP:127.0.0.1
EOF
  openssl req -x509 -newkey rsa:2048 -sha256 -days 3650 -nodes \
    -keyout "${OUT}/${base}_key.pem" \
    -out "${OUT}/${base}_cert.pem" \
    -config "${cnf}" >/dev/null 2>&1
  openssl x509 -in "${OUT}/${base}_cert.pem" -outform der -out "${OUT}/${base}_cert.der"
}

gen_cert server "test-alarm-server" "${SERVER_URI}"
gen_cert client "selfpatch-medkit-opcua-client" "${CLIENT_URI}"

echo "certs written to ${OUT}: server_cert.der client_cert.der (+ *_key.pem)"
