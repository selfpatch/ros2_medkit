#!/usr/bin/env bash
# OpenPLC Tank Demo - Integration Tests
# Validates: entity discovery, live data, the write surface the image was built
# with, and error handling.
#
# MEDKIT_OPCUA_VARIANT selects which write surface is expected, and it has to
# match how the gateway image was built (docker build --build-arg
# MEDKIT_OPCUA_READ_ONLY=ON|OFF). It defaults to read-only because that is what
# the plugin and the Dockerfile default to; against a default image the suite
# proves the read-only contract on a real PLC - the refusal, its vendor code,
# and that the tag did not move - rather than skipping the write section.
set -o pipefail

API="${GATEWAY_URL:-http://localhost:8080}/api/v1"
VARIANT="${MEDKIT_OPCUA_VARIANT:-read-only}"
PASS=0
FAIL=0

if [ "$VARIANT" != "read-only" ] && [ "$VARIANT" != "write-capable" ]; then
    echo "MEDKIT_OPCUA_VARIANT must be read-only or write-capable (got '$VARIANT')" >&2
    exit 2
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

assert() {
    local desc="$1" ok="$2"
    if [ "$ok" = "true" ]; then
        echo -e "  ${GREEN}PASS${NC}: $desc"
        PASS=$((PASS + 1))
    else
        echo -e "  ${RED}FAIL${NC}: $desc"
        FAIL=$((FAIL + 1))
    fi
}

# One request, both halves of the answer: HTTP_CODE is the status line, RESP the
# payload. An assertion that reads only the payload cannot tell a refusal from a
# route that is not registered, because both carry an error object.
request() {
    local method="$1" url="$2" data="${3-}"
    local out
    if [ -n "$data" ]; then
        out=$(curl -s -w '\n%{http_code}' -X "$method" "$url" \
            -H "Content-Type: application/json" -d "$data")
    else
        out=$(curl -s -w '\n%{http_code}' -X "$method" "$url")
    fi
    HTTP_CODE="${out##*$'\n'}"
    RESP="${out%$'\n'*}"
}

status_is() {
    [ "$HTTP_CODE" = "$1" ] && echo true || echo false
}

echo -e "${YELLOW}=== OpenPLC Tank Demo Integration Tests (${VARIANT} image) ===${NC}\n"

# 1. Wait for gateway + PLC entities
echo -e "${YELLOW}1. Wait for gateway + PLC entities${NC}"
for i in $(seq 1 60); do
    APPS=$(curl -s "$API/apps" 2>/dev/null | jq '[.items[].id]' 2>/dev/null)
    if echo "$APPS" | jq -e 'contains(["tank_process"])' >/dev/null 2>&1; then
        echo "  Ready after ${i}s"
        break
    fi
    if [ "$i" -eq 60 ]; then
        echo -e "  ${RED}Timeout waiting for PLC entities${NC}"
        exit 1
    fi
    sleep 2
done

# 2. Entity Discovery
echo -e "\n${YELLOW}2. Entity Discovery${NC}"
assert "plc_systems area" "$(curl -s "$API/areas" | jq '[.items[].id] | contains(["plc_systems"])' 2>/dev/null)"
assert "openplc_runtime component" "$(curl -s "$API/components" | jq '[.items[].id] | contains(["openplc_runtime"])' 2>/dev/null)"
assert "tank_process app" "$(curl -s "$API/apps" | jq '[.items[].id] | contains(["tank_process"])' 2>/dev/null)"
assert "fill_pump app" "$(curl -s "$API/apps" | jq '[.items[].id] | contains(["fill_pump"])' 2>/dev/null)"
assert "drain_valve app" "$(curl -s "$API/apps" | jq '[.items[].id] | contains(["drain_valve"])' 2>/dev/null)"

# 3. PLC Status
echo -e "\n${YELLOW}3. PLC Connection Status${NC}"
STATUS=$(curl -s "$API/components/openplc_runtime/x-plc-status")
assert "PLC connected" "$(echo "$STATUS" | jq '.connected' 2>/dev/null)"
assert "Zero errors" "$(echo "$STATUS" | jq '.error_count == 0' 2>/dev/null)"
# The build the image carries, read from the wire. The absent x-plc-operations
# capability below is also what a write-capable build with no writable point
# shows, so the marker is what distinguishes them.
if [ "$VARIANT" = "read-only" ]; then
    assert "write_capable false" "$(echo "$STATUS" | jq '.write_capable == false' 2>/dev/null)"
else
    assert "write_capable true" "$(echo "$STATUS" | jq '.write_capable == true' 2>/dev/null)"
fi

# 4. Live Data
echo -e "\n${YELLOW}4. Live Data${NC}"
DATA=$(curl -s "$API/apps/tank_process/x-plc-data")
LEVEL=$(echo "$DATA" | jq '.items[] | select(.name == "tank_level") | .value' 2>/dev/null)
TEMP=$(echo "$DATA" | jq '.items[] | select(.name == "tank_temperature") | .value' 2>/dev/null)
PRESS=$(echo "$DATA" | jq '.items[] | select(.name == "tank_pressure") | .value' 2>/dev/null)
assert "tank_level has value" "$([ -n "$LEVEL" ] && [ "$LEVEL" != "null" ] && echo true || echo false)"
assert "tank_temperature has value" "$([ -n "$TEMP" ] && [ "$TEMP" != "null" ] && echo true || echo false)"
assert "tank_pressure has value" "$([ -n "$PRESS" ] && [ "$PRESS" != "null" ] && echo true || echo false)"
echo "  Level=$LEVEL mm, Temp=$TEMP C, Pressure=$PRESS bar"

# 5. What the tree advertises about writing
echo -e "\n${YELLOW}5. Advertised write surface${NC}"
CAPS=$(curl -s "$API/apps/fill_pump" | jq '[.capabilities[].name]' 2>/dev/null)
OPS=$(curl -s "$API/apps/fill_pump/operations" | jq '[.items[].id]' 2>/dev/null)
if [ "$VARIANT" = "read-only" ]; then
    assert "no x-plc-operations capability" \
        "$(echo "$CAPS" | jq 'contains(["x-plc-operations"]) | not' 2>/dev/null)"
    assert "no set_pump_speed operation" \
        "$(echo "$OPS" | jq 'contains(["set_pump_speed"]) | not' 2>/dev/null)"
else
    assert "x-plc-operations capability" \
        "$(echo "$CAPS" | jq 'contains(["x-plc-operations"])' 2>/dev/null)"
    assert "set_pump_speed operation" \
        "$(echo "$OPS" | jq 'contains(["set_pump_speed"])' 2>/dev/null)"
fi

# 6. Write - Pump Speed
echo -e "\n${YELLOW}6. Write Pump Speed${NC}"
PUMP_BEFORE=$(curl -s "$API/apps/fill_pump/x-plc-data" | jq '.items[] | select(.name == "pump_speed") | .value' 2>/dev/null)
if [ "$VARIANT" = "read-only" ]; then
    # The vendor route is not registered at all, so this is the gateway's own
    # 404 rather than a plugin refusal.
    WRITE=$(curl -s -X POST "$API/apps/fill_pump/x-plc-operations/set_pump_speed" \
        -H "Content-Type: application/json" -d '{"value": 75.0}')
    assert "x-plc-operations set_pump_speed not routed" \
        "$(echo "$WRITE" | jq '.error_code == "resource-not-found"' 2>/dev/null)"
    # The SOVD write endpoint does reach the plugin, and the plugin refuses.
    PUT=$(curl -s -X PUT "$API/apps/fill_pump/data/pump_speed" \
        -H "Content-Type: application/json" -d '{"value": 75.0}')
    assert "PUT pump_speed refused with the vendor code" \
        "$(echo "$PUT" | jq '.vendor_code == "x-medkit-plugin-error"' 2>/dev/null)"
    assert "refusal names MEDKIT_OPCUA_READ_ONLY" \
        "$(echo "$PUT" | jq '.message | test("MEDKIT_OPCUA_READ_ONLY")' 2>/dev/null)"
    sleep 5
    PUMP=$(curl -s "$API/apps/fill_pump/x-plc-data" | jq '.items[] | select(.name == "pump_speed") | .value' 2>/dev/null)
    # The tag did not move. Tolerance rather than equality because the value is
    # a float coming back through the PLC; a write of 75 from rest is far larger
    # than any representation jitter.
    assert "pump_speed unchanged on the PLC" \
        "$(jq -n --argjson a "${PUMP_BEFORE:-null}" --argjson b "${PUMP:-null}" \
            '($a != null) and ($b != null) and (($a - $b) | fabs < 0.5)' 2>/dev/null)"
else
    WRITE=$(curl -s -X POST "$API/apps/fill_pump/x-plc-operations/set_pump_speed" \
        -H "Content-Type: application/json" -d '{"value": 75.0}')
    assert "Write pump speed OK" "$(echo "$WRITE" | jq '.status == "ok"' 2>/dev/null)"
    sleep 5
    PUMP=$(curl -s "$API/apps/fill_pump/x-plc-data" | jq '.items[] | select(.name == "pump_speed") | .value' 2>/dev/null)
    assert "Pump speed ~= 75" "$(echo "$PUMP" | jq '. >= 74 and . <= 76' 2>/dev/null)"
fi

# 7. Write - Valve Position
echo -e "\n${YELLOW}7. Write Valve Position${NC}"
VALVE_BEFORE=$(curl -s "$API/apps/drain_valve/x-plc-data" | jq '.items[] | select(.name == "valve_position") | .value' 2>/dev/null)
if [ "$VARIANT" = "read-only" ]; then
    PUT=$(curl -s -X PUT "$API/apps/drain_valve/data/valve_position" \
        -H "Content-Type: application/json" -d '{"value": 50.0}')
    assert "PUT valve_position refused with the vendor code" \
        "$(echo "$PUT" | jq '.vendor_code == "x-medkit-plugin-error"' 2>/dev/null)"
    sleep 5
    VALVE=$(curl -s "$API/apps/drain_valve/x-plc-data" | jq '.items[] | select(.name == "valve_position") | .value' 2>/dev/null)
    assert "valve_position unchanged on the PLC" \
        "$(jq -n --argjson a "${VALVE_BEFORE:-null}" --argjson b "${VALVE:-null}" \
            '($a != null) and ($b != null) and (($a - $b) | fabs < 0.5)' 2>/dev/null)"
else
    WRITE=$(curl -s -X POST "$API/apps/drain_valve/x-plc-operations/set_valve_position" \
        -H "Content-Type: application/json" -d '{"value": 50.0}')
    assert "Write valve position OK" "$(echo "$WRITE" | jq '.status == "ok"' 2>/dev/null)"
fi

# 8. Error Handling
echo -e "\n${YELLOW}8. Error Handling${NC}"
# Two answers that do not depend on the write surface, so they are identical in
# both variants, and each comes from a different layer. The vendor data route is
# a GET, registered either way, and its handler validates the entity through the
# gateway's plugin context before it reads the node map. The malformed body never
# reaches the plugin at all: the gateway's own data handler parses the body and
# rejects it before delegating the write.
request GET "$API/apps/nonexistent/x-plc-data"
assert "unknown entity on the vendor data route is 404 (got $HTTP_CODE)" "$(status_is 404)"
assert "that 404 is entity-not-found" \
    "$(echo "$RESP" | jq '.error_code == "entity-not-found"' 2>/dev/null)"

request PUT "$API/apps/fill_pump/data/pump_speed" 'bad'
assert "malformed body on PUT data is 400 (got $HTTP_CODE)" "$(status_is 400)"
assert "malformed body is the gateway's invalid-request, before the plugin" \
    "$(echo "$RESP" | jq '.error_code == "invalid-request"' 2>/dev/null)"

if [ "$VARIANT" = "read-only" ]; then
    # The plugin's own answers, on the two SOVD data verbs this build serves:
    # a read of an absent point, and a write of one.
    request GET "$API/apps/tank_process/data/nonexistent"
    assert "GET unknown data point is 404 (got $HTTP_CODE)" "$(status_is 404)"
    assert "that 404 is the plugin's own error" \
        "$(echo "$RESP" | jq '.vendor_code == "x-medkit-plugin-error"' 2>/dev/null)"

    # The refusal precedes the lookup, so an unknown point answers the build
    # rather than reporting whether that point exists.
    request PUT "$API/apps/fill_pump/data/nonexistent" '{"value": 1}'
    assert "PUT unknown data point is refused as the build: 501 (got $HTTP_CODE)" "$(status_is 501)"
    assert "that refusal names MEDKIT_OPCUA_READ_ONLY" \
        "$(echo "$RESP" | jq '.message | test("MEDKIT_OPCUA_READ_ONLY")' 2>/dev/null)"
else
    # The vendor write route is registered here, so it answers for itself: it
    # names the operation it could not find, and it parses the body itself.
    request POST "$API/apps/tank_process/x-plc-operations/nonexistent" '{"value":1}'
    assert "unknown operation is 404 (got $HTTP_CODE)" "$(status_is 404)"
    assert "that 404 is resource-not-found naming the operation" \
        "$(echo "$RESP" | jq '.error_code == "resource-not-found" and (.message | test("nonexistent"))' 2>/dev/null)"

    request POST "$API/apps/fill_pump/x-plc-operations/set_pump_speed" 'bad'
    assert "malformed body on the vendor route is 400 (got $HTTP_CODE)" "$(status_is 400)"
    assert "that 400 is invalid-request" \
        "$(echo "$RESP" | jq '.error_code == "invalid-request"' 2>/dev/null)"
fi

# 9. Standard SOVD Data Endpoint (DataProvider integration)
# NOTE: alarm trigger/clear tests are not feasible with the OpenPLC tank demo
# because the IEC 61131-3 simulation program continuously recalculates
# TankLevel from pump/drain physics. Direct writes to TankLevel are
# overridden on the next PLC cycle (100ms), so we cannot externally force
# the value below the alarm threshold. Alarm bridge is unit-tested via
# the OpcuaPoller::evaluate_alarms path instead.
echo -e "\n${YELLOW}9. Standard SOVD Data Endpoint${NC}"
SOVD_DATA=$(curl -s "$API/apps/tank_process/data" 2>/dev/null)
assert "SOVD /data returns items" "$(echo "$SOVD_DATA" | jq 'has("items")' 2>/dev/null)"
assert "SOVD /data has tank_level" "$(echo "$SOVD_DATA" | jq '[.items[].id] | contains(["tank_level"])' 2>/dev/null)"

# Cleanup - stop pump. Only the write-capable image can, and only it moved it.
if [ "$VARIANT" = "write-capable" ]; then
    curl -s -X POST "$API/apps/fill_pump/x-plc-operations/set_pump_speed" \
        -H "Content-Type: application/json" -d '{"value": 0}' >/dev/null 2>&1
fi

echo -e "\n${YELLOW}===== Test Summary =====${NC}"
TOTAL=$((PASS + FAIL))
echo -e "  Total: $TOTAL"
echo -e "  ${GREEN}Passed: $PASS${NC}"
if [ "$FAIL" -gt 0 ]; then
    echo -e "  ${RED}Failed: $FAIL${NC}"
    exit 1
else
    echo -e "  ${GREEN}All tests passed!${NC}"
fi
