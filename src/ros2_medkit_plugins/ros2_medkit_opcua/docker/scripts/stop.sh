#!/usr/bin/env bash
# Stop the OpenPLC + medkit gateway demo. The gateway's state volume stays, so
# faults and their freeze-frames survive the next start.sh.
STATE_VOLUME="${OPCUA_DEMO_STATE_VOLUME:-ros2-medkit-opcua-state}"

docker rm -f gateway openplc 2>/dev/null
docker network rm plc-demo 2>/dev/null
echo "Stopped."
echo "State kept in volume '$STATE_VOLUME'. Purge it with:"
echo "  docker volume rm $STATE_VOLUME"
