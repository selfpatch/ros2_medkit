#!/usr/bin/env bash
# Stop the OpenPLC + medkit gateway demo.
#
# The gateway's state volume is deliberately LEFT IN PLACE. It holds the entity
# freeze frames, faults.db and the rosbags, and a freeze-frame is only worth
# anything if it outlives the process that took it. This script removes the
# container, so state kept in the container's writable layer would go with it.
STATE_VOLUME="${OPCUA_DEMO_STATE_VOLUME:-ros2-medkit-opcua-state}"

docker rm -f gateway openplc 2>/dev/null
docker network rm plc-demo 2>/dev/null
echo "Stopped."
echo "State kept in volume '$STATE_VOLUME'. Purge it with:"
echo "  docker volume rm $STATE_VOLUME"
