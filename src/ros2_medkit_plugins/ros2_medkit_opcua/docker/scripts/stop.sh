#!/usr/bin/env bash
docker rm -f gateway openplc 2>/dev/null
docker network rm plc-demo 2>/dev/null
echo "Stopped."
