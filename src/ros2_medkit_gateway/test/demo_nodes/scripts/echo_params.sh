#!/bin/sh
printf '{"args": ["%s"], "env": {"ROBOT_ID": "%s"}}' "$*" "$ROBOT_ID"
