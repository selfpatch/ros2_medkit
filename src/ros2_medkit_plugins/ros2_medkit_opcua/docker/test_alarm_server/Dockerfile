# Copyright 2026 mfaferek93
#
# Standalone OPC-UA test fixture image for AlarmConditionType integration
# tests. The fixture exposes 3 conditions on tcp port 4842 and reads CLI
# commands from stdin; see ../../test/fixtures/test_alarm_server/.
#
# Build context: ros2_medkit repo root (the directory containing `src/`)
#   docker build -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
#                -t ros2_medkit_alarm_test_server:dev .

FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git python3 ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
# Copy only the open62541 source we need plus the fixture source. The plugin's
# FetchContent has already populated build/ros2_medkit_opcua/_deps/... in the
# host repo, so we depend on that being present on the host before docker build.
COPY build/ros2_medkit_opcua/_deps/open62541pp-src/3rdparty/open62541 /src/open62541
COPY src/ros2_medkit_plugins/ros2_medkit_opcua/test/fixtures/test_alarm_server \
     /src/test_alarm_server

RUN cmake -S /src/open62541 -B /tmp/o62-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/open62541 \
        -DBUILD_SHARED_LIBS=OFF \
        -DUA_ENABLE_SUBSCRIPTIONS_EVENTS=ON \
        -DUA_ENABLE_SUBSCRIPTIONS_ALARMS_CONDITIONS=ON \
        -DUA_NAMESPACE_ZERO=FULL \
        -DUA_BUILD_EXAMPLES=OFF \
        -DUA_BUILD_TOOLS=OFF \
        -DUA_FORCE_WERROR=OFF \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_C_FLAGS=-w \
    && cmake --build /tmp/o62-build -j"$(nproc)" --target install

RUN g++ -O2 -std=c++17 -w \
        -I/opt/open62541/include \
        /src/test_alarm_server/test_alarm_server.cpp \
        /opt/open62541/lib/libopen62541.a \
        -lpthread -o /opt/test_alarm_server

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends \
        libstdc++6 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/test_alarm_server /usr/local/bin/test_alarm_server
EXPOSE 4842
ENTRYPOINT ["/usr/local/bin/test_alarm_server"]
CMD ["--port", "4842"]
