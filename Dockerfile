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

# Multi-stage build for ros2_medkit gateway with all open-core plugins.
# Produces a minimal runtime image suitable for deployment and as a base
# for downstream plugin integration (mount .so + config).
#
# Build:  docker build -t ros2_medkit .
# Run:    docker run -p 8080:8080 ros2_medkit
# Config: docker run -v ./my_params.yaml:/etc/ros2_medkit/params.yaml ros2_medkit

# ============================================================================
# Stage 1: Builder
# ============================================================================
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS builder

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV COLCON_WS=/root/ws

# Build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-example-interfaces \
    python3-colcon-common-extensions \
    nlohmann-json3-dev \
    sqlite3 \
    libsqlite3-dev \
    libsystemd-dev \
    libssl-dev \
    pkg-config \
    cmake \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

# cpp-httplib: use system package on jazzy/rolling, build from source on humble
# (Ubuntu 22.04 either lacks the package or provides 0.10.x, we need >= 0.14)
RUN if apt-cache show libcpp-httplib-dev 2>/dev/null | grep -q "^Version: 0\.1[4-9]\|^Version: 0\.[2-9]"; then \
      apt-get update && apt-get install -y --no-install-recommends libcpp-httplib-dev && rm -rf /var/lib/apt/lists/*; \
    else \
      git clone --depth 1 --branch v0.14.3 https://github.com/yhirose/cpp-httplib.git /tmp/cpp-httplib && \
      cd /tmp/cpp-httplib && mkdir build && cd build && \
      cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DHTTPLIB_REQUIRE_OPENSSL=ON && \
      make install && \
      rm -rf /tmp/cpp-httplib; \
    fi

WORKDIR ${COLCON_WS}

# Copy shared cmake modules first (depended on by all packages)
COPY src/ros2_medkit_cmake/ ${COLCON_WS}/src/ros2_medkit_cmake/

# Copy core packages
COPY src/ros2_medkit_msgs/ ${COLCON_WS}/src/ros2_medkit_msgs/
COPY src/ros2_medkit_serialization/ ${COLCON_WS}/src/ros2_medkit_serialization/
COPY src/ros2_medkit_gateway/ ${COLCON_WS}/src/ros2_medkit_gateway/
COPY src/ros2_medkit_fault_manager/ ${COLCON_WS}/src/ros2_medkit_fault_manager/
COPY src/ros2_medkit_fault_reporter/ ${COLCON_WS}/src/ros2_medkit_fault_reporter/
COPY src/ros2_medkit_diagnostic_bridge/ ${COLCON_WS}/src/ros2_medkit_diagnostic_bridge/

# Copy open-core plugins
COPY src/ros2_medkit_discovery_plugins/ ${COLCON_WS}/src/ros2_medkit_discovery_plugins/
COPY src/ros2_medkit_plugins/ ${COLCON_WS}/src/ros2_medkit_plugins/

# Install ROS dependencies and build (skip tests for smaller image)
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys='ament_cmake_clang_format ament_cmake_clang_tidy test_msgs sqlite3 libcpp-httplib-dev rosbag2_storage_mcap' && \
    colcon build --cmake-args -DBUILD_TESTING=OFF"

# ============================================================================
# Stage 2: Runtime
# ============================================================================
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV COLCON_WS=/root/ws

# Runtime dependencies only (header-only libs like nlohmann-json and cpp-httplib
# are already compiled into the binaries, no need to install here)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-yaml-cpp-vendor \
    ros-${ROS_DISTRO}-example-interfaces \
    libsqlite3-0 \
    libsystemd0 \
    libssl3 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy built workspace from builder
COPY --from=builder ${COLCON_WS}/install/ ${COLCON_WS}/install/

# Default config - can be overridden via volume mount
COPY docker/gateway_docker_params.yaml /etc/ros2_medkit/params.yaml

# Plugin mount point for downstream (commercial) plugins
RUN mkdir -p /opt/ros2_medkit/plugins

# Entrypoint that sources ROS and starts the gateway
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["--params-file", "/etc/ros2_medkit/params.yaml"]
