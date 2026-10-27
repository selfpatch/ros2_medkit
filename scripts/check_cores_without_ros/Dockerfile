FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends g++-9 cmake make libsqlite3-dev libssl-dev nlohmann-json3-dev libyaml-cpp-dev && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY src/ros2_medkit_fault_detection src/ros2_medkit_fault_detection
COPY src/ros2_medkit_fault_reporter src/ros2_medkit_fault_reporter
COPY src/ros2_medkit_fault_manager src/ros2_medkit_fault_manager
COPY src/ros2_medkit_gateway src/ros2_medkit_gateway
COPY scripts/check_cores_without_ros scripts/check_cores_without_ros
RUN CXX=g++-9 cmake -S scripts/check_cores_without_ros -B /tmp/b && cmake --build /tmp/b -j4
RUN ldd /tmp/b/check_cores_without_ros /tmp/b/libcheck_context_plugin.so | tee /tmp/ldd.txt && ! grep -q rclcpp /tmp/ldd.txt
CMD ["sh", "-c", "ldd /tmp/b/check_cores_without_ros; /tmp/b/check_cores_without_ros"]
