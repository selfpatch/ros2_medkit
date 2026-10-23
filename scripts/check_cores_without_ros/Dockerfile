FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends g++-9 cmake make && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY src/ros2_medkit_fault_detection src/ros2_medkit_fault_detection
COPY src/ros2_medkit_fault_reporter src/ros2_medkit_fault_reporter
COPY scripts/check_cores_without_ros scripts/check_cores_without_ros
RUN CXX=g++-9 cmake -S scripts/check_cores_without_ros -B /tmp/b && cmake --build /tmp/b
RUN ldd /tmp/b/check_cores_without_ros | tee /tmp/ldd.txt && ! grep -q rclcpp /tmp/ldd.txt
CMD ["sh", "-c", "ldd /tmp/b/check_cores_without_ros; /tmp/b/check_cores_without_ros"]
