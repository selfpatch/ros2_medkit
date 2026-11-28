# LeRobot SO-101 example: so101-nexus MuJoCo simulation + LeRobot ACT policy + ROS 2 Jazzy
# + ros2_medkit built from this repository.
# Build from the repository root (docker-compose.yml does): docker build -f examples/lerobot-so101/Dockerfile .
# Jazzy ships Python 3.12, which LeRobot needs, so rclpy and LeRobot share one interpreter.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-colcon-common-extensions \
    libegl1 libgl1 libgl1-mesa-dri libosmesa6 ffmpeg curl jq \
    && rm -rf /var/lib/apt/lists/*

# ros2_medkit from this checkout, only the packages the example runs. The policy
# bridge and its messages are built when this checkout has them.
ENV MEDKIT_WS=/opt/medkit_ws
COPY src/ ${MEDKIT_WS}/src/
WORKDIR ${MEDKIT_WS}
RUN bash -c "source /opt/ros/jazzy/setup.bash && apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys='ament_cmake_clang_format ament_cmake_clang_tidy' && \
    rm -rf /var/lib/apt/lists/* && \
    MAKEFLAGS='-j 4' colcon build --parallel-workers 2 \
      --packages-up-to ros2_medkit_gateway ros2_medkit_fault_manager \
        ros2_medkit_diagnostic_bridge ros2_medkit_graph_watchdog \
        \$(test -d src/ros2_medkit_policy_bridge && echo ros2_medkit_policy_bridge) \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && rm -rf build log src"

# LeRobot in a venv that still sees the ROS 2 Python packages (rclpy, messages).
ENV VIRTUAL_ENV=/opt/lerobot_venv
RUN python3 -m venv --system-site-packages ${VIRTUAL_ENV}
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}
# CPU-only torch keeps the image small; ACT inference runs on the CPU in lockstep.
RUN pip install --no-cache-dir torch==2.15.0 torchvision==0.29.2 \
      --index-url https://download.pytorch.org/whl/cpu
COPY examples/lerobot-so101/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# MuJoCo renders off-screen (no GPU runtime needed).
ENV MUJOCO_GL=egl
# The trained checkpoint is mounted here (docker-compose.yml); nothing is downloaded at run time.
ENV POLICY_PATH=/opt/policy
ENV HF_HUB_OFFLINE=1

ENV COLCON_WS=/opt/example_ws
COPY examples/lerobot-so101/ros/lerobot_so101_example/ ${COLCON_WS}/src/lerobot_so101_example/
WORKDIR ${COLCON_WS}
# colcon with the venv's Python, so the node scripts start in the venv and can import LeRobot.
RUN bash -c "source ${MEDKIT_WS}/install/setup.bash && python3 -m colcon build"

CMD ["bash", "-c", "source /opt/example_ws/install/setup.bash && exec ros2 launch lerobot_so101_example example.launch.py"]
