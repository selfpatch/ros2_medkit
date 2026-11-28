#!/bin/bash
# Lint and test src/lerobot-medkit (and the lerobot-so101 example's ROS-free test),
# the way .github/workflows/lerobot-medkit.yml does.
# Runs in ros:jazzy-ros-base from the repository root. Builds only the message
# packages the plugin needs, installs the plugin with CPU-only torch into a venv
# that still sees the ROS 2 Python packages, and fails when any test is skipped:
# the rclpy tests skip without ROS, and a skip must not pass as a success here.
set -euo pipefail

# The ROS setup scripts read unset variables: source them without -u.
ros_source() { set +u; source "$1"; set -u; }

ros_source /opt/ros/jazzy/setup.bash
apt-get update
apt-get install -y --no-install-recommends python3-pip python3-venv python3-colcon-common-extensions

packages=(ros2_medkit_msgs)
if [ -d src/ros2_medkit_policy_msgs ]; then
  packages+=(ros2_medkit_policy_msgs)
fi
colcon build --base-paths src --packages-up-to "${packages[@]}" \
  --build-base /tmp/lm_build --install-base /tmp/lm_install --event-handlers console_direct-
ros_source /tmp/lm_install/setup.bash

python3 -m venv --system-site-packages /tmp/lm_venv
source /tmp/lm_venv/bin/activate
pip install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu
pip install --no-cache-dir -e 'src/lerobot-medkit[test]' \
  flake8 flake8-import-order flake8-quotes flake8-docstrings flake8-builtins flake8-comprehensions

python -m flake8 --config src/ros2_medkit_cmake/cmake/.flake8 src/lerobot-medkit

python -m pytest -p no:cacheprovider -rs src/lerobot-medkit/tests | tee /tmp/lm_pytest.log

# Every test must run. The only skip allowed is the introspection test when
# ros2_medkit_policy_msgs is not in this checkout.
skips=$(grep -E '^SKIPPED' /tmp/lm_pytest.log || true)
if [ -d src/ros2_medkit_policy_msgs ]; then
  unexpected=$skips
else
  unexpected=$(printf '%s\n' "$skips" | grep -v "ros2_medkit_policy_msgs" | grep -v '^$' || true)
fi
if [ -n "$unexpected" ]; then
  echo "error: tests were skipped that must run here:" >&2
  printf '%s\n' "$unexpected" >&2
  exit 1
fi

# The example's ROS-free test (its package is kept out of colcon by COLCON_IGNORE).
PYTHONPATH="examples/lerobot-so101/ros/lerobot_so101_example:${PYTHONPATH:-}" \
  python -m pytest -p no:cacheprovider -q examples/lerobot-so101/ros/lerobot_so101_example/test
