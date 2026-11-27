LeRobot SO-101 Example
======================

A simulated SO-101 arm runs a LeRobot ACT policy over ROS 2 and picks up a cube.
ros2_medkit, with stock packages and configuration only, reports what goes wrong:
a failed attempt (``EPISODE_FAILED``, with a rosbag of the attempt), a frozen camera,
slow policy inference and a dead policy process.

The example lives in the repository, in
`examples/lerobot-so101 <https://github.com/selfpatch/ros2_medkit/tree/main/examples/lerobot-so101>`_.
Its README covers running it with Docker Compose, the fault injection scripts, and how
to train the policy checkpoint on one GPU.

- 🐳 Docker Compose deployment, MuJoCo simulation, no GPU needed to run
- 🤖 LeRobot ACT policy, the same topics as a real SO-101 driver
- 🔧 Runtime fault injection scripts
- 📼 One rosbag per failed attempt
