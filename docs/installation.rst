Installation
============

This guide covers installation of ros2_medkit on Ubuntu 24.04 with ROS 2 Jazzy.

System Requirements
-------------------

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Requirement
     - Version
   * - Operating System
     - Ubuntu 24.04 LTS (Noble Numbat)
   * - ROS 2 Distribution
     - Jazzy Jalisco
   * - C++ Compiler
     - GCC 13+ (C++17 support required)
   * - CMake
     - 3.22+
   * - Python
     - 3.12+

Prerequisites
-------------

1. **Install ROS 2 Jazzy**

   Follow the official ROS 2 installation guide:
   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

   .. code-block:: bash

      # Add ROS 2 apt repository
      sudo apt update && sudo apt install -y software-properties-common
      sudo add-apt-repository universe
      sudo apt update && sudo apt install curl -y
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

      # Install ROS 2 Jazzy Desktop
      sudo apt update
      sudo apt install ros-jazzy-desktop ros-dev-tools

2. **Source ROS 2 environment**

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash

   Add this to your ``~/.bashrc`` for persistence:

   .. code-block:: bash

      echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

Installation from Source
------------------------

ros2_medkit is currently distributed as source code. Binary packages will be available in future releases.

1. **Create a workspace**

   .. code-block:: bash

      mkdir -p ~/ros2_medkit_ws/src
      cd ~/ros2_medkit_ws/src

2. **Clone the repository**

   .. code-block:: bash

      git clone --recurse-submodules https://github.com/selfpatch/ros2_medkit.git
      cd ros2_medkit

   If you already cloned without submodules, initialize them:

   .. code-block:: bash

      git submodule update --init --recursive

3. **Install dependencies**

   .. code-block:: bash

      cd ~/ros2_medkit_ws
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y

4. **Build the workspace**

   .. code-block:: bash

      cd ~/ros2_medkit_ws
      colcon build --symlink-install

5. **Source the workspace**

   .. code-block:: bash

      source install/setup.bash

   Add this to your ``~/.bashrc`` for persistence:

   .. code-block:: bash

      echo "source ~/ros2_medkit_ws/install/setup.bash" >> ~/.bashrc

Verifying Installation
----------------------

After installation, verify that everything works:

.. code-block:: bash

   # Check that the gateway node is available
   ros2 pkg list | grep ros2_medkit

   # Launch the gateway
   ros2 launch ros2_medkit_gateway gateway.launch.py &

   # Test the API
   curl http://localhost:8080/api/v1/health

   # Stop the gateway
   pkill -f gateway_node

You should see output like:

.. code-block:: json

   {"status": "healthy"}

Docker Installation
-------------------

For containerized deployments, see the `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_ repository
which includes Docker Compose examples with Nav2 and TurtleBot3.

See the :doc:`tutorials/docker` for detailed Docker usage instructions.

Development Installation
------------------------

For development with code coverage and debug symbols:

.. code-block:: bash

   cd ~/ros2_medkit_ws
   colcon build --symlink-install \
     --cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON

Troubleshooting
---------------

**rosdep fails to find packages**

   Make sure rosdep is initialized and updated:

   .. code-block:: bash

      sudo rosdep init  # Only needed once
      rosdep update

**Build fails with C++17 errors**

   Ensure you have GCC 13 or newer:

   .. code-block:: bash

      gcc --version  # Should show 13.x or higher

**Cannot find ros2_medkit packages after build**

   Make sure you source the workspace:

   .. code-block:: bash

      source ~/ros2_medkit_ws/install/setup.bash

Next Steps
----------

- :doc:`getting_started` - Quick tutorial to get started
- :doc:`tutorials/index` - Step-by-step tutorials
- :doc:`design/index` - Architecture documentation
