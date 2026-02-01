Demos Overview
==============

This section provides walkthroughs for running ros2_medkit with demo systems
of increasing complexity.

.. toctree::
   :maxdepth: 1

   demo-quick-start
   demo-sensor
   demo-turtlebot3

Learning Path
-------------

The demos are organized by complexity — start simple and progress to more
advanced scenarios:

1. :doc:`demo-quick-start` — **Fastest start** ⚡

   Built-in demo nodes with minimal setup. Perfect for exploring the gateway
   API without external dependencies.

   - ✅ No Docker required
   - ✅ Seconds to start
   - ✅ Simple automotive sensors
   - ✅ Fault injection examples

2. :doc:`demo-sensor` — **Lightweight diagnostics** 🔬

   Docker-based sensor simulation with comprehensive fault detection and
   multiple reporting paths.

   - 🐳 Docker Compose deployment
   - ✅ No Gazebo or GPU needed
   - 🔧 Runtime fault injection scripts
   - 📊 Dual reporting mechanisms

3. :doc:`demo-turtlebot3` — **Full robot simulation** 🤖

   Complete robotics stack with navigation, visualization, and realistic
   sensor data.

   - 🎮 Gazebo simulation
   - 🗺️ Nav2 navigation
   - 🚀 Production-like complexity
   - 🖥️ GPU recommended

Quick Comparison
----------------

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Feature
     - Quick Start
     - Sensor Demo
     - TurtleBot3
   * - **Setup Time**
     - < 1 minute
     - ~5 minutes
     - ~10 minutes
   * - **Startup Time**
     - ~5 seconds
     - ~10 seconds
     - ~60 seconds
   * - **Docker Required**
     - No
     - Yes
     - Yes
   * - **GPU Needed**
     - No
     - No
     - Recommended
   * - **CI Compatible**
     - Yes
     - Yes
     - Difficult
   * - **Image Size**
     - N/A (local)
     - ~500 MB
     - ~4 GB
   * - **Focus**
     - Basic API
     - Diagnostics
     - Navigation

Choose Your Demo
----------------

**I want to...**

- 🚀 **Get started quickly** → :doc:`demo-quick-start`
- 🔍 **Learn fault management** → :doc:`demo-sensor`
- 🤖 **Test with realistic robotics** → :doc:`demo-turtlebot3`
- 📚 **Understand the full API** → Start with :doc:`demo-quick-start`, then progress through all three
