Demos Overview
==============

This section provides walkthroughs for running ros2_medkit with demo systems
of increasing complexity.

.. toctree::
   :maxdepth: 1

   demo-sensor
   demo-turtlebot3

Learning Path
-------------

The demos are organized by complexity — start simple and progress to more
advanced scenarios:

**Start here:** :doc:`/getting_started` — **Quick tutorial** ⚡

   Learn the basics with built-in demo nodes. Perfect for exploring the gateway
   API without external dependencies.

   - ✅ No Docker required
   - ✅ Seconds to start
   - ✅ Simple automotive sensors
   - ✅ Interactive curl examples

Then explore these demos:

1. :doc:`demo-sensor` — **Lightweight diagnostics** 🔬

   Docker-based sensor simulation with comprehensive fault detection and
   multiple reporting paths.

   - 🐳 Docker Compose deployment
   - ✅ No Gazebo or GPU needed
   - 🔧 Runtime fault injection scripts
   - 📊 Dual reporting mechanisms

2. :doc:`demo-turtlebot3` — **Full robot simulation** 🤖

   Complete robotics stack with navigation, visualization, and realistic
   sensor data.

   - 🎮 Gazebo simulation
   - 🗺️ Nav2 navigation
   - 🚀 Production-like complexity
   - 🖥️ GPU recommended

Choose Your Demo
----------------

**I want to...**

- 🚀 **Get started quickly** → :doc:`/getting_started`
- 🔍 **Learn fault management** → :doc:`demo-sensor`
- 🤖 **Test with realistic robotics** → :doc:`demo-turtlebot3`
- 📚 **Understand the full API** → Start with :doc:`/getting_started`, then explore both demos
