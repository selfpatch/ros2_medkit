# check_cores_without_ros

Checks that the ROS-free cores (fault reporter core, fault manager core) build and run
without ROS. It builds them with plain CMake and gcc 9 on Ubuntu 20.04, the oldest compiler
the cores are checked with, then runs a small program that links them. The check fails if
the binary links `rclcpp`.

```bash
scripts/check_cores_without_ros/run.sh
```

The script sends only the needed source directories to `docker build` as the build
context. CI runs the same CMake steps in an `ubuntu:20.04` container (job `cores-without-ros`).
`COLCON_IGNORE` keeps colcon from treating this directory as a package.
