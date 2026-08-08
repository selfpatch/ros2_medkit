ros2_medkit_cmake
==================

This section contains design documentation for the ros2_medkit_cmake package.

Overview
--------

The ``ros2_medkit_cmake`` package is a build utility package that provides shared CMake
modules for all other ros2_medkit packages. It contains no runtime code - only CMake
macros and functions that are sourced via ``find_package(ros2_medkit_cmake REQUIRED)``
and ``include()``.

Modules
-------

The package provides these CMake modules, installed to the ament index:

1. **ros2_medkit_cmake-extras.cmake** - Ament extras hook

   - Automatically sourced after ``find_package(ros2_medkit_cmake)``
   - Appends the installed module directory to ``CMAKE_MODULE_PATH``
   - Enables transparent ``include(ROS2MedkitCcache)`` etc. in downstream packages

2. **ROS2MedkitCcache.cmake** - Compiler cache integration

   - Auto-detects ``ccache`` on the system
   - Sets ``CMAKE_C_COMPILER_LAUNCHER`` and ``CMAKE_CXX_COMPILER_LAUNCHER``
   - Respects existing launcher overrides (does not clobber explicit settings)
   - Must be included early in CMakeLists.txt, before ``add_library``/``add_executable``

3. **ROS2MedkitLinting.cmake** - Centralized clang-tidy configuration

   - Provides ``ENABLE_CLANG_TIDY`` option (default OFF; a local gate only, CI
     configures it OFF and runs ``run-clang-tidy`` over the compilation database)
   - Provides ``ros2_medkit_clang_tidy()`` function with optional ``HEADER_FILTER``,
     ``TIMEOUT`` and ``JOBS`` arguments
   - Provides ``ROS2_MEDKIT_CLANG_TIDY_JOBS`` (default ``min(host cores, 2)``),
     capped so one package fits an 8 GB machine; memory scales linearly at
     roughly 1.2 GiB per job. Switch it with ``./scripts/test.sh tidy --jobs <n>``
   - References the shared ``.clang-tidy`` config file from the installed module directory

4. **ROS2MedkitCompat.cmake** - Multi-distro compatibility layer

   - ``medkit_find_yaml_cpp()`` - Resolves yaml-cpp across Humble (no cmake target) and Jazzy (namespaced target)
   - ``medkit_find_cpp_httplib()`` - Finds cpp-httplib >= 0.14 via pkg-config, cmake config, or vendored fallback (``VENDORED_DIR`` param)
   - ``medkit_detect_compat_defs()`` - Detects rclcpp and rosbag2 versions, sets ``MEDKIT_RCLCPP_VERSION_MAJOR`` and ``MEDKIT_ROSBAG2_OLD_TIMESTAMP``
   - ``medkit_apply_compat_defs(target)`` - Applies compile definitions based on detected versions
   - ``medkit_target_dependencies(target ...)`` - Drop-in replacement for ``ament_target_dependencies`` that also works on Lyrical (where ``ament_target_dependencies`` was removed)

5. **ROS2MedkitTestDomain.cmake** - ``ROS_DOMAIN_ID`` allocation for tests

   - ``medkit_add_gtest()``, ``medkit_add_gmock()``, ``medkit_add_pytest_test()`` and
     ``medkit_add_launch_test()`` register a test behind the domain wrapper, so a test
     cannot be added without isolation
   - ``medkit_add_wrapped_test(<name> COMMAND <cmd...>)`` does the same for a test that
     builds its own command line
   - ``DOMAINS <n>`` on any of them for a test that holds several domains at once; the
     extras arrive as ``MEDKIT_SECONDARY_DOMAINS``
   - ``medkit_test_needs_no_domain(<test>)`` declares a test ROS-free
   - the runtime guard described below registers itself in every package; no package
     calls anything to get it
   - the wrapper scripts themselves live in ``scripts/``: ``medkit_domain.py`` (the band
     and the allocator), ``medkit_domain_runner.py`` (ament test runner replacement),
     ``medkit_run_with_domain.py`` (generic command wrapper)

Design Decisions
----------------

Test Domains Avoid the Ephemeral Port Range
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

RTPS gives a DDS domain the UDP slice ``[7400 + 250 * d, 7400 + 250 * d + 249]``, and both
CycloneDDS and Fast-DDS bind inside it without an effective ``SO_REUSEPORT`` exemption. The
Linux kernel hands out ephemeral ports from ``net.ipv4.ip_local_port_range``, 32768-60999 by
default, which maps back to domains 101-214. A domain in that band works until an unrelated
process on the machine is given one of its ports first, and then every node on it fails to
start with ``failed to bind to ANY:<port>: address in use``.

Test domains are therefore drawn only from 1-100 and 215-231, 117 in all. Domain 0 is left
to the developer shell, and 232 is dropped because its slice runs past 65535. The derivation
per DDS implementation is written out at the top of ``scripts/medkit_domain.py``, next to the
tuple it justifies.

The Allocation Happens at Run Time, Not at Configure Time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A test takes a domain when it starts and holds it, through an open TCP socket on
``32768 + domain``, for exactly as long as it runs. The lock is
``domain_coordinator.domain_id`` from ``ament_cmake_ros``, which ships on every distro the
project builds for, so this adds no dependency. Release is the kernel closing the socket,
which covers a crash and a ``SIGKILL`` as well as an ordinary exit.

This replaced a hand-written table of per-package domain pools. The table worked, but it had
to be edited every time a package grew past its pool, and the pools had to stay pairwise
disjoint because colcon runs a separate ``ctest`` per package and a CTest ``RESOURCE_LOCK``
only binds inside one ``ctest`` run. An OS-level lock has no such boundary: two processes
contending for a port do not care which ``ctest`` started them, so both the table and the
disjointness requirement went away with it.

The band is smaller than the number of tests the workspace can have in flight under
``ctest -j $(nproc)`` across parallel packages. A test that finds every domain held waits for
one, bounded by ``MEDKIT_TEST_DOMAIN_WAIT`` (180 s by default), and fails loudly if the wait
runs out. It never falls back to a literal, and never to domain 0 - a test on domain 0 does
not fail, it silently sees every node on the machine.

Upstream ``ament_add_ros_isolated_gtest`` is deliberately not used. It draws from
``domain_coordinator``'s own 1-100 selector, which does not include the 215-231 range, and it
skips allocation entirely when ``ROS_DOMAIN_ID`` is already set in the environment - which
would collapse a whole run onto one domain the moment a developer or a colcon extension
exported one. The same reasoning rules out ``colcon-ros-domain-id-coordinator``, which
allocates one domain per package task and pre-sets ``ROS_DOMAIN_ID``.

The Constraint Is Checked, Not Documented
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The failure mode is silence: a test registered with plain ``ament_add_gtest`` or
``add_launch_test`` still passes, it just runs on the default domain and shares it with
everything else on the machine. Nothing at configure time can see this, because the command
is assembled by ament and a later ``set_tests_properties`` can drop what was appended to it.

``test_dds_domain_allocation`` therefore runs on the machine that executes the tests. It
reads the generated CTest properties back and fails on any test whose command does not go
through a wrapper script, unless that test is a linter or carries the ``no_ros_domain`` label
written by ``medkit_test_needs_no_domain()``.

Neither gate is opt-in, and that is deliberate. An earlier shape had each package call
``medkit_add_domain_allocation_test()``; four packages never did, and their fourteen tests
ran on domain 0 with nothing to say so. A gate that only runs where somebody remembered to
register it reproduces exactly the maintenance burden this change exists to remove. So
``test_dds_domain_allocation`` is armed from ``ros2_medkit_cmake-extras.cmake``, the file
ament sources on ``find_package(ros2_medkit_cmake)``, which every package already calls. The
registration is deferred with ``cmake_language(DEFER)`` because the hook fires before a
package's tests exist and, in several packages, before ``BUILD_TESTING`` is defined.

That still leaves a package that never finds ``ros2_medkit_cmake`` at all, so
``test_dds_domain_coverage`` sweeps every package build directory in the workspace, applies
the same per-test rule to all of them, and separately reports a package that has tests but
no gate. Between the two, a newly added package is covered whether or not its author knows
the scheme exists.

The wrapper itself is covered by this package's own suite, which pins the band and its
endpoints, an unrelated process holding a lock port, an exhausted band, excess holders
waiting and being served as domains free up, and a holder killed with ``SIGKILL`` releasing
its domain.
The cross-talk probe carries its control: two independently allocated ROS nodes must hear
nothing from each other, and the same two forced onto one domain must hear each other,
because a zero from an instrument that cannot register a one means nothing. It starts both
peers itself, as concurrent subprocesses it joins, and fails if they did not overlap. An
earlier shape registered them as two CTest tests and needed ``ctest -j`` to run them side by
side; CTest cannot be asked for that, so the pair passed under a parallel run and failed
under a serial one. Concurrency a test depends on is the test's to arrange.

The one case that cannot live here is the launch test, which proves a launch test's children
join its domain. ``launch_testing_ament_cmake`` reaches the deprecated ``FindPythonLibs``
through ``python_cmake_module`` on Humble, and this project is declared ``NONE``: with no
language enabled CMake leaves ``CMAKE_LIBRARY_ARCHITECTURE`` and ``CMAKE_SIZEOF_VOID_P``
empty, so ``find_library`` never searches the multiarch directory and libpython is not found
however the development headers are installed. Since every package depends on this one, a
configure failure here stops the whole workspace, so that test lives in
``ros2_medkit_fault_reporter``, which compiles and already runs a launch test on every
distro. Keep test-only ``find_package`` calls out of this package.

Separate Package
~~~~~~~~~~~~~~~~

Shared CMake modules live in their own ament package rather than being inlined
into each consuming package. This avoids duplication and ensures all packages
use the same compatibility logic. Downstream packages declare
``<buildtool_depend>ros2_medkit_cmake</buildtool_depend>`` in their
``package.xml``.

Multi-Distro Strategy
~~~~~~~~~~~~~~~~~~~~~

Rather than maintaining separate branches per ROS 2 distribution, the compat
module detects version numbers at configure time and adapts. This keeps a single
source tree building on Humble, Jazzy, and Lyrical without ``#ifdef`` proliferation
in application code.
