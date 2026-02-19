# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Coverage environment helpers for gcov/lcov integration."""

import os


def get_coverage_env(package_name='ros2_medkit_gateway'):
    """Get GCOV_PREFIX env vars pointing to the target package's build dir.

    When running with ENABLE_COVERAGE=ON, subprocess nodes need GCOV_PREFIX
    set to write .gcda files to the correct build directory.

    Parameters
    ----------
    package_name : str
        The ROS 2 package whose build directory should receive .gcda files.
        Defaults to 'ros2_medkit_gateway' since that is the primary coverage
        target.

    Returns
    -------
    dict
        Environment variables dict with GCOV_PREFIX and GCOV_PREFIX_STRIP,
        or empty dict if coverage path cannot be determined.

    """
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix(package_name)
        # pkg_prefix is like /path/to/workspace/install/ros2_medkit_gateway
        # workspace is 2 levels up from install/package_name
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', package_name)

        if os.path.exists(build_dir):
            # GCOV_PREFIX_STRIP removes leading path components from compiled-in paths
            # GCOV_PREFIX prepends the new path for .gcda file output
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Ignore: if coverage environment cannot be determined,
        # return empty dict so tests proceed without coverage data.
        pass
    return {}
