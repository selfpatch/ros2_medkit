#!/bin/bash
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

# Release helper script for ros2_medkit.
#
# Usage:
#   ./scripts/release.sh bump <version>    - Bump all package.xml and version.hpp to <version>
#   ./scripts/release.sh verify [<version>] - Verify all packages have consistent versions
#                                             (if <version> given, checks against that specific version)
#
# Examples:
#   ./scripts/release.sh bump 0.4.0
#   ./scripts/release.sh verify
#   ./scripts/release.sh verify 0.4.0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${REPO_ROOT}/src"
VERSION_HPP="${SRC_DIR}/ros2_medkit_gateway/include/ros2_medkit_gateway/version.hpp"

usage() {
    echo "Usage: $0 {bump <version>|verify [<version>]}"
    echo ""
    echo "Commands:"
    echo "  bump <version>     Bump all package.xml files and version.hpp fallback"
    echo "  verify [<version>] Verify version consistency across all packages"
    exit 1
}

find_package_xmls() {
    find "${SRC_DIR}" -name "package.xml" -not -path "*/.worktrees/*" -not -path "*/build/*" -not -path "*/install/*" | sort
}

get_version() {
    local pkg_xml="$1"
    grep -oP '<version>\K[0-9]+\.[0-9]+\.[0-9]+(?=</version>)' "$pkg_xml"
}

get_package_name() {
    local pkg_xml="$1"
    grep -oP '<name>\K[^<]+' "$pkg_xml"
}

cmd_bump() {
    local target_version="$1"

    # Validate semver format
    if ! echo "$target_version" | grep -qP '^[0-9]+\.[0-9]+\.[0-9]+$'; then
        echo "Error: '$target_version' is not a valid semver (X.Y.Z)"
        exit 1
    fi

    echo "Bumping all packages to version ${target_version}..."
    echo ""

    local count=0
    while IFS= read -r pkg_xml; do
        local pkg_name
        pkg_name=$(get_package_name "$pkg_xml")
        local old_version
        old_version=$(get_version "$pkg_xml")
        local rel_path="${pkg_xml#"${REPO_ROOT}/"}"

        sed -i "s|<version>[0-9]\+\.[0-9]\+\.[0-9]\+</version>|<version>${target_version}</version>|" "$pkg_xml"
        echo "  ${pkg_name}: ${old_version} -> ${target_version}  (${rel_path})"
        count=$((count + 1))
    done < <(find_package_xmls)

    # Update version.hpp fallback
    if [ -f "$VERSION_HPP" ]; then
        local old_fallback
        old_fallback=$(grep -oP 'kGatewayVersion = "\K[0-9]+\.[0-9]+\.[0-9]+' "$VERSION_HPP" || echo "unknown")
        sed -i "s|kGatewayVersion = \"[0-9]\+\.[0-9]\+\.[0-9]\+\"|kGatewayVersion = \"${target_version}\"|" "$VERSION_HPP"
        echo "  version.hpp fallback: ${old_fallback} -> ${target_version}"
    else
        echo "  WARNING: version.hpp not found at ${VERSION_HPP}"
    fi

    echo ""
    echo "Bumped ${count} packages + version.hpp to ${target_version}."
    echo ""
    echo "Run '$0 verify ${target_version}' to confirm."
}

cmd_verify() {
    local expected_version="${1:-}"

    echo "Verifying package versions..."
    echo ""

    local all_ok=true
    local versions_seen=()

    while IFS= read -r pkg_xml; do
        local pkg_name
        pkg_name=$(get_package_name "$pkg_xml")
        local version
        version=$(get_version "$pkg_xml")
        local rel_path="${pkg_xml#"${REPO_ROOT}/"}"

        if [ -n "$expected_version" ] && [ "$version" != "$expected_version" ]; then
            echo "  MISMATCH: ${pkg_name} is ${version}, expected ${expected_version}  (${rel_path})"
            all_ok=false
        else
            echo "  OK: ${pkg_name} = ${version}  (${rel_path})"
        fi
        versions_seen+=("$version")
    done < <(find_package_xmls)

    # Check version.hpp fallback
    if [ -f "$VERSION_HPP" ]; then
        local hpp_version
        hpp_version=$(grep -oP 'kGatewayVersion = "\K[0-9]+\.[0-9]+\.[0-9]+' "$VERSION_HPP" || echo "unknown")
        if [ -n "$expected_version" ] && [ "$hpp_version" != "$expected_version" ]; then
            echo "  MISMATCH: version.hpp fallback is ${hpp_version}, expected ${expected_version}"
            all_ok=false
        else
            echo "  OK: version.hpp fallback = ${hpp_version}"
        fi
        versions_seen+=("$hpp_version")
    fi

    # Check consistency if no expected version given
    if [ -z "$expected_version" ]; then
        local unique_versions
        unique_versions=$(printf '%s\n' "${versions_seen[@]}" | sort -u)
        local unique_count
        unique_count=$(echo "$unique_versions" | wc -l)
        if [ "$unique_count" -gt 1 ]; then
            echo ""
            echo "WARNING: Found multiple versions:"
            echo "$unique_versions" | while read -r v; do echo "  - $v"; done
            all_ok=false
        fi
    fi

    echo ""
    if $all_ok; then
        echo "All versions are consistent."
        return 0
    else
        echo "Version mismatches found!"
        return 1
    fi
}

# Main
if [ $# -lt 1 ]; then
    usage
fi

case "$1" in
    bump)
        if [ $# -lt 2 ]; then
            echo "Error: bump requires a version argument"
            usage
        fi
        cmd_bump "$2"
        ;;
    verify)
        cmd_verify "${2:-}"
        ;;
    *)
        usage
        ;;
esac
