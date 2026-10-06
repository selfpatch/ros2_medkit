#!/usr/bin/env bash
# Copyright 2026 bburda
#
# Prints `code=true` or `code=false` for $GITHUB_OUTPUT. False only on a pull
# request whose every changed file is documentation:
#   - a .md or .rst file anywhere;
#   - under docs/: an image, stylesheet, font or font licence, docs/conf.py or
#     docs/Doxyfile.
# A symlink or submodule entry is never documentation, because colcon follows
# links. A CMakeLists.txt is never documentation, because colcon builds a lone
# one as a package. The gateway checks that read docs/api and the design docs
# are labelled `linter`, and format-lint runs those on every pull request.
#
# Runs in a checkout of the pull request merge commit, fetched with depth 2. Its
# first parent is the base branch tip, so the diff is what the merge adds.
set -euo pipefail

if [[ "${GITHUB_EVENT_NAME:-}" != "pull_request" ]]; then
  echo "code=true"
  exit 0
fi

# On a single-parent commit HEAD^1 diffs only the last commit, not the pull request.
if ! git rev-parse --verify --quiet 'HEAD^2' >/dev/null; then
  echo "::error::HEAD is not a merge commit - check out the pull request merge ref with fetch-depth 2" >&2
  exit 1
fi

is_docs() {
  case "$1" in
    CMakeLists.txt | */CMakeLists.txt) return 1 ;;
    *.md | *.rst) return 0 ;;
    docs/conf.py | docs/Doxyfile) return 0 ;;
    docs/*.png | docs/*.jpg | docs/*.jpeg | docs/*.gif | docs/*.svg | docs/*.webp | docs/*.ico) return 0 ;;
    docs/*.css | docs/*.woff | docs/*.woff2 | docs/*.txt) return 0 ;;
  esac
  return 1
}

# -z keeps every path byte-exact. --no-renames lists both sides of a rename, so
# moving a source file under docs/ still counts as a code change. A file, so
# that set -e sees a git failure.
diff_file=$(mktemp)
trap 'rm -f "${diff_file}"' EXIT
git diff --raw -z --no-renames 'HEAD^1' HEAD >"${diff_file}"

code_files=()
docs_files=()
# Each entry is ":<old mode> <new mode> <old sha> <new sha> <status>" NUL <path> NUL.
while IFS= read -r -d '' meta && IFS= read -r -d '' path; do
  read -r old_mode new_mode _ <<<"${meta#:}"
  case "${old_mode} ${new_mode}" in
    *120000* | *160000*) code_files+=("${path}") ;;
    *) if is_docs "${path}"; then docs_files+=("${path}"); else code_files+=("${path}"); fi ;;
  esac
done <"${diff_file}"

if [[ ${#code_files[@]} -eq 0 && ${#docs_files[@]} -gt 0 ]]; then
  echo "code=false"
  printf 'Docs-only change:\n' >&2
  printf '  %s\n' "${docs_files[@]}" >&2
else
  echo "code=true"
  printf 'Files outside the docs-only set:\n' >&2
  printf '  %s\n' "${code_files[@]:-<empty diff>}" >&2
fi
