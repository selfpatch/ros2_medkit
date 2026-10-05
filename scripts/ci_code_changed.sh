#!/usr/bin/env bash
# Copyright 2026 bburda
#
# Prints `code=true` or `code=false` for $GITHUB_OUTPUT. False only on a pull
# request whose every changed file is documentation: under docs/, or a .md or
# .rst file. The gateway checks that read docs/api and the design docs are
# labelled `linter`, and format-lint runs those on every pull request.
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

# --no-renames lists both sides of a rename, so moving a source file under docs/
# still counts as a code change. core.quotePath=false keeps non-ASCII names
# unquoted; a name git still quotes counts as code.
changed=$(git -c core.quotePath=false diff --no-renames --name-only 'HEAD^1' HEAD)
code_files=$(grep -vE '^(docs|\.github|scripts)/|\.(md|rst)$' <<<"${changed}" || true)

if [[ -z "${changed}" || -n "${code_files}" ]]; then
  echo "code=true"
  printf 'Files outside the docs-only set:\n%s\n' "${code_files:-<empty diff>}" >&2
else
  echo "code=false"
  printf 'Docs-only change:\n%s\n' "${changed}" >&2
fi
