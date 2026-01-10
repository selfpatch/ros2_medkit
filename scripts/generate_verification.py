"""Generate verification.rst from test files with @verifies tags."""

import os
from pathlib import Path
import re

# Determine workspace root relative to this script
# Script is in <workspace>/scripts/generate_verification.py
SCRIPT_DIR = Path(__file__).parent.resolve()
WORKSPACE_DIR = SCRIPT_DIR.parent
SRC_DIR = WORKSPACE_DIR / "src"
OUTPUT_FILE = WORKSPACE_DIR / "docs/requirements/verification.rst"


def parse_cpp_file(file_path):
    """Parse C++ test file for @verifies tags."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Regex to find TEST or TEST_F blocks
    # Matches: TEST_F(Suite, Name) { ... }
    test_pattern = re.compile(r"TEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)\s*\{")

    tests = []

    for match in test_pattern.finditer(content):
        test_name = match.group(2)
        start_index = match.end()

        # Extract a chunk of text after the match to search for comments
        search_window = content[start_index : start_index + 2000]
        lines = search_window.split("\n")

        # Auto-generate ID and Title
        test_id = "TEST_" + test_name
        test_title = test_name

        verifies_reqs = []
        description = []

        for line in lines:
            line = line.strip()
            if not line:
                continue

            # Stop if we hit code (not a comment)
            if not line.startswith("//"):
                # If it's an opening brace on a new line, ignore
                if line == "{":
                    continue
                break

            comment_content = line[2:].strip()

            # Parse tags
            # Support '@verifies REQ_...' (list) or 'Links to: REQ_...'
            tag_match = re.match(
                r"(?:@verifies|Links to:|Verifies:)\s*(.*)", comment_content
            )

            if tag_match:
                reqs_text = tag_match.group(1)
                # Extract anything looking like REQ_\w+
                reqs = re.findall(r"(REQ_\w+)", reqs_text)
                verifies_reqs.extend(reqs)
            else:
                # Treat other comments as description
                description.append(comment_content)

        if verifies_reqs:
            tests.append(
                {
                    "id": test_id,
                    "title": test_title,
                    "verifies": list(set(verifies_reqs)),
                    "description": "\n   ".join(description),
                    "file": str(file_path.relative_to(WORKSPACE_DIR)),
                    "test_func": test_name,
                }
            )

    return tests


def parse_py_file(file_path):
    """Parse Python test file for @verifies tags."""
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Regex to find python test methods
    # Matches: def test_something(self):
    test_pattern = re.compile(r"def\s+(test_\w+)\s*\(self\):")

    tests = []

    for match in test_pattern.finditer(content):
        test_name = match.group(1)
        start_index = match.end()

        # Extract a chunk of text after the match to search for docstrings/comments
        search_window = content[start_index : start_index + 2000]
        lines = search_window.split("\n")

        # Auto-generate ID and Title
        test_id = "TEST_" + test_name
        test_title = test_name

        verifies_reqs = []
        description = []

        in_docstring = False
        docstring_lines = []

        for line in lines:
            stripped = line.strip()
            if not stripped:
                continue

            # Check for docstring start/end
            if '"""' in stripped or "'''" in stripped:
                if in_docstring:
                    in_docstring = False
                    # End of docstring, parse collected lines
                    for dline in docstring_lines:
                        # Parse tags in docstring
                        tag_match = re.match(
                            r"(?:@verifies|Links to:|Verifies:)\s*(.*)", dline
                        )

                        if tag_match:
                            reqs_text = tag_match.group(1)
                            reqs = re.findall(r"(REQ_\w+)", reqs_text)
                            verifies_reqs.extend(reqs)
                        else:
                            description.append(dline)
                else:
                    in_docstring = True
                    # Handle one-line docstrings
                    if stripped.count('"""') == 2 or stripped.count("'''") == 2:
                        content_line = (
                            stripped.replace('"""', "").replace("'''", "").strip()
                        )
                        docstring_lines.append(content_line)
                        in_docstring = False
                        # Parse immediately
                        dline = content_line
                        tag_match = re.match(
                            r"(?:@verifies|Links to:|Verifies:)\s*(.*)", dline
                        )
                        if tag_match:
                            reqs_text = tag_match.group(1)
                            reqs = re.findall(r"(REQ_\w+)", reqs_text)
                            verifies_reqs.extend(reqs)
                        else:
                            description.append(dline)
                continue

            if in_docstring:
                docstring_lines.append(stripped)
                continue

            # Also check for comments #
            if stripped.startswith("#"):
                comment_content = stripped[1:].strip()
                tag_match = re.match(
                    r"(?:@verifies|Links to:|Verifies:)\s*(.*)", comment_content
                )

                if tag_match:
                    reqs_text = tag_match.group(1)
                    reqs = re.findall(r"(REQ_\w+)", reqs_text)
                    verifies_reqs.extend(reqs)
                continue

            # If we hit code that is not comment or docstring, stop
            if not in_docstring and not stripped.startswith("#"):
                break

        if verifies_reqs:
            tests.append(
                {
                    "id": test_id,
                    "title": test_title,
                    "verifies": list(set(verifies_reqs)),
                    "description": "\n   ".join(description),
                    "file": str(file_path.relative_to(WORKSPACE_DIR)),
                    "test_func": test_name,
                }
            )

    return tests


def generate_rst(tests):
    """Generate RST content from parsed tests."""
    lines = [
        "Verification",
        "============",
        "",
        "This section documents the test cases and their traceability to requirements.",
        "It is automatically generated from the source code.",
        "",
    ]

    for test in tests:
        lines.append(".. test:: " + test["title"])
        lines.append("   :id: " + test["id"])
        lines.append("   :status: verified")
        if test["verifies"]:
            lines.append("   :verifies: " + ", ".join(test["verifies"]))
        lines.append("")
        if test["description"]:
            lines.append("   " + test["description"])
            lines.append("")

        lines.append(
            "   **Implementation:** ``" + test["file"] + "`` "
            "(Test: ``" + test["test_func"] + "``)"
        )
        lines.append("")
        lines.append("")

    lines.append(".. needtable::")
    lines.append("   :filter: type == 'test'")
    lines.append("   :columns: id, title, status, verifies")
    lines.append("   :style: table")
    lines.append("")

    return "\n".join(lines)


def main():
    """Scan tests and generate verification.rst."""
    all_tests = []
    if not SRC_DIR.exists():
        print("Source directory " + str(SRC_DIR) + " does not exist.")
        return

    for root, dirs, files in os.walk(SRC_DIR):
        for file in files:
            file_path = Path(root) / file
            if file.endswith(".cpp"):
                all_tests.extend(parse_cpp_file(file_path))
            elif file.endswith(".py"):
                all_tests.extend(parse_py_file(file_path))

    # Sort by ID for consistency
    all_tests.sort(key=lambda x: x["id"])

    rst_content = generate_rst(all_tests)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write(rst_content)

    print("Generated " + str(OUTPUT_FILE) + " with " + str(len(all_tests)) + " tests.")


if __name__ == "__main__":
    main()
