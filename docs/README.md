# Documentation Development Guide

This directory contains the Sphinx-based documentation for the `ros2_medkit` project.

## Prerequisites

*   **Python 3.12+**
*   **PlantUML** and **Graphviz** (for generating diagrams)

### Installing System Dependencies (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install -y graphviz plantuml
```

## Setup

It is recommended to use a virtual environment for building the documentation.

1.  Create a virtual environment:
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

2.  Install Python dependencies:
    ```bash
    pip install -e .[dev]
    ```

## Building Documentation

To build the HTML documentation, run the following command from the `docs` directory:

```bash
sphinx-build -b html . _build/html
```

The generated HTML files will be located in `_build/html/index.html`.

## Live Preview (Development Mode)

For a better development experience, you can use `sphinx-autobuild` to automatically rebuild the documentation and refresh your browser when you make changes.

1.  Ensure you are in the `docs` directory and your virtual environment is activated:

    ```bash
    # Assuming you are in the project root
    cd docs
    source .venv/bin/activate
    ```

2.  Run the auto-build server:

    ```bash
    sphinx-autobuild . _build/html --port 8000 --host 0.0.0.0
    ```

Then open [http://127.0.0.1:8000](http://127.0.0.1:8000) in your browser.

## Directory Structure

*   `pyproject.toml`: Project configuration and dependencies.
*   `conf.py`: Sphinx configuration file.
*   `index.rst`: Main entry point for the documentation.
*   `requirements/`: Contains requirements specifications (split by category).
*   `design/`: Contains design documentation and PlantUML diagrams.
*   `_static/`: Custom static files (CSS, JS).
*   `_templates/`: Custom templates.

## Adding New Requirements

Requirements are managed using `sphinx-needs`. To add a new requirement:

1.  Identify the appropriate file in `requirements/` (e.g., `core.rst`, `system.rst`).
2.  Add a new `.. req::` directive:

    ```rst
    .. req:: Requirement Title
       :id: REQ_CATEGORY_XXX
       :status: open
       :tags: P

       Description of the requirement.
    ```

## CI/CD

The documentation is automatically built and deployed to GitHub Pages via GitHub Actions. The workflow is defined in `.github/workflows/docs.yml`.
