# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "ros2_medkit"
copyright = "2025, selfpatch"
author = "bburda"

version = "0.1.0"
release = "0.1.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx_needs",
    "sphinxcontrib.plantuml",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for Sphinx-Needs ------------------------------------------------
needs_types = [
    dict(
        directive="req",
        title="Requirement",
        prefix="REQ_",
        color="#BFD8D2",
        style="node",
    ),
    dict(
        directive="spec",
        title="Specification",
        prefix="SPEC_",
        color="#FEDCD2",
        style="node",
    ),
    dict(
        directive="impl",
        title="Implementation",
        prefix="IMPL_",
        color="#DF744A",
        style="node",
    ),
    dict(
        directive="test",
        title="Test Case",
        prefix="TEST_",
        color="#DCB239",
        style="node",
    ),
]

needs_extra_links = [
    {
        "option": "links",
        "incoming": "is linked by",
        "outgoing": "links to",
    },
    {
        "option": "verifies",
        "incoming": "is verified by",
        "outgoing": "verifies",
        "copy_link": False,
        "allow_dead_links": False,
    },
]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["custom.css"]


# -- Options for PlantUML ----------------------------------------------------
plantuml = "java -Djava.awt.headless=true -jar /usr/share/plantuml/plantuml.jar"

