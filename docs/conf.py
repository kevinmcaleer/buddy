# Configuration file for the Sphinx documentation builder.
#
# For a full list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# -- Path setup ---------------------------------------------------------------
# Add the project root so autodoc can find the modules.
sys.path.insert(0, os.path.abspath(".."))

# -- Project information ------------------------------------------------------
project = "Buddy Arm"
copyright = "2026, Kevin McAleer"
author = "Kevin McAleer"
release = "0.8.0"

# -- General configuration ----------------------------------------------------
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output --------------------------------------------------
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
