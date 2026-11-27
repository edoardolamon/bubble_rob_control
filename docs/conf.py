# docs/conf.py -- Sphinx configuration, used by:
#   sphinx-build -W --keep-going -b html docs docs/_build/html   (from the repository root)
# Run `doxygen docs/Doxyfile` first: Breathe reads Doxygen's XML.

project = "bubble_rob_control"
author = "SDCR course, University of Trento"
copyright = "2026, University of Trento"

extensions = ["breathe", "myst_parser"]

# Breathe: where Doxygen wrote its XML (relative to this folder)
breathe_projects = {"bubble_rob_control": "_build/doxygen/xml"}
breathe_default_project = "bubble_rob_control"

# MyST: pages are Markdown; ```{directive} blocks call Sphinx directives
source_suffix = {".md": "markdown"}
myst_heading_anchors = 3   # GitHub-style #section links inside the Markdown pages

exclude_patterns = ["_build"]

html_theme = "sphinx_rtd_theme"
html_title = "bubble_rob_control"
