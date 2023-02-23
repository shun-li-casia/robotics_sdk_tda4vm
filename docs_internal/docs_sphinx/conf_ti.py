# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))
# import os
# import sys
# sys.path.insert(0, os.path.abspath('..'))

import sphinx_rtd_theme

# -- Project information -----------------------------------------------------

project = 'Robotics SDK'
copyright = '2022, Texas Instruments Incorporated'

# The full version, including alpha/beta/rc tags
release = '8.5.0'
version = release

# -- General configuration ---------------------------------------------------
# version <= 1.8.5
# source_suffix = ['.rst', '.md']

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown'
}

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
   'recommonmark',
   'sphinx_markdown_tables',
   "sphinx_rtd_theme"
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [
    '_build',
    'Thumbs.db',
    '.DS_Store',
    'README.md',
    'source/scripts_internal',
    'source/ros1/nodes/ti_viz_nodes',
    'source/ros2/drivers/zed_capture',
    'source/ros2/drivers/mono_capture',
    'source/ros2/nodes/ti_sde',
    'source/ros2/nodes/ti_estop',
    'source/ros2/nodes/ti_vision_cnn',
    'source/ros2/nodes/ti_vl',
    'source/ros1/drivers/gscam/README.md',
    'source/ros1/drivers/gscam/CHANGELOG.rst',
    'source/ros2/drivers/gscam2/README.md',
]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_themes']

# html_theme = 'sphinx_rtd_theme'
# html_theme_options = {
#     'logo_only': False,
#     'display_version': True,
#     'prev_next_buttons_location': 'bottom',
#     'style_external_links': True,
#     # Toc options
#     'collapse_navigation': False,
#     'sticky_navigation': False,
#     'navigation_depth': 3,
#     'includehidden': True,
#     'titles_only': False
# }
# html_title = "Robotics Development Kit Documentation"

html_theme = 'sphinx_rtd_theme_ti'
html_theme_options = {}
html_theme_path = ["_themes"]
html_favicon = '_images/favicon.ico'
html_title = "Robotics SDK Documentation"

# Custom sidebar templates, maps document names to template names.
html_sidebars = { '**': ['globaltoc.html', 'relations.html', 'sourcelink.html',
                         'searchbox.html'], }

# If true, links to the reST sources are added to the pages.
html_show_sourcelink = False

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
html_show_sphinx = False

# If true, "(C) Copyright ..." is shown in the HTML footer. Default is True.
html_show_copyright = True

# Language to be used for generating the HTML full-text search index.
# Sphinx supports the following languages:
#   'da', 'de', 'en', 'es', 'fi', 'fr', 'hu', 'it', 'ja'
#   'nl', 'no', 'pt', 'ro', 'ru', 'sv', 'tr'
html_search_language = 'en'

# The name of a javascript file (relative to the configuration directory) that
# implements a search results scorer. If empty, the default will be used.
html_search_scorer = ''

def setup(app):
    app.add_stylesheet("theme_overrides.css")
