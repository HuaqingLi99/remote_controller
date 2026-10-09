from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

project = "remote_controller"
author = "Huaqing Li"
copyright = "2026, Huaqing Li"
release = "0.1.0"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

html_theme = "furo"

autodoc_default_options = {
    "members": True,
    "undoc-members": False,
    "show-inheritance": True,
    "member-order": "bysource",
}

napoleon_google_docstring = True
napoleon_numpy_docstring = False
