"""Sphinx 설정. 빌드: sphinx-build -b html docs docs/_build/html  (의존성: docs/requirements.txt)"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

project = "rover-lowlevel"
author = "shh444"
release = "0.1.0"
language = "ko"

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]
autodoc_mock_imports = ["mujoco", "dds_middleware_python"]   # 문서 빌드 환경에는 없어도 된다
autodoc_member_order = "bysource"
autodoc_default_options = {"members": True, "undoc-members": True, "show-inheritance": True}
autodoc_typehints = "description"

myst_enable_extensions = ["colon_fence", "deflist"]
myst_heading_anchors = 3
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "furo"
html_title = "Rover 저수준 제어 실습"
html_static_path = []
html_theme_options = {
    "source_repository": "https://github.com/shh444/rover-lowlevel",
    "source_branch": "main",
    "source_directory": "docs/",
}
