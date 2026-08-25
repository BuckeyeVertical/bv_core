"""Keep the BV mission venv isolated from per-user Python packages."""

import site
import sys
from pathlib import Path


user_site = site.getusersitepackages()
sys.path[:] = [path for path in sys.path if path != user_site]

# Debian preloads its mpl_toolkits namespace before the venv is processed.
toolkits = sys.modules.get("mpl_toolkits")
venv_toolkits = str(Path(__file__).parent / "mpl_toolkits")
if toolkits is not None and venv_toolkits not in toolkits.__path__:
    toolkits.__path__.insert(0, venv_toolkits)
