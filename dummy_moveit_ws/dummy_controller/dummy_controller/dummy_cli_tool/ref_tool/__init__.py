import os
import sys

# Fibre uses absolute imports like `import fibre.protocol`, so the parent of the
# `fibre` package directory must be on sys.path.
_cli_tool_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _cli_tool_dir not in sys.path:
    sys.path.insert(0, _cli_tool_dir)

_legacy_fibre = os.path.join(
    os.path.dirname(os.path.dirname(_cli_tool_dir)),
    "Firmware",
    "fibre",
    "python",
)
if os.path.isdir(_legacy_fibre) and _legacy_fibre not in sys.path:
    sys.path.insert(0, _legacy_fibre)

find_any = None
find_all = None
try:
    import fibre as _fibre

    find_any = _fibre.find_any
    find_all = _fibre.find_all
except Exception:
    pass

from .version import get_version_str

del get_version_str
