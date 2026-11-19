"""Keep every test away from ~/data: data goes to a temporary folder."""
import os
import tempfile
from pathlib import Path

_TMP = Path(tempfile.mkdtemp(prefix="tests_"))
os.environ["DATA_DIR"] = str(_TMP / "data")
