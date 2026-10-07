"""Direct-script compatibility for the shared robot package."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_core.config import PROFILE, profile_hash, load_profile  # noqa: E402,F401
