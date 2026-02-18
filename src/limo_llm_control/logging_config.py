"""
Central logging for remote PC (ROSA + tools).

Configure once at startup to get one run-dated log file plus console output.
Format is easy to grep and share for debugging.
"""

from __future__ import annotations

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

# Repo root: limo_llm_control -> src -> repo root
_PKG_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _PKG_DIR.parent.parent
DEFAULT_LOG_DIR = _REPO_ROOT / "logs"

# Set after configure_logging() so rosout_bridge can use the same file
_current_log_path: Optional[Path] = None


def get_current_log_path() -> Optional[Path]:
    """Return the path to the current run's log file, or None if not configured."""
    return _current_log_path


def configure_logging(
    log_dir: Optional[Path] = None,
    level: int = logging.INFO,
    console: bool = True,
) -> Path:
    """
    Configure central logging for this run.

    - Creates a run-dated log file under log_dir (default: repo_root/logs).
    - Adds a console handler so you still see output in the terminal.
    - Returns the path to the log file (so you can print "Log file: ..." at startup).

    Call once at application startup (e.g. in main.py).
    """
    global _current_log_path

    log_dir = log_dir or DEFAULT_LOG_DIR
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_path = log_dir / f"rosa_run_{stamp}.log"
    _current_log_path = log_path

    # Write a one-line header so shared logs are self-explanatory
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(f"# LIMO ROSA central log | run started {datetime.now().isoformat()} | file: {log_path}\n")
    # Reopen in append mode for the logging handler
    fh = logging.FileHandler(log_path, encoding="utf-8", mode="a")
    formatter = logging.Formatter(
        fmt="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    root = logging.getLogger("limo_llm_control")
    root.setLevel(level)
    # Avoid duplicate handlers if called twice
    root.handlers.clear()

    fh.setLevel(level)
    fh.setFormatter(formatter)
    root.addHandler(fh)

    if console:
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(level)
        ch.setFormatter(formatter)
        root.addHandler(ch)

    return log_path
