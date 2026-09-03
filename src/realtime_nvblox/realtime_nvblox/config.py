from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def deep_merge(base: dict, override: dict) -> dict:
    out = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    default_path = Path(__file__).resolve().parent / 'default.yaml'
    if not default_path.exists():
        # Installed ROS/pip package: setup.py installs config in share; caller should pass --config.
        defaults = {}
    else:
        with default_path.open('r', encoding='utf-8') as f:
            defaults = yaml.safe_load(f) or {}
    if path is None:
        if not defaults:
            raise FileNotFoundError('No config path supplied and source-tree default.yaml was not found.')
        return defaults
    with Path(path).expanduser().open('r', encoding='utf-8') as f:
        custom = yaml.safe_load(f) or {}
    return deep_merge(defaults, custom)
