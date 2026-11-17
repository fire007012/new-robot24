"""Resolve portable deployment paths relative to manager.json, never the caller cwd."""
import json
from pathlib import Path


def load_config(path):
    path = Path(path).resolve(strict=True)
    config = json.loads(path.read_text())
    for key in ('config_root','state_dir','token_file','setup','build_manifest'):
        if key in config:
            config[key] = str((path.parent / config[key]).resolve())
    return config
