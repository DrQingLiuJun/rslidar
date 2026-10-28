"""Load model YAML with base paths relative to its own location."""

from copy import deepcopy
from pathlib import Path
import yaml
from pcdet.config import cfg, merge_new_config


def load_model_config(filename):
    path = Path(filename).resolve()
    data = yaml.safe_load(path.read_text())

    def resolve(value):
        if isinstance(value, dict):
            return {
                key: (
                    str((path.parent / item).resolve())
                    if key == "_BASE_CONFIG_"
                    else resolve(item)
                )
                for key, item in value.items()
            }
        if isinstance(value, list):
            return [resolve(item) for item in value]
        return value

    config = merge_new_config(deepcopy(cfg), resolve(data))
    config.DATA_CONFIG.DATA_PATH = str(path.parent)
    return config
