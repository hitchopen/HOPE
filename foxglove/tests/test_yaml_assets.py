"""Parse every checked-in Foxglove YAML asset, independently of ROS availability."""
from pathlib import Path

import pytest
import yaml


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
YAML_FILES = sorted([*FOXGLOVE_DIR.rglob("*.yaml"), *FOXGLOVE_DIR.rglob("*.yml")])


@pytest.mark.parametrize("path", YAML_FILES, ids=lambda path: str(path.relative_to(FOXGLOVE_DIR)))
def test_yaml_asset_is_parseable(path):
    # A stream gives parser errors the actual filename as well as line/column.
    # PyYAML is required: missing syntax validation must fail, never silently skip.
    with path.open(encoding="utf-8") as source:
        yaml.safe_load(source)
