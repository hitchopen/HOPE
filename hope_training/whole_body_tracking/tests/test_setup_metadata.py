"""Fresh-clone packaging requirements for the train/play entry points."""

import ast
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[1]


def test_editable_install_includes_subpackages_and_hydra_dependencies() -> None:
    source = (_ROOT / "source" / "whole_body_tracking" / "setup.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    requirements = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "INSTALL_REQUIRES"
            for target in node.targets
        ):
            requirements = ast.literal_eval(node.value)
            break
    assert requirements is not None
    assert any(value.startswith("hydra-core") for value in requirements)
    assert any(value.startswith("omegaconf") for value in requirements)
    assert "packages=find_packages()" in source


def test_extension_metadata_is_hope_specific() -> None:
    metadata = (
        _ROOT
        / "source"
        / "whole_body_tracking"
        / "config"
        / "extension.toml"
    ).read_text(encoding="utf-8")
    assert 'title = "HOPE Whole-Body Tracking"' in metadata
    assert 'repository = "https://github.com/hitchopen/HOPE.git"' in metadata
    assert "Extension Template" not in metadata
    assert "IsaacLabExtensionTemplate" not in metadata
