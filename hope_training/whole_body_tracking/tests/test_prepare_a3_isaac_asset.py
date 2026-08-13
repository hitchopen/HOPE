"""Regression tests for the fresh-clone A3 Isaac asset preparation path."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[1]
_SCRIPT = _ROOT / "scripts" / "prepare_a3_isaac_asset.py"


def _load_prepare_module():
    spec = importlib.util.spec_from_file_location("hope_prepare_a3_asset", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


prepare_asset = _load_prepare_module()


def _write_source(root: Path, movable_axis: str = "0 0 1") -> None:
    (root / "urdf").mkdir(parents=True)
    (root / "meshes").mkdir(parents=True)
    (root / "meshes" / "body.STL").write_bytes(b"solid body\nendsolid body\n")
    (root / "urdf" / "model.urdf").write_text(
        f"""<?xml version="1.0"?>
<robot name="fixture">
  <link name="base"><visual><geometry><mesh filename="package://fixture/meshes/body.STL"/></geometry></visual></link>
  <link name="fixed_child"/>
  <link name="moving_child"/>
  <link name="pingbang_ball_Link"/>
  <joint name="fixed_joint" type="fixed">
    <parent link="base"/><child link="fixed_child"/><axis xyz="0.0"/>
  </joint>
  <joint name="moving_joint" type="revolute">
    <parent link="fixed_child"/><child link="moving_child"/><axis xyz="{movable_axis}"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="pingbang_ball_joint" type="fixed">
    <parent link="fixed_child"/><child link="pingbang_ball_Link"/><axis xyz="0 0 0"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )


def test_prepare_removes_invalid_fixed_axis_and_rewrites_meshes() -> None:
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        source = tmp / "source"
        output = tmp / "output"
        _write_source(source)

        prepare_asset.prepare(source, output, force=True)

        model = output / "urdf" / "model.urdf"
        root = ET.parse(model).getroot()
        fixed = root.find(".//joint[@name='fixed_joint']")
        moving = root.find(".//joint[@name='moving_joint']")
        mesh = root.find(".//mesh")
        assert fixed.find("axis") is None
        assert moving.find("axis").get("xyz") == "0 0 1"
        assert root.find(".//link[@name='pingbang_ball_Link']") is None
        assert root.find(".//joint[@name='pingbang_ball_joint']") is None
        assert mesh.get("filename") == "../meshes/body.STL"
        assert prepare_asset.check(output)


def test_check_rejects_an_invalid_movable_axis() -> None:
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        source = tmp / "source"
        output = tmp / "output"
        _write_source(source, movable_axis="0.0")
        prepare_asset.prepare(source, output, force=True)
        assert not prepare_asset.check(output)
