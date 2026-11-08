"""The published gripper model must load from a fresh, relocatable checkout."""
from pathlib import Path
import xml.etree.ElementTree as ET


MODEL = Path(__file__).resolve().parents[1] / "assets/a3_runtime/robots/A3PingPong-with-gripper"
URDF = MODEL / "urdf/model.urdf"


def test_single_root_unique_links_and_joints():
    root = ET.parse(URDF).getroot()
    links = [e.attrib["name"] for e in root.findall("link")]
    joints = root.findall("joint")
    assert len(links) == len(set(links))
    assert len(joints) == len({e.attrib["name"] for e in joints})
    parents = {j.find("child").attrib["link"]: j.find("parent").attrib["link"] for j in joints}
    assert len(parents) == len(joints)
    assert set(links) - parents.keys() == {"pelvis_link"}
    for link in links:
        seen = set()
        while link in parents:
            assert link not in seen
            seen.add(link)
            link = parents[link]
        assert link == "pelvis_link"


def test_elbow_and_ankle_pitch_origins():
    root = ET.parse(URDF).getroot()
    expected_origins = {"elbow": (0.01, 0, -0.1325), "ankle_pitch": (0, 0, -0.415)}
    for side in ("left", "right"):
        for joint, xyz in expected_origins.items():
            origin = root.find(f"./joint[@name='{side}_{joint}_joint']/origin")
            assert origin is not None
            assert tuple(map(float, origin.attrib["xyz"].split())) == xyz


def test_every_mesh_reference_is_local_and_present():
    meshes = list(ET.parse(URDF).iter("mesh"))
    assert meshes
    for mesh in meshes:
        filename = mesh.attrib["filename"]
        assert not Path(filename).is_absolute()
        path = (URDF.parent / filename).resolve()
        assert path.is_relative_to(MODEL.resolve())
        assert path.is_file(), filename
