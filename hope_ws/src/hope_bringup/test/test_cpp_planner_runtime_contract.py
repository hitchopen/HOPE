from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
BRINGUP = REPO_ROOT / "hope_ws/src/hope_bringup"


def test_generic_bringup_owns_exactly_one_cpp_packetizer_and_planner():
    source = (BRINGUP / "launch/hope_bringup.launch.py").read_text(encoding="utf-8")

    assert 'package="hope_planner_cpp"' in source
    assert 'executable="hope_ball_flight_packetizer"' in source
    assert 'executable="hope_planner_cpp_node"' in source
    assert '"start_flight_packetizer": "false"' in source
    assert 'package="hope_planner"' not in source
    assert 'executable="hope_planner_node"' not in source


def test_bringup_package_has_no_runtime_dependency_on_python_planner():
    package_xml = (BRINGUP / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>hope_planner_cpp</exec_depend>" in package_xml
    assert "<exec_depend>hope_planner</exec_depend>" not in package_xml
    assert (REPO_ROOT / "hope_ws/src/hope_planner/COLCON_IGNORE").is_file()

    retired_setup = (REPO_ROOT / "hope_ws/src/hope_planner/setup.py").read_text(
        encoding="utf-8"
    )
    assert "hope_planner_node =" not in retired_setup
    assert "planner_imitate_node =" not in retired_setup


def test_world_base_relay_is_owned_by_bringup():
    source = (BRINGUP / "launch/hope_world.launch.py").read_text(encoding="utf-8")
    assert 'package="hope_bringup"' in source
    assert 'executable="hope_base_pose_flat_relay"' in source
    assert 'package="hope_planner"' not in source
