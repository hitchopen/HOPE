"""Ground-truth pants assets and atomic VRPN profile selection (no ROS)."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from table_side_transform import transform_pose_xyzw


def asset_directory():
    source = Path(__file__).resolve().parents[1] / "config/pants_profiles"
    if source.is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("hope_bringup")) / "config/pants_profiles"


def load_profiles(directory=None):
    root = Path(directory) if directory else asset_directory()
    document = json.loads((root / "profiles.json").read_text())
    if document["schema_version"] != 1 or set(document["profiles"]) != {"v2", "v4"}:
        raise ValueError("unsupported pants catalog")
    profiles = document["profiles"]
    for profile, spec in profiles.items():
        if spec["body_frame"] != "pelvis_link" or spec["centroid_subtracted"]:
            raise ValueError("pants definition must preserve pelvis_link origin")
        if spec["marker_count"] != {"v2": 10, "v4": 17}[profile]:
            raise ValueError("unexpected marker count")
        for side, asset in spec["assets"].items():
            encoded = (root / asset["file"]).read_bytes()
            if hashlib.sha256(encoded).hexdigest() != asset["sha256"]:
                raise ValueError("pants asset hash mismatch")
            points = json.loads(encoded)["UCB_" + side]["markerPose"]
            expected = [v * 1000 for xyz in spec["marker_positions_m"] for v in xyz]
            if len(points) != len(expected) or any(abs(a-b) > 1e-6 for a,b in zip(points, expected)):
                raise ValueError("pants asset differs from ground truth")
    return profiles


@dataclass(frozen=True)
class Selection:
    profile: str = "v2"
    side: str = "P1"
    revision: int = 0

    def __post_init__(self):
        if self.profile not in {"v2", "v4"} or self.side not in {"P1", "P2"}:
            raise ValueError("pants_profile must be v2/v4 and table_side must be P1/P2")
        if type(self.revision) is not int or self.revision < 0:
            raise ValueError("invalid mocap configuration revision")

    @property
    def body(self):
        return "UCB_" + self.side


def update_selection(current, document):
    candidate = Selection(document["pants_profile"], document["table_side"], document["revision"])
    if candidate.revision < current.revision:
        raise ValueError("reordered mocap configuration")
    if candidate.revision == current.revision:
        if candidate != current:
            raise ValueError("mocap configuration changed without a new revision")
        return current
    # The supervisor enforces the same interlock at apply_config. Reject an
    # unexpected in-session change here too. A late subscriber may attach to
    # the already-confirmed initial snapshot even while the runner is active.
    if current.revision > 0 and (document.get("state") != "STOPPED" or document.get("busy") is not False):
        raise ValueError("pants/side changes require stopped lifecycle")
    return candidate


def validate_vrpn_setup(document, *, server, port, scale):
    """A confirmed selection must describe this running receiver, not an old one."""
    if not isinstance(document, dict):
        raise ValueError("invalid mocap configuration snapshot")
    if document.get("mocap_source") != "vrpn":
        raise ValueError("select QingTong / VRPN in Foxglove and confirm configuration")
    if document.get("vrpn_ip") != server or document.get("vrpn_port") != str(port):
        raise ValueError("VRPN endpoint changed; restart the Laptop mocap setup")
    units = document.get("vrpn_units")
    if units not in {"mm", "m"} or scale != {"mm": 0.001, "m": 1.0}[units]:
        raise ValueError("VRPN units changed; restart the Laptop mocap setup")


def normalize_pose(position, quaternion, scale, selection, length=2.74, width=1.525):
    if scale not in (0.001, 1.0):
        raise ValueError("VRPN position_scale must be 0.001 (mm) or 1.0 (m)")
    return transform_pose_xyzw(tuple(v * scale for v in position), quaternion,
                               selection.side, length, width)


def observation(selection, profiles, stamp, position, quaternion):
    """Carry pose and its profile/side atomically to the guarded base adapter."""
    return dict(schema_version=1, pants_profile=selection.profile, table_side=selection.side,
                revision=selection.revision, body=selection.body, stamp=list(stamp),
                position=list(position), quaternion_xyzw=list(quaternion),
                asset_sha256=profiles[selection.profile]["assets"][selection.side]["sha256"])


def validate_observation(document, profiles):
    if document["schema_version"] != 1:
        raise ValueError("unsupported VRPN observation")
    selection = Selection(document["pants_profile"], document["table_side"], document["revision"])
    asset = profiles[selection.profile]["assets"][selection.side]
    if document["body"] != selection.body or document["asset_sha256"] != asset["sha256"]:
        raise ValueError("VRPN selected body/asset mismatch")
    position, quaternion = normalize_pose(document["position"], document["quaternion_xyzw"],
                                           1.0, Selection())  # already side-transformed
    sec, nsec = document["stamp"]
    if type(sec) is not int or type(nsec) is not int or sec <= 0 or not 0 <= nsec < 1000000000:
        raise ValueError("invalid VRPN source timestamp")
    return selection, position, quaternion
