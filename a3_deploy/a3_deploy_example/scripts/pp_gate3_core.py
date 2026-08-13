#!/usr/bin/env python3
"""Pure helpers for the autonomous, physical-ball Gate3.

This module deliberately has no ROS dependency so the scenario, evidence, and
per-side verdict contracts can be unit-tested on the host.  A scenario contains
only initial ball state plus ``shot_id``; swing side is always supplied later by
the production planner/runner evidence.
"""

from __future__ import annotations

import ast
import math
import random
from dataclasses import asdict, dataclass, field
from typing import Any, Iterable, Mapping, Sequence


TABLE_LENGTH_M = 2.74
TABLE_WIDTH_M = 1.525
TABLE_Y_MAX_M = 0.0
TABLE_HEIGHT_M = 0.760
NET_X_M = 1.370
NET_HEIGHT_M = 0.1525
BALL_RADIUS_M = 0.020
# Contact counters are loss-detectable, but their accompanying ball pose is the
# first 250 Hz sample after the 1 kHz contact edge.  Accept only a tightly
# bounded observation delay and account for the distance the rebounding ball
# can travel during that measured delay.
CONTACT_EDGE_MAX_OBSERVATION_LAG_S = 0.010


@dataclass(frozen=True)
class ServeSpec:
    """One side-neutral initial state expressed in the table-surface frame."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]

    def world_position(self, table_height_m: float = TABLE_HEIGHT_M) -> tuple[float, float, float]:
        return (
            self.position[0],
            self.position[1],
            self.position[2] + float(table_height_m),
        )


def _finite_float(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite, got {value!r}")
    return result


def parse_serves_list(value: str | Sequence[float]) -> list[ServeSpec]:
    """Parse a flat ``6*N`` list without accepting side labels or dictionaries."""
    parsed: Any = ast.literal_eval(value) if isinstance(value, str) else value
    if not isinstance(parsed, (list, tuple)):
        raise ValueError("Gate3 serves must be a flat numeric list")
    if any(isinstance(item, (dict, list, tuple)) for item in parsed):
        raise ValueError(
            "Gate3 serves may contain only initial p/v numbers; side labels are forbidden"
        )
    flat = [_finite_float(item, f"serves[{index}]") for index, item in enumerate(parsed)]
    if not flat or len(flat) % 6:
        raise ValueError(
            f"Gate3 serves must contain exactly 6*N numbers, got {len(flat)}"
        )
    return [
        ServeSpec(tuple(flat[index : index + 3]), tuple(flat[index + 3 : index + 6]))
        for index in range(0, len(flat), 6)
    ]


def generate_v17_r10_random_serves(count: int, seed: int) -> list[ServeSpec]:
    """Build a reproducible, balanced, side-neutral fixed-station Gate3 sweep.

    The two lateral lanes sit inside the exported R10 FH/BH strike-position
    support when the immutable session anchor is ``y=-0.7625``.  We perturb
    launch position and velocity, but never encode a side in the wire format;
    the production planner still owns side selection from the measured path.
    Consecutive pairs contain one sample from each lane so a long test cannot
    accidentally become a one-sided draw.
    """
    if int(count) != count or count < 8:
        raise ValueError("V17-r10 random Gate3 requires at least 8 serves")
    rng = random.Random(int(seed))
    result: list[ServeSpec] = []
    # With station_y=-0.7625 these centers map to R10 reach-y -0.44/-0.09.
    lane_centers = (-1.2025, -0.8525)
    for pair_index in range((int(count) + 1) // 2):
        lane_order = [0, 1]
        if rng.random() < 0.5:
            lane_order.reverse()
        for lane in lane_order:
            if len(result) >= int(count):
                break
            # Keep every draw within the training position support while
            # varying the incoming trajectory enough to expose brittle timing.
            x = rng.uniform(2.36, 2.44)
            y = lane_centers[lane] + rng.uniform(-0.030, 0.030)
            z = rng.uniform(0.485, 0.510)
            vx = rng.uniform(-3.12, -2.96)
            vy = rng.uniform(-0.035, 0.035)
            vz = rng.uniform(2.08, 2.28)
            result.append(ServeSpec((x, y, z), (vx, vy, vz)))
    return result


def serves_to_flat_list(serves: Sequence[ServeSpec]) -> list[float]:
    """Serialize side-neutral serve specs to the existing flat ``6*N`` wire."""
    return [
        value
        for spec in serves
        for value in (*spec.position, *spec.velocity)
    ]


def table_to_world_position(
    position: Sequence[float], table_height_m: float = TABLE_HEIGHT_M
) -> tuple[float, float, float]:
    if len(position) != 3:
        raise ValueError("position must have three values")
    values = tuple(_finite_float(value, "position") for value in position)
    return values[0], values[1], values[2] + float(table_height_m)


def world_to_table_position(
    position: Sequence[float], table_height_m: float = TABLE_HEIGHT_M
) -> tuple[float, float, float]:
    if len(position) != 3:
        raise ValueError("position must have three values")
    values = tuple(_finite_float(value, "position") for value in position)
    return values[0], values[1], values[2] - float(table_height_m)


def _normalize_quaternion_wxyz(
    quaternion_wxyz: Sequence[float], label: str
) -> tuple[float, float, float, float]:
    values = tuple(_finite_float(value, label) for value in quaternion_wxyz)
    if len(values) != 4:
        raise ValueError(f"{label} quaternion must contain four values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 0.5 or norm > 1.5:
        raise ValueError(f"{label} quaternion norm is outside [0.5,1.5]")
    return tuple(value / norm for value in values)


def _quat_mul_wxyz(
    lhs: Sequence[float], rhs: Sequence[float]
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = lhs
    bw, bx, by, bz = rhs
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_rotate_wxyz(
    quaternion_wxyz: Sequence[float], vector_xyz: Sequence[float]
) -> tuple[float, float, float]:
    qw, qx, qy, qz = quaternion_wxyz
    vx, vy, vz = vector_xyz
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * vx
        + 2.0 * (qx * qy - qw * qz) * vy
        + 2.0 * (qx * qz + qw * qy) * vz,
        2.0 * (qx * qy + qw * qz) * vx
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * vy
        + 2.0 * (qy * qz - qw * qx) * vz,
        2.0 * (qx * qz - qw * qy) * vx
        + 2.0 * (qy * qz + qw * qx) * vy
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * vz,
    )


def base_pose_to_marker_pose(
    base_position_xyz: Sequence[float],
    base_quaternion_wxyz: Sequence[float],
    marker_to_base_xyz: Sequence[float],
    marker_to_base_quaternion_wxyz: Sequence[float],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float, float],
]:
    """Invert ``T_world_base = T_world_marker * T_marker_base``.

    Gate3's MuJoCo source reports the pelvis pose, while the production
    OptiTrack boundary reports the P1 rigid-body marker.  Publishing the
    inverted marker pose makes the exact calibrated production relay reconstruct
    the original MuJoCo pelvis pose instead of relying on an identity shortcut.
    """
    base_position = tuple(
        _finite_float(value, "base_position") for value in base_position_xyz
    )
    marker_to_base = tuple(
        _finite_float(value, "marker_to_base_xyz")
        for value in marker_to_base_xyz
    )
    if len(base_position) != 3 or len(marker_to_base) != 3:
        raise ValueError("base position and marker-to-base translation must be 3-D")
    q_world_base = _normalize_quaternion_wxyz(
        base_quaternion_wxyz, "base"
    )
    q_marker_base = _normalize_quaternion_wxyz(
        marker_to_base_quaternion_wxyz, "marker-to-base"
    )
    q_base_marker = (
        q_marker_base[0],
        -q_marker_base[1],
        -q_marker_base[2],
        -q_marker_base[3],
    )
    q_world_marker = _normalize_quaternion_wxyz(
        _quat_mul_wxyz(q_world_base, q_base_marker), "world-to-marker"
    )
    world_offset = _quat_rotate_wxyz(q_world_marker, marker_to_base)
    marker_position = tuple(
        base_position[index] - world_offset[index] for index in range(3)
    )
    return marker_position, q_world_marker


def calibrated_p1_marker_contract(
    config: Mapping[str, Any],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float, float],
]:
    """Resolve a calibrated P1 marker contract or fail closed."""
    world = config.get("hope_world", config)
    if not isinstance(world, Mapping):
        raise ValueError("hope_world configuration must be a mapping")
    contract = world.get("contract")
    offsets = world.get("mocap_to_base_link")
    if not isinstance(contract, Mapping) or not isinstance(offsets, Mapping):
        raise ValueError("hope_world contract/mocap_to_base_link is missing")
    p1 = offsets.get("p1")
    if not isinstance(p1, Mapping):
        raise ValueError("hope_world P1 marker contract is missing")
    if contract.get("venue_calibrated") is not True:
        raise ValueError("Motive world frame is not calibrated")
    if p1.get("calibrated") is not True:
        raise ValueError("P1 marker-to-base transform is not calibrated")
    for value, label in (
        (contract.get("calibration_sha256", ""), "world-frame receipt"),
        (p1.get("calibration_sha256", ""), "P1 marker receipt"),
    ):
        receipt = str(value)
        if len(receipt) != 64 or any(
            character not in "0123456789abcdef" for character in receipt
        ):
            raise ValueError(f"{label} must be a lowercase SHA256")
    translation = tuple(
        _finite_float(value, "P1 marker translation")
        for value in p1.get("xyz_m", ())
    )
    if len(translation) != 3:
        raise ValueError("P1 marker translation must contain three values")
    quaternion = _normalize_quaternion_wxyz(
        p1.get("quaternion_wxyz", ()), "P1 marker-to-base"
    )
    return translation, quaternion


def select_swing_side(
    intercept_y: float,
    base_y: float,
    previous_side: str | None,
    split_y: float = -0.25,
    hysteresis_y: float = 0.04,
) -> str:
    """Mirror the production planner's latched FH/BH split."""
    rel_y = float(intercept_y) - float(base_y)
    lo = float(split_y) - float(hysteresis_y)
    hi = float(split_y) + float(hysteresis_y)
    if previous_side == "forehand":
        return "backhand" if rel_y > hi else "forehand"
    if previous_side == "backhand":
        return "forehand" if rel_y < lo else "backhand"
    return "forehand" if rel_y < float(split_y) else "backhand"


@dataclass
class _Shot:
    shot_id: int
    samples: int = 0
    active_seen: bool = False
    inactive_seen: bool = False
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    max_gap_s: float = 0.0
    counter_monotonic: bool = True
    counter_jump: bool = False
    last_racket_count: int = 0
    last_table_count: int = 0
    last_net_count: int = 0
    racket_events: list[dict[str, Any]] = field(default_factory=list)
    incoming_table_events: list[dict[str, Any]] = field(default_factory=list)
    post_racket_table_events: list[dict[str, Any]] = field(default_factory=list)
    incoming_net_events: list[dict[str, Any]] = field(default_factory=list)
    post_racket_net_events: list[dict[str, Any]] = field(default_factory=list)
    peak_racket_force_n: float = 0.0


class PhysicalEvidenceAccumulator:
    """Accumulate loss-detectable plant contact and landing evidence by shot."""

    def __init__(
        self,
        expected_shot_ids: Iterable[int],
        *,
        min_samples: int = 20,
        max_sample_gap_s: float = 0.050,
    ) -> None:
        expected = [int(value) for value in expected_shot_ids]
        if not expected or any(value <= 0 for value in expected):
            raise ValueError("expected shot IDs must be positive")
        if len(set(expected)) != len(expected):
            raise ValueError("expected shot IDs must be unique")
        self.expected_shot_ids = expected
        self.min_samples = int(min_samples)
        self.max_sample_gap_s = float(max_sample_gap_s)
        self._shots: dict[int, _Shot] = {}
        self.unexpected_shot_ids: set[int] = set()

    @staticmethod
    def _event(
        stamp_ns: int,
        position: Sequence[float],
        velocity: Sequence[float],
        count: int,
        observation_lag_s: float,
    ) -> dict[str, Any]:
        return {
            "stamp_ns": int(stamp_ns),
            "position_world": [float(value) for value in position],
            "position_table": list(world_to_table_position(position)),
            "velocity_world": [float(value) for value in velocity],
            "count": int(count),
            "edge_observation_lag_s": float(observation_lag_s),
        }

    def ingest(
        self,
        *,
        stamp_ns: int,
        shot_id: int,
        active: bool,
        position: Sequence[float],
        velocity: Sequence[float],
        racket_contact_count: int,
        table_contact_count: int,
        net_contact_count: int,
        racket_normal_force_n: float = 0.0,
    ) -> None:
        shot_id = int(shot_id)
        if shot_id <= 0:
            return
        if shot_id not in self.expected_shot_ids:
            self.unexpected_shot_ids.add(shot_id)
        shot = self._shots.setdefault(shot_id, _Shot(shot_id=shot_id))
        stamp_ns = int(stamp_ns)
        observation_lag_s = 0.0
        if shot.last_stamp_ns is not None:
            if stamp_ns <= shot.last_stamp_ns:
                shot.counter_monotonic = False
            else:
                observation_lag_s = (
                    stamp_ns - shot.last_stamp_ns
                ) * 1.0e-9
                shot.max_gap_s = max(
                    shot.max_gap_s, observation_lag_s
                )
        shot.first_stamp_ns = stamp_ns if shot.first_stamp_ns is None else shot.first_stamp_ns
        shot.last_stamp_ns = stamp_ns
        shot.samples += 1
        shot.active_seen |= bool(active)
        shot.inactive_seen |= not bool(active)
        shot.peak_racket_force_n = max(
            shot.peak_racket_force_n, max(0.0, float(racket_normal_force_n))
        )
        # Parking intentionally resets the plant counters to zero.  It closes
        # the observation window; it is not a counter regression.
        if not active:
            return

        counts = (
            int(racket_contact_count),
            int(table_contact_count),
            int(net_contact_count),
        )
        previous = (
            shot.last_racket_count,
            shot.last_table_count,
            shot.last_net_count,
        )
        if any(current < old for current, old in zip(counts, previous)):
            shot.counter_monotonic = False
        if any(current - old > 1 for current, old in zip(counts, previous)):
            shot.counter_jump = True

        racket_delta = max(0, counts[0] - previous[0])
        table_delta = max(0, counts[1] - previous[1])
        net_delta = max(0, counts[2] - previous[2])
        had_racket = previous[0] > 0
        event = self._event(
            stamp_ns, position, velocity, 0, observation_lag_s
        )
        for offset in range(racket_delta):
            item = dict(event)
            item["count"] = previous[0] + offset + 1
            shot.racket_events.append(item)
        for offset in range(table_delta):
            item = dict(event)
            item["count"] = previous[1] + offset + 1
            (
                shot.post_racket_table_events
                if had_racket or racket_delta
                else shot.incoming_table_events
            ).append(item)
        for offset in range(net_delta):
            item = dict(event)
            item["count"] = previous[2] + offset + 1
            (
                shot.post_racket_net_events
                if had_racket or racket_delta
                else shot.incoming_net_events
            ).append(item)

        shot.last_racket_count, shot.last_table_count, shot.last_net_count = counts

    @staticmethod
    def _legal_table_top_event(
        event: Mapping[str, Any] | None,
        *,
        x_min: float,
        x_max: float,
        expected_vx_sign: int,
    ) -> bool:
        if event is None:
            return False
        x, y, z = (float(value) for value in event["position_world"])
        vx, _, vz = (float(value) for value in event["velocity_world"])
        observation_lag_s = float(
            event.get("edge_observation_lag_s", 0.0)
        )
        if not (
            0.0
            <= observation_lag_s
            <= CONTACT_EDGE_MAX_OBSERVATION_LAG_S
        ):
            return False
        top_center_z = TABLE_HEIGHT_M + BALL_RADIUS_M + 1.0e-4
        # The contact occurred somewhere since the prior sample.  The edge
        # sample is post-impulse, so its center can already be above the table
        # by roughly |vz|*lag.  The fixed 5 mm term covers contact softness;
        # the dynamic term accounts only for the measured sample interval.
        z_tolerance = (
            0.005
            + abs(vz) * observation_lag_s
            + 0.5 * 9.81 * observation_lag_s * observation_lag_s
        )
        return bool(
            x_min <= x <= x_max
            and TABLE_Y_MAX_M - TABLE_WIDTH_M + BALL_RADIUS_M
            <= y
            <= TABLE_Y_MAX_M - BALL_RADIUS_M
            and abs(z - top_center_z) <= z_tolerance
            and vz > 0.0
            and expected_vx_sign * vx > 0.0
        )

    @classmethod
    def _legal_incoming_bounce(cls, event: Mapping[str, Any] | None) -> bool:
        return cls._legal_table_top_event(
            event,
            x_min=BALL_RADIUS_M,
            x_max=NET_X_M - BALL_RADIUS_M,
            expected_vx_sign=-1,
        )

    @classmethod
    def _legal_landing(cls, event: Mapping[str, Any] | None) -> bool:
        return cls._legal_table_top_event(
            event,
            x_min=NET_X_M + BALL_RADIUS_M,
            x_max=TABLE_LENGTH_M - BALL_RADIUS_M,
            expected_vx_sign=1,
        )

    def report(self) -> dict[str, Any]:
        rows: list[dict[str, Any]] = []
        for shot_id in self.expected_shot_ids:
            shot = self._shots.get(shot_id, _Shot(shot_id=shot_id))
            landing = (
                shot.post_racket_table_events[0]
                if shot.post_racket_table_events
                else None
            )
            telemetry_complete = bool(
                shot.samples >= self.min_samples
                and shot.active_seen
                and shot.inactive_seen
                and shot.counter_monotonic
                and not shot.counter_jump
                and shot.max_gap_s <= self.max_sample_gap_s
            )
            incoming_bounce = (
                shot.incoming_table_events[0]
                if len(shot.incoming_table_events) == 1
                else None
            )
            incoming_bounce_pass = self._legal_incoming_bounce(incoming_bounce)
            contact_pass = bool(
                telemetry_complete
                and len(shot.incoming_table_events) == 1
                and incoming_bounce_pass
                and len(shot.incoming_net_events) == 0
                and len(shot.racket_events) == 1
                and float(shot.racket_events[0]["velocity_world"][0]) > 0.0
            )
            landing_pass = bool(
                contact_pass
                and landing is not None
                and not shot.post_racket_net_events
                and self._legal_landing(landing)
            )
            rows.append(
                {
                    "shot_id": shot_id,
                    "samples": shot.samples,
                    "first_stamp_ns": shot.first_stamp_ns,
                    "last_stamp_ns": shot.last_stamp_ns,
                    "max_sample_gap_s": shot.max_gap_s,
                    "telemetry_complete": telemetry_complete,
                    "counter_monotonic": shot.counter_monotonic,
                    "counter_jump": shot.counter_jump,
                    "incoming_table_events": shot.incoming_table_events,
                    "incoming_bounce_pass": incoming_bounce_pass,
                    "incoming_net_events": shot.incoming_net_events,
                    "racket_events": shot.racket_events,
                    "post_racket_table_events": shot.post_racket_table_events,
                    "post_racket_net_events": shot.post_racket_net_events,
                    "racket_contact_count": len(shot.racket_events),
                    "peak_racket_force_n": shot.peak_racket_force_n,
                    "landing_event": landing,
                    "contact_pass": contact_pass,
                    "landing_pass": landing_pass,
                }
            )
        measured = bool(
            rows
            and all(row["telemetry_complete"] for row in rows)
            and not self.unexpected_shot_ids
        )
        return {
            "schema_version": 1,
            "source": "mujoco_1khz_contact_edges",
            "expected_shot_ids": self.expected_shot_ids,
            "unexpected_shot_ids": sorted(self.unexpected_shot_ids),
            "min_samples_per_shot": self.min_samples,
            "max_sample_gap_limit_s": self.max_sample_gap_s,
            "physical_contact_measured": measured,
            "landing_measured": measured,
            "physical_contact_pass": bool(
                measured and all(row["contact_pass"] for row in rows)
            ),
            "landing_pass": bool(
                measured and all(row["landing_pass"] for row in rows)
            ),
            "rows": rows,
        }


def join_physical_evidence_by_side(
    serve_rows: Sequence[Mapping[str, Any]],
    physical_report: Mapping[str, Any],
    *,
    min_samples_per_side: int,
    min_contact_rate: float,
    min_landing_rate: float,
    exact_samples_per_side: int | None = None,
    min_contacts_per_side: int = 0,
    min_landings_per_side: int = 0,
    min_global_contacts: int = 0,
    min_global_landings: int = 0,
) -> dict[str, Any]:
    """Join measured outcomes to the planner-selected side; never average sides."""
    physical_rows = list(physical_report.get("rows", []))
    physical_row_ids = [int(row.get("shot_id") or 0) for row in physical_rows]
    physical_by_id = {
        int(row["shot_id"]): row for row in physical_rows
        if int(row.get("shot_id") or 0) > 0
    }
    serve_ids = [int(row.get("shot_id") or 0) for row in serve_rows]
    duplicate_serve_ids = sorted({
        shot_id for shot_id in serve_ids
        if shot_id > 0 and serve_ids.count(shot_id) > 1
    })
    duplicate_physical_ids = sorted({
        shot_id for shot_id in physical_row_ids
        if shot_id > 0 and physical_row_ids.count(shot_id) > 1
    })
    extra_physical_ids = sorted(set(physical_by_id) - set(serve_ids))
    per_side: dict[str, dict[str, Any]] = {}
    missing_shot_ids: list[int] = []
    unassigned_shot_ids: list[int] = []
    all_telemetry_complete = True
    for side in ("forehand", "backhand"):
        side_rows = [
            row for row in serve_rows if row.get("command_side") == side
        ]
        joined = []
        for row in side_rows:
            shot_id = int(row.get("shot_id") or 0)
            physical = physical_by_id.get(shot_id)
            if physical is None:
                missing_shot_ids.append(shot_id)
                all_telemetry_complete = False
                continue
            all_telemetry_complete &= bool(physical.get("telemetry_complete", False))
            joined.append(physical)
        total = len(side_rows)
        contact_count = sum(bool(row.get("contact_pass", False)) for row in joined)
        landing_count = sum(bool(row.get("landing_pass", False)) for row in joined)
        contact_rate = contact_count / total if total else 0.0
        landing_rate = landing_count / total if total else 0.0
        passed = bool(
            total >= int(min_samples_per_side)
            and (
                exact_samples_per_side is None
                or total == int(exact_samples_per_side)
            )
            and len(joined) == total
            and contact_count >= int(min_contacts_per_side)
            and landing_count >= int(min_landings_per_side)
            and contact_rate >= float(min_contact_rate)
            and landing_rate >= float(min_landing_rate)
        )
        per_side[side] = {
            "shots": total,
            "shot_ids": [int(row.get("shot_id") or 0) for row in side_rows],
            "contacts": contact_count,
            "legal_landings": landing_count,
            "contact_rate": contact_rate,
            "landing_rate": landing_rate,
            "pass": passed,
        }

    for row in serve_rows:
        if row.get("command_side") not in ("forehand", "backhand"):
            unassigned_shot_ids.append(int(row.get("shot_id") or 0))
    expected_ids = serve_ids
    all_shots_joined = bool(
        expected_ids
        and all(shot_id > 0 and shot_id in physical_by_id for shot_id in expected_ids)
        and not missing_shot_ids
        and not duplicate_serve_ids
        and not duplicate_physical_ids
        and not extra_physical_ids
        and set(physical_by_id) == set(expected_ids)
    )
    measured = bool(
        physical_report.get("physical_contact_measured", False)
        and physical_report.get("landing_measured", False)
        and all_shots_joined
        and all_telemetry_complete
    )
    global_contacts = sum(item["contacts"] for item in per_side.values())
    global_landings = sum(item["legal_landings"] for item in per_side.values())
    global_counts_pass = bool(
        global_contacts >= int(min_global_contacts)
        and global_landings >= int(min_global_landings)
    )
    passed = bool(
        measured
        and not unassigned_shot_ids
        and global_counts_pass
        and all(item["pass"] for item in per_side.values())
    )
    return {
        "physical_contact_measured": measured,
        "landing_measured": measured,
        "all_shots_joined": all_shots_joined,
        "all_sides_assigned": not unassigned_shot_ids,
        "missing_shot_ids": sorted(set(missing_shot_ids)),
        "extra_physical_shot_ids": extra_physical_ids,
        "duplicate_serve_shot_ids": duplicate_serve_ids,
        "duplicate_physical_shot_ids": duplicate_physical_ids,
        "unassigned_shot_ids": unassigned_shot_ids,
        "minimum_shots_per_side": int(min_samples_per_side),
        "exact_shots_per_side": (
            None if exact_samples_per_side is None
            else int(exact_samples_per_side)
        ),
        "minimum_contacts_per_side": int(min_contacts_per_side),
        "minimum_landings_per_side": int(min_landings_per_side),
        "minimum_contact_rate_per_side": float(min_contact_rate),
        "minimum_landing_rate_per_side": float(min_landing_rate),
        "global_contacts": global_contacts,
        "global_legal_landings": global_landings,
        "minimum_global_contacts": int(min_global_contacts),
        "minimum_global_landings": int(min_global_landings),
        "global_counts_pass": global_counts_pass,
        "by_side": per_side,
        "pass": passed,
    }


def physical_report_complete(
    report: Mapping[str, Any], expected_shot_ids: Iterable[int]
) -> bool:
    """Check that every expected shot reached a complete, loss-detectable window."""
    expected = [int(shot_id) for shot_id in expected_shot_ids]
    rows = list(report.get("rows", []))
    row_ids = [int(row.get("shot_id") or 0) for row in rows]
    return bool(
        expected
        and len(set(expected)) == len(expected)
        and row_ids == expected
        and report.get("physical_contact_measured", False)
        and report.get("landing_measured", False)
        and all(bool(row.get("telemetry_complete", False)) for row in rows)
    )


def serve_to_dict(spec: ServeSpec) -> dict[str, Any]:
    return asdict(spec)
