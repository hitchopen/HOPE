#!/usr/bin/env python3
"""Build the deterministic Schema31 Gate3 fixture from exact v5 bank rows.

The v5 artifact is a discrete complete-tuple training bank.  A continuous
Planner solution is therefore not allowed to stand in for a bank row.  This
tool selects an immutable 26-flight L0 ledger, validates that every selected
row is reachable through the bank's actual L0 sampling route, and integrates
the physical launch backwards through one table bounce.  The C++ Planner then
uses the CSV to compare its independent physical prediction before emitting
the exact bank tuple.

This fixture is simulator-only.  It does not promote the v5 candidate bank to
trajectory-certified or deployment-qualified status.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path

import numpy as np


BANK_SHA256 = "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e"
RECEIPT_SHA256 = "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493"
BANK_CONTRACT = "fixed_home_support_training_candidate_bank_v5"
FIXTURE_CONTRACT = "schema31_gate3_v5_l0_fixture_v1"
RECEIPT_CONTRACT = "gate3_schema31_v5_l0_fixture_receipt_v1"

HOME_X_M = -0.50
HOME_Y_M = -0.7625
POLICY_Z_OFFSET_M = 0.760
LAUNCH_X_M = 2.40
TABLE_WIDTH_M = 1.525
BALL_RADIUS_M = 0.020
BOUNCE_CENTER_Z_M = BALL_RADIUS_M + 1.0e-4
DRAG_K = 0.1220
RESTITUTION_H = 0.64
RESTITUTION_V = 0.9215
GRAVITY = np.asarray([0.0, 0.0, -9.81], dtype=np.float64)
INTEGRATION_DT_S = 1.0e-4

# Three L0 questions per side.  Each tuple is unique under the formal joint
# matcher and is present in sampling_pool_rows_by_reach_level[..., 0, :].
QUESTION_ROWS = {
    "FH0": 13511,
    "FH1": 26081,
    "FH2": 25891,
    "BH0": 43874,
    "BH1": 45754,
    "BH2": 30910,
}

# Preserve the established six clean + twenty rapid side-transition cadence,
# while making every question an actual v5 L0 row.
QUESTION_SEQUENCE = (
    "FH0", "BH0", "FH1", "BH1", "FH2", "BH2",
    "FH0", "FH1", "BH0", "BH1", "FH2", "BH2",
    "FH0", "BH0", "FH1", "BH1", "FH2", "BH2",
    "FH0", "BH0", "FH1", "BH1", "FH2", "BH2",
    "FH0", "BH0",
)

CSV_FIELDS = (
    "fixture_contract", "bank_sha256", "receipt_sha256", "flight_id",
    "bank_row_id", "clip", "reach_level", "swing_foot_sign",
    "contact_offset_x", "contact_offset_y", "contact_actor_z",
    "target_vx", "target_vy", "target_vz",
    "target_nx", "target_ny", "target_nz",
    "incoming_vx", "incoming_vy", "incoming_vz", "tts_s",
    "intended_land_x", "intended_land_y",
    "outgoing_vx", "outgoing_vy", "outgoing_vz",
    "launch_x", "launch_y", "launch_z_table",
    "launch_vx", "launch_vy", "launch_vz",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def _acceleration(velocity: np.ndarray) -> np.ndarray:
    return -DRAG_K * np.linalg.norm(velocity) * velocity + GRAVITY


def _rk4(state: np.ndarray, dt_s: float) -> np.ndarray:
    def derivative(value: np.ndarray) -> np.ndarray:
        return np.concatenate((value[3:], _acceleration(value[3:])))

    k1 = derivative(state)
    k2 = derivative(state + 0.5 * dt_s * k1)
    k3 = derivative(state + 0.5 * dt_s * k2)
    k4 = derivative(state + dt_s * k3)
    return state + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def _inverse_launch(endpoint: np.ndarray) -> tuple[np.ndarray, float]:
    """Integrate an incoming endpoint backwards through exactly one bounce."""

    state = endpoint.astype(np.float64, copy=True)
    elapsed_s = 0.0
    bounce_count = 0
    dt_s = -INTEGRATION_DT_S
    while elapsed_s < 3.0:
        previous = state.copy()
        previous_elapsed_s = elapsed_s
        state = _rk4(state, dt_s)
        elapsed_s += -dt_s

        if (
            bounce_count == 0
            and previous[2] > BOUNCE_CENTER_Z_M
            and state[2] <= BOUNCE_CENTER_Z_M
        ):
            fraction = (previous[2] - BOUNCE_CENTER_Z_M) / (
                previous[2] - state[2]
            )
            at_bounce = previous + fraction * (state - previous)
            at_bounce[2] = BOUNCE_CENTER_Z_M
            # Undo v_xy+ = e_h*v_xy- and v_z+ = -e_v*v_z-.
            at_bounce[3] /= RESTITUTION_H
            at_bounce[4] /= RESTITUTION_H
            at_bounce[5] = -at_bounce[5] / RESTITUTION_V
            remaining_s = (-dt_s) * (1.0 - fraction)
            state = _rk4(at_bounce, -remaining_s)
            bounce_count = 1

        if previous[0] < LAUNCH_X_M <= state[0]:
            fraction = (LAUNCH_X_M - previous[0]) / (state[0] - previous[0])
            launch = previous + fraction * (state - previous)
            launch[0] = LAUNCH_X_M
            full_flight_s = previous_elapsed_s + (-dt_s) * fraction
            break
    else:
        raise ValueError("inverse flight did not reach the fixed launch plane")

    if bounce_count != 1:
        raise ValueError(f"inverse flight crossed {bounce_count} table bounces, expected 1")
    if not np.all(np.isfinite(launch)) or launch[3] >= 0.0:
        raise ValueError("inverse launch state is non-finite or not incoming")
    if not (-TABLE_WIDTH_M <= launch[1] <= 0.0) or launch[2] <= BOUNCE_CENTER_Z_M:
        raise ValueError("inverse launch lies outside the physical table/air volume")
    return launch, full_flight_s


def _forward_endpoint(launch: np.ndarray) -> tuple[np.ndarray, int]:
    """Close the inverse calculation with the simulator's one-bounce map."""

    state = launch.astype(np.float64, copy=True)
    bounces = 0
    for _ in range(int(3.0 / INTEGRATION_DT_S)):
        previous = state.copy()
        state = _rk4(state, INTEGRATION_DT_S)
        if (
            state[2] <= BOUNCE_CENTER_Z_M
            and previous[2] > BOUNCE_CENTER_Z_M
            and state[5] < 0.0
        ):
            fraction = (previous[2] - BOUNCE_CENTER_Z_M) / (
                previous[2] - state[2]
            )
            at_bounce = previous + fraction * (state - previous)
            at_bounce[2] = BOUNCE_CENTER_Z_M
            at_bounce[3] *= RESTITUTION_H
            at_bounce[4] *= RESTITUTION_H
            at_bounce[5] = -RESTITUTION_V * at_bounce[5]
            remaining_s = INTEGRATION_DT_S * (1.0 - fraction)
            state = _rk4(at_bounce, remaining_s)
            bounces += 1
        if previous[0] > HOME_X_M + 0.58 >= state[0]:
            fraction = (previous[0] - (HOME_X_M + 0.58)) / (
                previous[0] - state[0]
            )
            endpoint = previous + fraction * (state - previous)
            endpoint[0] = HOME_X_M + 0.58
            return endpoint, bounces
    raise ValueError("forward closure did not reach the fixed strike plane")


def _routed_l0_mask(bank: np.lib.npyio.NpzFile) -> np.ndarray:
    rows = bank["sampling_pool_rows_by_reach_level"].astype(np.int64)
    sizes = bank["sampling_pool_size_by_reach_level"].astype(np.int64)
    row_ids = bank["row_id"]
    if (
        rows.ndim != 5
        or rows.shape[-2] != 3
        or sizes.shape != rows.shape[:-1]
        or np.any(sizes < 0)
        or np.any(sizes > rows.shape[-1])
    ):
        raise ValueError("v5 routed sampling-pool shape/size contract is invalid")
    mask = np.zeros(len(row_ids), dtype=bool)
    for cell in np.ndindex(rows.shape[:-2]):
        size = int(sizes[cell + (0,)])
        if size <= 0:
            raise ValueError(f"v5 cell {cell} has no routed L0 fallback")
        indices = rows[cell + (0,)][:size]
        if np.any(indices < 0) or np.any(indices >= len(row_ids)):
            raise ValueError(f"v5 cell {cell} routes an out-of-range L0 row")
        mask[np.unique(indices)] = True
    return mask


def build_fixture(bank_path: Path, receipt_path: Path) -> tuple[list[dict], dict]:
    bank_sha = _sha256(bank_path)
    receipt_sha = _sha256(receipt_path)
    if bank_sha != BANK_SHA256 or receipt_sha != RECEIPT_SHA256:
        raise ValueError(
            "v5 bank/receipt digest mismatch: "
            f"bank={bank_sha} receipt={receipt_sha}"
        )
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    semantics = receipt.get("optional_reach_semantics") or {}
    if not (
        receipt.get("bank_contract") == BANK_CONTRACT
        and receipt.get("qualification_scope") == "SIM_TRAINING_CANDIDATE_ONLY"
        and receipt.get("optional_reach_training_enabled") is True
        and receipt.get("optional_reach_trajectory_qualification")
        == "NOT_TRAJECTORY_CERTIFIED"
        and receipt.get("deployment_qualification") == "NOT_DEPLOYMENT_QUALIFIED"
        and receipt.get("optional_reach_deployment_qualification")
        == "NOT_DEPLOYMENT_QUALIFIED"
        and semantics.get("candidate_only") is True
        and semantics.get("deployment_use_forbidden") is True
        and semantics.get(
            "training_admission_is_not_trajectory_or_deployment_qualification"
        )
        is True
    ):
        raise ValueError("v5 receipt lost its simulator-candidate safety boundary")

    output_rows: list[dict] = []
    closure_rows: list[dict] = []
    with np.load(bank_path, allow_pickle=False) as bank:
        required = {
            "row_id", "clip", "contact_position_offset", "target_velocity",
            "target_normal", "incoming_velocity", "tts_center_s", "tts_min_s",
            "tts_max_s", "reach_level", "swing_foot_sign",
            "training_admissible", "optional_reach_training_admissible",
            "sampling_pool_rows_by_reach_level",
            "sampling_pool_size_by_reach_level", "intended_landing_xy",
            "predicted_outgoing_velocity",
        }
        missing = sorted(required.difference(bank.files))
        if missing:
            raise ValueError(f"v5 bank is missing fixture fields: {missing}")
        row_ids = bank["row_id"].astype(np.int64)
        if len(np.unique(row_ids)) != len(row_ids):
            raise ValueError("v5 row_id is not unique")
        index_by_id = {int(row_id): index for index, row_id in enumerate(row_ids)}
        routed_l0 = _routed_l0_mask(bank)
        positions = bank["contact_position_offset"].astype(np.float64)
        velocities = bank["target_velocity"].astype(np.float64)
        normals = bank["target_normal"].astype(np.float64)
        incoming = bank["incoming_velocity"].astype(np.float64)
        tts = bank["tts_center_s"].astype(np.float64)

        for flight_id, label in enumerate(QUESTION_SEQUENCE, start=1):
            row_id = QUESTION_ROWS[label]
            if row_id not in index_by_id:
                raise ValueError(f"selected v5 row_id={row_id} is absent")
            index = index_by_id[row_id]
            clip = int(bank["clip"][index])
            level = int(bank["reach_level"][index])
            foot_sign = int(bank["swing_foot_sign"][index])
            if (label.startswith("FH") and clip != 0) or (
                label.startswith("BH") and clip != 1
            ):
                raise ValueError(f"selected v5 row_id={row_id} has wrong side")
            if not (
                level == 0
                and foot_sign == 0
                and bool(bank["training_admissible"][index])
                and bool(bank["optional_reach_training_admissible"][index])
                and routed_l0[index]
            ):
                raise ValueError(f"selected v5 row_id={row_id} is outside routed L0")
            if not (
                float(bank["tts_min_s"][index]) == tts[index]
                == float(bank["tts_max_s"][index])
            ):
                raise ValueError(f"selected v5 row_id={row_id} has non-exact TTS")
            if not np.allclose(
                normals[index], velocities[index] / np.linalg.norm(velocities[index]),
                rtol=0.0, atol=2.0e-7,
            ):
                raise ValueError(f"selected v5 row_id={row_id} normal is incoherent")

            # The formal matcher must resolve the whole correlated tuple to one
            # and only one row.  Do not relax its 1e-4 / 1e-3 tolerances here.
            mask = (
                routed_l0
                & (bank["clip"].astype(np.int64) == clip)
                & (bank["reach_level"].astype(np.int64) == 0)
                & (bank["swing_foot_sign"].astype(np.int64) == 0)
                & np.all(np.abs(positions - positions[index]) <= 1.0e-4, axis=1)
                & np.all(np.abs(velocities - velocities[index]) <= 1.0e-4, axis=1)
                & np.all(np.abs(normals - normals[index]) <= 1.0e-4, axis=1)
                & np.all(np.abs(incoming - incoming[index]) <= 1.0e-4, axis=1)
                & (tts >= float(bank["tts_min_s"][index]) - 1.0e-3)
                & (tts <= float(bank["tts_max_s"][index]) + 1.0e-3)
            )
            if int(np.count_nonzero(mask)) != 1:
                raise ValueError(
                    f"selected v5 row_id={row_id} is not a unique formal joint tuple"
                )

            endpoint = np.concatenate((
                np.asarray([
                    HOME_X_M + positions[index, 0],
                    HOME_Y_M + positions[index, 1],
                    positions[index, 2] - POLICY_Z_OFFSET_M,
                ]),
                incoming[index],
            ))
            launch, flight_s = _inverse_launch(endpoint)
            closure, bounce_count = _forward_endpoint(launch)
            closure_error = np.abs(closure - endpoint)
            if bounce_count != 1 or np.max(closure_error) > 2.0e-3:
                raise ValueError(
                    f"selected v5 row_id={row_id} inverse closure failed: "
                    f"bounces={bounce_count} max_error={np.max(closure_error):.6g}"
                )

            land = bank["intended_landing_xy"][index].astype(np.float64)
            outgoing = bank["predicted_outgoing_velocity"][index].astype(np.float64)
            values = (
                FIXTURE_CONTRACT, BANK_SHA256, RECEIPT_SHA256, flight_id,
                row_id, clip, level, foot_sign,
                *positions[index], *velocities[index], *normals[index],
                *incoming[index], tts[index], *land, *outgoing, *launch,
            )
            output_rows.append(dict(zip(CSV_FIELDS, values, strict=True)))
            closure_rows.append({
                "flight_id": flight_id,
                "bank_row_id": row_id,
                "label": label,
                "inverse_flight_s": flight_s,
                "forward_bounces": bounce_count,
                "maximum_forward_closure_error": float(np.max(closure_error)),
                "launch_state_table_frame": launch.tolist(),
            })

    receipt_output = {
        "schema_version": 1,
        "receipt_contract": RECEIPT_CONTRACT,
        "fixture_contract": FIXTURE_CONTRACT,
        "simulator_only": True,
        "qualification_scope": "SIM_TRAINING_CANDIDATE_ONLY",
        "trajectory_qualification": "NOT_TRAJECTORY_CERTIFIED",
        "deployment_qualification": "NOT_DEPLOYMENT_QUALIFIED",
        "bank_sha256": BANK_SHA256,
        "bank_receipt_sha256": RECEIPT_SHA256,
        "expected_flights": len(output_rows),
        "flight_ids": list(range(1, len(output_rows) + 1)),
        "bank_row_ids": [int(row["bank_row_id"]) for row in output_rows],
        "serves_flat": [
            float(row[field])
            for row in output_rows
            for field in (
                "launch_x", "launch_y", "launch_z_table",
                "launch_vx", "launch_vy", "launch_vz",
            )
        ],
        "physics": {
            "drag_k": DRAG_K,
            "restitution_h": RESTITUTION_H,
            "restitution_v": RESTITUTION_V,
            "bounce_center_z_table_m": BOUNCE_CENTER_Z_M,
            "integration_dt_s": INTEGRATION_DT_S,
        },
        "rows": closure_rows,
    }
    return output_rows, receipt_output


def render_csv(rows: list[dict]) -> str:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({
            key: format(value, ".17g") if isinstance(value, (float, np.floating)) else value
            for key, value in row.items()
        })
    return stream.getvalue()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bank", required=True, type=Path)
    parser.add_argument("--receipt", required=True, type=Path)
    parser.add_argument("--planner-csv", required=True, type=Path)
    parser.add_argument("--json", required=True, type=Path)
    args = parser.parse_args()

    rows, receipt = build_fixture(args.bank, args.receipt)
    csv_text = render_csv(rows)
    _atomic_write(args.planner_csv, csv_text)
    receipt["planner_csv_sha256"] = hashlib.sha256(csv_text.encode()).hexdigest()
    _atomic_write(args.json, json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print(
        "[v5-fixture] generated "
        f"{len(rows)} simulator-only L0 flights; csv={args.planner_csv} "
        f"receipt={args.json}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
