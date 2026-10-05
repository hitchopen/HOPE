#!/usr/bin/env python3
"""Fail-closed Schema28/29/31 Gate3 question-to-bank joint-row provenance report.

The Planner's axis envelope is only a coarse OOD prefilter.  Formal Gate3 additionally requires
every emitted flight tuple to match one immutable row of the checkpoint-bound v3 bank.  Fields
are compared jointly; independent per-axis membership can never qualify a flight.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


BANK_SHA256 = "f74db60887fe0f701dee9157c8c7ea87038be78e805f615515f9b0f5d62c84ed"
RECEIPT_SHA256 = "3008fbb64c439db29e94a599f198cebb803c284ca8a52f1790e23a87896432fe"
V4_BANK_SHA256 = "894bea744fc5b119312b619c0368afe9ae048d1b076f67e80f25f7c1111907ee"
V4_RECEIPT_SHA256 = "809a39b7105229ca89d627fdf508a4d4763a709824e0d69b766bb96d5ec84eec"
V5_BANK_SHA256 = "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e"
V5_RECEIPT_SHA256 = "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493"
CONTRACT = "schema3_gate3_exact_joint_bank_row_provenance_v3"
V5_FIXTURE_CONTRACT = "schema31_gate3_v5_l0_fixture_v1"
V5_MAX_PHYSICAL_POSITION_ERROR_M = 0.03
V5_MAX_PHYSICAL_VELOCITY_ERROR_MPS = 0.10
V5_MAX_PHYSICAL_TTS_ERROR_S = 0.03
# Planner publishes the question tuple through float32 ROS fields while its
# audit CSV retains the fixture's double-precision parse.  This tolerance is
# only for comparing those two representations of the same fixture TTS; the
# bank-row and physical-residual gates below keep their existing tolerances.
V5_FIXTURE_WIRE_FLOAT32_TTS_TOLERANCE_S = 1.0e-6


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def _close_rows(values: np.ndarray, target: np.ndarray, tolerance: float) -> np.ndarray:
    return np.all(np.abs(values.astype(np.float64) - target) <= tolerance, axis=1)


def _coverage_summary(
    matches: list[dict], expected_flights: int, *, candidate_only: bool = False
) -> dict:
    """Return fail-closed Schema28 fixture coverage over distinct served flights."""

    level_counts = {
        str(level): sum(row["reach_level"] == level for row in matches)
        for level in (0, 1, 2)
    }
    side_counts = {
        "FH": sum(row["swing_sign"] == 1 for row in matches),
        "BH": sum(row["swing_sign"] == -1 for row in matches),
    }
    side_level_counts = {
        side: {
            str(level): sum(
                row["swing_sign"] == swing and row["reach_level"] == level
                for row in matches
            )
            for level in (0, 1, 2)
        }
        for side, swing in (("FH", 1), ("BH", -1))
    }
    failures: list[str] = []
    if expected_flights > 0 and len(matches) != expected_flights:
        failures.append(
            f"matched flight ledger has {len(matches)} distinct positive flight ids; "
            f"required exactly {expected_flights}"
        )
    if candidate_only:
        if level_counts != {"0": len(matches), "1": 0, "2": 0}:
            failures.append("candidate-only ledger emitted a nonzero reach level")
    else:
        for level in (0, 1, 2):
            if level_counts[str(level)] == 0:
                failures.append(f"formal ledger contains no level-{level} flight")
    for side in ("FH", "BH"):
        if side_counts[side] == 0:
            failures.append(f"formal ledger contains no {side} flight")
    if expected_flights >= 26 and not candidate_only:
        required_side_level = {"0": 5, "1": 4, "2": 4}
        for side in ("FH", "BH"):
            for level, required_count in required_side_level.items():
                if side_level_counts[side][level] < required_count:
                    failures.append(
                        f"formal ledger {side} level-{level} count "
                        f"{side_level_counts[side][level]} < {required_count}"
                    )
    return {
        "level_counts": level_counts,
        "side_counts": side_counts,
        "side_level_counts": side_level_counts,
        "failures": failures,
    }


def build_report(
    planner_csv: Path,
    bank_path: Path,
    receipt_path: Path,
    *,
    minimum_flights: int = 26,
    tuple_tolerance: float = 1.0e-4,
    tts_tolerance_s: float = 1.0e-3,
) -> dict:
    failures: list[str] = []
    bank_sha = _sha256(bank_path)
    receipt_sha = _sha256(receipt_path)
    known_pair = (bank_sha, receipt_sha) in {
        (BANK_SHA256, RECEIPT_SHA256),
        (V4_BANK_SHA256, V4_RECEIPT_SHA256),
        (V5_BANK_SHA256, V5_RECEIPT_SHA256),
    }
    if not known_pair:
        failures.append("bank/receipt digest pair is not an approved Gate3 artifact")
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    semantics = receipt.get("optional_reach_semantics") or {}
    candidate_only_v4 = bool(
        bank_sha == V4_BANK_SHA256
        and receipt_sha == V4_RECEIPT_SHA256
        and receipt.get("bank_contract") == "fixed_home_support_candidate_bank_v4"
        and receipt.get("optional_reach_training_enabled") is False
        and receipt.get("optional_reach_trajectory_qualification") == "NOT_PROVEN"
        and semantics.get("candidate_only") is True
    )
    candidate_only_v5 = bool(
        bank_sha == V5_BANK_SHA256
        and receipt_sha == V5_RECEIPT_SHA256
        and receipt.get("bank_contract")
        == "fixed_home_support_training_candidate_bank_v5"
        and receipt.get("optional_reach_training_enabled") is True
        and receipt.get("optional_reach_trajectory_qualification")
        == "NOT_TRAJECTORY_CERTIFIED"
        and receipt.get("qualification_scope") == "SIM_TRAINING_CANDIDATE_ONLY"
        and receipt.get("deployment_qualification") == "NOT_DEPLOYMENT_QUALIFIED"
        and receipt.get("optional_reach_deployment_qualification")
        == "NOT_DEPLOYMENT_QUALIFIED"
        and semantics.get("candidate_only") is True
        and semantics.get("deployment_use_forbidden") is True
        and semantics.get(
            "training_admission_is_not_trajectory_or_deployment_qualification"
        )
        is True
    )
    candidate_only = candidate_only_v4 or candidate_only_v5
    if bank_sha == V4_BANK_SHA256 and not candidate_only_v4:
        failures.append("v4 receipt does not preserve the candidate-only NOT_PROVEN boundary")
    if bank_sha == V5_BANK_SHA256 and not candidate_only_v5:
        failures.append(
            "v5 receipt does not preserve the simulator-training-candidate, "
            "non-trajectory/non-deployment-qualified boundary"
        )

    required = {
        "wire_valid", "flight_id", "revision_id", "target_actor_x",
        "target_actor_y", "target_actor_z", "racket_vx", "racket_vy", "racket_vz",
        "target_nx", "target_ny", "target_nz", "strike_vx", "strike_vy", "strike_vz",
        "session_home_x", "session_home_y", "swing_sign", "tts_s", "reach_level",
        "swing_foot_sign",
    }
    v5_fixture_required = {
        "question_fixture_active", "question_fixture_contract",
        "question_fixture_bank_sha256", "question_fixture_receipt_sha256",
        "question_fixture_bank_row_id", "question_fixture_physical_strike_x",
        "question_fixture_physical_strike_y", "question_fixture_physical_strike_z",
        "question_fixture_physical_strike_vx", "question_fixture_physical_strike_vy",
        "question_fixture_physical_strike_vz", "question_fixture_physical_tts_s",
        "question_fixture_position_error_max_m",
        "question_fixture_velocity_error_max_mps", "question_fixture_tts_error_s",
        "question_fixture_tts_s", "question_fixture_physical_match",
    }
    all_required = required | (v5_fixture_required if candidate_only_v5 else set())
    with planner_csv.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(all_required - set(reader.fieldnames or ()))
        if missing:
            return {
                "pass": False,
                "evidence_complete": False,
                "formal_gate_verdict": "FAIL",
                "qualification_status": "NOT_PROVEN",
                "contract": CONTRACT,
                "failures": [f"Planner CSV missing provenance columns: {missing}"],
            }
        valid_rows = [row for row in reader if int(round(_f(row, "wire_valid"))) == 1]

    revision_rows: dict[tuple[int, int], dict[str, str]] = {}
    tuple_fields = tuple(sorted(required - {"wire_valid", "flight_id", "revision_id"}))
    for row in valid_rows:
        flight = int(round(_f(row, "flight_id")))
        revision = int(round(_f(row, "revision_id")))
        if flight <= 0:
            continue
        key = (flight, revision)
        prior = revision_rows.get(key)
        if prior is None:
            revision_rows[key] = row
            continue
        # One wire revision is one atomic Planner tuple. Reusing the identity
        # while mutating any field would otherwise let the first row hide an
        # unqualified tuple later in the same revision.
        if any(_f(prior, field) != _f(row, field) for field in tuple_fields):
            failures.append(
                f"flight {flight} revision {revision} mutates its complete target tuple"
            )

    with np.load(bank_path, allow_pickle=False) as bank:
        row_ids = bank["row_id"].astype(np.int64)
        clip = bank["clip"].astype(np.int64)
        position = bank["contact_position_offset"].astype(np.float64)
        velocity = bank["target_velocity"].astype(np.float64)
        normal = bank["target_normal"].astype(np.float64)
        incoming_velocity = bank["incoming_velocity"].astype(np.float64)
        tts_min = bank["tts_min_s"].astype(np.float64)
        tts_max = bank["tts_max_s"].astype(np.float64)
        reach_level = bank["reach_level"].astype(np.int64)
        foot_sign = bank["swing_foot_sign"].astype(np.int64)
        training_admissible = bank["training_admissible"].astype(bool)
        source_row = bank["source_row"].astype(np.int64)
        source_flight = bank["source_flight_id"].astype(np.int64)
        sampled_for_training = np.zeros(len(row_ids), dtype=bool)
        optional_training_admissible = np.ones(len(row_ids), dtype=bool)
        if candidate_only:
            required_candidate_fields = {
                "sampling_pool_rows_by_reach_level",
                "sampling_pool_size_by_reach_level",
                "optional_reach_training_admissible",
            }
            missing_candidate_fields = required_candidate_fields.difference(bank.files)
            if missing_candidate_fields:
                raise ValueError(
                    "candidate bank lacks L0 training-universe fields: "
                    f"{sorted(missing_candidate_fields)}"
                )
            routed = bank["sampling_pool_rows_by_reach_level"].astype(np.int64)
            sizes = bank["sampling_pool_size_by_reach_level"].astype(np.int64)
            optional_training_admissible = bank[
                "optional_reach_training_admissible"
            ].astype(bool)
            if (
                routed.ndim != 5
                or routed.shape[-2] != 3
                or sizes.shape != routed.shape[:-1]
                or optional_training_admissible.shape != row_ids.shape
                or np.any(sizes[..., 0] <= 0)
                or np.any(sizes < 0)
                or np.any(sizes > routed.shape[-1])
            ):
                raise ValueError("candidate L0 routed-pool shapes/sizes are invalid")
            for cell in np.ndindex(routed.shape[:-2]):
                size = int(sizes[cell + (0,)])
                l0_rows = routed[cell + (0,)][:size]
                if (
                    np.any(l0_rows < 0)
                    or np.any(l0_rows >= len(row_ids))
                    or not np.all(reach_level[l0_rows] == 0)
                    or not np.all(foot_sign[l0_rows] == 0)
                    or not np.all(optional_training_admissible[l0_rows])
                ):
                    raise ValueError(
                        "candidate formal universe references a non-L0, "
                        "non-admissible, or invalid row"
                    )
                sampled_for_training[np.unique(l0_rows)] = True
        else:
            sampling_pool_rows = bank["sampling_pool_rows"].astype(np.int64).reshape(-1)
            if (
                np.any(sampling_pool_rows < 0)
                or np.any(sampling_pool_rows >= len(row_ids))
            ):
                raise ValueError("sampling_pool_rows contains an out-of-range bank index")
            sampled_for_training[np.unique(sampling_pool_rows)] = True

        matches = []
        for (flight, revision), row in sorted(revision_rows.items()):
            swing = int(round(_f(row, "swing_sign")))
            level = int(round(_f(row, "reach_level")))
            moving_foot = int(round(_f(row, "swing_foot_sign")))
            expected_clip = 0 if swing == 1 else 1 if swing == -1 else -1
            target_position = np.asarray(
                [
                    _f(row, "target_actor_x") - _f(row, "session_home_x"),
                    _f(row, "target_actor_y") - _f(row, "session_home_y"),
                    _f(row, "target_actor_z"),
                ],
                dtype=np.float64,
            )
            target_velocity = np.asarray(
                [_f(row, "racket_vx"), _f(row, "racket_vy"), _f(row, "racket_vz")],
                dtype=np.float64,
            )
            target_normal = np.asarray(
                [_f(row, "target_nx"), _f(row, "target_ny"), _f(row, "target_nz")],
                dtype=np.float64,
            )
            incoming = np.asarray(
                [_f(row, "strike_vx"), _f(row, "strike_vy"), _f(row, "strike_vz")],
                dtype=np.float64,
            )
            tts = _f(row, "tts_s")
            fixture_physical_evidence = None
            if candidate_only_v5:
                physical_position = np.asarray(
                    [
                        _f(row, "question_fixture_physical_strike_x"),
                        _f(row, "question_fixture_physical_strike_y"),
                        _f(row, "question_fixture_physical_strike_z"),
                    ],
                    dtype=np.float64,
                )
                physical_velocity = np.asarray(
                    [
                        _f(row, "question_fixture_physical_strike_vx"),
                        _f(row, "question_fixture_physical_strike_vy"),
                        _f(row, "question_fixture_physical_strike_vz"),
                    ],
                    dtype=np.float64,
                )
                physical_tts = _f(row, "question_fixture_physical_tts_s")
                fixture_tts = _f(row, "question_fixture_tts_s")
                expected_physical_position = np.asarray(
                    [
                        _f(row, "target_actor_x"),
                        _f(row, "target_actor_y"),
                        _f(row, "target_actor_z") - 0.760,
                    ],
                    dtype=np.float64,
                )
                recomputed_position_error = float(
                    np.max(np.abs(physical_position - expected_physical_position))
                )
                recomputed_velocity_error = float(
                    np.max(np.abs(physical_velocity - incoming))
                )
                recomputed_tts_error = abs(physical_tts - fixture_tts)
                reported_position_error = _f(
                    row, "question_fixture_position_error_max_m"
                )
                reported_velocity_error = _f(
                    row, "question_fixture_velocity_error_max_mps"
                )
                reported_tts_error = _f(row, "question_fixture_tts_error_s")
                fixture_physical_evidence = {
                    "physical_position": physical_position.tolist(),
                    "physical_velocity": physical_velocity.tolist(),
                    "physical_tts_s": physical_tts,
                    "position_error_max_m": recomputed_position_error,
                    "velocity_error_max_mps": recomputed_velocity_error,
                    "tts_error_s": recomputed_tts_error,
                }
                if int(round(_f(row, "question_fixture_active"))) != 1:
                    failures.append(f"flight {flight} v5 question fixture is not active")
                if row["question_fixture_contract"] != V5_FIXTURE_CONTRACT:
                    failures.append(f"flight {flight} v5 fixture contract mismatch")
                if row["question_fixture_bank_sha256"] != V5_BANK_SHA256:
                    failures.append(f"flight {flight} v5 fixture bank digest mismatch")
                if row["question_fixture_receipt_sha256"] != V5_RECEIPT_SHA256:
                    failures.append(f"flight {flight} v5 fixture receipt digest mismatch")
                if int(round(_f(row, "question_fixture_physical_match"))) != 1:
                    failures.append(f"flight {flight} v5 physical residual did not pass")
                if (
                    abs(fixture_tts - tts)
                    > V5_FIXTURE_WIRE_FLOAT32_TTS_TOLERANCE_S
                ):
                    failures.append(f"flight {flight} v5 fixture TTS differs from wire TTS")
                reported = (
                    reported_position_error,
                    reported_velocity_error,
                    reported_tts_error,
                )
                recomputed = (
                    recomputed_position_error,
                    recomputed_velocity_error,
                    recomputed_tts_error,
                )
                if not all(math.isfinite(value) for value in (*physical_position, *physical_velocity, physical_tts, *reported)):
                    failures.append(f"flight {flight} v5 physical residual is non-finite")
                if any(abs(a - b) > 1.0e-6 for a, b in zip(reported, recomputed)):
                    failures.append(f"flight {flight} v5 physical residual ledger is inconsistent")
                if recomputed_position_error > V5_MAX_PHYSICAL_POSITION_ERROR_M:
                    failures.append(
                        f"flight {flight} v5 physical position error "
                        f"{recomputed_position_error:.6f} > {V5_MAX_PHYSICAL_POSITION_ERROR_M:.6f} m"
                    )
                if recomputed_velocity_error > V5_MAX_PHYSICAL_VELOCITY_ERROR_MPS:
                    failures.append(
                        f"flight {flight} v5 physical velocity error "
                        f"{recomputed_velocity_error:.6f} > {V5_MAX_PHYSICAL_VELOCITY_ERROR_MPS:.6f} m/s"
                    )
                if recomputed_tts_error > V5_MAX_PHYSICAL_TTS_ERROR_S:
                    failures.append(
                        f"flight {flight} v5 physical TTS error "
                        f"{recomputed_tts_error:.6f} > {V5_MAX_PHYSICAL_TTS_ERROR_S:.6f} s"
                    )
            finite = all(
                math.isfinite(value)
                for value in (*target_position, *target_velocity, *target_normal, *incoming, tts)
            )
            mask = training_admissible & sampled_for_training & (clip == expected_clip)
            if candidate_only:
                if level != 0 or moving_foot != 0:
                    failures.append(
                        f"flight {flight} violates candidate-only runtime permission ({level},{moving_foot})"
                    )
                    mask[:] = False
                mask &= reach_level == 0
                mask &= foot_sign == 0
                mask &= optional_training_admissible
            else:
                mask &= reach_level == level
                mask &= foot_sign == moving_foot
            if finite:
                mask &= _close_rows(position, target_position, tuple_tolerance)
                mask &= _close_rows(velocity, target_velocity, tuple_tolerance)
                mask &= _close_rows(normal, target_normal, tuple_tolerance)
                mask &= _close_rows(incoming_velocity, incoming, tuple_tolerance)
                mask &= tts >= tts_min - tts_tolerance_s
                mask &= tts <= tts_max + tts_tolerance_s
            else:
                mask[:] = False
            indices = np.flatnonzero(mask)
            if len(indices) != 1:
                failures.append(
                    f"flight {flight} joint tuple matched {len(indices)} bank rows (required exactly 1)"
                )
                matches.append({
                    "flight_id": flight,
                    "revision_id": revision,
                    "matched": False,
                    "candidate_count": int(len(indices)),
                    "swing_sign": swing,
                    "reach_level": level,
                    "swing_foot_sign": moving_foot,
                    "target_position_home_relative": target_position.tolist(),
                    "target_velocity": target_velocity.tolist(),
                    "target_normal": target_normal.tolist(),
                    "incoming_velocity": incoming.tolist(),
                    "tts_s": tts,
                })
                continue
            index = int(indices[0])
            if candidate_only_v5 and int(round(_f(
                row, "question_fixture_bank_row_id"
            ))) != int(row_ids[index]):
                failures.append(
                    f"flight {flight} fixture row id does not equal matched v5 bank row"
                )
            matches.append({
                "flight_id": flight,
                "revision_id": revision,
                "matched": True,
                "bank_row_id": int(row_ids[index]),
                "bank_array_index": index,
                "source_row": int(source_row[index]),
                "source_flight_id": int(source_flight[index]),
                "swing_sign": swing,
                "reach_level": level,
                "swing_foot_sign": moving_foot,
                "tts_s": tts,
                **(
                    {"fixture_physical_evidence": fixture_physical_evidence}
                    if fixture_physical_evidence is not None
                    else {}
                ),
            })

    by_flight: dict[int, list[dict]] = {}
    for row in matches:
        by_flight.setdefault(int(row["flight_id"]), []).append(row)
    matched_flights = []
    for flight, rows in sorted(by_flight.items()):
        if not rows or not all(row.get("matched") is True for row in rows):
            continue
        permission_tuples = {
            (
                int(row["swing_sign"]),
                int(row["reach_level"]),
                int(row["swing_foot_sign"]),
            )
            for row in rows
        }
        if len(permission_tuples) != 1:
            failures.append(
                f"flight {flight} mutates side/reach/moving-foot permission across revisions"
            )
            continue
        representative = dict(rows[0])
        representative["revision_count"] = len(rows)
        representative["bank_row_ids"] = sorted(
            {int(row["bank_row_id"]) for row in rows}
        )
        matched_flights.append(representative)
    coverage = _coverage_summary(
        matched_flights, minimum_flights, candidate_only=candidate_only
    )
    failures.extend(coverage["failures"])
    return {
        "pass": not failures,
        "evidence_complete": bool(matches) and not failures,
        "formal_gate_verdict": "PASS" if not failures else "FAIL",
        "qualification_status": (
            "PROVEN_CANDIDATE_L0_TUPLE_ONLY"
            if candidate_only and not failures
            else "PROVEN"
            if not failures
            else "NOT_PROVEN"
        ),
        "contract": CONTRACT,
        "axis_envelope_role": "coarse_prefilter_only_not_joint_tuple_certification",
        "bank_sha256": bank_sha,
        "receipt_sha256": receipt_sha,
        "minimum_flights": minimum_flights,
        "flight_count": len(by_flight),
        "matched_flight_count": len(matched_flights),
        "revision_tuple_count": len(matches),
        "matched_revision_tuple_count": sum(
            row.get("matched") is True for row in matches
        ),
        "level_counts": coverage["level_counts"],
        "side_counts": coverage["side_counts"],
        "side_level_counts": coverage["side_level_counts"],
        "matches": matches,
        "failures": failures,
        "continuous_correlated_planner_admission": "NOT_PROVEN",
        "candidate_only": candidate_only,
        "v5_fixture_physical_residual_contract": (
            "required_0p03m_0p10mps_0p03s_v1" if candidate_only_v5 else "not_applicable"
        ),
        "optional_reach_trajectory_qualification": (
            "NOT_TRAJECTORY_CERTIFIED" if candidate_only_v5 else "NOT_PROVEN"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner-csv", type=Path, required=True)
    parser.add_argument("--bank", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--minimum-flights", type=int, default=26)
    args = parser.parse_args()
    try:
        report = build_report(
            args.planner_csv,
            args.bank,
            args.receipt,
            minimum_flights=args.minimum_flights,
        )
    except (OSError, ValueError, KeyError) as exc:
        report = {
            "pass": False,
            "evidence_complete": False,
            "formal_gate_verdict": "FAIL",
            "qualification_status": "NOT_PROVEN",
            "contract": CONTRACT,
            "failures": [f"provenance evidence unavailable: {exc}"],
        }
    args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    verdict = "PASS/PROVEN" if report["pass"] else "FAIL/NOT_PROVEN"
    print(
        f"[question-provenance] {verdict}: flights={report.get('flight_count', 0)} "
        f"matched={report.get('matched_flight_count', 0)} "
        f"levels={report.get('level_counts', {})} sides={report.get('side_counts', {})}"
    )
    for failure in report.get("failures", ()):
        print(f"[question-provenance] FAIL: {failure}")
    print(f"[question-provenance] JSON: {args.json}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
