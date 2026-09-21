from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts/pp_gate3_v5_question_fixture.py"
SPEC = importlib.util.spec_from_file_location("pp_gate3_v5_fixture", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
ROOT = Path(__file__).parents[4]
BANK = ROOT / "hope_training/whole_body_tracking/assets/question_bank/fixed_home_support_training_candidate_bank_v5.npz"
RECEIPT = BANK.with_suffix(".receipt.json")


def test_v5_fixture_is_exact_routed_l0_and_closes_physical_launches():
    rows, receipt = MODULE.build_fixture(BANK, RECEIPT)
    assert len(rows) == receipt["expected_flights"] == 26
    assert [int(row["flight_id"]) for row in rows] == list(range(1, 27))
    assert {int(row["reach_level"]) for row in rows} == {0}
    assert {int(row["swing_foot_sign"]) for row in rows} == {0}
    assert {int(row["clip"]) for row in rows} == {0, 1}
    assert max(
        item["maximum_forward_closure_error"] for item in receipt["rows"]
    ) < 2.0e-3
    assert len(receipt["serves_flat"]) == 26 * 6

    with np.load(BANK, allow_pickle=False) as bank:
        by_id = {
            int(row_id): index for index, row_id in enumerate(bank["row_id"])
        }
        for row in rows:
            index = by_id[int(row["bank_row_id"])]
            assert np.array_equal(
                np.asarray([
                    row["contact_offset_x"], row["contact_offset_y"],
                    row["contact_actor_z"],
                ]),
                bank["contact_position_offset"][index].astype(np.float64),
            )
