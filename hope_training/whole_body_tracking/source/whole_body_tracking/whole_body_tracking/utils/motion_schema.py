"""Motion-array schema helpers that do not require Isaac Sim."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np


def select_motion_bodies(
    array: np.ndarray,
    body_indexes: Sequence[int],
    source: str,
    field: str,
    articulation_body_count: int | None = None,
) -> np.ndarray:
    """Return tracked bodies from either supported motion representation.

    Complete reference motion artifacts store every articulation body and therefore need
    the live articulation indexes applied.  The public starter clips store only
    the tracked bodies, already in ``MotionCommandCfg.body_names`` order.  The
    old loader applied articulation indexes to both representations, turning a
    useful schema error into a CUDA out-of-bounds assertion on a fresh clone.
    """

    indexes = np.asarray(body_indexes, dtype=np.int64).reshape(-1)
    if indexes.size == 0:
        raise ValueError("MotionLoader resolved no tracked body indexes")
    if int(indexes.min()) < 0:
        raise ValueError(
            f"MotionLoader body indexes must be non-negative, got {int(indexes.min())}"
        )
    if array.ndim < 2:
        raise ValueError(
            f"motion file {source!r} field {field!r} must have a body axis, "
            f"got shape {array.shape}"
        )

    body_count = int(array.shape[1])
    tracked_count = int(indexes.size)
    # Compact public clips carry exactly the tracked bodies.  This form remains
    # unambiguous even when a tracked articulation index happens to be smaller
    # than ``tracked_count``.
    if body_count == tracked_count:
        return array

    max_index = int(indexes.max())
    # Full reference clips carry the complete, unmerged URDF body list (32 for
    # A3). Isaac's URDF importer may merge fixed children into a smaller live
    # articulation, so accept an explicit count or any wider complete array.
    if body_count > max_index and (
        articulation_body_count is None or body_count >= articulation_body_count
    ):
        return array[:, indexes]

    raise ValueError(
        f"motion file {source!r} stores {body_count} bodies in {field}, but "
        f"the prepared articulation maps a tracked body to index {max_index}. "
        f"Provide either {tracked_count} tracked bodies in configured order or "
        "the complete articulation-body arrays."
    )
