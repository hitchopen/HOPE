"""Batch CommandTerm reset logging without changing metric/reset semantics."""

from __future__ import annotations

import torch


def batched_command_term_reset(term, env_ids=None) -> dict[str, float]:
    """Match IsaacLab's metric means, in-place row clearing, counter reset and resampling.

    Independent metrics use one host transfer per shape/dtype/device group and alias wave. Clearing
    uses assignment and fused copies, rather than multiplication: NaN/Inf reset rows must become
    zero too. Unusual metrics sharing storage retain upstream's sequential semantics.
    """
    if env_ids is None:
        env_ids = slice(None)
    waves = {}
    storage_occurrences = {}
    for name, value in term.metrics.items():
        storage = (value.device, value.untyped_storage().data_ptr())
        # Empty tensors all have pointer zero but cannot overlap any nonempty storage.
        wave = storage_occurrences.get(storage, 0) if value.numel() > 0 else 0
        if value.numel() > 0:
            storage_occurrences[storage] = wave + 1
        groups = waves.setdefault(wave, {})
        groups.setdefault((tuple(value.shape), value.dtype, value.device), []).append((name, value))

    extras = dict.fromkeys(term.metrics)
    # A clear can affect a later view of the same storage. Process occurrences in order,
    # batching only independent storages within each wave; one alias must not force every
    # unrelated metric back onto the per-scalar GPU synchronization path.
    for wave in sorted(waves):
        groups = waves[wave]
        for items in groups.values():
            tensors = [value for _, value in items]
            stacked = torch.stack(tensors)
            selected = stacked[:, env_ids].reshape(len(items), -1)
            means = selected.mean(dim=1).cpu().tolist()
            for (name, _), mean in zip(items, means):
                extras[name] = mean
            stacked[:, env_ids] = 0.0
            torch._foreach_copy_(tensors, list(stacked.unbind(0)))

    term.command_counter[env_ids] = 0
    term._resample(env_ids)
    return extras
