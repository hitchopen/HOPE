"""Reuse unchanged reference tensors and aggregate log values without changing PPO."""
from contextlib import contextmanager
import torch


def reference_cache(motion):
    """Cache only command-based styles, invalidating on every in-place input write.

    A control-step index alone is insufficient: a reset, replay restore or next
    ball can change the reference multiple times within that same step. Tensor
    version counters detect these writes without a CUDA-to-host synchronization.
    Inference tensors have no version counters, so deliberately bypass the cache
    if any input is one of those. Legacy drift-dependent styles also bypass it.
    """
    mode = getattr(motion.cfg, 'step_reference_mode', '')
    if motion.step_motion is None or mode not in ('target_reach', 'discrete_four_stroke'):
        return None
    try:
        target = motion._env.command_manager.get_term('racket_target')
        inputs = (motion.time_steps, motion.clip_id, target.racket_target_pos_w,
                  target.base_target_pos_w, motion._env.scene.env_origins)
        if mode == 'discrete_four_stroke':
            inputs += (motion.hold_counter, target._stroke_step, target._stroke_initial_tts)
        key = (id(motion.motion), id(motion.step_motion),
               id(motion.cfg.stand_contact_xyz), id(motion.cfg.step_contact_xyz),
               tuple((id(value), value._version) for value in inputs))
    except (AttributeError, KeyError, RuntimeError):
        return None
    previous = getattr(motion, '_reference_value_cache', None)
    if previous is None or previous[0] != key:
        previous = (key, {})
        motion._reference_value_cache = previous
    return previous[1]


def aggregate_episode_infos(episode_infos, device):
    """Match rsl_rl's element-weighted means using one concatenation per key.

    Keep the first dictionary's keys, skip missing keys in later records, and
    preserve the original tensors/dicts. This changes logging work only.
    """
    if not episode_infos:
        return []
    target = torch.device(device)
    result = {}
    for key in episode_infos[0]:
        values = [info[key] for info in episode_infos if key in info]
        if all(not torch.is_tensor(value) for value in values):
            tensor = torch.tensor(values, dtype=torch.float32, device=target)
        else:
            parts = [torch.empty(0, device=target)]
            for value in values:
                if not torch.is_tensor(value):
                    value = torch.tensor([value], dtype=torch.float32, device=target)
                else:
                    value = value.reshape(1) if value.ndim == 0 else value
                    if value.device != target:
                        value = value.to(target)
                parts.append(value)
            tensor = torch.cat(parts)
        result[key] = tensor.mean().reshape(1)
    return [result]


class BatchedScalarWriter:
    """Keep every local scalar, publishing one complete W&B row per iteration."""
    def __init__(self, writer, local_add_scalar, remote_log):
        self._writer = writer
        self._local_add_scalar = local_add_scalar
        self._remote_log = remote_log
        self._pending = None

    def __getattr__(self, name):
        return getattr(self._writer, name)

    @contextmanager
    def scalar_batch(self, step):
        if self._pending is not None:
            raise RuntimeError('Nested scalar batches are not supported')
        self._pending = {}
        self._batch_step = step
        try:
            yield
        finally:
            pending = self._pending
            self._pending = None
            if pending:
                self._remote_log(pending, step=step)

    def add_scalar(self, tag, scalar_value, global_step=None, walltime=None, new_style=False):
        if self._pending is None or global_step != self._batch_step:
            return self._writer.add_scalar(tag, scalar_value, global_step, walltime, new_style)
        value = scalar_value.detach().item() if torch.is_tensor(scalar_value) else scalar_value
        self._local_add_scalar(tag, value, global_step, walltime, new_style)
        self._pending[tag] = value
