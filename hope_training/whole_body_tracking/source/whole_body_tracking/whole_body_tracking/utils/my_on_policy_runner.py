"""Minimal rsl_rl PPO runner glue for HOPE PingPong training.

The base ``rsl_rl.runners.OnPolicyRunner`` already writes periodic (every ``save_interval``) and final
local checkpoints. This subclass only replaces the logging writer with a local, offline no-op sink so
training pulls in **no** Weights & Biases / TensorBoard / external logging service, and adds no gate,
lineage, receipt, or ONNX-export coupling (export is a separate script). Per-iteration console
progress from rsl_rl is preserved; the only shipped machine-readable metric is ``success_rate`` from
``scripts/evaluate.py``.
"""

from __future__ import annotations

from rsl_rl.runners import OnPolicyRunner


class _LocalNullWriter:
    """A local, offline stand-in for rsl_rl's summary writer.

    Implements the small surface rsl_rl calls on ``self.writer`` (``add_scalar``, ``log_config``,
    ``save_model``, ``save_file``, ``stop``/``flush``) as no-ops, and returns a no-op for anything
    else, so training never depends on TensorBoard or Weights & Biases. Checkpoints are still written
    locally by the runner's ``save()``.
    """

    def __init__(self, *args, **kwargs) -> None:
        pass

    def add_scalar(self, *args, **kwargs) -> None:
        pass

    def log_config(self, *args, **kwargs) -> None:
        pass

    def save_model(self, *args, **kwargs) -> None:
        pass

    def save_file(self, *args, **kwargs) -> None:
        pass

    def stop(self, *args, **kwargs) -> None:
        pass

    def flush(self, *args, **kwargs) -> None:
        pass

    def close(self, *args, **kwargs) -> None:
        pass

    def __getattr__(self, _name):
        # Any other writer method (across rsl_rl versions) becomes a no-op.
        def _noop(*args, **kwargs):
            return None

        return _noop


class HOPEOnPolicyRunner(OnPolicyRunner):
    """rsl_rl OnPolicyRunner with local-only, offline logging (no W&B / TensorBoard)."""

    def _prepare_logging_writer(self) -> None:
        if self.log_dir is not None and self.writer is None and not self.disable_logs:
            self.logger_type = "local"
            self.writer = _LocalNullWriter(log_dir=self.log_dir)
