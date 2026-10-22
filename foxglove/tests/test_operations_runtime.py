"""Shutdown ownership when DDS has already been closed externally."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_operations_runtime import drain_and_destroy


def test_closed_dds_and_status_errors_still_join_cleanup_before_destroy():
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    order, errors = [], []
    logger = Mock()

    def close():
        entered.set()
        assert release.wait(3)
        order.append("worker cleanup")
        finished.set()
        raise RuntimeError("cleanup diagnostic")

    def destroy():
        assert finished.is_set()
        order.append("destroy")

    node = SimpleNamespace(
        get_logger=lambda: logger,
        stop_accepting=Mock(side_effect=RuntimeError("status publisher closed")),
        close=close,
        context=SimpleNamespace(ok=lambda: False),
        _hope_rpc_tracker=SimpleNamespace(pending=1),
        destroy_node=destroy,
    )
    executor = Mock()
    executor.shutdown.side_effect = lambda **_kwargs: order.append("executor shutdown")

    def drain():
        try:
            drain_and_destroy(node, executor, response_grace_s=0)
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=drain)
    thread.start()
    try:
        assert entered.wait(1)
        assert thread.is_alive()
        executor.shutdown.assert_not_called()
        assert order == []
    finally:
        release.set()
        thread.join(3)
    assert not thread.is_alive()
    assert errors == []
    assert order == ["worker cleanup", "executor shutdown", "destroy"]
    executor.spin_once.assert_not_called()
    diagnostics = str(logger.error.call_args_list)
    assert "status publisher closed" in diagnostics
    assert "cleanup diagnostic" in diagnostics
    assert "DDS context closed before pending RPC responses could drain" in diagnostics
