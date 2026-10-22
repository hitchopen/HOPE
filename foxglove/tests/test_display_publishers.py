import os
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest
pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_display_publishers import display_publisher
from rclpy.event_handler import PublisherEventCallbacks


def test_only_default_warning_waitables_removed(monkeypatch):
    monkeypatch.delenv("HOPE_DISPLAY_QOS_EVENTS", raising=False)
    node = Mock()
    kind, qos = object(), object()
    display_publisher(node, kind, "/hope/display", qos)
    args, kwargs = node.create_publisher.call_args
    assert args == (kind, "/hope/display", qos)
    assert kwargs["event_callbacks"].use_default_callbacks is False
    callback = PublisherEventCallbacks(matched=lambda _: None)
    display_publisher(node, kind, "/hope/display", qos, event_callbacks=callback)
    assert node.create_publisher.call_args.kwargs["event_callbacks"] is callback


def test_diagnostic_override_restores_default_callbacks(monkeypatch):
    monkeypatch.setenv("HOPE_DISPLAY_QOS_EVENTS", "1")
    node = Mock()
    display_publisher(node, str, "/hope/display", 10)
    assert "event_callbacks" not in node.create_publisher.call_args.kwargs
