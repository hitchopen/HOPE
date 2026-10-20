"""Enforce the explicit forecast-only dependency and legacy source boundaries."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).parents[1]


def test_v2_cannot_construct_commands_or_publish_control_topics():
    source=(ROOT/'src/flight_prediction_v2_node.cpp').read_text()
    for name in ['RacketTargetPlanner','planner_locked','Schema2Packer','FlightDecisionV2','StrikeTarget',
                 'swing_sign','policy_z_offset','target_land','/racket_command']:
        assert name not in source, name
    assert '/hope/shadow/flight_v2/trajectory' in source
    assert 'remapping must retain v2 shadow namespace' in source


def test_inverse_and_scheduling_sources_remain_pinned_upstream():
    for file,digest in json.loads((ROOT/'test/fixtures/scoped_production_sources.json').read_text()).items():
        assert hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==digest, file
