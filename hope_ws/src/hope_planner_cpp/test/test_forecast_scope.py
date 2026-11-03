"""Check the runtime graph, whole upstream diff, and untouched production bytes."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).parents[1]
REPO = ROOT.parents[2]
BASE = '91aecabfd44cb258ac998158e724be9017d883a7'
FORBIDDEN_PATH = re.compile(
    r'ckf_strike_shadow|strike_target_selector|strike_flight_detector|'
    r'shadow_runner_input|hybrid_trajectory|ball_flight_state_machine|BallForecast|HybridTrajectory')


def test_v2_cannot_construct_commands_or_publish_control_topics():
    forbidden = ['RacketTargetPlanner', 'planner_locked', 'Schema2Packer',
                 'Schema3Packer', 'FlightDecisionV2', 'RacketCommand', 'swing_sign',
                 'policy_z_offset', 'target_land']
    for name in ['common_ball_physics', 'flight_state_v2', 'flight_spin_estimator',
                 'flight_transport_v2', 'probabilistic_ball_tracker',
                 'probabilistic_physics', 'nominal_event_forecast', 'hit_plane_manager']:
        for path in [ROOT / 'src' / (name + '.cpp'),
                     ROOT / 'include/hope_planner_cpp' / (name + '.hpp')]:
            for symbol in forbidden:
                assert symbol not in path.read_text(), (path, symbol)
    source = (ROOT / 'src/flight_prediction_v2_node.cpp').read_text()
    for symbol in forbidden:
        assert symbol not in source, symbol
    assert '/hope/shadow/flight_v2/trajectory' in source
    assert 'remapping must retain v2 shadow namespace' in source
    assert 'resolve_topic_name(topic) != topic' in source
    cmake = (ROOT / 'CMakeLists.txt').read_text()
    v2 = cmake[cmake.index('# V2\'s forward-only'):]
    assert not FORBIDDEN_PATH.search(v2)
    assert 'hope_planner_core' not in v2


def test_production_sources_and_headers_remain_pinned_upstream():
    for file, digest in json.loads((ROOT / 'test/fixtures/pr85_upstream_sources.json').read_text())['sources'].items():
        data = (ROOT / file).read_bytes()
        if file == 'src/planner_node.cpp':
            # This milestone intentionally integrates the shared owner into the
            # existing node. Pin that reviewed integration separately; every
            # other protected downstream source still uses the upstream pin.
            manifest = json.loads((ROOT / 'docs/v2_incoming_scope_manifest.json').read_text())
            integration = manifest['planner_integration']
            assert integration['upstream_sha256'] == digest
            assert integration['behavioral_gates']
            digest = integration['sha256']
        assert hashlib.sha256(data).hexdigest() == digest, file


def test_entire_upstream_diff_has_no_historical_pipeline_and_complete_inventory():
    if not (REPO / '.git').exists():
        # An exported clean source tree has no Git metadata. The manifest and
        # source/binary gates still apply; the Git audit runs in the review checkout.
        return
    def git(*args):
        return subprocess.check_output(['git', '-C', str(REPO), *args], text=True).splitlines()
    paths = set(git('diff', '--name-only', BASE)) | set(git('ls-files', '--others', '--exclude-standard'))
    assert not [p for p in paths if FORBIDDEN_PATH.search(p)]
    manifest = ROOT / 'docs/integrated_planner_pr85_scope_manifest.json'
    if not manifest.exists():
        manifest = ROOT / 'docs/integrated_planner_review_scope_manifest.json'
    if not manifest.exists():
        manifest = ROOT / 'docs/flight_prediction_minimal_scope_manifest.json'
    entries = json.loads(manifest.read_text())['files']
    incoming_manifest = ROOT / 'docs/v2_incoming_scope_manifest.json'
    if incoming_manifest.exists():
        incoming = json.loads(incoming_manifest.read_text())
        assert incoming['extends'] == manifest.name
        merged = {entry['path']: entry for entry in entries}
        merged.update({entry['path']: entry for entry in incoming['files']})
        entries = list(merged.values())
    assert paths == {entry['path'] for entry in entries}
    assert all(entry['why_needed'] and entry['owner'] and entry['tests'] for entry in entries)


def test_legacy_defaults_command_types_and_original_sample_members_are_unchanged():
    if not (REPO / '.git').exists():
        import pytest
        pytest.skip('Exact baseline Git comparison runs in the review checkout')
    relative = str((ROOT / 'include/hope_planner_cpp/types.hpp').relative_to(REPO))
    original = subprocess.check_output(['git', '-C', str(REPO), 'show',
                                        BASE + ':' + relative], text=True)
    current = (REPO / relative).read_text()
    def definition(source, name):
        found = re.search(r'struct ' + name + r'\s*\{.*?^\};', source, re.M | re.S)
        assert found, name
        return found.group()
    for name in ['BallPhysics', 'TableParams', 'PlannerConfig', 'EstimatorConfig',
                 'SpinEstimatorConfig', 'SpinEstimate', 'BallState',
                 'StrikeTarget', 'RacketCommand']:
        assert definition(original, name) == definition(current, name), name
    # Appended V2 provenance is explicit; no old member is reordered or altered.
    assert definition(current, 'BallSample').startswith(definition(original, 'BallSample')[:-3])
