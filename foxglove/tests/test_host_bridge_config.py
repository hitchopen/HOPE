from pathlib import Path
import importlib.util
import yaml
import re

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('host_bridge_config', ROOT/'foxglove/laptop/host_bridge_config.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_host_endpoint_preserves_exact_control_boundary():
    source = (ROOT/'foxglove/a3/bridge_params_control.yaml').read_text()
    original = yaml.safe_load(source)['/**']['ros__parameters']
    params = module.host_config(source)['/**']['ros__parameters']
    assert params['address'] == '127.0.0.1' and params['port'] == 8767
    for key in ['service_whitelist', 'client_topic_whitelist', 'param_whitelist', 'capabilities']:
        assert params[key] == original[key]
    assert '^/hope/live/.*' in params['topic_whitelist']
    assert 'display_subscription_nice' not in params
    assert 'display_ball_marker_hz' not in params
    assert module.host_config(yaml.safe_dump(module.host_config(source))) == module.host_config(source)


def test_live_scene_has_no_hdu_bridge_route():
    for name in ['bridge_params.yaml', 'bridge_params_control.yaml']:
        p = yaml.safe_load((ROOT/'foxglove/a3'/name).read_text())['/**']['ros__parameters']
        rules = p['topic_whitelist']
        assert not any(re.fullmatch(rule, '/hope/live/scene') for rule in rules)
        assert any(re.fullmatch(rule, '/hope/runner/mode') for rule in rules)
