from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

spec = spec_from_file_location('field_display', Path(__file__).resolve().parents[2] / 'scripts/macos/field-display.py')
display = module_from_spec(spec)
spec.loader.exec_module(display)


def test_receipt_identity_and_session_reconfiguration():
    hdu, laptop = '192.168.200.2', '192.168.100.6'
    receipt = dict(HOPE_LIFECYCLE_HDU_IP=hdu, HOPE_LIFECYCLE_LAPTOP_IP=laptop,
                   HOPE_LIFECYCLE_ROOT=str(display.ROOT), HOPE_LIFECYCLE_SESSION_ID='session1')
    keys = display.display_keys(receipt, hdu, laptop)
    assert set(keys) == {'hope-host-bridge', 'hope-live-scene'}
    assert not display.display_keys(receipt, hdu, '192.168.100.7')
    assert not display.display_keys({}, hdu, laptop)
    changed = display.display_keys(dict(receipt, HOPE_LIFECYCLE_SESSION_ID='session2'), hdu, laptop)
    assert keys['hope-host-bridge'] == changed['hope-host-bridge']
    assert keys['hope-live-scene'] != changed['hope-live-scene']
