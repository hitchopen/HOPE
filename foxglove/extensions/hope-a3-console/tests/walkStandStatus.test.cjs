const assert = require('node:assert/strict');
const test = require('node:test');
const { walkStandEntryReason, walkStandPhase } = require('./loadSource.cjs')('walkStandStatus');
const english = require('./loadSource.cjs')('i18n').createTranslator('en');
const loaded = [1, 123456, 1, 0, .02, 0, 0, 0, 0, 0];
const reason = (extra = {}) => walkStandEntryReason({ runnerFresh: true, runnerFault: false,
  mode: 'PD_STAND', telemetryFresh: true, state: loaded, pending: false, estop: false, ...extra }, english);

test('walk stand opens only from a fresh, healthy PD stand', () => {
  assert.equal(reason(), undefined);
  assert.match(reason({ runnerFresh: false }), /Runner status/);
  assert.match(reason({ runnerFault: true }), /command fault/);
  assert.match(reason({ estop: true }), /E-stop/);
  for (const mode of ['PASSIVE', 'MOTION', 'SERVE', 'TELEOP', undefined]) {
    assert.match(reason({ mode }), /PD stand\) first/);
  }
  assert.match(reason({ mode: 'WALK_STAND' }), /returns to PD stand/);
  assert.match(reason({ pending: true }), /pending/);
});

test('policy availability blocks only on fresh telemetry; the Runner still decides', () => {
  assert.match(reason({ state: [1, 123456, 0, 0, 0, 0, 0, 0, 0, 0] }), /HumanLike/);
  assert.equal(reason({ telemetryFresh: false, state: undefined }), undefined);
});

test('phase follows the shared locomotion telemetry only in WALK_STAND', () => {
  assert.equal(walkStandPhase('PD_STAND', [1, 1, 1, 2]), undefined);
  assert.equal(walkStandPhase('TELEOP', [1, 1, 1, 2]), undefined);
  assert.equal(walkStandPhase('WALK_STAND', [1, 1, 1, 1]), 'entering');
  assert.equal(walkStandPhase('WALK_STAND', [1, 1, 1, 2]), 'active');
  assert.equal(walkStandPhase('WALK_STAND', [1, 1, 1, 3]), 'stopping');
  assert.equal(walkStandPhase('WALK_STAND', undefined), 'waiting');
});
