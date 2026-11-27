const assert = require('node:assert/strict');
const test = require('node:test');
const { decodeDockState, dockRepositionReason, dockPhaseName, formatDockStatus } = require('./loadSource.cjs')('dockStatus');
const english = require('./loadSource.cjs')('i18n').createTranslator('en');
const sample = [1, 77, 2, 1, .1, -.2, .05, .2236, 3.5, 1, 0, -.5, -.7625, 0, .02];
const reason = (extra = {}) => dockRepositionReason({ runnerFresh: true, runnerFault: false,
  mode: 'PD_STAND', telemetryFresh: true, baseFresh: true, serving: false, pending: false,
  estop: false, ...extra }, english);

test('decodes typed arrays and rejects wrong schema or length', () => {
  assert.deepEqual([...decodeDockState(new Float64Array(sample))], sample);
  assert.deepEqual([...decodeDockState(sample)], sample);
  assert.equal(decodeDockState(sample.slice(1)), undefined);
  assert.equal(decodeDockState([2, ...sample.slice(1)]), undefined);
  assert.equal(decodeDockState([...sample.slice(0, 14), Number.NaN]), undefined);
});

test('Reposition opens from Stand, Walk stand or Teleop with a fresh base pose and dock telemetry', () => {
  assert.equal(reason(), undefined);
  assert.equal(reason({ mode: 'WALK_STAND' }), undefined);
  assert.equal(reason({ mode: 'TELEOP' }), undefined);
  assert.equal(reason({ mode: 'MOTION' }), undefined);
  assert.match(reason({ telemetryFresh: false }), /no Reposition loaded/);
  assert.match(reason({ mode: 'DOCK' }), /Stand stops it/);
  for (const mode of ['PASSIVE', 'SERVE']) {
    assert.match(reason({ mode }), /Select Stand or Teleop/);
  }
  assert.match(reason({ baseFresh: false }), /base pose/);
  assert.match(reason({ serving: true }), /serve/);
  assert.match(reason({ estop: true }), /E-stop/);
  assert.match(reason({ runnerFault: true }), /command fault/);
});

test('status line shows phase, error and result', () => {
  assert.equal(dockPhaseName(sample), 'ALIGN');
  const line = formatDockStatus(sample, english);
  assert.match(line, /^ALIGN · Δx 0\.100 m · Δy -0\.200 m · Δyaw 2\.9° · 0\.224 m · try 1 · 3\.5 s · RUNNING$/);
  const failed = [...sample]; failed[2] = 6; failed[9] = 3; failed[10] = 5;
  assert.match(formatDockStatus(failed, english), /FAILED · START_TOO_FAR$/);
  assert.match(formatDockStatus(undefined, english), /Waiting/);
});
