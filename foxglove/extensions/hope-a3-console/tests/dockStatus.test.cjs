const assert = require('node:assert/strict');
const test = require('node:test');
const load = require('./loadSource.cjs');
const { decodeDockState, dockRepositionReason, formatDockStatus } = load('dockStatus');
const { createTranslator } = load('i18n');
const t = createTranslator('en');
const sample = [1, 9, 2, 1, 0.12, -0.04, 0.02, 0.13, 4.5, 1, 0, -0.5, -0.7625, 0, 0.01];

test('CDR Float64Array and JSON arrays both decode dock telemetry', () => {
  const backing = new Float64Array([0, ...sample, 0]);
  for (const input of [sample, new Float64Array(sample), backing.subarray(1, 16)]) {
    assert.equal(JSON.stringify(decodeDockState(input)), JSON.stringify(sample));
  }
});

test('rejects a short, wrong-schema, or non-finite dock packet', () => {
  for (const input of [null, {}, new DataView(new ArrayBuffer(120)), sample.slice(1),
    [2, ...sample.slice(1)], [...sample.slice(0, 14), NaN]]) {
    assert.equal(decodeDockState(input), undefined);
  }
});

test('Reposition stays disabled until Auto-dock, Stand, a fresh base pose, and a quiet serve', () => {
  const ready = { autoDock: 'on', runnerFresh: true, runnerFault: false, mode: 'PD_STAND',
    baseFresh: true, serveActive: false, pending: false };
  assert.equal(dockRepositionReason(ready, t), undefined);
  assert.match(dockRepositionReason({ ...ready, autoDock: 'off' }, t), /off/);
  assert.match(dockRepositionReason({ ...ready, runnerFresh: false }, t), /Runner status/);
  assert.match(dockRepositionReason({ ...ready, mode: 'DOCK' }, t), /Stand stops/);
  assert.match(dockRepositionReason({ ...ready, serveActive: true }, t), /serve/);
  assert.match(dockRepositionReason({ ...ready, mode: 'PASSIVE' }, t), /Stand/);
  assert.match(dockRepositionReason({ ...ready, baseFresh: false }, t), /base pose/);
  assert.match(dockRepositionReason({ ...ready, pending: true }, t), /pending/);
});

test('status line reports the world-frame error and phase', () => {
  assert.equal(formatDockStatus(undefined, t), 'WAITING FOR DOCK TELEMETRY');
  assert.match(formatDockStatus(sample, t), /ALIGN · dx 0\.120 m · dy -0\.040 m · dyaw 1\.1 deg/);
  assert.match(formatDockStatus(sample, t), /attempt 1 · 4\.5 s · RUNNING/);
});
