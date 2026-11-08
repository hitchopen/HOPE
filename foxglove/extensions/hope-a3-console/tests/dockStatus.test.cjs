const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const source = fs.readFileSync(path.join(__dirname, '../src/dockStatus.ts'), 'utf8');
const exportsObject = {};
const code = ts.transpileModule(source, { compilerOptions: { module: ts.ModuleKind.CommonJS } }).outputText;
vm.runInNewContext(code, { exports: exportsObject, ArrayBuffer, DataView });
const { decodeDockState, dockRepositionReason, formatDockStatus } = exportsObject;
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
  assert.equal(dockRepositionReason(ready), undefined);
  assert.match(dockRepositionReason({ ...ready, autoDock: 'off' }), /off/);
  assert.match(dockRepositionReason({ ...ready, runnerFresh: false }), /Runner status/);
  assert.match(dockRepositionReason({ ...ready, mode: 'DOCK' }), /Stand stops/);
  assert.match(dockRepositionReason({ ...ready, serveActive: true }), /serve/);
  assert.match(dockRepositionReason({ ...ready, mode: 'PASSIVE' }), /Stand/);
  assert.match(dockRepositionReason({ ...ready, baseFresh: false }), /base pose/);
  assert.match(dockRepositionReason({ ...ready, pending: true }), /pending/);
});

test('status line reports the world-frame error and phase', () => {
  assert.equal(formatDockStatus(undefined), 'WAITING FOR DOCK TELEMETRY');
  assert.match(formatDockStatus(sample), /ALIGN · dx 0\.120 m · dy -0\.040 m · dyaw 1\.1 deg/);
  assert.match(formatDockStatus(sample), /attempt 1 · 4\.5 s · RUNNING/);
});
