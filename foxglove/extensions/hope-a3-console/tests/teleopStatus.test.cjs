const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const source = fs.readFileSync(path.join(__dirname, '../src/teleopStatus.ts'), 'utf8');
const exportsObject = {};
const code = ts.transpileModule(source, { compilerOptions: { module: ts.ModuleKind.CommonJS } }).outputText;
vm.runInNewContext(code, { exports: exportsObject, ArrayBuffer, DataView });
const { decodeTeleopState, xboxInputFresh, teleopEntryReason } = exportsObject;
const entry = (runnerFresh, runnerFault, mode, telemetryFresh, state, pending) =>
  teleopEntryReason({ runnerFresh, runnerFault, mode, telemetryFresh, state, pending });
const state = [1, 123456, 1, 0, .02, 0, 0, 0, 0, 0];

test('CDR Float64Array and JSON arrays both unlock teleop in Stand', () => {
  const backing = new Float64Array([0, ...state, 0]);
  for (const input of [state, new Float64Array(state), backing.subarray(1, 11)]) {
    const decoded = decodeTeleopState(input);
    assert.equal(JSON.stringify(decoded), JSON.stringify(state));
    assert.equal(entry(true, false, 'PD_STAND', true, decoded, false), undefined);
  }
});

test('rejects malformed telemetry instead of enabling a button', () => {
  for (const input of [null, {}, new DataView(new ArrayBuffer(80)), state.slice(1),
    [2, ...state.slice(1)], [...state.slice(0, 9), NaN], [...state.slice(0, 9), '0']]) {
    assert.equal(decodeTeleopState(input), undefined);
  }
});

test('continuous input remains live between 250 ms repaint timer ticks', () => {
  const start = 1000000;
  for (let elapsed = 0; elapsed < 2000; elapsed += 20) {
    const renderNow = start + elapsed;
    assert.equal(xboxInputFresh(renderNow, renderNow - 5, renderNow), true);
  }
  // This old cached clock falsely treated a new packet as future-dated.
  assert.equal(xboxInputFresh(start + 200, start + 195, start), false);
});

test('expired and genuinely future-dated input stays stale', () => {
  assert.equal(xboxInputFresh(1000, 1000, 1501), false);
  assert.equal(xboxInputFresh(1501, 1000, 1501), false);
  assert.equal(xboxInputFresh(1000, 1101, 1000), false);
  assert.equal(xboxInputFresh(undefined, undefined, 1000), false);
});

test('button explains missing Runner, Stand, telemetry, input and fault', () => {
  assert.match(entry(false, false, undefined, false, undefined, false), /Start the system/);
  assert.match(entry(true, false, 'PASSIVE', true, state, false), /Select Stand/);
  assert.match(entry(true, false, 'PD_STAND', false, state, false), /telemetry/);
  const oldInput = [...state]; oldInput[4] = .3;
  assert.match(entry(true, false, 'PD_STAND', true, oldInput, false), /Xbox input/);
  assert.match(entry(true, true, 'PD_STAND', true, state, false), /fault/);
});


test('Stand -> Entering -> Active -> Stopping -> Stand has accurate feedback', () => {
  const current = [...state];
  assert.equal(entry(true, false, 'PD_STAND', true, current, false), undefined);
  for (const [phase, text] of [[1, /Entering Teleop/], [2, /Teleop is ACTIVE/], [3, /Stopping and settling/]]) {
    current[3] = phase;
    const reason = entry(true, false, 'TELEOP', true, current, false);
    assert.match(reason, text);
    assert.doesNotMatch(reason, /Select Stand/);
  }
  // Mode and phase arrive on separate topics. Fresh PD_STAND is authoritative
  // even if the final STOPPING telemetry packet has not yet been replaced.
  assert.equal(entry(true, false, 'PD_STAND', true, current, false), undefined);
  assert.match(entry(false, false, 'TELEOP', true, current, false), /Start the system/);
  assert.match(entry(true, false, 'TELEOP', false, undefined, false), /waiting for its current phase/);
});
