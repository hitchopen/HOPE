const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const api = {};
Object.assign(api, require('./loadSource.cjs')('mocapSafety'));
const english = require('./loadSource.cjs')('i18n').createTranslator('en');
const safetyText = (state) => api.mocapSafetyText(state, english);
test('mocap safety accepts CDR typed arrays and rejects malformed status', () => {
  for (const value of [[1,42,7,8,1,0,1,1], new Float64Array([1,42,7,8,1,0,1,1])]) {
    assert.match(safetyText(api.decodeMocapSafety(value)), /Stopped: mocap lost/);
  }
  for (const value of [[], [1,42,7,8,1,0,1], [1,42,7,8,1,NaN,1,1],
    [1,42,7,8,1,2,1,1], [1,0,7,8,1,0,1,1]]) {
    assert.equal(api.decodeMocapSafety(value), undefined);
  }
});
test('restored stream retains stop reason; offline modes and startup are explicit', () => {
  assert.match(safetyText(api.decodeMocapSafety([1,42,7,8,1,1,1,1])), /explicitly re-enter/);
  assert.equal(safetyText(api.decodeMocapSafety([1,42,7,8,1,1,1,0])), 'Mocap fresh');
  assert.match(safetyText(api.decodeMocapSafety([1,42,7,8,0,0,0,0])), /Offline/);
  assert.match(safetyText(api.decodeMocapSafety([1,42,7,8,1,0,0,0])), /entry blocked/);
  assert.match(safetyText(undefined), /unavailable/);
});
test('held pelvis is not displayed as fresh and operator losses have independent reasons', () => {
  const packet=[1,42,7,8,1,0,0,0,1,25.5,1,1,1,0,0];
  assert.match(safetyText(api.decodeMocapSafety(packet)), /Holding last pelvis pose.*PD_STAND at 1 s/);
  packet[13]=36;
  assert.match(safetyText(api.decodeMocapSafety(packet)), /Stopped: Foxglove UI lost/);
  packet[13]=37;
  assert.match(safetyText(api.decodeMocapSafety(packet)), /Stopped: operator computer lost/);
  packet[5]=1;
  assert.match(safetyText(api.decodeMocapSafety(packet)), /links restored/);
});
