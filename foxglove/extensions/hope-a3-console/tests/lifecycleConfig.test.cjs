const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const exportsObject = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname, '../src/lifecycleConfig.ts'), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS },
}).outputText, { exports: exportsObject });
const { mergeConfirmedConfig } = exportsObject;
const draft = { mocap_source: '', table_side: '', pants_profile: '', gripper_profile: '', operator_profile: '' };

test('legacy supervisor fills received menus without the newer operator field', () => {
  const result = mergeConfirmedConfig(draft, { mocap_source: 'vrpn', table_side: 'P1', pants_profile: 'v2', gripper_profile: 'new_20261015' }, false);
  assert.equal(result.mocap_source, 'vrpn');
  assert.equal(result.table_side, 'P1');
  assert.equal(result.pants_profile, 'v2');
  assert.equal(result.gripper_profile, 'new_20261015');
  assert.equal(result.operator_profile, '');
  assert.equal(draft.table_side, '');
});

test('asynchronous and missing fields preserve prior values; explicit empty values apply', () => {
  const first = mergeConfirmedConfig(draft, { table_side: 'P1' }, false);
  const next = mergeConfirmedConfig(first, { table_side: undefined, operator_profile: 'mac' }, false);
  assert.equal(next.table_side, 'P1');
  assert.equal(next.operator_profile, 'mac');
  assert.equal(mergeConfirmedConfig(next, { table_side: '' }, false).table_side, '');
});

test('incoming telemetry cannot overwrite unsaved operator edits', () => {
  const edited = { ...draft, table_side: 'P2' };
  assert.equal(mergeConfirmedConfig(edited, { table_side: 'P1' }, true), edited);
});
