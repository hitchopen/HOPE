const assert = require('node:assert/strict');
const test = require('node:test');
const { lifecycleResultView: viewInVm } = require('./loadSource.cjs')('lifecycleResult');
// The module runs in a vm realm; compare plain data, not prototypes.
const lifecycleResultView = (...args) => JSON.parse(JSON.stringify(viewInVm(...args)));

test('a saved config revision reads as applied once the live runtime readback reaches it', () => {
  const pending = 'CONFIG_SAVED_PENDING_RUNTIME_REVISION_29';
  assert.deepEqual(lifecycleResultView(pending, { applied: true, revision: 29 }), { kind: 'config', revision: 29, applied: true });
  assert.deepEqual(lifecycleResultView(pending, { applied: true, revision: 30 }), { kind: 'config', revision: 29, applied: true });
  // Older readback, unapplied or stale/missing readback: still waiting.
  assert.deepEqual(lifecycleResultView(pending, { applied: true, revision: 28 }), { kind: 'config', revision: 29, applied: false });
  assert.deepEqual(lifecycleResultView(pending, { applied: false, revision: 29 }), { kind: 'config', revision: 29, applied: false });
  assert.deepEqual(lifecycleResultView(pending, undefined), { kind: 'config', revision: 29, applied: false });
  assert.deepEqual(lifecycleResultView('CONFIG_SAVED_REVISION_7', { applied: true, revision: 7 }), { kind: 'config', revision: 7, applied: true });
});

test('idle results are hidden; everything else stays verbatim for diagnosis', () => {
  for (const result of [undefined, '', 'CONFIG_LOADED', 'CONFIG_NOT_CONFIRMED']) {
    assert.deepEqual(lifecycleResultView(result, { applied: true, revision: 3 }), { kind: 'hidden' });
  }
  for (const result of ['CONFIG_SAVED_MOCAP_FAILED: timeout', 'PREFLIGHT_REJECTED: x', 'START_COMPLETE_RUNNER_PASSIVE',
    'CONFIG_SAVED_PENDING_RUNTIME_REVISION_', 'CONFIG_SAVED_PENDING_RUNTIME_REVISION_29 extra']) {
    assert.deepEqual(lifecycleResultView(result, { applied: true, revision: 99 }), { kind: 'raw', text: result });
  }
});

test('an unchanged re-confirmation reads as the same configuration revision', () => {
  assert.deepEqual(lifecycleResultView('CONFIG_UNCHANGED_REVISION_45', { applied: true, revision: 45 }), { kind: 'config', revision: 45, applied: true });
  assert.equal(lifecycleResultView('CONFIG_UNCHANGED_REVISION_45 extra', undefined).kind, 'raw');
});
