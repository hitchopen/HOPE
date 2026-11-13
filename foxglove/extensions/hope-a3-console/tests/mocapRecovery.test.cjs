const assert = require('node:assert/strict');
const test = require('node:test');
const {startBlockReason, mocapRecoveryText} = require('./loadSource.cjs')('mocapRecovery');
const good = {lifecycleState:'STOPPED', lifecycleBusy:false, calibrationBusy:false,
  configApplied:true, configTouched:false, fieldBusy:false, fieldFresh:true,
  kernelMode:false, ballReady:true, playerReady:true, side:'P1'};

test('each START gate gives an explicit reason and all good evidence enables Start', () => {
  assert.equal(startBlockReason(good), undefined);
  for (const bad of [{lifecycleState:'RUNNING'}, {lifecycleBusy:true}, {calibrationBusy:true},
    {configApplied:false}, {configTouched:true}, {fieldBusy:true}, {fieldFresh:false},
    {ballReady:false}]) {
    assert.equal(typeof startBlockReason({...good,...bad}), 'string');
  }
  assert.equal(startBlockReason({...good,playerReady:false}), undefined);
});
test('PASSIVE startup permits absent player evidence and retains field readiness', () => {
  assert.equal(startBlockReason({...good,playerReady:false}), undefined);
  assert.notEqual(startBlockReason({...good,fieldBusy:true}), undefined);
  assert.match(mocapRecoveryText({state:'VERIFYING',retry_after_s:60}), /等待新的玩家位姿/);
});
test('existing FAILED cleanup and kernel semantics are retained', () => {
  assert.equal(startBlockReason({...good,lifecycleState:'FAILED',ballReady:false,playerReady:false}), undefined);
  assert.equal(startBlockReason({...good,kernelMode:true,ballReady:false,playerReady:false}), undefined);
  assert.notEqual(startBlockReason({...good,lifecycleState:'FAILED',fieldFresh:false}), undefined);
});
test('old backend and exhausted recovery have visible actionable status', () => {
  assert.match(mocapRecoveryText(undefined), /尚未提供/);
  assert.match(mocapRecoveryText({state:'NEEDS_ATTENTION',retry_after_s:0}), /3 次.*VRPN.*重新连接/);
  assert.match(mocapRecoveryText({state:'WRONG_SIDE',retry_after_s:0}), /桌端/);
  assert.match(mocapRecoveryText({state:'BLOCKED',retry_after_s:0}), /暂停/);
});
