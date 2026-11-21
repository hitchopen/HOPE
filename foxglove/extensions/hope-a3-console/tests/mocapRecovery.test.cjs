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
  assert.match(mocapRecoveryText({state:'ASSET_NOT_TRACKED',retry_after_s:0}), /裤子刚体/);
  assert.match(mocapRecoveryText({state:'BLOCKED',retry_after_s:0}), /暂停/);
});

test('Start checklist chips are exactly the startBlockReason conditions, in order', () => {
  const {startChecks} = require('./loadSource.cjs')('mocapRecovery');
  const reasons = {busy:startBlockReason({...good,lifecycleBusy:true}), state:startBlockReason({...good,lifecycleState:'RUNNING'}),
    calibration:startBlockReason({...good,calibrationBusy:true}), config:startBlockReason({...good,configApplied:false}),
    field:startBlockReason({...good,fieldFresh:false}), ball:startBlockReason({...good,ballReady:false})};
  const flags = ['lifecycleBusy','calibrationBusy','configApplied','configTouched','fieldBusy','fieldFresh','kernelMode','ballReady','playerReady'];
  let combinations = 0;
  for (const lifecycleState of ['STOPPED','CONFIG_ERROR','FAILED','RUNNING','STARTING','KILLING','NO DATA']) {
    for (let bits = 0; bits < 2 ** flags.length; bits++) {
      const evidence = {...good, lifecycleState};
      flags.forEach((flag, index) => { evidence[flag] = (bits & (1 << index)) !== 0; });
      const checks = startChecks(evidence);
      const reason = startBlockReason(evidence);
      assert.equal(reason === undefined, checks.every(check => check.ok), JSON.stringify(evidence));
      const firstFailing = checks.find(check => !check.ok);
      assert.equal(reason, firstFailing ? reasons[firstFailing.id] : undefined, JSON.stringify(evidence));
      combinations++;
    }
  }
  assert.equal(combinations, 7 * 512);
});

const { mocapConnectionView } = require('./loadSource.cjs')('mocapRecovery');
const allowed = (o = {}) => ({ state: 'BLOCKED', attempts: 0, max_attempts: 3, retry_after_s: 0, manual_allowed: true,
  blocked_reason: '', manual_blocked_reason: '', manual_blocked_code: '', error: '', ...o });
const view = (o = {}) => mocapConnectionView({ source: 'motive', target: 'Motive 169.254.10.221', recovery: allowed(),
  anyTracked: false, selectedTracked: false, kernelMode: false, requesting: false, calibrating: false, ...o });
test('mocap connection row: Motive reconnect allowed while stopped, stable text without poses', () => {
  const v = view();
  assert.equal(v.canReconnect, true);
  assert.equal(v.tone, 'warn');
  assert.match(v.buttonTitle, /169\.254\.10\.221/);  // the address is in the row header and hover text
  assert.equal(view({ kernelMode: true }).tone, 'idle');
});
test('mocap connection row: robot refusal and cooldown disable the button with a reason', () => {
  for (const code of ['RUNNING', 'TELEMETRY', 'BUSY', 'STOPPING', 'CONFIG', 'NONE']) {
    const v = view({ recovery: allowed({ manual_allowed: false, manual_blocked_code: code, manual_blocked_reason: 'x y' }) });
    assert.equal(v.canReconnect, false);
    assert.notEqual(v.buttonTitle, 'x y');
  }
  assert.equal(view({ recovery: allowed({ manual_allowed: false, manual_blocked_code: 'OTHER', manual_blocked_reason: 'odd' }) }).buttonTitle, 'odd');
  assert.equal(view({ recovery: allowed({ manual_allowed: false, retry_after_s: 7 }) }).canReconnect, false);
  // Never enable on missing or older robot status, even if it claims manual_allowed.
  assert.equal(view({ recovery: undefined }).canReconnect, false);
  assert.equal(view({ recovery: allowed({ manual_blocked_code: undefined }) }).canReconnect, false);
});
test('mocap connection row: in-flight, live, wrong body, failure and none', () => {
  assert.deepEqual([view({ requesting: true }).tone, view({ requesting: true }).canReconnect], ['busy', false]);
  assert.equal(view({ recovery: allowed({ state: 'RECONNECTING' }) }).tone, 'busy');
  assert.equal(view({ anyTracked: true, selectedTracked: true }).tone, 'ok');
  assert.equal(view({ anyTracked: true, selectedTracked: false }).tone, 'warn');
  assert.equal(view({ recovery: allowed({ error: 'UNMANAGED_BRIDGE_PRESENT' }) }).tone, 'bad');
  const none = view({ source: 'none', target: '' });
  assert.deepEqual([none.tone, none.canReconnect], ['idle', false]);
});
test('mocap connection row: disabled while time calibration owns the interlock', () => {
  const v = view({ calibrating: true });
  assert.equal(v.canReconnect, false);
  assert.notEqual(v.buttonTitle, '');
});
test('normal-mode START blocked only by the missing mocap receiver says so', () => {
  const blocked = {...good, configApplied:false};
  assert.notEqual(startBlockReason({...blocked, mocapDown:true}), startBlockReason(blocked));
  // Unconfirmed edits still win: they are the actionable cause.
  assert.equal(startBlockReason({...blocked, mocapDown:true, configTouched:true}), startBlockReason({...blocked, configTouched:true}));
});
