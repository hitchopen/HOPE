const test = require('node:test');
const assert = require('node:assert/strict');
const {parseLiveTelemetry, liveTelemetryFresh, liveSourceFresh} = require('./loadSource.cjs')('liveTelemetry');
const packet = (extra = {}) => JSON.stringify({timestamp_s:100,session_id:'a',generation:3,shot:3,robot_available:false,robot_age_ms:null,play_state_age_ms:10,play_state:{state:'RALLY',reason:'',observation_missing:true,forwarding:false},robot_xyz:null,robot_rpy_deg:null,racket_xyz:null,racket_speed:null,ball_xyz:null,target_xyz:null,frozen:null,policy_frozen:null,...extra});
test('new ball drops both previous generation ghosts', () => {
 const x=parseLiveTelemetry(packet({frozen:{generation:2,ball_xyz:[1,2,3]},policy_frozen:{generation:2,racket_xyz:[1,2,3]}}));
 assert.equal(x.frozen,null); assert.equal(x.policy_frozen,null);
});
test('ball and racket ghosts are independent complete snapshots', () => {
 const a=parseLiveTelemetry(packet({frozen:{generation:3,ball_xyz:[1,2,3]}}));
 const b=parseLiveTelemetry(packet({policy_frozen:{generation:3,racket_xyz:[4,5,6]}}));
 assert.ok(a.frozen); assert.equal(a.policy_frozen,null); assert.equal(b.frozen,null); assert.ok(b.policy_frozen);
});
test('missing observations retain logical state without inventing robot values', () => {
 const x=parseLiveTelemetry(packet()); assert.equal(x.play_state.state,'RALLY'); assert.equal(x.play_state.observation_missing,true); assert.equal(x.robot_xyz,null); assert.equal(x.racket_speed,null);
});
test('bridge heartbeat cannot conceal stale source and stale deliveries expire', () => {
 const x=parseLiveTelemetry(packet()); assert.equal(liveTelemetryFresh(x,100010,100100),true); assert.equal(liveTelemetryFresh(x,100010,101010),false); assert.equal(liveTelemetryFresh(x,105000,105000),false); assert.equal(liveSourceFresh(1000),false); assert.equal(liveSourceFresh(null),false); assert.equal(liveSourceFresh(99),true);
});
test('invalid snapshots are rejected, new session can restart ball number', () => {
 assert.equal(parseLiveTelemetry('broken'),undefined); assert.equal(parseLiveTelemetry(packet({generation:-1})),undefined); const x=parseLiveTelemetry(packet({session_id:'b',generation:0})); assert.equal(x.generation,0);
});
test('Runner mode requires its own fresh source and cannot be inferred from ball state', () => {
 const {liveRunnerLabel} = require('./loadSource.cjs')('liveTelemetry');
 assert.equal(liveRunnerLabel(parseLiveTelemetry(packet({runner_mode:'PD_STAND',runner_age_ms:50}))), '站立 · PD_STAND');
 assert.equal(liveRunnerLabel(parseLiveTelemetry(packet({runner_mode:'MOTION',runner_age_ms:1500}))), '状态未知');
 assert.equal(liveRunnerLabel(parseLiveTelemetry(packet())), '状态未知');
 assert.equal(liveRunnerLabel(undefined), '状态未知');
});
