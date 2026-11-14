const assert = require('node:assert/strict');
const test = require('node:test');
const { decodeBallPlayState, currentBallPlayState } = require('./loadSource.cjs')('ballPlayState');
const status = {schema:2, producer_instance_id:'producer', intent_revision:'0', local_stopped:false, session_id:'test', state:'OPPONENT_RALLY', reason:'qualified_launch',
  generation:'9007199254740993', forwarding:true, history_known:true, serve_armed:false,
  striker_bounces:0, receiver_bounces:0};
test('five states, conservative forwarding and integer identity', () => {
  for (const state of ['INVALID','RALLY','SERVE','OPPONENT_RALLY','OPPONENT_SERVE']) {
    const p = {...status,state,forwarding:state.startsWith('OPPONENT_')};
    assert.deepEqual(JSON.parse(JSON.stringify(decodeBallPlayState(JSON.stringify(p)))),p);
  }
  for (const p of [{...status,state:'INVALID'}, {...status,observation_missing:true},
    {...status,serve_armed:true}, {...status,striker_bounces:-1}, {...status,schema:1}]) {
    assert.equal(decodeBallPlayState(JSON.stringify(p)),undefined);
  }
  assert.equal(decodeBallPlayState('not JSON'),undefined);
});
test('local status freshness is independent of robot session or mode', () => {
  assert.equal(currentBallPlayState(status,100,500),status);
  assert.equal(currentBallPlayState({...status,session_id:'unrelated'},100,500).state,status.state);
  for (const args of [[100,1100],[600,500],[undefined,500]])
    assert.equal(currentBallPlayState(status,...args),undefined);
  const stopped = {...status,state:'INVALID',forwarding:false,local_stopped:true};
  assert.equal(decodeBallPlayState(JSON.stringify(stopped)).local_stopped,true);
  for(const patch of [{forwarding:true},{serve_armed:true},{receive_armed:true},{state:'RALLY'}])
    assert.equal(decodeBallPlayState(JSON.stringify({...stopped,...patch})),undefined);
});
test('reads the classifier phase or fault alongside the five-state label', () => {
  const fault = {...status,state:'INVALID',forwarding:false,substate:'Double bounce'};
  assert.equal(decodeBallPlayState(JSON.stringify(fault)).substate,'Double bounce');
  assert.equal(decodeBallPlayState(JSON.stringify({...fault,substate:42})),undefined);
});

test('local receiver Ready opens sample gate without fabricated history', () => {
  const ready = {...status, state:'OPPONENT_SERVE', history_known:false, receive_armed:true};
  assert.equal(decodeBallPlayState(JSON.stringify(ready)).receive_armed,true);
  for(const patch of [{state:'RALLY'}, {state:'OPPONENT_RALLY'}, {serve_armed:true}, {receive_armed:'true'}])
    assert.equal(decodeBallPlayState(JSON.stringify({...ready,...patch})),undefined);
});

test('display reads only localhost, never the robot bridge, and rejects unavailable snapshots', async () => {
  const { readLocalBallPlayState, LOCAL_BALL_STATUS_URL } = require('./loadSource.cjs')('ballPlayState');
  assert.equal(LOCAL_BALL_STATUS_URL, 'http://127.0.0.1:8768/ball-state');
  const signal = {};
  const accepted = await readLocalBallPlayState(async (url, options) => {
    assert.equal(url, LOCAL_BALL_STATUS_URL);
    assert.equal(options.method, 'GET'); assert.equal(options.cache, 'no-store');
    assert.equal(options.credentials, 'omit'); assert.equal(options.signal, signal);
    return {ok:true, text:async () => JSON.stringify(status)};
  }, signal);
  assert.equal(accepted.state, 'OPPONENT_RALLY');
  for (const response of [{ok:false}, {ok:true,text:async ()=>'bad'},
    {ok:true,text:async ()=>'x'.repeat(4097)}])
    assert.equal(await readLocalBallPlayState(async () => response, signal), undefined);
  await assert.rejects(readLocalBallPlayState(async () => {throw new Error('offline');},signal), /offline/);
});

test('observed incoming return can forward while pre-net bounce history stays unknown', () => {
  const value = {...status, history_known:false, return_confirmed:true};
  assert.equal(decodeBallPlayState(JSON.stringify(value)).return_confirmed,true);
  for(const patch of [{state:'RALLY'}, {state:'SERVE'}, {state:'OPPONENT_SERVE'},
    {state:'INVALID'}, {return_confirmed:'true'}])
    assert.equal(decodeBallPlayState(JSON.stringify({...value,...patch})),undefined);
});

test('forecast advisory is display-only and cannot change forwarding', () => {
  const value = {...status, forecast_advisory:'predicted_net_collision'};
  const decoded = decodeBallPlayState(JSON.stringify(value));
  assert.equal(decoded.state, 'OPPONENT_RALLY');
  assert.equal(decoded.forwarding, true);
  assert.equal(decoded.reason, status.reason);
  assert.equal(decoded.forecast_advisory, 'predicted_net_collision');
  assert.equal(decodeBallPlayState(JSON.stringify({...value, forecast_advisory:42})), undefined);
});

test('only explicit local buttons initialize the classifier', () => {
  const { localIntentForAction } = require('./loadSource.cjs')('ballPlayState');
  for (const key of ['prepareServe','readyToServe']) assert.equal(localIntentForAction(key),'SERVE');
  for (const key of ['stand','killAllAndCollect','passive','estop']) assert.equal(localIntentForAction(key),'STOP');
  assert.equal(localIntentForAction('ready'),'READY');
  for (const key of ['FAULT','ABORTED','PD_STAND','MOTION','STOPPED','RUNNING','COMPLETE','setReceiver'])
    assert.equal(localIntentForAction(key),undefined);
});

test('local intent waits for local application, never robot acknowledgement', async () => {
  const { sendLocalBallIntent, LOCAL_BALL_STATUS_URL } = require('./loadSource.cjs')('ballPlayState');
  let calls=0;
  await sendLocalBallIntent(async (url, options) => {
    ++calls;
    if(calls===2) {
      assert.equal(url,LOCAL_BALL_STATUS_URL+'/intent/READY');
      assert.equal(options.method,'POST'); assert.equal(options.credentials,'omit');
      assert.equal(options.headers['X-Hope-Producer'],'producer');
      assert.equal(options.headers['X-Hope-Intent-Revision'],'0');
      assert.equal(options.body,undefined);
      return {status:202,json:async()=>({intent_revision:'1'})};
    }
    assert.equal(url,LOCAL_BALL_STATUS_URL); assert.equal(options.method,'GET');
    return {ok:true,text:async()=>JSON.stringify({...status,intent_revision:calls>=4?'1':'0'})};
  }, 'READY', {aborted:false});
  assert.equal(calls,4);
});

test('stale producer or competing local intent is rejected without retry', async () => {
  const { sendLocalBallIntent } = require('./loadSource.cjs')('ballPlayState');
  let calls=0;
  await assert.rejects(sendLocalBallIntent(async () => {
    ++calls; return calls===1?{ok:true,text:async()=>JSON.stringify(status)}:{status:409};
  },'READY',{aborted:false}),/LOCAL_BALL_INTENT_REJECTED/);
  assert.equal(calls,2);
  await assert.rejects(sendLocalBallIntent(async()=>({ok:false}),'STOP',{aborted:false}),/LOCAL_BALL_UNAVAILABLE/);
});

test('Console sends local intent independently and has no robot-conditioned ball display', () => {
  const fs=require('node:fs'), path=require('node:path');
  const source=fs.readFileSync(path.join(__dirname,'../src/HopeA3Console.tsx'),'utf8');
  const invoke=source.slice(source.indexOf('const invoke ='),source.indexOf('const fieldCommand ='));
  assert.ok(invoke.indexOf('applyLocalButton(key)')<invoke.indexOf('context.callService == undefined'));
  assert.equal((invoke.match(/applyLocalButton\(key\)/g)||[]).length,1);
  assert.ok(!source.includes('currentBallPlayState(localBall?.state, snapshot.lifecycleSession'));
  assert.ok(!source.includes('ballServeRequested'));
});

test('missing prehistory is accepted; absent observations cannot forward',()=>{
 assert.ok(decodeBallPlayState(JSON.stringify({...status,history_known:false})));
 assert.ok(decodeBallPlayState(JSON.stringify({...status,observation_missing:true,forwarding:false})));
 assert.equal(decodeBallPlayState(JSON.stringify({...status,observation_missing:'false'})),undefined);
});
