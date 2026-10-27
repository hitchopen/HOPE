const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const api = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname,'../src/operatorHeartbeat.ts'),'utf8'),
  {compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2020}}).outputText,
  {exports:api, ArrayBuffer, DataView, TextEncoder});
test('UI echoes only matching live boot/session challenges with its own role', () => {
  const adverts=[], sent=[], closed=[];
  const client = new api.OperatorHeartbeat({advertise:(...args)=>adverts.push(args),
    publish:(topic,msg)=>sent.push({topic,msg}), unadvertise:topic=>closed.push(topic)});
  const challenge=[1,42,api.sessionFingerprint('test'),1];
  client.echo(challenge,43,'test'); client.echo(challenge,42,'wrong');
  client.echo([1,42,NaN,1],42,'test');
  assert.equal(sent.length,0);
  client.echo(new Float64Array(challenge),42,'test');
  assert.equal(adverts.length,1);
  assert.equal(adverts[0][0],api.OPERATOR_ECHO);
  assert.ok(adverts[0][2].datatypes.has('std_msgs/Float64MultiArray'));
  assert.deepEqual(Array.from(sent[0].msg.data),[...challenge,1]);
  assert.equal(sent[0].msg.layout.data_offset,0);
  client.echo(challenge,42,'test');
  assert.equal(sent.length,1); // never replay cached challenges on a timer
  client.echo([...challenge.slice(0,3),2],42,'test');
  assert.equal(sent.length,2);
  client.close(); assert.equal(closed[0],api.OPERATOR_ECHO);
});
test('unsupported or failed publishing cannot crash Console rendering', () => {
  const challenge=[1,42,api.sessionFingerprint('test'),1];
  for (const context of [{},{advertise:()=>{throw Error('disconnected');},publish:()=>{}}]) {
    const client=new api.OperatorHeartbeat(context);
    assert.doesNotThrow(()=>client.echo(challenge,42,'test'));
  }
});
test('a reconnect retries advertisement rather than permanently losing the heartbeat', () => {
  const challenge=[1,42,api.sessionFingerprint('test'),1];
  let attempts=0, disconnected=true, published=0;
  const client=new api.OperatorHeartbeat({advertise:()=>attempts++,publish:()=>{
    if(disconnected) throw Error('disconnected'); published++;
  }});
  client.echo(challenge,42,'test'); disconnected=false;
  client.echo(challenge,42,'test');
  assert.equal(attempts,2); assert.equal(published,1);
});
test('session identity matches the native/Python contract, including UTF-8', () => {
  assert.equal(api.sessionFingerprint('test'),2279734720943415);
  assert.equal(api.sessionFingerprint('机器人'),2963676621229307);
  assert.equal(api.sessionFingerprint('normal-session'),4029071744270237);
});
