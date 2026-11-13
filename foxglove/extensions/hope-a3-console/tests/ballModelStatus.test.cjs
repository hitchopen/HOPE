const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const exportsObject = {};
Object.assign(exportsObject, require('./loadSource.cjs')('ballModelStatus'));
const { ballModelDisplay } = exportsObject;
const status = {state:'CONFIGURED', configured_model:'ppq004', age_s:1, endpoints:{
  hdu:{configured_model:'ppq004', runtime:'NOT_RUNNING'},
  laptop:{configured_model:'ppq004', runtime_model:'ppq004', runtime:'VERIFIED'},
}};
test('inherited PPQ004 shows the actual model and separate stopped/loaded endpoints', () => {
  const d=ballModelDisplay(status,true); assert.equal(d.value,'ppq004'); assert.equal(d.ready,true);
  assert.match(d.text,/Planner：配置 PPQ004，未运行/); assert.match(d.text,/球包器：配置 PPQ004，已加载 PPQ004/);
});
test('mismatched runtime is explicit and blocks admission even if saved config agrees', () => {
  const d=ballModelDisplay({...status,state:'MISMATCH',endpoints:{...status.endpoints,
    hdu:{configured_model:'ppq004',runtime_model:'ppq002',runtime:'MISMATCH'}}},true);
  assert.equal(d.ready,false); assert.match(d.text,/运行参数不匹配（PPQ002）/);
});
test('old, absent or disconnected status cannot masquerade as current selection', () => {
  for(const [s,f] of [[undefined,true],[status,false],[{...status,age_s:46},true],[{...status,age_s:null},true]]) {
    const d=ballModelDisplay(s,f); assert.equal(d.ready,false); assert.equal(d.value,'');
  }
});

const verifiedAt = 1793718000;
const cachedEndpoint = {configured_model:'ppq004', runtime_model:'ppq004', runtime:'CACHED_VERIFIED',
  verified_at_unix_s:verifiedAt, verification_age_s:15, verification_cached:true,
  verification_scope:'process_identity', verification_cache_ttl_s:60};
const cachedStatus = {...status, state:'CONFIGURED', age_s:5,
  endpoints:{hdu:cachedEndpoint, laptop:cachedEndpoint}};
test('cache displays original measurement age instead of resetting freshness with the outer status', () => {
  const d=ballModelDisplay(cachedStatus,true,undefined,(verifiedAt+20)*1000);
  assert.equal(d.ready,true);
  assert.match(d.text,/20 秒前（同一进程缓存）/);
  assert.match(d.text,/START 前将重新实时核验/);
  assert.doesNotMatch(d.text,/已加载|无法读取|未全部运行/);
  // A slow second endpoint must not hide time elapsed after the first receipt.
  assert.match(ballModelDisplay(cachedStatus,true,undefined,(verifiedAt+32)*1000).text,/32 秒前/);
});
test('cached result retains its original measurement time across new status receipts', () => {
  const later={...cachedStatus, age_s:0, endpoints:{hdu:{...cachedEndpoint,verification_age_s:30},
    laptop:{...cachedEndpoint,verification_age_s:30}}};
  const first=ballModelDisplay(cachedStatus,true,undefined,(verifiedAt+20)*1000);
  const second=ballModelDisplay(later,true,undefined,(verifiedAt+30)*1000);
  const time=new Date(verifiedAt*1000).toLocaleTimeString(undefined,{hour12:false});
  assert.ok(first.text.includes(time) && second.text.includes(time));
  assert.match(second.text,/30 秒前/);
});
test('malformed cached identity or time fails visibly and cannot enable START', () => {
  for(const bad of [{verification_age_s:undefined},{verification_age_s:NaN},{verification_age_s:-1},
    {verification_age_s:60},{verified_at_unix_s:undefined},{verified_at_unix_s:verifiedAt+50},
    {verification_scope:'live_parameters'},{verification_cached:false},{verification_cache_ttl_s:undefined}]) {
    const d=ballModelDisplay({...cachedStatus,endpoints:{hdu:{...cachedEndpoint,...bad},laptop:cachedEndpoint}},
      true,undefined,(verifiedAt+20)*1000);
    assert.equal(d.ready,false);assert.match(d.text,/缓存的核验时间或进程身份不可用/);
  }
});
test('cached status does not override mismatches or outer disconnection and stale status gates', () => {
  const mismatch={...cachedStatus,state:'MISMATCH',endpoints:{...cachedStatus.endpoints,
    laptop:{configured_model:'ppq004',runtime:'MISMATCH',runtime_model:'ppq002'}}};
  const d=ballModelDisplay(mismatch,true,undefined,(verifiedAt+20)*1000);
  assert.equal(d.ready,false);assert.match(d.text,/参数不一致/);
  for(const [s,f] of [[cachedStatus,false],[{...cachedStatus,age_s:46},true]])
    assert.equal(ballModelDisplay(s,f,undefined,(verifiedAt+20)*1000).ready,false);
});
test('English cache description distinguishes a past measurement from a fresh live query', () => {
  const {createTranslator}=require('./loadSource.cjs')('i18n');
  const d=ballModelDisplay(cachedStatus,true,createTranslator('en'),(verifiedAt+20)*1000);
  assert.match(d.text,/20s ago \(same-process cache\)/);
  assert.match(d.text,/START checks live parameters again/);
  assert.doesNotMatch(d.text,/Cannot read|not all running|[\u3400-\u9fff]/);
});
