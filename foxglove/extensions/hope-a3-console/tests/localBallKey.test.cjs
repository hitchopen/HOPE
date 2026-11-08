const assert = require('node:assert/strict');
const test = require('node:test');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const ts = require('typescript');
function moduleWithStorage(storage) {
  const exports={};
  const source=fs.readFileSync(path.join(__dirname,'../src/ballPlayState.ts'),'utf8');
  vm.runInNewContext(ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2020}}).outputText,
    {exports,window:{localStorage:storage},setTimeout});
  return exports;
}
test('Desktop key persists only in local app storage and malformed imports do not overwrite it',()=>{
  const values=new Map();const storage={getItem:k=>values.get(k),setItem:(k,v)=>values.set(k,v)};
  const api=moduleWithStorage(storage),key='a'.repeat(64);
  assert.equal(api.readLocalControlKey(),'');
  assert.equal(api.saveLocalControlKey(key+'\n'),key);
  assert.equal(moduleWithStorage(storage).readLocalControlKey(),key);
  for(const bad of ['',key.slice(1),'x'.repeat(64),key+'extra'])assert.throws(()=>api.saveLocalControlKey(bad));
  assert.equal(api.readLocalControlKey(),key);
  assert.equal(values.size,1);
});
test('unavailable local storage is explicit on import, not silently saved to a shared layout',()=>{
  const api=moduleWithStorage({getItem:()=>{throw Error('denied');},setItem:()=>{throw Error('denied');}});
  assert.equal(api.readLocalControlKey(),'');
  assert.throws(()=>api.saveLocalControlKey('a'.repeat(64)),/denied/);
});
test('private key is sent only as an HTTP header on localhost status and intent requests',async()=>{
  const api=moduleWithStorage({getItem:()=>null}),key='a'.repeat(64);let revision='0',calls=0;
  await api.sendLocalBallIntent(async(url,options)=>{
    ++calls;assert.ok(url.startsWith('http://127.0.0.1:8768/ball-state'));
    assert.ok(!url.includes(key));assert.equal(options.headers['X-Hope-Local-Key'],key);
    if(options.method==='POST'){revision='1';return {status:202,json:async()=>({intent_revision:revision})};}
    return {ok:true,text:async()=>JSON.stringify({schema:2,session_id:'test',producer_instance_id:'p',intent_revision:revision,
      local_stopped:false,state:'SERVE',reason:'test',generation:'0',forwarding:false,history_known:false,serve_armed:true,
      striker_bounces:0,receiver_bounces:0})};
  },'SERVE',{aborted:false},key);
  assert.equal(calls,3);
});
