const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const exportsObject = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname, '../src/ballModelStatus.ts'), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS },
}).outputText, { exports: exportsObject });
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
