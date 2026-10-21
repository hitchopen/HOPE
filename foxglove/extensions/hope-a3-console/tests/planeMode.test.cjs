const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const output = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname, '../src/planeMode.ts'), 'utf8'), {compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText, {exports:output});
const {decodePlaneMode, planeModeRequest} = output;
const diag = (values) => ({status:[{name:'hope_planner_cpp/realtime',values:Object.entries(values).map(([key,value])=>({key,value}))}]});
test('keeps requested mode distinct from the last published flight', () => {
 const state=decodePlaneMode(diag({plane_mode_requested:'per_ball',plane_mode_last_command:'fixed',plane_last_flight:'9',plane_reference_x:'-0.5',plane_world_x:'0.08',plane_mode_configurable:'true',plane_runtime_supported:'true',session_home_plane_x:'0.58'}));
 assert.equal(state.requested,'per_ball'); assert.equal(state.lastCommand,'fixed');
 assert.equal(state.flight,9); assert.equal(state.planeX,.08); assert.equal(state.supported,true);
});
test('missing telemetry never invents an applied plane', () => {
 for(const value of [undefined,null,{},diag({}),diag({plane_mode_requested:'continuous'})]) assert.equal(decodePlaneMode(value),undefined);
 const state=decodePlaneMode(diag({plane_mode_requested:'stable_per_ball',plane_mode_last_command:'none',plane_reference_x:'nan'}));
 assert.equal(state.lastCommand,undefined);assert.equal(state.referenceX,undefined);assert.equal(state.supported,false);
});
test('all three choices send exactly one atomic string parameter', () => {
 for(const mode of ['fixed','per_ball','stable_per_ball']) {
  const req=planeModeRequest(mode);assert.equal(req.parameters.length,1);
  assert.equal(req.parameters[0].name,'mode');assert.equal(req.parameters[0].value.type,4);
  assert.equal(req.parameters[0].value.string_value,mode);
 }
 assert.throws(()=>planeModeRequest('continuous'));
});
