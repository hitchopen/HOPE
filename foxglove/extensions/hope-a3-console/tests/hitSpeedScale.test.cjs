const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const output = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname, '../src/hitSpeedScale.ts'), 'utf8'), {compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText, {exports:output});
const {decodeHitSpeedScale, hitSpeedScaleRequest, parseHitSpeedScale} = output;
const diag = (values) => ({status:[{name:'hope_planner_cpp/realtime',values:Object.entries(values).map(([key,value])=>({key,value}))}]});
test('valid input sends exactly one double scale', () => {
 for(const scale of [0,.5,1,1.4,1e-20]) {
  const req=hitSpeedScaleRequest(scale);assert.equal(req.parameters.length,1);
  assert.equal(req.parameters[0].name,'scale');assert.equal(req.parameters[0].value.type,3);
  assert.equal(req.parameters[0].value.double_value,scale);
  assert.equal(parseHitSpeedScale(String(scale)),scale);
 }
 for(const value of [-1,NaN,Infinity,-Infinity]) assert.throws(()=>hitSpeedScaleRequest(value));
 for(const value of ['',' ','NaN','Infinity','-0.1','abc']) assert.equal(parseHitSpeedScale(value),undefined);
});
test('requested scale and last published scale remain distinct', () => {
 const state=decodeHitSpeedScale(diag({hit_speed_scale:'.8',hit_speed_scale_last_command:'1',hit_speed_scale_configurable:'true'}));
 assert.equal(state.scale,.8);assert.equal(state.lastCommand,1);assert.equal(state.configurable,true);
});
test('missing or invalid telemetry never claims the default is applied', () => {
 for(const value of [undefined,null,{},diag({}),diag({hit_speed_scale:'nan'}),diag({hit_speed_scale:'-1'})]) assert.equal(decodeHitSpeedScale(value),undefined);
 const state=decodeHitSpeedScale(diag({hit_speed_scale:'1',hit_speed_scale_last_command:'nan'}));
 assert.equal(state.lastCommand,undefined);assert.equal(state.configurable,false);
});
