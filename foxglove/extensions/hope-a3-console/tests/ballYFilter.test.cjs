const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const output = {};
const source = fs.readFileSync(path.join(__dirname, '../src/ballYFilter.ts'), 'utf8');
vm.runInNewContext(ts.transpileModule(source, {compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText, {exports:output});
const {decodeBallYFilter, parseBallYBounds, ballYFilterRequest} = output;
const diag = (values) => ({status:[{values:Object.entries(values).map(([key,value])=>({key,value}))}]});
test('accepts Planner state including disabled without confusing defaults with receipt', () => {
  for (const enabled of ['true','false']) {
    const state = decodeBallYFilter(diag({ball_y_filter_enabled:enabled,ball_y_filter_min_m:'-1.3895904353652275',ball_y_filter_max_m:'-0.5543593239924284',ball_y_filter_runtime_config:'true',ball_y_filter_persistent:'true'}));
    assert.equal(state.enabled, enabled==='true'); assert.equal(state.configurable,true); assert.equal(state.persistent,true);
  }
});
test('invalid, missing and reversed telemetry never becomes an enabled range', () => {
  for (const value of [null,undefined,{}, {status:7}, diag({}), diag({ball_y_filter_enabled:'true',ball_y_filter_min_m:'nan',ball_y_filter_max_m:'0'}),diag({ball_y_filter_enabled:'true',ball_y_filter_min_m:'1',ball_y_filter_max_m:'0'})]) assert.equal(decodeBallYFilter(value),undefined);
});
test('blank, nonfinite, equal and reversed manual bounds are rejected', () => {
  for(const [lo,hi] of [['','0'],['  ','0'],['-1',''],['NaN','0'],['-Infinity','0'],['-1','-1'],['0','-1']]) assert.equal(parseBallYBounds(lo,hi),undefined);
  assert.equal(parseBallYBounds('-1.2','-0.6').min,-1.2);
});
test('one atomic request contains exact typed enable and both editable limits', () => {
  const r=ballYFilterRequest({enabled:false,min:-1.2,max:-0.6});
  assert.equal(r.parameters.length,3);
  assert.equal(r.parameters[0].name,'enabled'); assert.equal(r.parameters[0].value.type,1); assert.equal(r.parameters[0].value.bool_value,false);
  assert.equal(r.parameters[1].name,'min_m'); assert.equal(r.parameters[1].value.type,3); assert.equal(r.parameters[1].value.double_value,-1.2);
  assert.equal(r.parameters[2].value.double_value,-0.6);
});
