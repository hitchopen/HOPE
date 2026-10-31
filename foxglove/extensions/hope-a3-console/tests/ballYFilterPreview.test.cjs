const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
function load(name) {
  const exports = {};
  const source = fs.readFileSync(path.join(__dirname, '../src', name + '.ts'), 'utf8');
  vm.runInNewContext(ts.transpileModule(source, {compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText,
    {exports, require: (name) => load(name.replace('./', ''))});
  return exports;
}
const {withBallYPreview, readBallYFilterDraft, ballYFilterDraftEdited, PREVIEW_LAYER} = load('ballYFilterPreview');
const draft = {enabled: true, min: '-1.2', max: '-0.6'};
const options = {kernelMode: false};
const xml = (config) => decodeURIComponent(config.layers[PREVIEW_LAYER].url.split(',')[1]);
test('offline preview preserves the actual saved 3D view and uses local geometry, not a ROS topic', () => {
  const original = {cameraState:{distance:4.5},followTf:'world',layers:{table:{visible:true}},topics:{'/hope/ball/marker':{visible:true}}};
  const before = JSON.stringify(original);
  const result = withBallYPreview(original, draft, options);
  assert.equal(JSON.stringify(original), before);
  assert.equal(result.cameraState, original.cameraState);
  assert.equal(result.layers.table, original.layers.table);
  assert.equal(result.topics['/hope/ball/marker'].visible, true);
  assert.equal(result.topics['/hope/planner/y_filter_markers'].visible, false);
  assert.equal(result.layers[PREVIEW_LAYER].visible, true);
  assert.match(result.layers[PREVIEW_LAYER].url, /^data:text\/xml/);
  assert.match(xml(result), /child link="hope_preview_left"\/><origin xyz="1.37 -0.6 0.025"/);
  assert.match(xml(result), /child link="hope_preview_right"\/><origin xyz="1.37 -1.2 0.025"/);
  assert.match(result.layers[PREVIEW_LAYER].label, /本地预览.*非生效回执/);
});
test('edit updates the same layer; uncheck, invalid values and kernel mode hide it', () => {
  const first = withBallYPreview({}, draft, options);
  const changed = withBallYPreview(first, {...draft, max:'-0.4'}, options);
  assert.equal(Object.keys(changed.layers).length,1);
  assert.notEqual(first.layers[PREVIEW_LAYER].url, changed.layers[PREVIEW_LAYER].url);
  for (const update of [{...draft, enabled:false},{...draft,min:''},{...draft,min:'0'},{...draft,max:'Infinity'}]) {
    assert.equal(withBallYPreview(first, update, options).layers[PREVIEW_LAYER].visible,false);
  }
  assert.equal(withBallYPreview(first,draft,{kernelMode:true}).layers[PREVIEW_LAYER].visible,false);
});
test('local drafts restore without being mistaken for Planner receipts; fill is clipped to table', () => {
  assert.equal(readBallYFilterDraft({ballYFilterDraft:draft}), draft);
  assert.equal(readBallYFilterDraft({}).enabled, false);
  assert.match(xml(withBallYPreview({}, {...draft,min:'-2',max:'1'},options)), /box size="2.74 1.525 0.004"/);
  assert.doesNotMatch(xml(withBallYPreview({}, {...draft,min:'1',max:'2'},options)), /hope_preview_lane/);
});
test('migrates the accidentally persisted original default to the disabled public preset', () => {
  const legacy = {ballYFilterDraft:{enabled:true,min:'-1.38959',max:'-0.554359'}};
  assert.equal(readBallYFilterDraft(legacy).min, '-1.525');
  assert.equal(readBallYFilterDraft({}).min, '-1.525');
  assert.equal(ballYFilterDraftEdited(legacy), false);
  assert.equal(ballYFilterDraftEdited({}), false);
});
test('applied/restored values can sync with fresh Planner receipts; user edits remain protected', () => {
  const applied = {ballYFilterDraft:{enabled:true,min:'-1.25',max:'-0.554359'},ballYFilterDraftEdited:false};
  assert.equal(readBallYFilterDraft(applied).min, '-1.25');
  assert.equal(readBallYFilterDraft(applied).enabled, false);
  assert.equal(readBallYFilterDraft({...applied,ballYFilterDraftEdited:true}).enabled, true);
  assert.equal(ballYFilterDraftEdited(applied), false);
  assert.equal(ballYFilterDraftEdited({...applied,ballYFilterDraftEdited:true}), true);
  const explicitOldValue = {ballYFilterDraft:{enabled:true,min:'-1.38959',max:'-0.554359'},ballYFilterDraftEdited:true};
  assert.equal(readBallYFilterDraft(explicitOldValue).min, '-1.38959');
  assert.equal(ballYFilterDraftEdited(explicitOldValue), true);
  assert.equal(ballYFilterDraftEdited({ballYFilterDraft:draft}), true);
});
