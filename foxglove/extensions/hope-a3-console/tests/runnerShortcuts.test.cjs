const assert = require('node:assert/strict');
const test = require('node:test');
const shortcutHarness = require('./shortcutHarness.cjs');

function harness() {
  const calls = [];
  const disabled = new Set();
  let dialog = false;
  const h = shortcutHarness(action => ({
    matches: selector => selector === ':disabled' && disabled.has(action),
    click: () => calls.push(action),
  }), () => dialog);
  return {...h, calls, disabled, setDialog: value => {dialog=value;}};
}

test('I/R/S map to the existing buttons, with uppercase and no-code fallback', () => {
  const h = harness();
  for (const [key, options] of [['i', {}], ['R', {}], ['I', {code:''}], ['r', {code:''}], ['s', {}], ['S', {code:''}]]) {
    assert.equal(h.down(key, options).defaultPrevented, true); h.up(key);
  }
  assert.deepEqual(h.calls, ['stand','ready','stand','ready','serve','serve']);
  assert.equal(h.down('x').defaultPrevented, false);
});

test('Passive requires simultaneous arrows in either order, once until both release', () => {
  for (const [first, second] of [['ArrowLeft','ArrowRight'], ['ArrowRight','ArrowLeft']]) {
    const h = harness();
    h.down(first); h.up(first); h.down(second);
    assert.deepEqual(h.calls, []); h.up(second);
    h.down(first); h.down(second);
    h.down(second, {repeat:true}); h.down(first, {repeat:true});
    h.up(second); h.down(second);
    assert.deepEqual(h.calls, ['passive']);
    h.up(first); h.up(second);
    h.down(second); h.down(first);
    assert.deepEqual(h.calls, ['passive','passive']);
  }
});

test('held keys never repeat even without the browser repeat flag', () => {
  const h = harness();
  h.down('i'); h.down('i'); h.down('i', {repeat:true});
  assert.deepEqual(h.calls, ['stand']);
  h.up('i'); h.down('i');
  assert.deepEqual(h.calls, ['stand','stand']);
});

test('disabled buttons and robot-switch fieldsets never dispatch', () => {
  const h = harness();
  for (const action of ['stand','ready','serve','passive']) h.disabled.add(action);
  h.down('i'); h.down('s'); h.down('r'); h.down('ArrowLeft'); h.down('ArrowRight');
  h.disabled.clear();
  h.down('r', {repeat:true});
  assert.deepEqual(h.calls, []);
  h.up('r'); h.down('r');
  assert.deepEqual(h.calls, ['ready']);
});

test('editing, composition and modified shortcuts do not send commands', () => {
  for (const options of [
    {ctrlKey:true}, {metaKey:true}, {altKey:true}, {isComposing:true}, {keyCode:229},
    {path:[{nodeType:1,isContentEditable:true,matches:()=>false}]},
    ...['input','textarea','select',"[role='textbox']","[role='combobox']","[role='spinbutton']","[role='slider']"]
      .map(tag => ({path:[{nodeType:1,isContentEditable:false,matches:s=>s.split(', ').includes(tag)}]})),
  ]) {
    const h = harness();
    for (const key of ['i','s','r','ArrowLeft','ArrowRight']) assert.equal(h.down(key, options).defaultPrevented, false);
    assert.deepEqual(h.calls, []);
  }
});

test('modal dialogs and hidden documents suppress all shortcuts', () => {
  for (const mode of ['dialog','hidden']) {
    const h = harness();
    if (mode === 'dialog') h.setDialog(true); else h.doc.hidden=true;
    h.down('i'); h.down('s'); h.down('r'); h.down('ArrowLeft'); h.down('ArrowRight');
    assert.deepEqual(h.calls, []);
  }
});

test('focus loss and visibility changes clear partially held chords', () => {
  for (const kind of ['focusout','blur','visibilitychange']) {
    const h = harness();
    h.down('ArrowLeft'); h.reset(kind); h.down('ArrowRight');
    assert.deepEqual(h.calls, []);
    h.up('ArrowRight'); h.down('ArrowLeft'); h.down('ArrowRight');
    assert.deepEqual(h.calls, ['passive']);
  }
});

test('editing after the first arrow invalidates the chord', () => {
  const h = harness();
  h.down('ArrowLeft'); h.down('ArrowRight', {ctrlKey:true});
  h.down('ArrowRight'); assert.deepEqual(h.calls, []);
});

test('unmount removes key listeners and pending chord state', () => {
  const h = harness();
  h.down('ArrowLeft'); h.remove();
  h.down('ArrowRight'); h.down('i'); h.down('s'); h.down('r');
  assert.deepEqual(h.calls, []);
});

test('S dispatches only one enabled Serve stage and refuses ambiguous stages', () => {
  const calls = [];
  let enabled = [true, false];
  const h = shortcutHarness(() => enabled.map((allowed, index) => ({
    matches: () => !allowed, click: () => calls.push(index),
  })));
  h.down('s');
  enabled = [false, true];
  h.down('s', {repeat:true});
  assert.deepEqual(calls, [0]);
  h.up('s'); h.down('s');
  assert.deepEqual(calls, [0,1]);
  for (const stages of [[true,true], [false,false]]) {
    enabled = stages; h.up('s'); h.down('s');
  }
  assert.deepEqual(calls, [0,1]);
});

test('document-level keys dispatch after a button loses focus and from sibling panels', () => {
  const h = harness();
  h.reset('focusout');
  h.down('s', {path:[h.doc]}); h.up('s');
  h.down('i', {path:[{nodeType:1,isContentEditable:false,matches:()=>false},h.doc]});
  h.down('ArrowLeft'); h.down('ArrowRight');
  assert.deepEqual(h.calls,['serve','stand','passive']);
});

test('multiple Consoles route only to the selected visible panel', () => {
  const calls = [];
  const a = shortcutHarness(() => ({matches:()=>false,click:()=>calls.push('a')}));
  const b = shortcutHarness(() => ({matches:()=>false,click:()=>calls.push('b')}),()=>false,a.doc);
  a.down('s'); a.up('s'); assert.deepEqual(calls,[]);
  a.select(); a.down('s'); a.up('s'); assert.deepEqual(calls,['a']);
  b.select(); b.down('s'); b.up('s'); assert.deepEqual(calls,['a','b']);
  b.panel.getClientRects = () => [];
  a.down('s'); a.up('s'); assert.deepEqual(calls,['a','b','a']);
  a.panel.isConnected = false;
  a.down('s'); a.up('s'); assert.deepEqual(calls,['a','b','a']);
  a.remove(); b.remove();
});
