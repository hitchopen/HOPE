const assert = require('node:assert/strict');
const test = require('node:test');
const { KeyboardWalk } = require('./loadSource.cjs')('keyboardWalk');
const harness = require('./shortcutHarness.cjs');

function setup() {
  const frames = [], clicks = [];
  let target = {boot:7,fingerprint:9,standing:true};
  const walk = new KeyboardWalk({advertise(){}, publish(_, m){frames.push(Array.from(m.data));}}, () => target);
  const h = harness(action => ({matches:()=>false,click:()=>clicks.push(action)}),()=>false,undefined,()=>clicks.push('rigid'),walk);
  return {...h, frames, clicks, walk, setTarget:t=>{target=t;}};
}
test('six physical keypad directions and immediate release, including NumLock off', () => {
  const h=setup();
  for (const [key, axis] of Object.entries({8:[1,0,0],5:[-1,0,0],4:[0,1,0],6:[0,-1,0],7:[0,0,1],9:[0,0,-1]})) {
    const code='Numpad'+key; h.down(key,{code});
    assert.deepEqual(h.frames.at(-1).slice(6),[1,...axis]);h.up(code);
    assert.deepEqual(h.frames.at(-1).slice(6),[0,0,0,0]);
  }
  h.down('ArrowUp',{code:'Numpad8'});assert.deepEqual(h.frames.at(-1).slice(7),[1,0,0]);
  assert.deepEqual(h.clicks,[]);
});
test('initial I stands only; repeat, editing, mode/identity changes and focus loss cannot start a walk', () => {
  const h=setup();
  h.down('i');h.up('i');assert.deepEqual(h.clicks,['stand']);assert.deepEqual(h.frames,[]);
  h.setTarget(undefined); h.down('Numpad8');h.setTarget({boot:7,fingerprint:9,standing:true});
  h.down('Numpad8',{repeat:true});assert.deepEqual(h.frames,[]);h.up('Numpad8');
  h.down('Numpad4',{path:[{nodeType:1,isContentEditable:true,matches:()=>false}]});assert.deepEqual(h.frames,[]);
  h.down('Numpad8');h.down('Numpad4');h.down('Numpad6');assert.deepEqual(h.frames.at(-1).slice(6),[1,1,0,0]);
  h.up('Numpad6');assert.deepEqual(h.frames.at(-1).slice(6),[1,1,1,0]);
  h.reset('blur');assert.deepEqual(h.frames.at(-1).slice(6),[0,0,0,0]);h.down('Numpad8',{repeat:true});
  const n=h.frames.length;h.doc.timer();assert.equal(h.frames.length,n);
  h.up('Numpad8');h.down('Numpad8');h.setTarget({boot:8,fingerprint:10,standing:true});h.doc.timer();
  assert.deepEqual(h.frames.at(-1).slice(1,3),[7,9]);assert.equal(h.frames.at(-1)[6],0);
  h.setTarget({boot:8,fingerprint:10,standing:false});h.up('Numpad8');h.down('Numpad8');assert.equal(h.walk.active,false);
});
