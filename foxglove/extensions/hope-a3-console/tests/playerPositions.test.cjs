const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const api = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname,'../src/playerPositions.ts'),'utf8'),
  {compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText, {exports:api});
function pose(x, nanosec=0) {
  return {header:{frame_id:'hope_table_global',stamp:{sec:1,nanosec}},
    pose:{position:{x,y:0,z:1},orientation:{x:0,y:0,z:0,w:1}}};
}
test('strict physical ends, expiry and either player name govern assignment', () => {
  for (const [x,p1,p2] of [[-.01,true,false],[0,false,false],[2.74,false,false],[2.75,false,true]]) {
    const p=api.decodePlayerPosition(pose(x),100);
    assert.equal(api.playerSideAvailable('P1',[undefined,p],100),p1);
    assert.equal(api.playerSideAvailable('P2',[p],100),p2);
    assert.equal(api.playerSideAvailable('P1',[p],351),false);
    assert.equal(api.playerSideAvailable('P2',[p],351),false);
  }
});
test('replayed timestamps cannot renew a stationary pose; live 150/300 Hz samples can', () => {
  let p=api.decodePlayerPosition(pose(-.5),100);
  p=api.decodePlayerPosition(pose(-.5),400,p);
  assert.equal(api.playerSideAvailable('P1',[p],400),false);
  for (const hz of [150,300]) {
    p=undefined;
    for(let i=0;i<hz;i++) {
      const now=i*1000/hz;
      p=api.decodePlayerPosition(pose(-.5,Math.round(i*1e9/hz)),now,p);
      assert.equal(api.playerSideAvailable('P1',[p],now),true);
    }
  }
});
test('invalid frames, positions, orientation and timestamps fail closed', () => {
  for(const edit of [p=>p.header.frame_id='world',p=>p.pose.position.x=NaN,
    p=>p.pose.orientation.w=0,p=>p.header.stamp.sec=0]) {
    const p=pose(-.5); edit(p); assert.equal(api.decodePlayerPosition(p,100),undefined);
  }
});
