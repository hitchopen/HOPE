const assert = require('node:assert/strict');
const test = require('node:test');
const {ballStatusDisplay,freshValue,poseDetails,mocapStatusDisplay} = require('./loadSource.cjs')('liveStatus');
const state={state:'SERVE',reason:'waiting_for_ball',serve_armed:true,forwarding:false,local_stopped:false};
test('waiting for first visible serve is distinct from a lost tracked ball',()=>{
 assert.equal(ballStatusDisplay(state).title,'等待发球可见');
 for(const name of ['SERVE','RALLY','OPPONENT_SERVE','OPPONENT_RALLY']) {
  const visible={...state,state:name,serve_armed:false,reason:'qualified_launch'};
  const missing=ballStatusDisplay({...visible,observation_missing:true,reason:'ball_observation_missing'});
  assert.equal(missing.title,ballStatusDisplay(visible).title);
  assert.match(missing.detail,/状态保留/);
 }
 assert.equal(ballStatusDisplay({...state,local_stopped:true}).title,'已停止接收');
 assert.equal(ballStatusDisplay(undefined).tone,'unavailable');
});
test('old or future telemetry never appears current',()=>{
 assert.equal(freshValue({value:true,receivedAt:100},1099),true);
 assert.equal(freshValue({value:true,receivedAt:100},1100),undefined);
 assert.equal(freshValue({value:true,receivedAt:100},99),undefined);
});
test('pose detail accepts Foxglove and ROS timestamps and rejects stale or invalid geometry',()=>{
 const m={header:{stamp:{sec:10,nsec:1000000}},pose:{position:{x:1,y:2,z:3},orientation:{x:0,y:0,z:0,w:1}}};
 assert.match(poseDetails(m,10010),/1\.000  2\.000  3\.000/);
 assert.equal(poseDetails(m,12000),undefined);
 m.header.stamp={sec:10,nanosec:1000000};assert.ok(poseDetails(m,10010));
 m.pose.position.x=NaN;assert.equal(poseDetails(m,10010),undefined);
});

test('mocap card shows measured position without a running Runner',()=>{
 const pose={value:{header:{frame_id:'world',stamp:{sec:10,nsec:0}},
   pose:{position:{x:3.31,y:-1,z:.6},orientation:{x:0,y:0,z:0,w:1}}},receivedAt:10000};
 const tf={value:true,receivedAt:10000};
 const live=mocapStatusDisplay(pose,tf,10050);
 assert.equal(live.tone,'good');
 assert.match(live.detail,/3\.310/);
 assert.match(live.detail,/姿态/);
 assert.notEqual(mocapStatusDisplay(pose,{...tf,value:false},10050).tone,'good');
 assert.notEqual(mocapStatusDisplay(undefined,tf,10050).tone,'good');
 assert.notEqual(mocapStatusDisplay(pose,undefined,10050).tone,'good');
 assert.notEqual(mocapStatusDisplay(pose,tf,11001).tone,'good');
 assert.notEqual(mocapStatusDisplay({...pose,receivedAt:11001}, {...tf,receivedAt:11001},11001).tone,'good');
 assert.notEqual(mocapStatusDisplay({...pose,value:{...pose.value,header:{...pose.value.header,frame_id:'wrong'}}},tf,10050).tone,'good');
});
