const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const output = {};
Object.assign(output, require('./loadSource.cjs')('robotConnection'));
test('switch opens only new endpoint without mutating either robot', () => {
 const oldRobot={ip:'192.168.8.182',writes:[],commands:[]};
 const newRobot={ip:'192.168.8.183',writes:[],commands:[]};
 let saved,opened;
 output.switchRobot(newRobot.ip, ip => {saved=ip}, url => {opened=new URL(url)});
 assert.equal(saved,newRobot.ip);
 assert.equal(opened.searchParams.get('ds.url'),'ws://192.168.8.183:8766');
 assert.equal(oldRobot.writes.length+oldRobot.commands.length+newRobot.writes.length+newRobot.commands.length,0);
});
test('bad addresses never persist or open a connection', () => {
 for(const ip of ['','192.168.8.256','192.168.008.2','127.0.0.1','1.2.3.4','192.168.1.2&x=1']) {
  assert.throws(()=>output.switchRobot(ip,()=>assert.fail('saved'),()=>assert.fail('opened')));
 }
});
test('reopened old layout follows the fresh new robot receipt', () => {
 const state=output.reconcileRobotTarget('192.168.8.182',false,'','192.168.200.202');
 assert.equal(state.target,'192.168.200.202');assert.equal(state.pending,'');
});
test('an unconfigured robot never confirms a saved target', () => {
 const state=output.reconcileRobotTarget('192.168.8.182',false,'','');
 assert.equal(state.pending,'');
});
test('editing a target does not change current robot identity or complete a switch', () => {
 const draft=output.reconcileRobotTarget('192.168.200.203',true,'','192.168.200.202');
 assert.equal(draft.target,'192.168.200.203');assert.equal(draft.edited,true);
 const waiting=output.reconcileRobotTarget('192.168.200.203',true,'192.168.200.203','192.168.200.202');
 assert.equal(waiting.pending,'192.168.200.203');
 const confirmed=output.reconcileRobotTarget(waiting.target,waiting.edited,waiting.pending,'192.168.200.203');
 assert.equal(confirmed.pending,'');assert.equal(confirmed.target,'192.168.200.203');assert.equal(confirmed.edited,false);
});

test('saved old address cannot replace the observed control endpoint', () => {
  const identity = output.decodeRobotIdentity({machine_id:'a'.repeat(32),boot_id:'b'.repeat(36),
    connection:{hdu_ip:'192.168.200.202',laptop_ip:'192.168.100.89'}});
  assert.equal(output.reconcileRobotTarget('192.168.8.196',false,'',identity.connection.hdu_ip).target,'192.168.200.202');
  assert.equal(output.decodeRobotIdentity({machine_id:'bad',boot_id:'b'.repeat(36)}),undefined);
  const ambiguous = output.decodeRobotIdentity({machine_id:'a'.repeat(32),boot_id:'b'.repeat(36),connection:null});
  assert.equal(ambiguous.connection,undefined);
});

test('repeated confirmation of current robot never opens another data source', () => {
 let confirms=0;
 for(let i=0;i<3;i++) output.confirmRobotConnection('192.168.200.202','192.168.200.202',true,
  ip=>{assert.equal(ip,'192.168.200.202');confirms++},()=>assert.fail('switch'),()=>assert.fail('opened'));
 assert.equal(confirms,3);
});
test('missing or stale identity cannot redirect confirmation to a direct window', () => {
 for(const [host,fresh] of [[undefined,true],['192.168.200.202',false]]) {
  assert.throws(()=>output.confirmRobotConnection('192.168.200.202',host,fresh,
   ()=>assert.fail('confirmed'),()=>assert.fail('switch'),()=>assert.fail('opened')));
 }
});
test('explicit different target still opens only that robot without confirming old connection', () => {
 let saved,opened;
 output.confirmRobotConnection('192.168.200.203','192.168.200.202',true,
  ()=>assert.fail('confirmed wrong robot'),ip=>{saved=ip},url=>{opened=new URL(url)});
 assert.equal(saved,'192.168.200.203');
 assert.equal(opened.searchParams.get('ds.url'),'ws://192.168.200.203:8766');
});
