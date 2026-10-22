const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const output = {};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(path.join(__dirname, '../src/robotConnection.ts'), 'utf8'), {compilerOptions:{module:ts.ModuleKind.CommonJS}}).outputText, {exports:output,encodeURIComponent});
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
