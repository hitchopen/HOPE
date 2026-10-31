const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');
const output = {};
Object.assign(output, require('./loadSource.cjs')('timeCalibration'));
const {timeCalibrationControls} = output;
const ready = {state:'IDLE',fresh:true,busy:false,ntpFresh:true,ntpPass:true,lifecycleStopped:true,lifecycleBusy:false,configConfirmed:true};
test('manual resync accepts passing and failing NTP, including a completed request', () => {
  for (const state of ['IDLE','COMPLETE','REJECTED']) for (const ntpPass of [true,false]) {
    const result=timeCalibrationControls({...ready,state,ntpPass});
    assert.equal(result.ready,true); assert.equal(result.blocksStart,false);
    if(ntpPass) assert.match(result.title,/NTP 正常/);
  }
});
test('missing or stale evidence and active lifecycle keep resync disabled', () => {
  for(const change of [{fresh:false},{ntpFresh:false},{ntpPass:undefined},{lifecycleStopped:false},{lifecycleBusy:true},{configConfirmed:false},{busy:true},{state:'NO DATA'}])
    assert.equal(timeCalibrationControls({...ready,...change}).ready,false,JSON.stringify(change));
});
test('maintenance and failed recovery cannot be bypassed by a passing NTP gate', () => {
  for(const state of ['RUNNING','INTERRUPTED','FAILED_SAFE_STOP']) {
    const result=timeCalibrationControls({...ready,state});
    assert.equal(result.ready,false); assert.equal(result.blocksStart,true);
  }
});
