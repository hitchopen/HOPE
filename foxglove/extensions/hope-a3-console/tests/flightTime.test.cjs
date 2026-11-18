const assert = require('node:assert/strict');
const test = require('node:test');
const {decodeFlightTime, flightTimeRequest, parseFlightTime, FLIGHT_TIME_SERVICE} = require('./loadSource.cjs')('flightTime');
const diag = values => ({status:[{name:'hope_planner_cpp/realtime',values:Object.entries(values).map(([key,value])=>({key,value}))}]});
const current = {flight_time_s:'.5', flight_time_fh_s:'.4', flight_time_bh_s:'.6', flight_time_max_s:'2', flight_time_configurable:'true'};
test('flight time request uses the dedicated double service and rejects nonpositive or nonfinite input', () => {
 assert.equal(FLIGHT_TIME_SERVICE, '/hope/planner/set_flight_time');
 for (const seconds of [.3,.5,.7]) {
  const req = flightTimeRequest(seconds);
  assert.equal(req.parameters.length,1);
  assert.equal(req.parameters[0].name,'flight_time_s');
  assert.equal(req.parameters[0].value.type,3);
  assert.equal(req.parameters[0].value.double_value,seconds);
  assert.equal(parseFlightTime(String(seconds)),seconds);
 }
 for (const bad of [0,-1,NaN,Infinity,-Infinity]) assert.throws(()=>flightTimeRequest(bad));
 for (const bad of ['', ' ', '0', '-.1', 'NaN', 'Infinity', 'abc']) assert.equal(parseFlightTime(bad),undefined);
});
test('current side settings, last requested duration and solver fallback prediction remain distinct', () => {
 const state=decodeFlightTime(diag({...current,flight_time_last_requested_s:'.7',flight_time_last_command_s:'.402',flight_time_persistent:'true'}));
 assert.equal(state.seconds,.5);assert.equal(state.forehand,.4);assert.equal(state.backhand,.6);
 assert.equal(state.lastRequested,.7);assert.equal(state.lastCommand,.402);
 assert.equal(state.maxSeconds,2);assert.equal(state.persistent,true);assert.equal(state.configurable,true);
});
test('old and malformed backend telemetry cannot enable flight-time controls', () => {
 for(const data of [null,undefined,{},diag({}),diag({flight_time_s:'.5'}),diag({...current,flight_time_s:'nan'}),diag({...current,flight_time_max_s:'0'})]) assert.equal(decodeFlightTime(data),undefined);
 const state=decodeFlightTime(diag({...current,flight_time_configurable:'false',flight_time_last_command_s:'nan'}));
 assert.equal(state.lastCommand,undefined);assert.equal(state.lastRequested,undefined);
 assert.equal(state.configurable,false);assert.equal(state.persistent,false);
});
