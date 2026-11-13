const assert = require('node:assert/strict');
const test = require('node:test');
const loadSource = require('./loadSource.cjs');
const { configForSave, sameMocapBinding, CONFIG_KEYS, decodeConfigRuntime, configRuntimeReady, savedConfigObserved } = loadSource('configRuntime');
const { applyMessage, TOPICS, INITIAL_SNAPSHOT } = loadSource('HopeA3Console', '\nexport { applyMessage, TOPICS, INITIAL_SNAPSHOT };');
function report(overrides = {}) {
  return { schema_version: 1, source: '/var/lib/hope-lifecycle/config.json', revision: 54,
    config_sha256: 'a'.repeat(64), applied: true, state: 'APPLIED', reason: 'match',
    values: Object.fromEntries(CONFIG_KEYS.map(k => [k, k === 'hdu_wifi_ip' ? '192.168.8.182' : k === 'table_side' ? 'P1' : k.endsWith('_confirmed') ? 'false' : 'value'])),
    services: [{service:'coordinator',matches:true,expected_peers:'192.168.8.150',actual_peers:'192.168.8.150'}], ...overrides };
}
const decode = r => decodeConfigRuntime(JSON.stringify(r));
test('only a fresh complete applied report permits controls', () => {
  const status=decode(report());
  assert.equal(configRuntimeReady(status,100,101),true);
  for (const time of [-1,1600]) assert.equal(configRuntimeReady(status,100,time),false);
  assert.equal(configRuntimeReady(decode(report({applied:false})),100,101),false);
  assert.equal(decode(report({values:{table_side:'P1'}})),undefined);
  assert.equal(configRuntimeReady(decode(report({services:[]})),100,101),false);
  assert.equal(configRuntimeReady(decode(report({services:[{service:'coordinator',matches:false,expected_peers:'192.168.8.150',actual_peers:'192.168.8.194'}]})),100,101),false);
});
test('service success alone cannot acknowledge a new saved generation', () => {
  const request={revision:54,values:report().values};
  assert.equal(savedConfigObserved(decode(report()),request),false);
  assert.equal(savedConfigObserved(decode(report({revision:55})),request),true);
  assert.equal(savedConfigObserved(decode(report({revision:55,values:{...request.values,table_side:'P2'}})),request),false);
});
test('actual Console reducer keeps fields and revision in the same snapshot', () => {
  const state={...INITIAL_SNAPSHOT,lifecycleConfig:{},lastReceived:{}};
  const receive=(topic,data,at=100)=>applyMessage(state,{topic,message:{data}},at);
  receive(TOPICS.configRuntime,JSON.stringify(report()));
  receive(TOPICS.tableSide,'P2',110);
  receive(TOPICS.hduWifiIp,'192.168.8.196',110);
  receive(TOPICS.lifecycleConfigRevision,99,110);
  assert.equal(state.lifecycleConfig.table_side,'P1');
  assert.equal(state.lifecycleConfig.hdu_wifi_ip,'192.168.8.182');
  assert.equal(state.lifecycleConfigRevision,54);
  assert.equal(state.lastReceived[TOPICS.hduWifiIp],100);
  receive(TOPICS.configRuntime,'invalid',120);
  assert.equal(state.configRuntime,undefined);
  receive(TOPICS.tableSide,'P2',130);
  assert.equal(state.lifecycleConfig.table_side,'P1');
  assert.equal(configRuntimeReady(state.configRuntime,120,130),false);
});

test('changing the setup clears old attestations before sending and awaiting acknowledgement', () => {
  const saved = {...report().values, mocap_source:'vrpn', vrpn_pelvis_asset_confirmed:'true', vrpn_world_frame_confirmed:'true'};
  for (const key of ['hdu_wifi_ip','vrpn_ip','vrpn_port','vrpn_units','pants_profile','table_side','motive_ip','mocap_source']) {
    const draft = {...saved,[key]:'changed'};
    assert.equal(sameMocapBinding(draft,saved),false);
    const sent = configForSave(draft,saved);
    assert.equal(sent.vrpn_pelvis_asset_confirmed,'false');
    assert.equal(sent.vrpn_world_frame_confirmed,'false');
    assert.equal(savedConfigObserved(decode(report({revision:55,values:sent})),{revision:54,values:sent}),true);
  }
  assert.equal(configForSave({...saved,laptop_wifi_ip:'192.168.8.151'},saved).vrpn_world_frame_confirmed,'true');
  for (const value of [true,1,'yes','']) assert.equal(decode(report({values:{...saved,vrpn_world_frame_confirmed:value}})),undefined);
});
