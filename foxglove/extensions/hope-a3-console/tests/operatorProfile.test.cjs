const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');

// Exercise the actual panel and callbacks without a browser, ROS connection,
// robot, or exported test-only production API. JSX remains a React element tree.
function panelHarness(profile = 'linux', lifecycleState = 'STOPPED', lifecycleBusy = false, field, fieldAgeMs = 0, players = [], extraSnapshot = {}, initialState = {}) {
  let now = Date.now();
  const state = [];
  let cursor = 0;
  let refCursor = 0;
  const refs = [];
  const savedStates = [];
  const published = [];
  let layoutMounted = false;
  const modules = new Map();
  let api;
  const calls = [];
  const localRequests = [];
  let localRevision = 0;
  let localOffline = false, robotAccepted = true;
  const localFetch = async (url, options) => {
    if(localOffline) throw new Error('local offline');
    if(options.method === 'POST') {
      localRequests.push(url.split('/').pop()); ++localRevision;
      return { status:202, json:async()=>({intent_revision:String(localRevision)}) };
    }
    return {ok:true,text:async()=>JSON.stringify({schema:2,session_id:'unrelated-local-session',
      producer_instance_id:'local-process',intent_revision:String(localRevision),local_stopped:false,
      state:'INVALID',reason:'test',generation:'0',forwarding:false,history_known:false,
      serve_armed:false,striker_bounces:0,receiver_bounces:0})};
  };
  const subscribed = [];
  const config = {
    laptop_wifi_ip: '192.0.2.10', hdu_wifi_ip: '192.0.2.20',
    mdu_internal_ip: '192.0.2.30', motive_ip: '', table_side: 'P1',
    operator_profile: profile, gripper_profile: 'legacy',
    pants_profile: 'v2', mocap_source: 'none', vrpn_ip: '', vrpn_port: '3883', vrpn_units: 'mm',
  };
  const react = {
    ...require('react'),
    useState(initial) {
      const index = cursor++;
      if (!(index in state)) {
        state[index] = initial === api.EMPTY_CONFIG ? { ...config }
          : initial === api.INITIAL_SNAPSHOT ? {
            ...initial, lifecycleState, lifecycleBusy,
            playerP1: players[0], playerP2: players[1],
            field: field ? { recording: { state: 'OFF', output: '', error: '' }, ...field } : undefined,
            lifecycleConfig: { ...config },
            lastReceived: { [api.TOPICS.lifecycleState]: Date.now(),
              [api.TOPICS.fieldStatus]: field ? Date.now() - fieldAgeMs : undefined },
            ...extraSnapshot,
          } : initial;
      }
      return [state[index], (value) => {
        state[index] = typeof value === 'function' ? value(state[index]) : value;
      }];
    },
    useRef: (current) => refs[refCursor++] ?? (refs[refCursor - 1] = { current: current === api.INITIAL_SNAPSHOT ? { ...current, lifecycleState, lifecycleBusy, lifecycleConfig: { ...config }, ...extraSnapshot } : current }),
    useMemo: (callback) => callback(),
    useCallback: (callback) => callback,
    useEffect: () => {},
    useLayoutEffect: (callback) => { if (!layoutMounted) { layoutMounted = true; callback(); } },
  };
  function load(source, extra = '') {
    const exports = {};
    const code = ts.transpileModule(source + extra, {
      compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020, jsx: ts.JsxEmit.ReactJSX },
    }).outputText;
    vm.runInNewContext(code, {
      exports, ArrayBuffer, DataView, TextEncoder, AbortController, setTimeout, clearTimeout,
      Date: class extends Date { static now() { return now; } },
      window: { fetch:localFetch, setTimeout: () => 0, clearTimeout:()=>{}, confirm: () => { throw new Error('blocking confirmation interrupts heartbeat'); } },
      require(name) {
        if (name.endsWith('.css')) return {};
        if (name === 'react') return react;
        if (name.startsWith('./')) {
          const sourcePath = ['.ts', '.tsx'].map((extension) =>
            path.join(__dirname, '../src', name + extension)).find(fs.existsSync);
          if (sourcePath) {
            if (!modules.has(sourcePath)) modules.set(sourcePath, load(fs.readFileSync(sourcePath, 'utf8')));
            return modules.get(sourcePath);
          }
        }
        return require(name);
      },
    });
    return exports;
  }
  api = load(fs.readFileSync(path.join(__dirname, '../src/HopeA3Console.tsx'), 'utf8'),
    '\nexport { HopeA3Console, TOPICS, CONFIG_FIELDS, EMPTY_CONFIG, INITIAL_SNAPSHOT, applyMessage, confirmationBlocked };');
  const context = {
    initialState,
    saveState(value) { savedStates.push(value); },
    watch() {},
    subscribe(topics) { subscribed.push(...topics); },
    advertise() {},
    publish(topic, message) { published.push({ topic, message }); },
    async callService(service, payload) {
      calls.push({ service, payload });
      return payload.parameters ? { results: payload.parameters.map(() => ({ successful: true })) } : { success: robotAccepted, message: robotAccepted ? 'ok' : 'rejected' };
    },
  };
  return {
    api, calls, subscribed, savedStates, published, localRequests,
    setLocalOffline(value) { localOffline=value; },
    setRobotAccepted(value) { robotAccepted=value; },
    advance(ms) { now += ms; },
    frame(events, didSeek = false) { context.onRender({ currentFrame: events, didSeek }, () => {}); },
    render() { cursor = 0; refCursor = 0; return api.HopeA3Console({ context }); },
  };
}

function elements(node) {
  if (Array.isArray(node)) return node.flatMap(elements);
  if (node == null || typeof node !== 'object') return [];
  return [node, ...elements(node.props?.children)];
}
// 2.0 rows (policy, table side) carry the 1.9 button label as their accessible name.
const button = (tree, name) => elements(tree).find((node) =>
  node.type === 'button' && (node.props.children === name || node.props['aria-label'] === name));

test('operator profile is subscribed and decoded from the supervisor', () => {
  const harness = panelHarness();
  harness.render();
  const topic = '/hope/lifecycle/config/operator_profile';
  assert.ok(harness.subscribed.some((subscription) => subscription.topic === topic));
  for (const profile of ['linux', 'mac']) {
    const snapshot = { lifecycleConfig: {}, lastReceived: {} };
    harness.api.applyMessage(snapshot, { topic, message: { data: profile } }, 123);
    assert.equal(snapshot.lifecycleConfig.operator_profile, profile);
    assert.equal(snapshot.lastReceived[topic], 123);
  }
});

test('Linux/Mac selection submits the selected profile with the current addresses', async () => {
  const harness = panelHarness();
  let tree = harness.render();
  assert.equal(button(tree, 'Linux').props['aria-pressed'], true);
  button(tree, 'Mac').props.onClick();
  tree = harness.render();
  assert.equal(button(tree, 'Mac').props['aria-pressed'], true);
  assert.equal(button(tree, '确认配置').props.disabled, false);
  button(tree, '确认配置').props.onClick();
  await Promise.resolve();
  assert.equal(harness.calls.length, 1);
  assert.equal(harness.calls[0].service, '/hope/lifecycle/apply_config');
  const fields = Object.fromEntries(harness.calls[0].payload.parameters.map((parameter) =>
    [parameter.name, parameter.value.string_value]));
  assert.equal(Object.keys(fields).length, Object.keys(harness.api.CONFIG_FIELDS).length);
  assert.equal(fields.gripper_profile, 'legacy');
  assert.equal(fields.operator_profile, 'mac');
  assert.equal(fields.laptop_wifi_ip, '192.0.2.10');
});

test('missing or unknown profile cannot be confirmed; active or busy sessions lock selection', () => {
  for (const profile of ['', 'windows', 'MAC']) {
    const tree = panelHarness(profile).render();
    assert.equal(button(tree, '确认配置').props.disabled, true);
  }
  for (const [status, busy] of [['RUNNING', false], ['STOPPED', true], ['STARTING', true]]) {
    const tree = panelHarness('linux', status, busy).render();
    assert.equal(button(tree, 'Linux').props.disabled, true);
    assert.equal(button(tree, 'Mac').props.disabled, true);
    assert.equal(button(tree, '确认配置').props.disabled, true);
  }
});

test('pants asset buttons ignore table position and lock until STOP', () => {
  for (const [p1, p2] of [[true, false], [false, true], [false, false], [true, true]]) {
    const field = { mode: 'OPTITRACK' };
    const players = [{x:p1 ? -.5 : 1, receivedAt:Date.now(), sec:1, nanosec:0},
      {x:p2 ? 3 : 1, receivedAt:Date.now(), sec:1, nanosec:0}];
    let tree = panelHarness('linux', 'STOPPED', false, field, 0, players).render();
    assert.equal(button(tree, 'P1 裤子 · UCB_P1').props.disabled, false);
    assert.equal(button(tree, 'P2 裤子 · UCB_P2').props.disabled, false);
    for (const state of ['STARTING', 'RUNNING', 'FAILED', 'KILLING']) {
      tree = panelHarness('linux', state, false, field, 0, players).render();
      assert.equal(button(tree, 'P1 裤子 · UCB_P1').props.disabled, true);
      assert.equal(button(tree, 'P2 裤子 · UCB_P2').props.disabled, true);
    }
  }
});

test('pants remain selectable without tracking while stopped, including Kernel', () => {
  for (const players of [[], [{x:-.5, receivedAt:Date.now()-251}, {x:3, receivedAt:Date.now()-251}]]) {
    const tree = panelHarness('linux', 'STOPPED', false, undefined, 0, players).render();
    assert.equal(button(tree, 'P1 裤子 · UCB_P1').props.disabled, false);
    assert.equal(button(tree, 'P2 裤子 · UCB_P2').props.disabled, false);
  }
  const tree = panelHarness('linux', 'STOPPED', false, {mode:'KERNEL'}).render();
  assert.equal(button(tree, 'P1 裤子 · UCB_P1').props.disabled, false);
  assert.equal(button(tree, 'P2 裤子 · UCB_P2').props.disabled, false);
});

test('panel subscribes to raw unrotated player topics and decodes them directly', () => {
  const harness = panelHarness(); harness.render();
  assert.ok(harness.subscribed.some(s => s.topic === harness.api.TOPICS.playerP1));
  assert.ok(harness.subscribed.some(s => s.topic === harness.api.TOPICS.playerP2));
  const snapshot = { lifecycleConfig:{}, lastReceived:{} };
  const message = {header:{frame_id:'hope_table_global', stamp:{sec:1,nanosec:0}},
    pose:{position:{x:3,y:0,z:1},orientation:{x:0,y:0,z:0,w:1}}};
  harness.api.applyMessage(snapshot, {topic:harness.api.TOPICS.playerP2,message}, 100);
  assert.equal(snapshot.playerP2.x, 3);
  harness.api.applyMessage(snapshot, {topic:harness.api.TOPICS.playerP2,message}, 400);
  assert.equal(snapshot.playerP2.receivedAt, 100);
});

test('PASSIVE START and pants selection permit absent position evidence', () => {
  for (const verified of [true, false, undefined]) {
    for (const visible of [true, false]) {
    const players = visible ? [{x:-.5, receivedAt:Date.now(), sec:1, nanosec:0}] : [];
    const field = {mode:'OPTITRACK', player_positions: verified == null ? undefined : {P1:verified, P2:false},
      ball_model_status:{state:'CONFIGURED', configured_model:'ppq004', age_s:0, endpoints:{}}};
    const tree = panelHarness('linux','STOPPED',false,field,0,players,{lifecycleConfigRevision:1,configRuntime:{applied:true,services:[],mocapProcesses:[]},lastReceived:{'/hope/lifecycle/config/status':Date.now(),'/hope/lifecycle/state':Date.now(),'/hope/field/status':Date.now()}}).render();
    assert.equal(button(tree,'P1 裤子 · UCB_P1').props.disabled, false);
    assert.equal(button(tree,'▶ 启动系统').props.disabled, false);
    }
  }
});

test('Foxglove-normalized pose messages enable physical sides and START through the real decoder', () => {
  const decoder = panelHarness().api;
  const snapshot = { lifecycleConfig:{}, lastReceived:{} };
  const now = Date.now();
  for (const [topic, x] of [[decoder.TOPICS.playerP1, -.5], [decoder.TOPICS.playerP2, 3.2]]) {
    decoder.applyMessage(snapshot, {topic, message: {
      header: {frame_id: 'hope_table_global', stamp: {sec: 1793113809, nsec: 720394517}},
      pose: {position: {x, y: 0, z: 1}, orientation: {x: 0, y: 0, z: 0, w: 1}},
    }}, now);
  }
  const field = {mode:'OPTITRACK', player_positions:{P1:true, P2:true},
    ball_model_status:{state:'CONFIGURED', configured_model:'ppq004', age_s:0, endpoints:{}}};
  const tree = panelHarness('linux', 'STOPPED', false, field, 0,
    [snapshot.playerP1, snapshot.playerP2], {lifecycleConfigRevision:51,configRuntime:{applied:true,services:[],mocapProcesses:[]},lastReceived:{'/hope/lifecycle/config/status':Date.now(),'/hope/lifecycle/state':Date.now(),'/hope/field/status':Date.now()}}).render();
  assert.equal(button(tree, 'P1 裤子 · UCB_P1').props.disabled, false);
  assert.equal(button(tree, 'P2 裤子 · UCB_P2').props.disabled, false);
  assert.equal(button(tree, '▶ 启动系统').props.disabled, false);
});

test('automatic stop reason is displayed independently and expires with its Runner boot', () => {
  for (const [boot, age, expected] of [[42,0,true], [43,0,false], [42,1001,false]]) {
    const now=Date.now();
    const tree=panelHarness('linux','RUNNING',false,undefined,0,[], {
      runnerAlive:true, runnerBoot:42,
      mocapSafety:{boot,required:true,fresh:false,stops:1,interrupted:true},
      lastAction:'ENTER_MOTION', lastResult:'APPLIED', lastReason:'MODE_CHANGED',
      lastReceived:{'/hope/runner/alive':now,'/hope/runner/boot_id':now,
        '/hope/runner/mocap_safety_hdu_flat':now-age},
    }).render();
    const text=elements(tree).flatMap(e=>typeof e.props?.children==='string' ? [e.props.children] : []);
    assert.equal(text.some(t=>t.startsWith('已停止：动捕中断')), expected);
    assert.ok(text.includes('ENTER_MOTION → APPLIED / MODE_CHANGED'));
  }
});
test('UInt64 CDR boot ids decode from BigInt without permitting lossy identities', () => {
  const harness=panelHarness();
  const snapshot={lastReceived:{}, lifecycleConfig:{}};
  for (const [wire, expected] of [
    [42, 42], [42n, 42], [2n**52n, 2**52],
    [0n, undefined], [-1n, undefined], [1.5, undefined],
    [2n**52n + 1n, undefined], [2n**60n, undefined],
  ]) {
    harness.api.applyMessage(snapshot,{topic:harness.api.TOPICS.runnerBoot,message:{data:wire}},100);
    assert.equal(snapshot.runnerBoot,expected,`boot ${wire}`);
  }
});

test('decoded UInt64 boot matches the safety packet in the rendered panel', () => {
  const harness=panelHarness();
  for (const boot of [42, 42n]) {
    const now=Date.now();
    const snapshot={lastReceived:{}, lifecycleConfig:{}};
    for (const [topic, data] of [
      [harness.api.TOPICS.runnerAlive, true],
      [harness.api.TOPICS.runnerBoot, boot],
      [harness.api.TOPICS.mocapSafety, [1,42,7,1,1,0,1,1,0,120,1,1,1,0,0]],
    ]) {
      harness.api.applyMessage(snapshot,{topic,message:{data}},now);
    }
    const tree=panelHarness('linux','RUNNING',false,undefined,0,[],snapshot).render();
    const text=elements(tree).flatMap(e=>typeof e.props?.children==='string' ? [e.props.children] : []);
    assert.ok(text.includes('已停止：动捕中断 · 正在受控恢复到 PD_STAND'));
    assert.ok(!text.includes('动捕安全状态不可用'));
  }
});

test('UInt64 flight ids decode exactly and render in the MOTION swing label', () => {
  const harness=panelHarness();
  const snapshot={lastReceived:{}, lifecycleConfig:{}};
  for (const [wire, expected] of [
    [42, 42], [42n, 42], [0n, 0], [9007199254740991n, Number.MAX_SAFE_INTEGER],
    [9007199254740992n, undefined], [9007199254740993n, undefined],
    [18446744073709551615n, undefined],
  ]) {
    harness.api.applyMessage(snapshot,{topic:harness.api.TOPICS.commandFlight,message:{data:wire}},100);
    assert.equal(snapshot.commandFlight,expected,`flight ${wire}`);
  }
  for (const [topic, data] of [
    [harness.api.TOPICS.commandFlight, 42n],
    [harness.api.TOPICS.commandCountdown, .125],
    [harness.api.TOPICS.runnerMode, 'MOTION'],
  ]) {
    harness.api.applyMessage(snapshot,{topic,message:{data}},Date.now());
  }
  const tree=panelHarness('linux','RUNNING',false,undefined,0,[],snapshot).render();
  assert.ok(elements(tree).some(e=>e.props?.detail?.includes('SWING INPUT · AUDIT F42 · 0.13s')));
});

test('shared scalar decoder preserves finite signed floats and rejects invalid values', () => {
  const harness=panelHarness();
  const snapshot={lastReceived:{}, lifecycleConfig:{}};
  for (const [wire, expected] of [
    [-.125, -.125], [0, 0], [1.25, 1.25], [-42n, -42],
    [-9007199254740993n, undefined],
    [NaN, undefined], [Infinity, undefined], [-Infinity, undefined],
    ['42', undefined], [true, undefined], [null, undefined], [undefined, undefined],
  ]) {
    harness.api.applyMessage(snapshot,{topic:harness.api.TOPICS.ntpOffset,message:{data:wire}},100);
    assert.equal(snapshot.ntpOffsetMs,expected,`offset ${wire}`);
  }
});

 test('policy list offers only 14200 Best and respects stopped/mocap gates', async () => {
  const best='14200 Best · Serve + receive';
  const harness=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK', policy_model:'model14200best', policy_models_age_s:0, policy_models_config_revision:1, policy_models:{model14200best:{available:true,reason:''}}},0,[],{lifecycleConfigRevision:1},{language:'en'});
  const tree=harness.render();
  const selected=button(tree,best);
  assert.equal(selected.props.disabled,false);
  assert.equal(selected.props['aria-pressed'],true);
  for (const name of ['14600','11999 · Receive only','12600 · Serve + receive','17400 · Serve + receive','12000 · Serve + receive','14200 · Serve + receive','13800 · Serve + receive']) assert.equal(button(tree,name),undefined);
  selected.props.onClick();
  await Promise.resolve();
  assert.equal(harness.calls.length,1);
  assert.match(JSON.stringify(harness.calls[0].payload),/set_policy_model/);
  assert.match(JSON.stringify(harness.calls[0].payload),/model14200best/);
  const locked=panelHarness('linux','RUNNING',false,{mode:'OPTITRACK'},0,[],{lifecycleConfigRevision:1},{language:'en'}).render();
  assert.equal(button(locked,best).props.disabled,true);
});

test('only a verified installed 14200 Best is selectable, in both normal play and Kernel', () => {
  const best='14200 Best · Serve + receive';
  const installed={model14200best:{available:true,reason:''},model14600:{available:true,reason:''}};
  const render=(field)=>panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_models_age_s:1,policy_models_config_revision:1,policy_models:installed,...field},0,[],{lifecycleConfigRevision:1},{language:"en"}).render();
  assert.equal(button(render({}),best).props.disabled,false);
  for(const field of [{policy_models:undefined},{policy_models_age_s:31},{policy_models_age_s:null},{policy_models_config_revision:2}]) assert.equal(button(render(field),best).props.disabled,true);
  const broken=render({policy_models:{...installed,model14200best:{available:false,reason:'MDU model failed verification'}}});
  assert.equal(button(broken,best).props.disabled,true);
  assert.equal(button(broken,best).props.title,'MDU model failed verification');
  const kernel=render({mode:'KERNEL'});
  assert.equal(button(kernel,best).props.disabled,false);
  assert.equal(button(kernel,'14600'),undefined);
});

test('only 14200 Best is offered in the policy dropdown (operator request 2026-11-18)', () => {
  const best='14200 Best · Serve + receive';
  const text=(tree)=>JSON.stringify(elements(tree).map((n)=>typeof n.props?.children==='string'?n.props.children:''));
  const installed={model14200best:{available:true,reason:''},model19000:{available:true,reason:'',receive_only:true},
    model26500:{available:true,reason:'',receive_only:true},model400:{available:true,reason:''}};
  const harness=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_model:'model19000',policy_models_age_s:0,policy_models_config_revision:1,policy_models:installed},0,[],{lifecycleConfigRevision:1},{language:'en'});
  const tree=harness.render();
  for (const gone of ['19000 GRU · Receive only','26500 · Receive only','400 · Serve + receive']) {
    assert.equal(elements(tree).some((n)=>n.type==='button'&&JSON.stringify(n.props.children).includes(gone)),false,gone);
  }
  assert.equal(button(tree,best).props.disabled,false);
  button(tree,best).props.onClick();
  assert.deepEqual(JSON.parse(harness.calls.at(-1).payload.parameters[0].value.string_value),{op:'set_policy_model',model:'model14200best'});
  // A model that is no longer offered is still named when the robot reports it.
  assert.match(text(tree),/The robot has 19000 GRU selected/);
});

// Hidden from the dropdown 2026-11-18 (hidden: true in POLICY_ROWS); re-enable when 19000 GRU / 26500 are shown again.
test.skip('19000 GRU is a separate receive-only option next to 14200 Best, never offered in Kernel', () => {
  const gru='19000 GRU · Receive only', best='14200 Best · Serve + receive';
  const text=(tree)=>JSON.stringify(elements(tree).map((n)=>typeof n.props?.children==='string'?n.props.children:''));
  const installed={model14200best:{available:true,reason:''},model19000:{available:true,reason:'',receive_only:true}};
  const render=(field)=>panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_models_age_s:1,policy_models_config_revision:1,policy_models:installed,...field},0,[],{lifecycleConfigRevision:1},{language:'en'}).render();
  const normal=render({policy_model:'model14200best'});
  assert.equal(button(normal,best).props.disabled,false);
  assert.equal(button(normal,gru).props.disabled,false);
  const harness=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_model:'model14200best',policy_models_age_s:0,policy_models_config_revision:1,policy_models:installed},0,[],{lifecycleConfigRevision:1},{language:'en'});
  button(harness.render(),gru).props.onClick();
  assert.deepEqual(JSON.parse(harness.calls.at(-1).payload.parameters[0].value.string_value),{op:'set_policy_model',model:'model19000'});
  assert.doesNotMatch(text(render({policy_model:'model19000'})),/The robot has 19000 GRU selected/);
  assert.equal(button(render({policy_models:{...installed,model19000:{available:false,reason:'policy bundle not installed: model19000'}}}),gru).props.disabled,true);
});

test.skip('26500 is its own receive-only option beside 19000 GRU and 14200 Best', () => {
  const row='26500 · Receive only';
  const installed={model14200best:{available:true,reason:''},model19000:{available:true,reason:'',receive_only:true},model26500:{available:true,reason:'',receive_only:true}};
  const harness=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_model:'model19000',policy_models_age_s:0,policy_models_config_revision:1,policy_models:installed},0,[],{lifecycleConfigRevision:1},{language:'en'});
  const tree=harness.render();
  for(const name of [row,'19000 GRU · Receive only','14200 Best · Serve + receive']) assert.equal(button(tree,name).props.disabled,false);
  button(tree,row).props.onClick();
  assert.deepEqual(JSON.parse(harness.calls.at(-1).payload.parameters[0].value.string_value),{op:'set_policy_model',model:'model26500'});
});

test('14200 Best shows the installed bundle and flags another selected model', () => {
  const text=(tree)=>JSON.stringify(elements(tree).map((n)=>typeof n.props?.children==='string'?n.props.children:''));
  const render=(field)=>panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_models_age_s:1,policy_models_config_revision:1,policy_models:{model14200best:{available:true,reason:''}},...field},0,[],{lifecycleConfigRevision:1},{language:'en'}).render();
  assert.match(text(render({policy_model:'model14200best',csv:{name:'v23-14200-best.csv',sha256:'14c75e8e2f17234e26d2498c7b75da9b2f2b8aad48f8e31659901c56fa856630',frames:468,hz:100}})),/Bundle best-v1/);
  assert.match(text(render({policy_model:'model14200best',csv:{name:'v23-14200-best.csv',sha256:'fa37dd8ffc89390163d30cda29ebea47c4e56db4317ceafcc4d33fcd971ff4c9',frames:468,hz:100}})),/Bundle earlier-staged/);
  assert.match(text(render({policy_model:'model14200best',csv:null})),/Bundle not identified/);
  assert.match(text(render({policy_model:'model17400'})),/The robot has 17400 selected/);
  // installed but not selected: the catalog's own serve CSV identifies the bundle
  assert.match(text(render({policy_model:'model14600',policy_models:{model14200best:{available:true,reason:'',serve_csv:{name:'v23-14200-best.csv',sha256:'14c75e8e2f17234e26d2498c7b75da9b2f2b8aad48f8e31659901c56fa856630'}}}})),/Bundle best-v1/);
});

test('mode buttons switch mode only; the 14200 Best selection stays', async () => {
  const modeButton=(tree,mode)=>elements(tree).find((n)=>n.type==='button'&&n.props.onClick&&JSON.stringify(n.props.children).includes(mode==='KERNEL'?'Kernel':'Normal play'));
  const flush=async()=>{ for (let i=0;i<6;i++) await Promise.resolve(); };
  const toKernel=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_model:'model14200best'},0,[],{lifecycleConfigRevision:1},{language:'en'});
  modeButton(toKernel.render(),'KERNEL').props.onClick(); await flush();
  assert.deepEqual(toKernel.calls.map((c)=>JSON.parse(c.payload.parameters[0].value.string_value)),[{op:'set_mode',mode:'KERNEL'}]);
  const toNormal=panelHarness('linux','STOPPED',false,{mode:'KERNEL',policy_model:'model14200best'},0,[],{lifecycleConfigRevision:1},{language:'en'});
  modeButton(toNormal.render(),'OPTITRACK').props.onClick(); await flush();
  assert.deepEqual(toNormal.calls.map((c)=>JSON.parse(c.payload.parameters[0].value.string_value)),[{op:'set_mode',mode:'OPTITRACK'}]);
  const keep=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_model:'model17400'},0,[],{lifecycleConfigRevision:1},{language:'en'});
  modeButton(keep.render(),'OPTITRACK').props.onClick(); await flush();
  assert.deepEqual(keep.calls.map((c)=>JSON.parse(c.payload.parameters[0].value.string_value)),[{op:'set_mode',mode:'OPTITRACK'}]);
});

test('old saved robot target does not restore a pending switch or masquerade as live identity', () => {
 const live='192.168.200.202';
 const h=panelHarness('linux','STOPPED',false,undefined,0,[],{
  lifecycleConfig:{hdu_wifi_ip:'192.168.8.196'},
  configRuntimeSeen:true,configRuntime:{services:[],mocapProcesses:[],robotIdentity:{machine_id:'a'.repeat(32),boot_id:'b'.repeat(36),connection:{hdu_ip:live,laptop_ip:'192.168.100.89'}}},
  lastReceived:{'/hope/lifecycle/state':Date.now(),'/hope/lifecycle/config/status':Date.now()},
 },{robotTarget:'192.168.8.182'});
 const tree=h.render();const all=elements(tree);
 assert.equal(all.find(n=>n.type==='fieldset' && n.props.className==='robot-controls').props.disabled,false);
 const status=all.find(n=>n.props?.role==='status').props.children;
 assert.ok(status.includes(live));assert.ok(!status.includes('192.168.8.182'));assert.ok(!status.includes('正在切换'));
});

test('fresh status with an empty robot address says address unknown rather than no data', () => {
 const h=panelHarness('linux','STOPPED',false,undefined,0,[],{
  lifecycleConfig:{hdu_wifi_ip:''},
  lastReceived:{'/hope/lifecycle/state':Date.now(),'/hope/lifecycle/config/hdu_wifi_ip':Date.now()},
 },{robotTarget:'192.168.8.182'});
 const status=elements(h.render()).find(n=>n.props?.role==='status').props.children;
 assert.ok(status.includes('STOPPED'));assert.ok(status.includes('连接身份尚未验证'));assert.ok(!status.includes('192.168.8.182'));
});

test('language switch persists independently and preserves unsaved controls without service calls', () => {
 const initial = {robotTarget:'192.168.200.202',ballYFilterDraft:{enabled:false,min:'-1.3',max:'-0.55'},ballYFilterDraftEdited:true,other:'keep'};
 const h=panelHarness('linux','STOPPED',false,undefined,0,[],{},initial);
 let tree=h.render();
 const target=elements(tree).find(n=>n.props?.['aria-label']==='Robot connection IP');
 target.props.onChange({target:{value:'192.168.200.203'}});
 button(tree,'Mac').props.onClick();
 button(h.render(),'English').props.onClick();
 tree=h.render();
 assert.equal(button(tree,'English').props['aria-pressed'],true);
 assert.equal(button(tree,'Mac').props['aria-pressed'],true);
 assert.equal(elements(tree).find(n=>n.props?.['aria-label']==='Robot connection IP').props.value,'192.168.200.203');
 assert.equal(button(tree,'Confirm current connection').props.disabled,false);
 assert.deepEqual(JSON.parse(JSON.stringify(h.savedStates.at(-1))),{...initial,language:'en'});
 assert.equal(h.calls.length,0);
 button(tree,'Chinese').props.onClick();
 assert.equal(button(h.render(),'中文').props['aria-pressed'],true);
 assert.equal(h.calls.length,0);
 const reopened=panelHarness('linux','STOPPED',false,undefined,0,[],{}, {...initial,language:'en'});
 assert.equal(button(reopened.render(),'English').props['aria-pressed'],true);
});

test('pending notice and confirmation prompts follow language changes', async () => {
 const h=panelHarness('linux','FAILED');
 let tree=h.render();
 elements(tree).find(n=>n.props?.['aria-label']==='Robot connection IP').props.onChange({target:{value:'192.168.200.203'}});
 // A mismatched draft is rejected locally; its notice must not freeze in Chinese.
 const configInput=elements(tree).find(n=>n.type==='input' && n.props.value==='192.0.2.10');
 configInput.props.onChange({target:{value:'192.0.2.11'}});
 // Generate a local IPv4 validation error using the actual connection callback.
 elements(h.render()).find(n=>n.props?.['aria-label']==='Robot connection IP').props.onChange({target:{value:'bad'}});
 button(h.render(),'确认当前连接').props.onClick();
 button(h.render(),'English').props.onClick();
 tree=h.render();
 const notice=elements(tree).find(n=>n.props?.className==='operator-notice ');
 assert.match(JSON.stringify(notice),/Enter a valid local IPv4/);
 assert.doesNotMatch(JSON.stringify(notice),/请输入/);
 await button(tree,'▶ Retry start · clean up first').props.onClick();
 assert.match(JSON.stringify(elements(h.render()).find(n=>n.props?.role==='dialog')),/Clean up the failed session/);
 assert.doesNotMatch(JSON.stringify(elements(h.render()).find(n=>n.props?.role==='dialog')),/[\u3400-\u9fff]/);
 assert.equal(h.calls.length,0);
 button(h.render(),'Chinese').props.onClick();
 await button(h.render(),'▶ 重试启动 · 先自动清理').props.onClick();
 assert.match(JSON.stringify(elements(h.render()).find(n=>n.props?.role==='dialog')),/清理上次失败会话/);
});

function serveHarness() {
  return panelHarness('linux', 'RUNNING', false, undefined, 0, [], {}, {language:'en'});
}
function serveFrame(h, {ui = 1, computer = 1, boot = 42, session = 'test', role = 'SERVER',
  mode = 'PD_STAND', serve = 'IDLE', required = 1, seq = 1,
  pelvis = true, mocapRequired = 1} = {}) {
  const values = {
    runnerAlive: true, runnerBoot: boot, runnerFault: false, runnerMode: mode,
    localRole: role, serveCapability: 'AVAILABLE', serveState: serve, lifecycleSession: session,
    baseFresh: pelvis, tfReady: pelvis,
    mocapSafety: [1, boot, 2279734720943415, seq, mocapRequired, 1, 0, 0, 0, 0, required, ui, computer, 0, 0],
  };
  h.frame(Object.entries(values).map(([key, data]) => ({topic:h.api.TOPICS[key], message:{data}})));
}
function action(tree, label) {
  return elements(tree).find(n => n.props?.label === label);
}

function consoleKeyboard(h) {
  const shortcutHarness = require('./shortcutHarness.cjs');
  return shortcutHarness(shortcut => {
    const tree = h.render();
    const controls = elements(tree).filter(n => n.props?.shortcut === shortcut);
    const switching = elements(tree).some(n => n.type === 'fieldset' && n.props.disabled);
    return controls.map(control => ({matches: () => switching || control.props.disabled || control.props.busy,
      click: control.props.onClick}));
  }, () => elements(h.render()).some(n => n.props?.role === 'dialog'));
}

test('Console shortcuts use the actual Init/Ready/Passive services and local intents', async () => {
  const h = serveHarness(); h.render(); serveFrame(h, {role:'RECEIVER'});
  const keyboard = consoleKeyboard(h);
  keyboard.down('i'); keyboard.up('i');
  keyboard.down('r'); keyboard.up('r');
  keyboard.down('ArrowLeft'); keyboard.down('ArrowRight');
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(h.calls.map(call => call.service), [
    '/hope/runner/enter_pd_stand', '/hope/runner/enter_motion', '/hope/runner/emergency_passive',
  ]);
  assert.deepEqual(h.localRequests, ['STOP','READY','STOP']);
});

test('Ready shortcut respects Runner role/mode, stale state, fault and request busy gates', () => {
  for (const state of [{role:'SERVER'}, {mode:'PASSIVE'}, {mode:'MOTION'}, {stale:true}, {fault:true}]) {
    const h = serveHarness(); h.render(); serveFrame(h, {role:'RECEIVER',...state});
    if (state.stale) h.advance(1600);
    if (state.fault) h.frame([{topic:h.api.TOPICS.runnerFault,message:{data:true}}]);
    consoleKeyboard(h).down('r');
    assert.equal(h.calls.length, 0, JSON.stringify(state));
  }
  const h = serveHarness(); h.render(); serveFrame(h, {role:'RECEIVER'});
  const keyboard = consoleKeyboard(h);
  keyboard.down('r'); keyboard.up('r'); keyboard.down('r');
  assert.equal(h.calls.length, 1);
});

test('S starts the current Serve stage without a dialog and needs a fresh press for stage two', async () => {
  const h = serveHarness(); h.render(); serveFrame(h);
  assert.ok(action(h.render(), 'Stand'));
  const keyboard = consoleKeyboard(h);
  keyboard.down('s');
  assert.equal(elements(h.render()).some(n=>n.props?.role==='dialog'), false);
  assert.deepEqual(h.calls.map(call=>call.service), ['/hope/runner/prepare_serve']);
  await new Promise(resolve=>setImmediate(resolve));
  serveFrame(h,{mode:'SERVE',serve:'WAIT_READY_TO_SERVE'});
  keyboard.down('s', {repeat:true});
  assert.equal(h.calls.length,1);
  keyboard.up('s'); keyboard.down('s');
  await new Promise(resolve=>setImmediate(resolve));
  assert.deepEqual(h.calls.map(call=>call.service), ['/hope/runner/prepare_serve','/hope/runner/ready_to_serve']);
  assert.deepEqual(h.localRequests,['SERVE','SERVE']);
  assert.equal(elements(h.render()).some(n=>n.props?.role==='dialog'), false);
});

test('S does nothing between Serve stages or for a receiver', () => {
  for (const frame of [{role:'RECEIVER'}, {mode:'PASSIVE'}, {mode:'SERVE',serve:'PREPARING'},
    {mode:'SERVE',serve:'SERVING'}, {serve:'FAULT'}]) {
    const h=serveHarness(); h.render(); serveFrame(h,frame);
    consoleKeyboard(h).down('s');
    assert.equal(h.calls.length,0,JSON.stringify(frame));
  }
});

test('immediate Serve keeps click-time state/identity checks and synchronous duplicate protection', () => {
  for (const change of [h=>h.advance(1001), h=>serveFrame(h,{role:'RECEIVER'}),
    h=>serveFrame(h,{serve:'PREPARING'}), h=>serveFrame(h,{boot:43}),
    h=>serveFrame(h,{session:'other'}), h=>h.frame([],true)]) {
    const h=serveHarness(); h.render(); serveFrame(h);
    const oldClick=action(h.render(),'Raise & load').props.onClick;
    change(h); oldClick();
    assert.equal(h.calls.length,0);
  }
  const h=serveHarness(); h.render(); serveFrame(h);
  const click=action(h.render(),'Raise & load').props.onClick;
  click(); click();
  assert.equal(h.calls.length,1);
});

test('Passive remains available on a Runner fault; stale Runner disables every shortcut', () => {
  const h = serveHarness(); h.render(); serveFrame(h);
  h.frame([{topic:h.api.TOPICS.runnerFault,message:{data:true}}]);
  const keyboard = consoleKeyboard(h);
  keyboard.down('i'); keyboard.up('i');
  keyboard.down('ArrowRight'); keyboard.down('ArrowLeft');
  assert.deepEqual(h.calls.map(call => call.service), ['/hope/runner/emergency_passive']);
  keyboard.up('ArrowRight'); keyboard.up('ArrowLeft');
  h.advance(1600);
  keyboard.down('i'); keyboard.down('s'); keyboard.down('r'); keyboard.down('ArrowLeft'); keyboard.down('ArrowRight');
  assert.equal(h.calls.length, 1);
});

test('confirmation stays open with telemetry and no heartbeat publication; one explicit submit', async () => {
  const h = serveHarness(); h.render(); serveFrame(h);
  action(h.render(), 'Open gripper').props.onClick();
  for(let seq=2;seq<=26;seq++) {
    h.advance(200); serveFrame(h, {seq});
    assert.ok(elements(h.render()).some(n=>n.props?.role==='dialog'));
  }
  assert.equal(h.published.length,0); // telemetry updates never publish operator heartbeats
  assert.equal(h.calls.length,0);
  const confirm=button(h.render(),'Confirm and send');
  assert.equal(confirm.props.disabled,false);
  confirm.props.onClick(); confirm.props.onClick();
  await Promise.resolve();
  assert.equal(h.calls.length,1);
  assert.equal(h.calls[0].service,'/hope/runner/open_gripper');
  assert.ok(!elements(h.render()).some(n=>n.props?.role==='dialog'));
});

test('missing UI and computer heartbeats do not block explicit submission', () => {
  const h=serveHarness(); h.render(); serveFrame(h);
  action(h.render(),'Open gripper').props.onClick();
  serveFrame(h,{ui:0,computer:0,seq:2});
  assert.equal(button(h.render(),'Confirm and send').props.disabled,false);
  assert.equal(h.calls.length,0);
  button(h.render(),'Confirm and send').props.onClick();
  assert.equal(h.calls.length,1);
  assert.equal(h.published.length,0);
});

test('cancel invalidates an old confirmation callback without sending a request', () => {
  const h=serveHarness(); h.render(); serveFrame(h,{ui:0,computer:0});
  action(h.render(),'Open gripper').props.onClick();
  const oldConfirm=button(h.render(),'Confirm and send');
  button(h.render(),'Cancel').props.onClick();
  oldConfirm.props.onClick();
  assert.equal(h.calls.length,0);
  assert.ok(!elements(h.render()).some(n=>n.props?.role==='dialog'));
});

test('stale status, role changes, new boot/session and seek invalidate pending gripper consent', () => {
  for(const change of [h=>h.advance(1001), h=>serveFrame(h,{role:'RECEIVER',seq:2}),
    h=>serveFrame(h,{boot:43,seq:2}), h=>serveFrame(h,{session:'other',seq:2}),
    h=>h.frame([],true)]) {
    const h=serveHarness(); h.render(); serveFrame(h);
    action(h.render(),'Open gripper').props.onClick();
    const confirm=button(h.render(),'Confirm and send');
    change(h); confirm.props.onClick();
    assert.equal(h.calls.length,0);
  }
});

test('Serve sends immediately without confirmation or operator heartbeats', () => {
  const h=serveHarness(); h.render(); serveFrame(h,{mode:'SERVE',serve:'WAIT_READY_TO_SERVE',required:0,ui:0,computer:0});
  const serve=action(h.render(),'Serve');
  assert.ok(serve); serve.props.onClick();
  assert.equal(elements(h.render()).some(n=>n.props?.role==='dialog'), false);
  assert.equal(h.calls.length,1);
  assert.equal(h.calls[0].service,'/hope/runner/ready_to_serve');
});


test('local Serve intent survives rejected robot request; remote feedback never sends local intents', async () => {
  const h=serveHarness();h.setRobotAccepted(false);h.render();serveFrame(h);
  action(h.render(),'Raise & load').props.onClick();
  await new Promise(resolve=>setImmediate(resolve));
  assert.deepEqual(h.localRequests,['SERVE']);
  assert.equal(h.calls.length,1);
  for(const mode of ['PD_STAND','MOTION','SERVE','PASSIVE']) {
    serveFrame(h,{mode,serve:'FAULT'});h.render();
  }
  await new Promise(resolve=>setImmediate(resolve));
  assert.deepEqual(h.localRequests,['SERVE']);
});

test('local endpoint failure never blocks the existing robot request', async () => {
  const h=serveHarness();h.setLocalOffline(true);h.render();serveFrame(h);
  action(h.render(),'Raise & load').props.onClick();
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(h.calls.length,1);assert.deepEqual(h.localRequests,[]);
  assert.ok(elements(h.render()).some(n=>n.props?.role==='alert'));
});

// ---- 2.0 behavior changes ----
const flush = () => new Promise(resolve => setImmediate(resolve));
const services = h => h.calls.map(call => call.service);
function allowRoleChange(h, allowed = true) {
  h.frame([{topic:h.api.TOPICS.roleChangeAllowed, message:{data:allowed}}]);
}

test('Receive as SERVER switches role first, waits for the reported RECEIVER role, then sends Ready', async () => {
  const h = serveHarness(); h.render(); serveFrame(h); allowRoleChange(h);
  const receive = action(h.render(), 'Receive');
  assert.equal(receive.props.disabled, false);
  receive.props.onClick();
  await flush();
  assert.deepEqual(services(h), ['/hope/runner/set_receiver']);
  // Accepted but not yet reported: Ready is not sent on the service reply alone.
  h.advance(50); serveFrame(h, {seq:2}); await flush();
  assert.deepEqual(services(h), ['/hope/runner/set_receiver']);
  h.advance(50); serveFrame(h, {role:'RECEIVER', seq:3}); await flush();
  assert.deepEqual(services(h), ['/hope/runner/set_receiver', '/hope/runner/enter_motion']);
  assert.deepEqual(h.localRequests, ['READY']);
});

test('Raise as RECEIVER switches role first, then uses the unchanged immediate Start to Serve path', async () => {
  const h = serveHarness(); h.render(); serveFrame(h, {role:'RECEIVER'}); allowRoleChange(h);
  const raise = action(h.render(), 'Raise & load');
  assert.equal(raise.props.disabled, false);
  raise.props.onClick();
  await flush();
  assert.deepEqual(services(h), ['/hope/runner/set_server']);
  h.advance(50); serveFrame(h, {role:'SERVER', seq:2}); await flush();
  assert.deepEqual(services(h), ['/hope/runner/set_server', '/hope/runner/prepare_serve']);
  assert.equal(elements(h.render()).some(n => n.props?.role === 'dialog'), false);
});

test('role switching is never attempted when the Runner does not allow it', async () => {
  for (const allowed of [false, undefined]) {
    const h = serveHarness(); h.render(); serveFrame(h);
    if (allowed != undefined) allowRoleChange(h, allowed);
    const receive = action(h.render(), 'Receive');
    assert.equal(receive.props.disabled, true);
    receive.props.onClick(); await flush();
    assert.equal(h.calls.length, 0);
    consoleKeyboard(h).down('r'); await flush();
    assert.equal(h.calls.length, 0);
  }
});

test('auto role switch keeps every other Ready / Start to Serve gate', () => {
  const node = (tree, shortcut, index = 0) => elements(tree).filter(n => n.props?.shortcut === shortcut)[index];
  for (const state of [{mode:'MOTION'}, {mode:'PASSIVE'}, {mode:'TELEOP'}]) {
    const h = serveHarness(); h.render(); serveFrame(h, state); allowRoleChange(h);
    assert.equal(node(h.render(), 'ready').props.disabled, true, JSON.stringify(state));
  }
  for (const state of [{mode:'PASSIVE'}, {serve:'PREPARING'}, {serve:'FAULT'}]) {
    const h = serveHarness(); h.render(); serveFrame(h, {role:'RECEIVER', ...state}); allowRoleChange(h);
    assert.equal(node(h.render(), 'serve').props.disabled, true, JSON.stringify(state));
  }
  const h = serveHarness(); h.render(); serveFrame(h); allowRoleChange(h);
  h.frame([{topic:h.api.TOPICS.runnerFault, message:{data:true}}]);
  assert.equal(node(h.render(), 'ready').props.disabled, true);
});

test('a rejected role change, an intervening command or a changed Runner state sends no follow-up', async () => {
  const rejected = serveHarness(); rejected.setRobotAccepted(false);
  rejected.render(); serveFrame(rejected); allowRoleChange(rejected);
  action(rejected.render(), 'Receive').props.onClick(); await flush();
  rejected.advance(50); serveFrame(rejected, {role:'RECEIVER', seq:2}); await flush();
  assert.deepEqual(services(rejected), ['/hope/runner/set_receiver']);

  const superseded = serveHarness(); superseded.render(); serveFrame(superseded); allowRoleChange(superseded);
  action(superseded.render(), 'Receive').props.onClick(); await flush();
  action(superseded.render(), 'PASSIVE').props.onClick(); await flush();
  superseded.advance(50); serveFrame(superseded, {role:'RECEIVER', seq:2}); await flush();
  assert.deepEqual(services(superseded), ['/hope/runner/set_receiver', '/hope/runner/emergency_passive']);

  const changed = serveHarness(); changed.render(); serveFrame(changed); allowRoleChange(changed);
  action(changed.render(), 'Receive').props.onClick(); await flush();
  changed.advance(50); serveFrame(changed, {role:'RECEIVER', mode:'PASSIVE', seq:2}); await flush();
  assert.deepEqual(services(changed), ['/hope/runner/set_receiver']);

  // Already SERVER: Raise is the unchanged immediate 1.9 path, no role request.
  const direct = serveHarness(); direct.render(); serveFrame(direct); allowRoleChange(direct);
  action(direct.render(), 'Raise & load').props.onClick(); await flush();
  assert.deepEqual(services(direct), ['/hope/runner/prepare_serve']);
});

test('PASSIVE is one click with no dialog; the teleop bar cannot request Teleop', async () => {
  const h = serveHarness(); h.render(); serveFrame(h);
  action(h.render(), 'PASSIVE').props.onClick();
  assert.equal(elements(h.render()).some(n => n.props?.role === 'dialog'), false);
  await flush();
  assert.deepEqual(services(h), ['/hope/runner/emergency_passive']);
  const source = fs.readFileSync(path.join(__dirname, '../src/HopeA3Console.tsx'), 'utf8');
  assert.ok(!source.includes('invoke("teleop")'));
});

test('actual Confirm callback keeps Host window and invokes no robot service', () => {
 const ip='192.168.200.202';
 const h=panelHarness('linux','STOPPED',false,undefined,0,[],{
  configRuntimeSeen:true,
  configRuntime:{hostConnection:{hdu_ip:ip,laptop_ip:'192.168.100.89'},
   robotIdentity:{machine_id:'a'.repeat(32),boot_id:'b'.repeat(36)},mocapProcesses:[],services:[]},
  lastReceived:{'/hope/lifecycle/config/status':Date.now()},
 },{robotTarget:ip});
 for(let i=0;i<3;i++) button(h.render(),'确认当前连接').props.onClick();
 assert.equal(h.calls.length,0);
 assert.equal(h.savedStates.at(-1).robotTarget,ip);
 const notice=elements(h.render()).find(n=>n.props?.className==='operator-notice ');
 assert.match(JSON.stringify(notice),/保持当前窗口/);
 h.advance(1600);
 button(h.render(),'确认当前连接').props.onClick();
 const stale=elements(h.render()).find(n=>n.props?.className==='operator-notice ');
 assert.match(JSON.stringify(stale),/等待当前连接的实时机器人身份/);
 assert.equal(h.calls.length,0);
});

test('pelvis-dependent Ready and both Serve stages fail closed at render and dispatch', async () => {
  for (const [role, mode, serve, label, key] of [
    ['RECEIVER','PD_STAND','IDLE','Ready','r'],
    ['SERVER','PD_STAND','IDLE','Start to Serve · Raise hand','s'],
    ['SERVER','SERVE','WAIT_READY_TO_SERVE','Serve · Play','s'],
  ]) {
    const h=serveHarness(); h.render(); serveFrame(h,{role,mode,serve});
    // 2.x state graph: Ready = receive node, Start to Serve = raise node, Serve = serve node.
    const nodeId = key==='r' ? 'receive' : serve==='IDLE' ? 'raise' : 'serve';
    const find=()=>elements(h.render()).find(n=>n.props?.id === nodeId && n.props?.shortcut !== undefined);
    const active=find(); assert.ok(active, label); assert.equal(active.props.disabled,false,label);
    const oldClick=active.props.onClick;
    serveFrame(h,{role,mode,serve,pelvis:false});
    assert.equal(find().props.disabled,true,label);
    const keyboard=consoleKeyboard(h); keyboard.down(key); keyboard.up(key);
    oldClick(); await Promise.resolve();
    assert.equal(h.calls.length,0,label); assert.equal(h.localRequests.length,0,label);
    serveFrame(h,{role,mode,serve}); h.advance(501);
    oldClick(); assert.equal(h.calls.length,0,`${label} stale click`);
  }
});

test('pelvis-independent Runner mode is exempt only with fresh matching safety evidence', () => {
  const h=serveHarness(); h.render(); serveFrame(h,{role:'RECEIVER',pelvis:false,mocapRequired:0});
  const ready=()=>elements(h.render()).find(n=>n.props?.shortcut==='ready');
  assert.equal(ready().props.disabled,false);
  h.advance(1001); assert.equal(ready().props.disabled,true);
  serveFrame(h,{role:'RECEIVER',pelvis:false,mocapRequired:0});
  h.frame([{topic:h.api.TOPICS.runnerBoot,message:{data:43}}]);
  assert.equal(ready().props.disabled,true);
});

test('MOTION cannot show completed Ready after pelvis disappears', () => {
  const h=serveHarness();h.render();serveFrame(h,{role:'RECEIVER',mode:'MOTION',pelvis:false});
  // 2.x: the receive node is not shown as the reached (current) state without a fresh pelvis.
  assert.equal(elements(h.render()).find(n=>n.props?.shortcut==='ready').props.current,false);
});

test('fresh Host pelvis cannot override missing Runner pelvis; bounded hold follows Runner admission', () => {
  const h=serveHarness(); h.render(); serveFrame(h,{role:'RECEIVER'});
  const ready=()=>elements(h.render()).find(n=>n.props?.shortcut==='ready');
  for (const [fresh,held,blocked] of [[0,0,true],[1,0,false],[0,1,false]]) {
    h.frame([{topic:h.api.TOPICS.mocapSafety,
      message:{data:[1,42,2279734720943415,2,1,fresh,0,0,held,25,0,1,1,0,0]}}]);
    assert.equal(ready().props.disabled,blocked);
  }
});
