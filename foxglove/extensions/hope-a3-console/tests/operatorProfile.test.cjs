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
      exports, ArrayBuffer, DataView, TextEncoder,
      Date: class extends Date { static now() { return now; } },
      window: { setTimeout: () => 0, confirm: () => { throw new Error('blocking confirmation interrupts heartbeat'); } },
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
      return payload.parameters ? { results: payload.parameters.map(() => ({ successful: true })) } : { success: true, message: 'ok' };
    },
  };
  return {
    api, calls, subscribed, savedStates, published,
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
const button = (tree, name) => elements(tree).find((node) =>
  node.type === 'button' && node.props.children === name);

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
  assert.equal(button(tree, 'CONFIRM CONFIG').props.disabled, false);
  button(tree, 'CONFIRM CONFIG').props.onClick();
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
    assert.equal(button(tree, 'CONFIRM CONFIG').props.disabled, true);
  }
  for (const [status, busy] of [['RUNNING', false], ['STOPPED', true], ['STARTING', true]]) {
    const tree = panelHarness('linux', status, busy).render();
    assert.equal(button(tree, 'Linux').props.disabled, true);
    assert.equal(button(tree, 'Mac').props.disabled, true);
    assert.equal(button(tree, 'CONFIRM CONFIG').props.disabled, true);
  }
});

test('table assignment buttons require live end-position evidence and lock until STOP', () => {
  for (const [p1, p2] of [[true, false], [false, true], [false, false], [true, true]]) {
    const field = { mode: 'OPTITRACK' };
    const players = [{x:p1 ? -.5 : 1, receivedAt:Date.now(), sec:1, nanosec:0},
      {x:p2 ? 3 : 1, receivedAt:Date.now(), sec:1, nanosec:0}];
    let tree = panelHarness('linux', 'STOPPED', false, field, 0, players).render();
    assert.equal(button(tree, 'UCB_P1 · DIRECT').props.disabled, !p1);
    assert.equal(button(tree, 'UCB_P2 · ROTATE 180°').props.disabled, !p2);
    for (const state of ['STARTING', 'RUNNING', 'FAILED', 'KILLING']) {
      tree = panelHarness('linux', state, false, field, 0, players).render();
      assert.equal(button(tree, 'UCB_P1 · DIRECT').props.disabled, true);
      assert.equal(button(tree, 'UCB_P2 · ROTATE 180°').props.disabled, true);
    }
  }
});

test('missing/stale position status disables both sides; offline Kernel remains selectable', () => {
  for (const players of [[], [{x:-.5, receivedAt:Date.now()-251}, {x:3, receivedAt:Date.now()-251}]]) {
    const tree = panelHarness('linux', 'STOPPED', false, undefined, 0, players).render();
    assert.equal(button(tree, 'UCB_P1 · DIRECT').props.disabled, true);
    assert.equal(button(tree, 'UCB_P2 · ROTATE 180°').props.disabled, true);
  }
  const tree = panelHarness('linux', 'STOPPED', false, {mode:'KERNEL'}).render();
  assert.equal(button(tree, 'UCB_P1 · DIRECT').props.disabled, false);
  assert.equal(button(tree, 'UCB_P2 · ROTATE 180°').props.disabled, false);
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

test('START requires the HDU verdict as well as raw UI evidence; side buttons stay raw-driven', () => {
  for (const verified of [true, false, undefined]) {
    const players = [{x:-.5, receivedAt:Date.now(), sec:1, nanosec:0}];
    const field = {mode:'OPTITRACK', player_positions: verified == null ? undefined : {P1:verified, P2:false},
      ball_model_status:{state:'CONFIGURED', configured_model:'ppq004', age_s:0, endpoints:{}}};
    const tree = panelHarness('linux','STOPPED',false,field,0,players,{lifecycleConfigRevision:1}).render();
    assert.equal(button(tree,'UCB_P1 · DIRECT').props.disabled, false);
    assert.equal(button(tree,'START SYSTEM').props.disabled, verified !== true);
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
    [snapshot.playerP1, snapshot.playerP2], {lifecycleConfigRevision:51}).render();
  assert.equal(button(tree, 'UCB_P1 · DIRECT').props.disabled, false);
  assert.equal(button(tree, 'UCB_P2 · ROTATE 180°').props.disabled, false);
  assert.equal(button(tree, 'START SYSTEM').props.disabled, false);
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
  assert.ok(elements(tree).some(e=>e.props?.label==='SWING INPUT · AUDIT F42 · 0.13s'));
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

 test('model12000 selection preserves existing policies and respects stopped/mocap gates', async () => {
  const harness=panelHarness('linux','STOPPED',false,{mode:'OPTITRACK', policy_model:'model12000', policy_models_age_s:0, policy_models_config_revision:1, policy_models:{model12000:{available:true,reason:''}}},0,[],{lifecycleConfigRevision:1});
  const tree=harness.render();
  const selected=button(tree,'12000 · Serve + receive');
  assert.equal(selected.props.disabled,false);
  assert.equal(selected.props['aria-pressed'],true);
  for (const name of ['14600','11999 · Receive only','12600 · Serve + receive','17400 · Serve + receive']) assert.ok(button(tree,name));
  selected.props.onClick();
  await Promise.resolve();
  assert.equal(harness.calls.length,1);
  assert.match(JSON.stringify(harness.calls[0].payload),/set_policy_model/);
  assert.match(JSON.stringify(harness.calls[0].payload),/model12000/);
  for (const [state,mode] of [['RUNNING','OPTITRACK'],['STOPPED','KERNEL']]) {
    const locked=panelHarness('linux',state,false,{mode},0,[],{lifecycleConfigRevision:1}).render();
    assert.equal(button(locked,'12000 · Serve + receive').props.disabled,true);
  }
});

test('only verified installed models are selectable and disabled models explain why', () => {
  const names={model14600:'14600',model12600:'12600 · Serve + receive',model17400:'17400 · Serve + receive',model12000:'12000 · Serve + receive'};
  const installed=Object.fromEntries(Object.keys(names).map(name=>[name,{available:true,reason:''}]));
  const render=(field)=>panelHarness('linux','STOPPED',false,{mode:'OPTITRACK',policy_models_age_s:1,policy_models_config_revision:1,policy_models:installed,...field},0,[],{lifecycleConfigRevision:1}).render();
  const tree=render({});
  for(const name of Object.values(names)) assert.equal(button(tree,name).props.disabled,false);
  assert.equal(button(tree,'11999 · Receive only').props.disabled,true);
  for(const field of [{policy_models:undefined},{policy_models_age_s:31},{policy_models_age_s:null},{policy_models_config_revision:2}]) {
    const stale=render(field);
    for(const name of Object.values(names)) assert.equal(button(stale,name).props.disabled,true);
  }
  const broken=render({policy_models:{...installed,model17400:{available:false,reason:'MDU model failed verification'}}});
  assert.equal(button(broken,names.model17400).props.disabled,true);
  assert.equal(button(broken,names.model17400).props.title,'MDU model failed verification');
  const kernel=render({mode:'KERNEL'});
  assert.equal(button(kernel,names.model14600).props.disabled,false);
  for(const model of ['model12600','model17400','model12000']) assert.equal(button(kernel,names[model]).props.disabled,true);
});

test('old saved robot target does not restore a pending switch or masquerade as live identity', () => {
 const live='192.168.200.202';
 const h=panelHarness('linux','STOPPED',false,undefined,0,[],{
  lifecycleConfig:{hdu_wifi_ip:live},
  lastReceived:{'/hope/lifecycle/state':Date.now(),'/hope/lifecycle/config/hdu_wifi_ip':Date.now()},
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
 assert.ok(status.includes('STOPPED'));assert.ok(status.includes('地址尚未配置'));assert.ok(!status.includes('192.168.8.182'));
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
 assert.equal(button(tree,'CONFIRM connection').props.disabled,false);
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
 button(h.render(),'CONFIRM 连接').props.onClick();
 button(h.render(),'English').props.onClick();
 tree=h.render();
 const notice=elements(tree).find(n=>n.props?.className==='operator-notice ');
 assert.match(JSON.stringify(notice),/Enter a valid local IPv4/);
 assert.doesNotMatch(JSON.stringify(notice),/请输入/);
 await button(tree,'RETRY START · clean up automatically').props.onClick();
 assert.match(JSON.stringify(elements(h.render()).find(n=>n.props?.role==='dialog')),/Clean up the failed session/);
 assert.doesNotMatch(JSON.stringify(elements(h.render()).find(n=>n.props?.role==='dialog')),/[\u3400-\u9fff]/);
 assert.equal(h.calls.length,0);
 button(h.render(),'Chinese').props.onClick();
 await button(h.render(),'RETRY START · 自动清理后重试').props.onClick();
 assert.match(JSON.stringify(elements(h.render()).find(n=>n.props?.role==='dialog')),/清理上次失败会话/);
});

function serveHarness() {
  return panelHarness('linux', 'RUNNING', false, undefined, 0, [], {}, {language:'en'});
}
function serveFrame(h, {ui = 1, computer = 1, boot = 42, session = 'test', role = 'SERVER',
  mode = 'PD_STAND', serve = 'IDLE', required = 1, seq = 1} = {}) {
  const values = {
    runnerAlive: true, runnerBoot: boot, runnerFault: false, runnerMode: mode,
    localRole: role, serveCapability: 'AVAILABLE', serveState: serve, lifecycleSession: session,
    mocapSafety: [1, boot, 2279734720943415, seq, 1, 1, 0, 0, 0, 0, required, ui, computer, 0, 0],
    operatorChallenge: [1, boot, 2279734720943415, seq],
  };
  h.frame(Object.entries(values).map(([key, data]) => ({topic:h.api.TOPICS[key], message:{data}})));
}
function action(tree, label) {
  return elements(tree).find(n => n.props?.label === label);
}

test('confirmation stays open beyond watchdog deadline while real onRender continues echoing; one explicit submit', async () => {
  const h = serveHarness(); h.render(); serveFrame(h);
  action(h.render(), 'Start to Serve · Raise hand').props.onClick();
  for(let seq=2;seq<=26;seq++) {
    h.advance(200); serveFrame(h, {seq});
    assert.ok(elements(h.render()).some(n=>n.props?.role==='dialog'));
  }
  assert.equal(h.published.length,26); // five seconds of live challenges, no timer replay
  assert.equal(h.calls.length,0);
  const confirm=button(h.render(),'Confirm and send');
  assert.equal(confirm.props.disabled,false);
  confirm.props.onClick(); confirm.props.onClick();
  await Promise.resolve();
  assert.equal(h.calls.length,1);
  assert.equal(h.calls[0].service,'/hope/runner/prepare_serve');
  assert.ok(!elements(h.render()).some(n=>n.props?.role==='dialog'));
});

test('heartbeat loss blocks submission, recovery never sends automatically, and cancel sends nothing', () => {
  const h=serveHarness(); h.render(); serveFrame(h);
  action(h.render(),'Start to Serve · Raise hand').props.onClick();
  const oldConfirm=button(h.render(),'Confirm and send');
  serveFrame(h,{ui:0,seq:2});
  oldConfirm.props.onClick(); // race: fresh telemetry arrived after the last render
  assert.equal(h.calls.length,0);
  assert.equal(button(h.render(),'Confirm and send').props.disabled,true);
  serveFrame(h,{ui:1,computer:0,seq:3});
  assert.equal(button(h.render(),'Confirm and send').props.disabled,true);
  serveFrame(h,{seq:4});
  assert.equal(button(h.render(),'Confirm and send').props.disabled,false);
  assert.equal(h.calls.length,0);
  button(h.render(),'Cancel').props.onClick();
  oldConfirm.props.onClick();
  assert.equal(h.calls.length,0);
  assert.ok(!elements(h.render()).some(n=>n.props?.role==='dialog'));
});

test('stale status, role changes, new boot/session and seek invalidate pending serve consent', () => {
  for(const change of [h=>h.advance(1001), h=>serveFrame(h,{role:'RECEIVER',seq:2}),
    h=>serveFrame(h,{boot:43,seq:2}), h=>serveFrame(h,{session:'other',seq:2}),
    h=>h.frame([],true)]) {
    const h=serveHarness(); h.render(); serveFrame(h);
    action(h.render(),'Start to Serve · Raise hand').props.onClick();
    const confirm=button(h.render(),'Confirm and send');
    change(h); confirm.props.onClick();
    assert.equal(h.calls.length,0);
  }
});

test('Serve also uses live confirmation; explicit offline watchdog setting remains supported', () => {
  const h=serveHarness(); h.render(); serveFrame(h,{mode:'SERVE',serve:'WAIT_READY_TO_SERVE',required:0,ui:0,computer:0});
  const serve=elements(h.render()).find(n=>n.props?.next && n.props?.onClick && n.props?.label?.includes('Serve'));
  assert.ok(serve); serve.props.onClick();
  const confirm=button(h.render(),'Confirm and send');
  assert.equal(confirm.props.disabled,false);
  confirm.props.onClick();
  assert.equal(h.calls.length,1);
  assert.equal(h.calls[0].service,'/hope/runner/ready_to_serve');
});
