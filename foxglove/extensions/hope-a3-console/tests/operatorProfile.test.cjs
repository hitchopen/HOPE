const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');

// Exercise the actual panel and callbacks without a browser, ROS connection,
// robot, or exported test-only production API. JSX remains a React element tree.
function panelHarness(profile = 'linux', lifecycleState = 'STOPPED', lifecycleBusy = false, field, fieldAgeMs = 0, players = [], extraSnapshot = {}) {
  const state = [];
  let cursor = 0;
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
    useRef: (current) => ({ current: current === api.INITIAL_SNAPSHOT ? { ...current, lifecycleConfig: { ...config } } : current }),
    useMemo: (callback) => callback(),
    useCallback: (callback) => callback,
    useEffect: () => {},
    useLayoutEffect: (callback) => callback(),
  };
  function load(source, extra = '') {
    const exports = {};
    const code = ts.transpileModule(source + extra, {
      compilerOptions: { module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
    }).outputText;
    vm.runInNewContext(code, {
      exports, ArrayBuffer, DataView,
      window: { setTimeout: () => 0 },
      require(name) {
        if (name.endsWith('.css')) return {};
        if (name === 'react') return react;
        if (name.startsWith('./')) {
          const sourcePath = ['.ts', '.tsx'].map((extension) =>
            path.join(__dirname, '../src', name + extension)).find(fs.existsSync);
          if (sourcePath) return load(fs.readFileSync(sourcePath, 'utf8'));
        }
        return require(name);
      },
    });
    return exports;
  }
  api = load(fs.readFileSync(path.join(__dirname, '../src/HopeA3Console.tsx'), 'utf8'),
    '\nexport { HopeA3Console, TOPICS, CONFIG_FIELDS, EMPTY_CONFIG, INITIAL_SNAPSHOT, applyMessage };');
  const context = {
    watch() {},
    subscribe(topics) { subscribed.push(...topics); },
    async callService(service, payload) {
      calls.push({ service, payload });
      return { results: payload.parameters.map(() => ({ successful: true })) };
    },
  };
  return {
    api, calls, subscribed,
    render() { cursor = 0; return api.HopeA3Console({ context }); },
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
  const players = [{x:-.5, receivedAt:Date.now(), sec:1, nanosec:0}];
  for (const verified of [true, false, undefined]) {
    const field = {mode:'OPTITRACK', player_positions: verified == null ? undefined : {P1:verified, P2:false},
      ball_model_status:{state:'CONFIGURED', configured_model:'ppq004', age_s:0, endpoints:{}}};
    const tree = panelHarness('linux','STOPPED',false,field,0,players,{lifecycleConfigRevision:1}).render();
    assert.equal(button(tree,'UCB_P1 · DIRECT').props.disabled, false);
    assert.equal(button(tree,'START SYSTEM').props.disabled, verified !== true);
  }
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
    assert.equal(text.some(t=>t.startsWith('Stopped: mocap lost')), expected);
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
    assert.ok(text.includes('Stopped: mocap lost · controlled PD_STAND recovery'));
    assert.ok(!text.includes('Mocap safety status unavailable'));
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
