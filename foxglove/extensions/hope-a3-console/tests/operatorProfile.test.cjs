const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const ts = require('typescript');

// Exercise the actual panel and callbacks without a browser, ROS connection,
// robot, or exported test-only production API. JSX remains a React element tree.
function panelHarness(profile = 'linux', lifecycleState = 'STOPPED', lifecycleBusy = false) {
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
            lifecycleConfig: { ...config },
            lastReceived: { [api.TOPICS.lifecycleState]: Date.now() },
          } : initial;
      }
      return [state[index], (value) => {
        state[index] = typeof value === 'function' ? value(state[index]) : value;
      }];
    },
    useRef: (current) => ({ current }),
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
