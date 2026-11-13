const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const test = require("node:test");
const ts = require("typescript");
function harness(component, initialState) {
  const states = [],
    refs = [],
    deps = [],
    pending = [],
    added = [],
    variables = [],
    subscriptions = [];
  let si = 0,
    ri = 0,
    ei = 0;
  const react = {
    ...require("react"),
    useCallback: (fn) => fn,
    useState(initial) {
      const i = si++;
      if (!(i in states)) states[i] = initial;
      return [
        states[i],
        (v) => {
          states[i] = typeof v === "function" ? v(states[i]) : v;
        },
      ];
    },
    useRef(initial) {
      const i = ri++;
      return refs[i] ?? (refs[i] = { current: initial });
    },
    useEffect(fn, ds) {
      const i = ei++;
      if (!deps[i] || ds.some((v, j) => v !== deps[i][j])) {
        deps[i] = ds;
        pending.push(fn);
      }
    },
  };
  const exports = {};
  vm.runInNewContext(
    ts.transpileModule(
      fs.readFileSync(path.join(__dirname, "../src/ReplayTelemetry.tsx"), "utf8") +
        "\nexport { ReplayTelemetry, ReplayViewControls };",
      {
        compilerOptions: {
          module: ts.ModuleKind.CommonJS,
          target: ts.ScriptTarget.ES2020,
          jsx: ts.JsxEmit.ReactJSX,
        },
      },
    ).outputText,
    {
      exports,
      require: (name) => (name.endsWith(".css") ? {} : name === "react" ? react : require(name)),
    },
  );
  const context = {
    initialState,
    setDefaultPanelTitle() {},
    watch() {},
    subscribe(t) {
      subscriptions.push(...t);
    },
    unsubscribeAll() {},
    setVariable(k, v) {
      variables.push([k, v]);
    },
    layout: {
      addPanel(v) {
        added.push(v);
      },
    },
  };
  const render = () => {
    si = ri = ei = 0;
    const tree = exports[component]({ context });
    while (pending.length) pending.shift()();
    return tree;
  };
  render();
  return {
    context,
    added,
    variables,
    subscriptions,
    render,
    frame(s) {
      let ack = 0;
      context.onRender(s, () => ++ack);
      const tree = render();
      assert.equal(ack, 1);
      return tree;
    },
  };
}
function elements(n) {
  return Array.isArray(n)
    ? n.flatMap(elements)
    : n && typeof n === "object"
      ? [n, ...elements(n.props?.children)]
      : [];
}
const text = (n) =>
  Array.isArray(n)
    ? n.map(text).join(" ")
    : n && typeof n === "object"
      ? text(n.props?.children)
      : n == null
        ? ""
        : String(n);
const click = (tree, label) =>
  elements(tree)
    .find((e) => e.type === "button" && text(e) === label)
    .props.onClick();
const telemetry = {
  timestamp_s: 100,
  shot: 5,
  robot_xyz: [1, 2, 3],
  robot_rpy_deg: [10, 20, 30],
  racket_xyz: [0, 0, 0],
  racket_speed: 2,
  target_xyz: [0.1, 0.2, 0.3],
  frozen: {
    timestamp_s: 99.9,
    ball_xyz: [0.1, 0.2, 0.3],
  },
  policy_frozen: {
    timestamp_s: 99.95,
    racket_xyz: [0.2, 0.3, 0.4],
    delta_xyz: [0.1, 0.1, 0.1],
    sample_age_ms: 5,
  },
};
const event = { topic: "/hope/replay/telemetry", message: { data: JSON.stringify(telemetry) } };
test("recorded numeric panel uses playback time and clears data across seek or stale intervals", () => {
  const h = harness("ReplayTelemetry");
  assert.deepEqual(
    h.subscriptions.map((x) => x.topic),
    ["/hope/replay/telemetry"],
  );
  let tree = h.frame({ currentFrame: [event], currentTime: { sec: 100, nsec: 0 } });
  assert.match(text(tree), /1.000  2.000  3.000/);
  assert.match(text(tree), /100.0  100.0  100.0/);
  tree = h.frame({ currentFrame: [], currentTime: { sec: 100, nsec: 0 } });
  assert.match(text(tree), /历史 # 5/);
  tree = h.frame({ currentFrame: [], currentTime: { sec: 100, nsec: 300000000 } });
  assert.doesNotMatch(text(tree), /1.000  2.000  3.000/);
  h.frame({ currentFrame: [event], currentTime: { sec: 100, nsec: 0 } });
  tree = h.frame({ didSeek: true, currentFrame: [], currentTime: { sec: 90, nsec: 0 } });
  assert.doesNotMatch(text(tree), /1.000  2.000  3.000/);
});
test("all-view reset only broadcasts camera reset and does not change timeline or control state", () => {
  const h = harness("ReplayTelemetry");
  click(h.render(), "↺ 重置三视");
  click(h.render(), "↺ 重置三视");
  assert.equal(h.added.length, 0);
  assert.equal(h.variables.length, 2);
  assert.equal(h.variables[0][0], "hopeReplayCameraReset");
  assert.notEqual(h.variables[0][1], h.variables[1][1]);
});
test("local camera reset preserves full existing 3D config and handles each broadcast only once", () => {
  const cameraState = { distance: 5, perspective: true };
  const h = harness("ReplayViewControls", { view: "侧视", cameraState });
  h.frame({ variables: new Map([["hopeReplayCameraReset", "one"]]) });
  assert.equal(h.added.length, 1);
  const req = h.added[0];
  assert.equal(req.type, "3D");
  assert.equal(req.position, "sibling");
  assert.equal(req.updateIfExists, true);
  const old = {
    topics: { ball: { visible: true } },
    followTf: "world",
    cameraState: { distance: 99 },
    custom: "preserved",
  };
  assert.equal(JSON.stringify(req.getState(old)), JSON.stringify({ ...old, cameraState }));
  assert.equal(old.cameraState.distance, 99);
  h.frame({ variables: new Map([["hopeReplayCameraReset", "one"]]) });
  assert.equal(h.added.length, 1);
  h.frame({ variables: new Map([["hopeReplayCameraReset", "two"]]) });
  assert.equal(h.added.length, 2);
  assert.equal(h.render(), null);
});

for (const component of ["ReplayTelemetry", "ReplayViewControls"]) {
  test(component + " acknowledges every frame even when Foxglove reuses renderDone", () => {
    const h = harness(component, { view: "侧视", cameraState: { distance: 5 } });
    let acknowledged = 0;
    const done = () => {
      ++acknowledged;
    };
    for (let i = 0; i < 3; ++i) {
      h.context.onRender(
        { currentFrame: [event], currentTime: { sec: 100, nsec: 0 }, variables: new Map() },
        done,
      );
      h.render();
      assert.equal(acknowledged, i + 1);
      h.render();
      assert.equal(acknowledged, i + 1);
    }
  });
}

test("scheduled-hit policy ghost is independent of observed ball crossing", () => {
  const h = harness("ReplayTelemetry");
  const draw = (data) =>
    h.frame({
      currentFrame: [{ ...event, message: { data: JSON.stringify(data) } }],
      currentTime: { sec: 100, nsec: 0 },
    });
  let tree = draw({ ...telemetry, frozen: null });
  assert.match(text(tree), /0.200  0.300  0.400/);
  assert.match(text(tree), /100.0  100.0  100.0/);
  assert.match(text(tree), /等待过平面/);
  assert.doesNotMatch(text(tree), /等待击球时刻/);
  tree = draw({ ...telemetry, policy_frozen: null });
  assert.doesNotMatch(text(tree), /0.200  0.300  0.400/);
  assert.match(text(tree), /等待击球时刻/);
});

test("missing observations retain the logical ball state and handedness is explicit", () => {
  const h = harness("ReplayTelemetry");
  const data = {
    ...telemetry,
    target_swing_sign: 1,
    policy_swing_sign: 1,
    play_state: {
      state: "OPPONENT_RALLY",
      reason: "ball_observation_missing",
      observation_missing: true,
      forwarding: false,
    },
  };
  let tree = h.frame({
    currentFrame: [{ ...event, message: { data: JSON.stringify(data) } }],
    currentTime: { sec: 100, nsec: 0 },
  });
  assert.match(text(tree), /对方回球/);
  assert.match(text(tree), /观测缺失 · 状态保留/);
  assert.match(text(tree), /转发暂停/);
  assert.match(text(tree), /目标\s+正手\s+· Runner 标记\s+正手/);
  assert.equal(elements(tree).filter((e) => e.type === "button").length, 1);
  data.play_state = {
    state: "INVALID",
    reason: "double_bounce",
    observation_missing: false,
    forwarding: false,
  };
  tree = h.frame({
    currentFrame: [{ ...event, message: { data: JSON.stringify(data) } }],
    currentTime: { sec: 100, nsec: 0 },
  });
  assert.match(text(tree), /double_bounce/);
});
