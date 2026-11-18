const assert = require('node:assert/strict');
const test = require('node:test');
const { lifecycleButtons } = require('./loadSource.cjs')('lifecycleButtons');
const { en } = require('./loadSource.cjs')('messages.en');
const { zh } = require('./loadSource.cjs')('messages.zh');

const base = { fresh: true, state: 'STOPPED', lifecycleBusy: false, startPending: false, stopPending: false,
  startBlocked: false, kernel: false };
const view = (patch) => {
  const { start, stop } = lifecycleButtons({ ...base, ...patch });
  return { start: [start.enabled, start.label, start.tone, start.reason], stop: [stop.enabled, stop.label, stop.tone, stop.reason] };
};

test('stopped: start on the left is live, stop on the right is grey with a reason', () => {
  assert.deepEqual(view({}), {
    start: [true, 'console2.startSystem', 'go', undefined],
    stop: [false, 'console2.stopBig', 'off', 'console2.stopNotRunning'],
  });
  assert.deepEqual(view({ kernel: true }).start, [true, 'console2.startKernel', 'kern', undefined]);
  assert.deepEqual(view({ state: 'CONFIG_ERROR' }).start, [true, 'console2.startSystem', 'go', undefined]);
});

test('stopped but a start check fails: start is grey and points at the checks', () => {
  assert.deepEqual(view({ startBlocked: true }), {
    start: [false, 'console2.startSystem', 'off', 'startChecks'],
    stop: [false, 'console2.stopBig', 'off', 'console2.stopNotRunning'],
  });
});

test('running: start is grey, stop is the only live button', () => {
  assert.deepEqual(view({ state: 'RUNNING', startBlocked: true }), {
    start: [false, 'console2.startRunning', 'off', 'console2.startWhileRunning'],
    stop: [true, 'console2.stopBig', 'stop', undefined],
  });
  // A running system never offers start, even if the start checks happen to pass.
  assert.equal(view({ state: 'RUNNING' }).start[0], false);
  // Backend busy while running: the backend would reject the kill, so stop is grey too.
  assert.deepEqual(view({ state: 'RUNNING', lifecycleBusy: true }).stop,
    [false, 'console2.stopBig', 'off', 'console2.stopBackendBusy']);
});

test('starting (backend or a start request in flight): start striped, stop grey', () => {
  for (const patch of [{ state: 'STARTING', lifecycleBusy: true, startBlocked: true }, { startPending: true }]) {
    assert.deepEqual(view(patch), {
      start: [false, 'console2.starting', 'busy', 'console2.lifeWaiting'],
      stop: [false, 'console2.stopBig', 'off', 'console2.stopWhileStarting'],
    });
  }
});

test('stopping (backend or a stop request in flight): stop striped, start grey', () => {
  for (const patch of [{ state: 'KILLING', lifecycleBusy: true, startBlocked: true }, { state: 'RUNNING', stopPending: true }]) {
    assert.deepEqual(view(patch), {
      start: [false, 'console2.startSystem', 'off', 'console2.startWhileStopping'],
      stop: [false, 'console2.killing', 'busy', 'console2.lifeWaiting'],
    });
  }
});

test('failed: retry start and stop (cleanup only) are both live', () => {
  assert.deepEqual(view({ state: 'FAILED' }), {
    start: [true, 'console2.retryStart', 'go', undefined],
    stop: [true, 'console2.stopBig', 'stop', undefined],
  });
  assert.deepEqual(view({ state: 'FAILED', startBlocked: true }).start, [false, 'console2.retryStart', 'off', 'startChecks']);
  assert.deepEqual(view({ state: 'FAILED', lifecycleBusy: true }).stop, [false, 'console2.stopBig', 'off', 'console2.stopBackendBusy']);
});

test('stale or unknown lifecycle: both grey, waiting', () => {
  assert.deepEqual(view({ fresh: false, state: 'RUNNING' }), {
    start: [false, 'console2.startSystem', 'off', 'console2.lifeWaiting'],
    stop: [false, 'console2.stopBig', 'off', 'console2.lifeWaiting'],
  });
  assert.deepEqual(view({ state: undefined, startBlocked: true }).stop, [false, 'console2.stopBig', 'off', 'console2.stopNotRunning']);
});

test('stop is live exactly where the backend accepts kill_all_and_collect', () => {
  const states = ['STOPPED', 'CONFIG_ERROR', 'STARTING', 'RUNNING', 'KILLING', 'FAILED', 'UNKNOWN', undefined];
  for (const state of states) for (const lifecycleBusy of [false, true]) for (const fresh of [false, true])
    for (const startPending of [false, true]) for (const stopPending of [false, true]) for (const startBlocked of [false, true]) {
      const input = { ...base, state, lifecycleBusy, fresh, startPending, stopPending, startBlocked };
      const { start, stop } = lifecycleButtons(input);
      const backendAccepts = fresh && (state === 'RUNNING' || state === 'FAILED') && !lifecycleBusy;
      assert.equal(stop.enabled, backendAccepts && !startPending && !stopPending, JSON.stringify(input));
      const startAllowed = fresh && ['STOPPED', 'CONFIG_ERROR', 'FAILED', 'UNKNOWN', undefined].includes(state);
      assert.equal(start.enabled, startAllowed && !startBlocked && !startPending && !stopPending, JSON.stringify(input));
      // Never both live unless the run failed (retry vs. cleanup only).
      if (start.enabled && stop.enabled) assert.equal(state, 'FAILED');
      // Every grey button explains itself; live buttons carry no reason.
      for (const button of [start, stop]) assert.equal(button.reason == undefined, button.enabled, JSON.stringify(input));
      for (const button of [start, stop]) {
        for (const key of [button.label, button.reason]) {
          if (key == undefined || key === 'startChecks') continue;
          assert.ok(key in en && key in zh, key);
        }
      }
    }
});
