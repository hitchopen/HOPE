const assert = require('node:assert/strict');
const test = require('node:test');
const { decodeDockTuning, dockTuningValid, encodeDockTuning, dockTuningResult } = require('./loadSource.cjs')('dockTuning');
const { decodeDockState } = require('./loadSource.cjs')('dockStatus');
const DEG = Math.PI / 180;
const schema2 = [2, 77, 5, 0, 0, 0, 0, 0.07, 5.6, 2, 0, -0.5, -0.7625, 0, 0.02,
  1700000000000, 1, 0.1, 5 * DEG, 0.2, 0.1, 0.1, 0.3, 1.5];

test('schema 2 dock state decodes for both status and tuning', () => {
  assert.equal(decodeDockState(schema2)?.length, 24);
  const echo = decodeDockTuning(schema2);
  assert.equal(echo.seq, 1700000000000);
  assert.equal(dockTuningResult(echo.result), 'applied');
  assert.equal(echo.values.targetX, -0.5);
  assert.equal(echo.values.tolCm, 10);
  assert.equal(echo.values.tolYawDeg, 5);
  assert.equal(echo.values.yawRateDeg, 17.2);
  assert.equal(decodeDockTuning([1, ...schema2.slice(1, 15)]), undefined);
});

test('encode converts units and validation follows the Runner envelope', () => {
  const values = { ...decodeDockTuning(schema2).values, targetX: -0.45, tolCm: 8 };
  assert.equal(dockTuningValid(values), true);
  const wire = encodeDockTuning(77, 9, values);
  assert.equal(wire.length, 13);
  assert.deepEqual([...wire.slice(0, 5)], [1, 77, 9, -0.45, -0.7625]);
  assert.ok(Math.abs(wire[6] - 0.08) < 1e-12);
  assert.ok(Math.abs(wire[7] - 5 * DEG) < 1e-12);
  for (const bad of [{ tolCm: 1 }, { forward: 0.5 }, { maxStart: 4 }, { targetY: Number.NaN }]) {
    assert.equal(dockTuningValid({ ...values, ...bad }), false);
  }
});
