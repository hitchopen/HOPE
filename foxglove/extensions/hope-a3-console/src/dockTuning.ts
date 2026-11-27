// Reposition (auto-dock) tuning. The Console publishes one request on
// /hope/runner/dock_config_hdu_flat; the HDU gateway forwards it to the Runner,
// which validates it, applies it outside DOCK and saves dock.yaml. The applied
// values and result come back in dock state schema 2 (24 doubles).
export const DOCK_CONFIG_TOPIC = "/hope/runner/dock_config_hdu_flat";

export type DockTuning = {
  targetX: number; targetY: number; targetYawDeg: number;
  tolCm: number; tolYawDeg: number;
  forward: number; backward: number; lateral: number; yawRateDeg: number;
  maxStart: number;
};
export type DockTuningEcho = { values: DockTuning; wireValues: number[]; seq: number; result: number };

export const DOCK_TUNING_FIELDS: { key: keyof DockTuning; min: number; max: number; step: number }[] = [
  { key: "targetX", min: -5, max: 5, step: 0.01 },
  { key: "targetY", min: -5, max: 5, step: 0.01 },
  { key: "targetYawDeg", min: -180, max: 180, step: 1 },
  { key: "tolCm", min: 2, max: 30, step: 1 },
  { key: "tolYawDeg", min: 0.5, max: 20, step: 0.5 },
  { key: "forward", min: 0.05, max: 0.4, step: 0.01 },
  { key: "backward", min: 0.05, max: 0.25, step: 0.01 },
  { key: "lateral", min: 0.05, max: 0.25, step: 0.01 },
  { key: "yawRateDeg", min: 5, max: 60, step: 1 },
  { key: "maxStart", min: 0.1, max: 3, step: 0.1 },
];

const DEG = Math.PI / 180;
const round = (value: number, digits: number) => Number(value.toFixed(digits));

/** Schema-2 dock state carries the current limits; schema 1 (older Runner) does not. */
export function decodeDockTuning(state: number[] | undefined): DockTuningEcho | undefined {
  if (state?.[0] !== 2 || state.length !== 24) { return undefined; }
  const v = (i: number) => state[i] ?? 0;
  return {
    seq: v(15), result: v(16),
    wireValues: [...state.slice(11, 14), ...state.slice(17, 24)],
    values: {
      targetX: round(v(11), 3), targetY: round(v(12), 4), targetYawDeg: round(v(13) / DEG, 1),
      tolCm: round(v(17) * 100, 1), tolYawDeg: round(v(18) / DEG, 1),
      forward: round(v(19), 3), backward: round(v(20), 3), lateral: round(v(21), 3),
      yawRateDeg: round(v(22) / DEG, 1), maxStart: round(v(23), 2),
    },
  };
}

export function dockTuningValid(values: DockTuning): boolean {
  return DOCK_TUNING_FIELDS.every(({ key, min, max }) =>
    Number.isFinite(values[key]) && values[key] >= min && values[key] <= max);
}

/** Wire schema 1: [1, boot, seq, x, y, yaw rad, tol m, tol rad, fwd, back, lat, yaw rad/s, start m]. */
export function encodeDockTuning(boot: number, seq: number, echo: DockTuningEcho, edits: Partial<DockTuning>): number[] {
  // Send untouched fields exactly as reported, including angles rounded for display.
  const values = DOCK_TUNING_FIELDS.map(({ key }, index) => {
    const value = edits[key];
    if (value == undefined) { return echo.wireValues[index]!; }
    if (key === "tolCm") { return value / 100; }
    if (key === "targetYawDeg" || key === "tolYawDeg" || key === "yawRateDeg") { return value * DEG; }
    return value;
  });
  return [1, boot, seq, ...values];
}

export const DOCK_TUNING_RESULTS = ["none", "applied", "invalid", "busy", "notSaved"] as const;
export function dockTuningResult(code: number): (typeof DOCK_TUNING_RESULTS)[number] {
  return DOCK_TUNING_RESULTS[code] ?? "none";
}
