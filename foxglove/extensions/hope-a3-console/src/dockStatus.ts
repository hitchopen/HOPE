import { defaultTranslator, Translator } from "./i18n";

// Reposition (auto-dock) telemetry: /hope/runner/dock_state_hdu_flat, schema 1,
// 15 doubles: schema, boot, phase, attempt, error x/y/yaw, distance, elapsed,
// result, reason, target x/y/yaw, pose age. Published only when the Runner
// loaded --auto-dock, so fresh telemetry also means Reposition is available.
const PHASES = ["IDLE", "ENTRY", "ALIGN", "SETTLE", "VERIFY", "DONE", "FAILED"] as const;
const RESULTS = ["NONE", "RUNNING", "DONE", "FAILED"] as const;
const REASONS = [
  "NONE", "TIMEOUT", "MOCAP_STALE", "MOCAP_INVALID", "WRONG_FRAME",
  "START_TOO_FAR", "ATTEMPTS_EXHAUSTED", "STOPPED",
] as const;

// Foxglove's CDR decoder delivers Float64MultiArray.data as a typed array;
// JSON data sources deliver an ordinary array.
export function decodeDockState(value: unknown): number[] | undefined {
  const values: unknown[] | undefined = Array.isArray(value)
    ? value
    : ArrayBuffer.isView(value) && !(value instanceof DataView)
      ? Array.from(value as unknown as ArrayLike<unknown>)
      : undefined;
  // Schema 1: 15 doubles. Schema 2 appends tuning seq/result and 7 limits (24).
  if (!((values?.length === 15 && values[0] === 1) || (values?.length === 24 && values[0] === 2)) ||
      !values.every((item): item is number => typeof item === "number" && Number.isFinite(item))) {
    return undefined;
  }
  return values;
}

export function dockRepositionReason({ runnerFresh, runnerFault, mode, telemetryFresh, baseFresh,
  serving, pending, estop }: {
  runnerFresh: boolean;
  runnerFault: boolean;
  mode: string | undefined;
  telemetryFresh: boolean;
  baseFresh: boolean;
  serving: boolean;
  pending: boolean;
  estop: boolean;
}, t: Translator = defaultTranslator): string | undefined {
  if (!runnerFresh) { return t("dockStatus.startTheSystemAndWaitForRunnerStatus"); }
  if (runnerFault) { return t("dockStatus.runnerReportsACommandFault"); }
  if (estop) { return t("dockStatus.resetTheSoftwareEStopFirst"); }
  if (!telemetryFresh) { return t("dockStatus.runnerHasNoRepositionLoaded"); }
  if (mode === "DOCK") { return t("dockStatus.repositionIsInProgressStandStopsIt"); }
  if (serving) { return t("dockStatus.finishOrAbortTheServeBeforeReposition"); }
  if (mode !== "PD_STAND" && mode !== "WALK_STAND" && mode !== "TELEOP") { return t("dockStatus.selectStandBeforeReposition"); }
  if (!baseFresh) { return t("dockStatus.waitForAFreshBasePose"); }
  if (pending) { return t("dockStatus.modeRequestPending"); }
  return undefined;
}

export function dockPhaseName(state: number[] | undefined): string | undefined {
  return state == undefined ? undefined : PHASES[state[2] ?? -1] ?? "UNKNOWN";
}

export function formatDockStatus(state: number[] | undefined, t: Translator = defaultTranslator): string {
  if (state == undefined) { return t("dockStatus.waitingForDockTelemetry"); }
  const phase = PHASES[state[2] ?? -1] ?? "UNKNOWN";
  const result = RESULTS[state[9] ?? -1] ?? "UNKNOWN";
  const reason = REASONS[state[10] ?? -1] ?? "UNKNOWN";
  const yawDeg = ((state[6] ?? 0) * 180) / Math.PI;
  return t("dockStatus.line", {
    phase, dx: (state[4] ?? 0).toFixed(3), dy: (state[5] ?? 0).toFixed(3),
    dyaw: yawDeg.toFixed(1), distance: (state[7] ?? 0).toFixed(3),
    attempt: state[3] ?? 0, elapsed: (state[8] ?? 0).toFixed(1), result,
    reason: reason === "NONE" ? "" : ` · ${reason}`,
  });
}
