import { defaultTranslator, Translator } from "./i18n";

// Foxglove's CDR decoder delivers Float64MultiArray.data as a typed array.
// JSON data sources deliver an ordinary array; support both representations.
const PHASES = ["IDLE", "ENTRY", "ALIGN", "SETTLE", "VERIFY", "DONE", "FAILED"];
const RESULTS = ["NONE", "RUNNING", "DONE", "FAILED"];
const REASONS = [
  "NONE", "TIMEOUT", "MOCAP_STALE", "MOCAP_INVALID", "WRONG_FRAME",
  "START_TOO_FAR", "ATTEMPTS_EXHAUSTED", "STOPPED",
];

export function decodeDockState(value: unknown): number[] | undefined {
  const values: unknown[] | undefined = Array.isArray(value)
    ? value
    : ArrayBuffer.isView(value) && !(value instanceof DataView)
      ? Array.from(value as unknown as ArrayLike<unknown>)
      : undefined;
  if (values?.length !== 15 || values[0] !== 1 ||
      !values.every((item): item is number => typeof item === "number" && Number.isFinite(item))) {
    return undefined;
  }
  return values;
}

export function dockRepositionReason({ autoDock, runnerFresh, runnerFault, mode, baseFresh,
  serveActive, pending }: {
  autoDock: string | undefined;
  runnerFresh: boolean;
  runnerFault: boolean;
  mode: string | undefined;
  baseFresh: boolean;
  serveActive: boolean;
  pending: boolean;
}, t: Translator = defaultTranslator): string | undefined {
  if (autoDock !== "on") { return t("dockStatus.autoDockIsOff"); }
  if (!runnerFresh) { return t("dockStatus.startTheSystemAndWaitForRunnerStatus"); }
  if (runnerFault) { return t("dockStatus.runnerReportsACommandFault"); }
  if (mode === "DOCK") { return t("dockStatus.repositionIsInProgressStandStopsIt"); }
  if (serveActive) { return t("dockStatus.finishOrAbortTheServeBeforeReposition"); }
  if (mode !== "PD_STAND") { return t("dockStatus.selectStandBeforeReposition"); }
  if (!baseFresh) { return t("dockStatus.waitForAFreshBasePose"); }
  if (pending) { return t("dockStatus.modeRequestPending"); }
  return undefined;
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
