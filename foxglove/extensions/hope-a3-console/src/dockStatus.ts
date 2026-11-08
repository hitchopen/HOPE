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
}): string | undefined {
  if (autoDock !== "on") { return "Auto-dock is off."; }
  if (!runnerFresh) { return "Start the system and wait for Runner status."; }
  if (runnerFault) { return "Runner reports a command fault."; }
  if (mode === "DOCK") { return "Reposition is in progress. Stand stops it and waits for the feet."; }
  if (serveActive) { return "Finish or abort the serve before Reposition."; }
  if (mode !== "PD_STAND") { return "Select Stand before Reposition."; }
  if (!baseFresh) { return "Wait for a fresh base pose."; }
  if (pending) { return "Mode request pending."; }
  return undefined;
}

export function formatDockStatus(state: number[] | undefined): string {
  if (state == undefined) { return "WAITING FOR DOCK TELEMETRY"; }
  const phase = PHASES[state[2] ?? -1] ?? "UNKNOWN";
  const result = RESULTS[state[9] ?? -1] ?? "UNKNOWN";
  const reason = REASONS[state[10] ?? -1] ?? "UNKNOWN";
  const yawDeg = ((state[6] ?? 0) * 180) / Math.PI;
  return `${phase} · dx ${(state[4] ?? 0).toFixed(3)} m · dy ${(state[5] ?? 0).toFixed(3)} m · dyaw ${yawDeg.toFixed(1)} deg · distance ${(state[7] ?? 0).toFixed(3)} m · attempt ${state[3] ?? 0} · ${(state[8] ?? 0).toFixed(1)} s · ${result}${reason === "NONE" ? "" : ` · ${reason}`}`;
}
