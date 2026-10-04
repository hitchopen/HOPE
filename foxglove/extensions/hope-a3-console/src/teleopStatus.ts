// Foxglove's CDR decoder delivers Float64MultiArray.data as a typed array.
// JSON data sources deliver an ordinary array; support both representations.
export function decodeTeleopState(value: unknown): number[] | undefined {
  const values: unknown[] | undefined = Array.isArray(value)
    ? value
    : ArrayBuffer.isView(value) && !(value instanceof DataView)
      ? Array.from(value as unknown as ArrayLike<unknown>)
      : undefined;
  if (values?.length !== 10 || values[0] !== 1 ||
      !values.every((x): x is number => typeof x === "number" && Number.isFinite(x))) {
    return undefined;
  }
  return values;
}

export function xboxInputFresh(receivedAt: number | undefined, sourceWallMs: number | undefined,
  now: number): boolean {
  return receivedAt != undefined && sourceWallMs != undefined &&
    now - receivedAt >= 0 && now - receivedAt <= 500 &&
    now - sourceWallMs >= -100 && now - sourceWallMs < 500;
}

export type TeleopXboxInput = {
  connected: boolean;
  enabled: boolean;
  lt: boolean;
  axes: number[];
  action_pending?: boolean;
  estop_requested?: boolean;
};

export function teleopEntryReason({ runnerFresh, runnerFault, mode, telemetryFresh, state, pending,
  xboxFresh, xbox, estop }: {
  runnerFresh: boolean;
  runnerFault: boolean;
  mode: string | undefined;
  telemetryFresh: boolean;
  state: number[] | undefined;
  pending: boolean;
  xboxFresh: boolean;
  xbox: TeleopXboxInput | undefined;
  estop: boolean;
}): string | undefined {
  if (!runnerFresh) { return "Start the system and wait for Runner status."; }
  if (runnerFault) { return "Runner reports a command fault."; }
  if (estop || xbox?.estop_requested === true) { return "Reset the software E-stop before entering Teleop."; }
  if (mode === "TELEOP") {
    if (!telemetryFresh || state == undefined) { return "Teleop selected; waiting for its current phase."; }
    if (state[3] === 3) { return "Stopping and settling the feet; Stand will become available automatically."; }
    if (state[3] === 1) { return "Entering Teleop; wait for ACTIVE, then release and hold LT."; }
    if (state[3] === 2) { return "Teleop is ACTIVE. Center the sticks, release LT, then hold LT to move."; }
    return "Teleop selected; waiting for its current phase.";
  }
  if (mode !== "PD_STAND") { return "Select Stand before entering Teleop."; }
  if (!xboxFresh || xbox?.connected !== true) {
    return "Connect Xbox and wait for fresh controller input.";
  }
  if (xbox.lt || xbox.enabled || xbox.axes.length !== 3 ||
      !xbox.axes.every((x) => Number.isFinite(x) && Math.abs(x) < 1e-6)) {
    return "Center the sticks and release LT before entering Teleop.";
  }
  if (pending || xbox.action_pending === true) { return "Mode request pending."; }
  if (telemetryFresh && state != undefined && state[2] !== 1) {
    return "Runner has no teleop policy loaded.";
  }
  // This button submits a discrete request, just like Xbox Y. Teleop telemetry
  // is a separate display stream and may lag/reconnect independently. Runner
  // alone checks policy availability, neutral input and its 200 ms watchdog
  // at execution time; no cached input-age sample can grant that authority.
  return undefined;
}
