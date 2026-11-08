import { Translator, defaultTranslator } from "./i18n";
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
}, t: Translator = defaultTranslator): string | undefined {
  if (!runnerFresh) { return t("teleopStatus.startTheSystemAndWaitForRunnerStatus"); }
  if (runnerFault) { return t("teleopStatus.runnerReportsACommandFault"); }
  if (estop || xbox?.estop_requested === true) { return t("teleopStatus.resetTheSoftwareEStopBeforeEnteringTeleop"); }
  if (mode === "TELEOP") {
    if (!telemetryFresh || state == undefined) { return t("teleopStatus.teleopSelectedWaitingForItsCurrentPhase"); }
    if (state[3] === 3) { return t("teleopStatus.stoppingAndSettlingTheFeetStandWillBecome"); }
    if (state[3] === 1) { return t("teleopStatus.enteringTeleopWaitForActiveThenReleaseAnd"); }
    if (state[3] === 2) { return t("teleopStatus.teleopIsActiveCenterTheSticksReleaseLt"); }
    return t("teleopStatus.teleopSelectedWaitingForItsCurrentPhase");
  }
  if (mode !== "PD_STAND") { return t("teleopStatus.selectStandBeforeEnteringTeleop"); }
  if (!xboxFresh || xbox?.connected !== true) {
    return t("teleopStatus.connectXboxAndWaitForFreshControllerInput");
  }
  if (xbox.lt || xbox.enabled || xbox.axes.length !== 3 ||
      !xbox.axes.every((x) => Number.isFinite(x) && Math.abs(x) < 1e-6)) {
    return t("teleopStatus.centerTheSticksAndReleaseLtBeforeEntering");
  }
  if (pending || xbox.action_pending === true) { return t("teleopStatus.modeRequestPending"); }
  if (telemetryFresh && state != undefined && state[2] !== 1) {
    return t("teleopStatus.runnerHasNoTeleopPolicyLoaded");
  }
  // This button submits a discrete request, just like Xbox Y. Teleop telemetry
  // is a separate display stream and may lag/reconnect independently. Runner
  // alone checks policy availability, neutral input and its 200 ms watchdog
  // at execution time; no cached input-age sample can grant that authority.
  return undefined;
}
