import { Translator, defaultTranslator } from "./i18n";

export type MocapRecovery = {
  state: string;
  attempts: number;
  max_attempts: number;
  retry_after_s: number;
  manual_allowed: boolean;
  blocked_reason: string;
  /** Why an operator reconnect is refused now ("" = allowed). Absent on older robots. */
  manual_blocked_reason?: string;
  /** Stable code for manual_blocked_reason: RUNNING, TELEMETRY, BUSY, STOPPING, CONFIG, NONE, OTHER or "". */
  manual_blocked_code?: string;
  error: string;
};

export type MocapConnectionInput = {
  /** Saved mocap system: "motive" | "vrpn" | "none". */
  source: string;
  /** Saved receiver address shown to the operator, e.g. "Motive 169.254.10.221". */
  target: string;
  /** Fresh robot field status; undefined while it is stale or missing. */
  recovery: MocapRecovery | undefined;
  /** Live player poses: any tracked / the selected pants body tracked. */
  anyTracked: boolean;
  selectedTracked: boolean;
  kernelMode: boolean;
  /** A reconnect request from this panel is in flight. */
  requesting: boolean;
  /** Time calibration owns the hardware interlock (the robot would refuse the reconnect). */
  calibrating: boolean;
};

export type MocapConnectionView = {
  tone: "ok" | "warn" | "bad" | "idle" | "busy";
  text: string;
  canReconnect: boolean;
  /** Why the button is disabled (or what it does). */
  buttonTitle: string;
};

const MANUAL_BLOCK_KEYS = {
  RUNNING: "mocapConnection.blockRunning", TELEMETRY: "mocapConnection.blockTelemetry",
  BUSY: "mocapConnection.blockBusy", STOPPING: "mocapConnection.blockStopping",
  CONFIG: "mocapConnection.blockConfig", NONE: "mocapConnection.blockNone",
} as const;

/**
 * The Start card's "动捕连接" row (operator request 2026-11-18): live mocap state in one stable line plus
 * the operator reconnect. The robot decides whether a reconnect may run (manual_allowed); this only
 * explains it. Receiver-process presence is left to the hover text: with Motive unplugged it flaps.
 */
export function mocapConnectionView(input: MocapConnectionInput, t: Translator = defaultTranslator): MocapConnectionView {
  const { source, target, recovery, anyTracked, selectedTracked, kernelMode, requesting, calibrating } = input;
  const reconnecting = requesting || recovery?.state === "RECONNECTING";
  let blockedTitle: string | undefined;
  if (calibrating) {
    blockedTitle = t("mocapConnection.blockCalibrating");
  } else if (recovery == undefined) {
    blockedTitle = t("mocapConnection.blockNoStatus");
  } else if (recovery.manual_blocked_code == undefined) {
    blockedTitle = t("mocapConnection.blockOldRobot");
  } else if (recovery.manual_blocked_code !== "") {
    const code = recovery.manual_blocked_code;
    blockedTitle = Object.prototype.hasOwnProperty.call(MANUAL_BLOCK_KEYS, code)
      ? t(MANUAL_BLOCK_KEYS[code as keyof typeof MANUAL_BLOCK_KEYS])
      : (recovery.manual_blocked_reason ?? code);
  } else if (recovery.retry_after_s > 0) {
    blockedTitle = t("mocapRecovery.cooldown", { seconds: recovery.retry_after_s });
  }
  const canReconnect = !reconnecting && recovery?.manual_allowed === true && blockedTitle == undefined;
  const buttonTitle = reconnecting ? t("mocapRecovery.reconnecting")
    : blockedTitle ?? t("mocapConnection.buttonHint", { target });
  if (source === "none") {
    return { tone: "idle", text: t("mocapConnection.none"), canReconnect: false, buttonTitle: t("mocapConnection.blockNone") };
  }
  if (reconnecting) {
    return { tone: "busy", text: t("mocapRecovery.reconnecting"), canReconnect: false, buttonTitle };
  }
  if (anyTracked) {
    return selectedTracked
      ? { tone: "ok", text: t("mocapConnection.live"), canReconnect, buttonTitle }
      : { tone: "warn", text: t("mocapRecovery.wrongSide"), canReconnect, buttonTitle };
  }
  if (recovery != undefined && recovery.error !== "") {
    return { tone: kernelMode ? "idle" : "bad", text: t("mocapRecovery.failed", { error: recovery.error }), canReconnect, buttonTitle };
  }
  return kernelMode
    ? { tone: "idle", text: t("mocapConnection.noPoseKernel"), canReconnect, buttonTitle }
    : { tone: "warn", text: t("mocapConnection.noPose"), canReconnect, buttonTitle };
}

export function mocapRecoveryText(status: MocapRecovery | undefined, t: Translator = defaultTranslator): string {
  if (!status) {return t("mocapRecovery.unavailable");}
  const labels = {
    HEALTHY: "mocapRecovery.healthy", ASSET_NOT_TRACKED: "mocapRecovery.wrongSide",
    WAITING: "mocapRecovery.waiting", BLOCKED: "mocapRecovery.blocked",
    RECONNECTING: "mocapRecovery.reconnecting", VERIFYING: "mocapRecovery.verifying",
    NEEDS_ATTENTION: "mocapRecovery.attention",
  } as const;
  const text = status.state === "FAILED" ? t("mocapRecovery.failed", { error: status.error })
    : t(Object.prototype.hasOwnProperty.call(labels, status.state)
      ? labels[status.state as keyof typeof labels] : "mocapRecovery.unavailable");
  return text + (status.retry_after_s > 0 && !["HEALTHY", "RECONNECTING"].includes(status.state)
    ? " " + t("mocapRecovery.cooldown", { seconds: status.retry_after_s }) : "");
}

export type StartEvidence = {
  lifecycleState: string; lifecycleBusy: boolean; calibrationBusy: boolean;
  configApplied: boolean; configTouched: boolean; fieldBusy: boolean;
  fieldFresh: boolean; kernelMode: boolean; ballReady: boolean;
  playerReady: boolean; side: string;
  /** Normal mode: every HDU service matches; only the laptop mocap receiver is missing. */
  mocapDown?: boolean;
};

export function startBlockReason(e: StartEvidence, t: Translator = defaultTranslator): string | undefined {
  if (e.lifecycleBusy) {return t("mocapRecovery.startBusy");}
  if (!["STOPPED", "CONFIG_ERROR", "FAILED"].includes(e.lifecycleState)) {return t("mocapRecovery.startState");}
  if (e.calibrationBusy) {return t("mocapRecovery.startCalibration");}
  if (!e.configApplied || e.configTouched) {
    return e.mocapDown === true && !e.configTouched ? t("mocapRecovery.startMocapDown") : t("mocapRecovery.startConfig");
  }
  if (!e.fieldFresh || e.fieldBusy) {return t("mocapRecovery.startField");}
  // Preserve the explicit FAILED cleanup path and kernel-mode semantics.
  if (!e.kernelMode && e.lifecycleState !== "FAILED") {
    if (!e.ballReady) {return t("mocapRecovery.startBall");}
  }
  return undefined;
}

export type StartCheckId = "busy" | "state" | "calibration" | "config" | "field" | "ball";

/**
 * Display-only checklist for the Start card. Each entry is one condition of
 * startBlockReason, in the same order; startBlockReason remains the gate.
 * tests/mocapRecovery.test.cjs proves both agree for every evidence combination.
 */
export function startChecks(e: StartEvidence): { id: StartCheckId; ok: boolean }[] {
  const checks: { id: StartCheckId; ok: boolean }[] = [
    { id: "busy", ok: !e.lifecycleBusy },
    { id: "state", ok: ["STOPPED", "CONFIG_ERROR", "FAILED"].includes(e.lifecycleState) },
    { id: "calibration", ok: !e.calibrationBusy },
    { id: "config", ok: e.configApplied && !e.configTouched },
    { id: "field", ok: e.fieldFresh && !e.fieldBusy },
  ];
  if (!e.kernelMode && e.lifecycleState !== "FAILED") {
    checks.push({ id: "ball", ok: e.ballReady });
  }
  return checks;
}
