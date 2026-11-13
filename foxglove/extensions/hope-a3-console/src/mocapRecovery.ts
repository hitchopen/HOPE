import { Translator, defaultTranslator } from "./i18n";

export type MocapRecovery = {
  state: string;
  attempts: number;
  max_attempts: number;
  retry_after_s: number;
  manual_allowed: boolean;
  blocked_reason: string;
  error: string;
};

export function mocapRecoveryText(status: MocapRecovery | undefined, t: Translator = defaultTranslator): string {
  if (!status) {return t("mocapRecovery.unavailable");}
  const labels = {
    HEALTHY: "mocapRecovery.healthy", WRONG_SIDE: "mocapRecovery.wrongSide",
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

type StartEvidence = {
  lifecycleState: string; lifecycleBusy: boolean; calibrationBusy: boolean;
  configApplied: boolean; configTouched: boolean; fieldBusy: boolean;
  fieldFresh: boolean; kernelMode: boolean; ballReady: boolean;
  playerReady: boolean; side: string;
};

export function startBlockReason(e: StartEvidence, t: Translator = defaultTranslator): string | undefined {
  if (e.lifecycleBusy) {return t("mocapRecovery.startBusy");}
  if (!["STOPPED", "CONFIG_ERROR", "FAILED"].includes(e.lifecycleState)) {return t("mocapRecovery.startState");}
  if (e.calibrationBusy) {return t("mocapRecovery.startCalibration");}
  if (!e.configApplied || e.configTouched) {return t("mocapRecovery.startConfig");}
  if (!e.fieldFresh || e.fieldBusy) {return t("mocapRecovery.startField");}
  // Preserve the explicit FAILED cleanup path and kernel-mode semantics.
  if (!e.kernelMode && e.lifecycleState !== "FAILED") {
    if (!e.ballReady) {return t("mocapRecovery.startBall");}
  }
  return undefined;
}
