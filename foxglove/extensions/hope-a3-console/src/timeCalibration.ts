import { Translator, defaultTranslator } from "./i18n";

type CalibrationEvidence = {
  state: string;
  fresh: boolean;
  busy: boolean;
  ntpFresh: boolean;
  ntpPass?: boolean;
  lifecycleStopped: boolean;
  lifecycleBusy: boolean;
  configConfirmed: boolean;
};

export function timeCalibrationControls(evidence: CalibrationEvidence, t: Translator = defaultTranslator): {
  ready: boolean;
  title: string;
  blocksStart: boolean;
} {
  const { state, fresh, busy, ntpFresh, ntpPass, lifecycleStopped, lifecycleBusy, configConfirmed } = evidence;
  const blocksStart = busy || ["RUNNING", "INTERRUPTED", "FAILED_SAFE_STOP"].includes(state);
  let reason: string | undefined;
  if (!fresh) {
    reason = t("timeCalibration.waitingForTheTimeCalibrationCoordinator");
  } else if (state === "FAILED_SAFE_STOP" || state === "INTERRUPTED") {
    reason = t("timeCalibration.manualRecoveryIsRequiredBeforeAnotherTimeCalibration");
  } else if (busy || state === "RUNNING") {
    reason = t("timeCalibration.timeCalibrationIsAlreadyRunning");
  } else if (!ntpFresh || typeof ntpPass !== "boolean") {
    reason = t("timeCalibration.waitingForAFreshNtpGate");
  } else if (!lifecycleStopped || lifecycleBusy) {
    reason = t("timeCalibration.stopTheManagedLifecycleBeforeCalibratingTime");
  } else if (!configConfirmed) {
    reason = t("timeCalibration.confirmTheThreeRunnerAddressesAndTableSide");
  } else if (!["IDLE", "REJECTED", "COMPLETE"].includes(state)) {
    reason = t("timeCalibration.waitingForAReadyTimeCalibrationState");
  }
  return {
    ready: reason == undefined,
    blocksStart,
    title: reason ?? (ntpPass
      ? t("timeCalibration.ntpIsHealthyManualResynchronizationIsAvailableWith")
      : t("timeCalibration.runReceiptSafeMduBaselineRecoveryThenThe")),
  };
}
