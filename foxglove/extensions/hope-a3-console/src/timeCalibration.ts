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

export function timeCalibrationControls(evidence: CalibrationEvidence): {
  ready: boolean;
  title: string;
  blocksStart: boolean;
} {
  const { state, fresh, busy, ntpFresh, ntpPass, lifecycleStopped, lifecycleBusy, configConfirmed } = evidence;
  const blocksStart = busy || ["RUNNING", "INTERRUPTED", "FAILED_SAFE_STOP"].includes(state);
  let reason: string | undefined;
  if (!fresh) {
    reason = "Waiting for the time-calibration coordinator";
  } else if (state === "FAILED_SAFE_STOP" || state === "INTERRUPTED") {
    reason = "Manual recovery is required before another time calibration";
  } else if (busy || state === "RUNNING") {
    reason = "Time calibration is already running";
  } else if (!ntpFresh || typeof ntpPass !== "boolean") {
    reason = "Waiting for a fresh NTP gate";
  } else if (!lifecycleStopped || lifecycleBusy) {
    reason = "Stop the managed lifecycle before calibrating time";
  } else if (!configConfirmed) {
    reason = "Confirm the three Runner addresses and table side before calibrating time";
  } else if (!["IDLE", "REJECTED", "COMPLETE"].includes(state)) {
    reason = "Waiting for a ready time-calibration state";
  }
  return {
    ready: reason == undefined,
    blocksStart,
    title: reason ?? (ntpPass
      ? "NTP is healthy; manual resynchronization is available with operator confirmation"
      : "Run receipt-safe MDU baseline recovery, then the fixed 10.4 sequence"),
  };
}
