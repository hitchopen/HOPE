import { defaultTranslator, Translator } from "./i18n";

// WALK_STAND is the Runner's HumanLike locomotion policy at zero velocity. It
// shares the teleop telemetry stream: state[2] = policy loaded, state[3] =
// phase (0 off, 1 entering, 2 active, 3 stopping).
export type WalkStandPhase = "entering" | "active" | "stopping" | "waiting";

export function walkStandPhase(mode: string | undefined, state: number[] | undefined): WalkStandPhase | undefined {
  if (mode !== "WALK_STAND") { return undefined; }
  const phase = state?.[3];
  if (phase === 1) { return "entering"; }
  if (phase === 2) { return "active"; }
  if (phase === 3) { return "stopping"; }
  return "waiting";
}

export function walkStandEntryReason({ runnerFresh, runnerFault, mode, telemetryFresh, state,
  pending, estop }: {
  runnerFresh: boolean;
  runnerFault: boolean;
  mode: string | undefined;
  telemetryFresh: boolean;
  state: number[] | undefined;
  pending: boolean;
  estop: boolean;
}, t: Translator = defaultTranslator): string | undefined {
  if (!runnerFresh) { return t("walkStand.startTheSystemAndWaitForRunnerStatus"); }
  if (runnerFault) { return t("walkStand.runnerReportsACommandFault"); }
  if (estop) { return t("walkStand.resetTheSoftwareEStopFirst"); }
  if (mode === "WALK_STAND") { return t("walkStand.alreadyActiveStandReturnsToPdStand"); }
  if (mode !== "PD_STAND") { return t("walkStand.selectStandFirst"); }
  if (pending) { return t("walkStand.modeRequestPending"); }
  // Runner alone decides at execution time; stale telemetry does not block.
  if (telemetryFresh && state != undefined && state[2] !== 1) {
    return t("walkStand.runnerHasNoHumanLikePolicyLoaded");
  }
  return undefined;
}
