// START (left) and STOP (right) share one row at the bottom of the lifecycle card.
// Exactly one rule decides each button; tests/lifecycleButtons.test.cjs walks every state.
//
// Backend contract (hope_lifecycle_supervisor):
//   start            accepted in STOPPED / CONFIG_ERROR / FAILED (FAILED = clean up and retry)
//   kill_all_and_collect  accepted only in RUNNING / FAILED and only while not busy
// A greyed button always carries a reason (shown as its tooltip); a striped button is the
// action that is currently in progress.

export type LifecycleButtonTone = "go" | "kern" | "stop" | "busy" | "off";

export type StartLabel =
  | "console2.startSystem"
  | "console2.startKernel"
  | "console2.retryStart"
  | "console2.starting"
  | "console2.startRunning";
export type StopLabel = "console2.stopBig" | "console2.killing";
export type LifecycleButtonReason =
  | "console2.lifeWaiting"
  | "console2.startWhileRunning"
  | "console2.startWhileStopping"
  | "console2.stopNotRunning"
  | "console2.stopWhileStarting"
  | "console2.stopBackendBusy"
  /** START is blocked by a start check; the card shows the failing check and its reason. */
  | "startChecks";

export interface LifecycleButton<L> {
  enabled: boolean;
  label: L;
  tone: LifecycleButtonTone;
  /** Set exactly when the button is not enabled. */
  reason?: LifecycleButtonReason;
}

export interface LifecycleButtonInput {
  /** Lifecycle state topic is fresh. */
  fresh: boolean;
  /** Raw backend lifecycle state (RUNNING, STOPPED, ...). */
  state: string | undefined;
  /** Backend reports a lifecycle operation in progress. */
  lifecycleBusy: boolean;
  /** This panel has sent START and is waiting for the reply. */
  startPending: boolean;
  /** This panel has sent STOP and is waiting for the reply. */
  stopPending: boolean;
  /** startBlockReason(...) returned a reason. */
  startBlocked: boolean;
  kernel: boolean;
}

// A function declaration, not a generic arrow: the test loader parses every file as TSX.
function off<L>(label: L, reason: LifecycleButtonReason): LifecycleButton<L> {
  return { enabled: false, label, tone: "off", reason };
}

export function lifecycleButtons(input: LifecycleButtonInput): {
  start: LifecycleButton<StartLabel>;
  stop: LifecycleButton<StopLabel>;
} {
  const idleStart: StartLabel = input.kernel ? "console2.startKernel" : "console2.startSystem";
  const goTone: LifecycleButtonTone = input.kernel ? "kern" : "go";
  if (!input.fresh) {
    return { start: off(idleStart, "console2.lifeWaiting"), stop: off("console2.stopBig", "console2.lifeWaiting") };
  }
  const state = input.state ?? "UNKNOWN";
  if (state === "STARTING" || input.startPending) {
    return {
      start: { enabled: false, label: "console2.starting", tone: "busy", reason: "console2.lifeWaiting" },
      stop: off("console2.stopBig", "console2.stopWhileStarting"),
    };
  }
  if (state === "KILLING" || input.stopPending) {
    return {
      start: off(idleStart, "console2.startWhileStopping"),
      stop: { enabled: false, label: "console2.killing", tone: "busy", reason: "console2.lifeWaiting" },
    };
  }
  if (state === "RUNNING") {
    return {
      start: off("console2.startRunning", "console2.startWhileRunning"),
      stop: input.lifecycleBusy
        ? off("console2.stopBig", "console2.stopBackendBusy")
        : { enabled: true, label: "console2.stopBig", tone: "stop" },
    };
  }
  const failed = state === "FAILED";
  const startLabel: StartLabel = failed ? "console2.retryStart" : idleStart;
  return {
    start: input.startBlocked
      ? off(startLabel, "startChecks")
      : { enabled: true, label: startLabel, tone: goTone },
    stop: !failed
      ? off("console2.stopBig", "console2.stopNotRunning")
      : input.lifecycleBusy
        ? off("console2.stopBig", "console2.stopBackendBusy")
        : { enabled: true, label: "console2.stopBig", tone: "stop" },
  };
}
