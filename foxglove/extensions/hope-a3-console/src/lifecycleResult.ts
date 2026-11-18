// Display of /hope/lifecycle/last_result under the START/STOP buttons.
//
// The supervisor sets CONFIG_SAVED_PENDING_RUNTIME_REVISION_<n> after saving a configuration and
// never rewrites it when the runtime later confirms that revision, so the raw string reads as
// "stuck" long after the robot applied it. Config results are therefore shown against the live
// /hope/lifecycle/config/status readback; everything else stays verbatim for diagnosis.

export type LifecycleResultView =
  | { kind: "hidden" }
  | { kind: "config"; revision: number; applied: boolean }
  | { kind: "raw"; text: string };

// CONFIG_UNCHANGED_REVISION_<n>: the operator re-confirmed the saved values (no new revision).
const CONFIG_SAVED = /^CONFIG_(?:SAVED(?:_PENDING_RUNTIME)?|UNCHANGED)_REVISION_(\d+)$/;

export function lifecycleResultView(
  result: string | undefined,
  /** Fresh runtime readback (applied must already include freshness), or undefined. */
  runtime: { applied: boolean; revision: number } | undefined,
): LifecycleResultView {
  if (result == undefined || result === "" || result === "CONFIG_LOADED" || result === "CONFIG_NOT_CONFIRMED") {
    // Nothing happened yet / the config card already says the configuration is unconfirmed.
    return { kind: "hidden" };
  }
  const match = CONFIG_SAVED.exec(result);
  if (match != undefined) {
    const revision = Number(match[1]);
    return { kind: "config", revision, applied: runtime?.applied === true && runtime.revision >= revision };
  }
  return { kind: "raw", text: result };
}
