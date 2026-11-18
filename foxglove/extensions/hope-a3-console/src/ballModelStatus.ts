import { Translator, defaultTranslator } from "./i18n";

export type BallModelStatus = {
  state: string;
  configured_model: string | null;
  age_s: number | null;
  endpoints: Record<string, { configured_model?: string | null; runtime_model?: string | null;
    runtime: string; mismatches?: string[]; error?: string; verified_at_unix_s?: number;
    verification_age_s?: number; verification_cached?: boolean; verification_scope?: string;
    verification_cache_ttl_s?: number }>;
};

export function ballModelDisplay(status: BallModelStatus | undefined, fieldFresh: boolean,
  t: Translator = defaultTranslator, nowMs: number = Date.now()) {
  if (!fieldFresh || status == undefined || status.age_s == null || !Number.isFinite(status.age_s) || status.age_s > 45) {
    return { value: "", text: t("ballModelStatus.waitingForFreshParameterVerificationFromBothEndpoints"), ready: false };
  }
  const labels: Record<string, string> = { LOADED: t("ballModelStatus.bothEndpointsLoadedAndMatching"), CONFIGURED: t("ballModelStatus.bothEndpointsConfiguredConsistentlyNotAllRunning"),
    MISMATCH: t("ballModelStatus.parameterMismatchNormalStartupBlocked"), UNAVAILABLE: t("ballModelStatus.verificationUnavailable"), CHECKING: t("ballModelStatus.verifying") };
  let hasCached = false;
  let invalidCached = false;
  const parts = [["hdu", "Planner"], ["laptop", t("ballModelStatus.ballBridge")]].map(([key, label]) => {
    const e = status.endpoints[key!];
    if (e == undefined) { return t("ballModelStatus.noReceipt", {endpoint: label}); }
    const model = e.configured_model?.toUpperCase() ?? t("ballModelStatus.unknown");
    let runtime = e.runtime === "VERIFIED" ? t("ballModelStatus.loaded", {model: e.runtime_model?.toUpperCase() ?? model})
      : e.runtime === "NOT_RUNNING" ? t("ballModelStatus.notRunning") : e.runtime === "MISMATCH"
        ? t("ballModelStatus.runtimeParameterMismatch", {model: e.runtime_model?.toUpperCase() ?? t("ballModelStatus.unknownModel")}) : t("ballModelStatus.cannotRead");
    if (e.runtime === "CACHED_VERIFIED") {
      hasCached = true;
      const verifiedAt = e.verified_at_unix_s;
      const measuredAge = e.verification_age_s;
      const ttl = e.verification_cache_ttl_s;
      if (e.verification_cached !== true || e.verification_scope !== "process_identity" ||
          verifiedAt == undefined || !Number.isFinite(verifiedAt) || verifiedAt <= 0 || verifiedAt > nowMs / 1000 + 5 ||
          measuredAge == undefined || !Number.isFinite(measuredAge) || measuredAge < 0 ||
          ttl == undefined || !Number.isFinite(ttl) || ttl <= 0 || ttl > 60 || measuredAge >= ttl) {
        invalidCached = true;
        runtime = t("ballModelStatus.invalidCachedReceipt");
      } else {
        // The outer age measures status delivery, not the parameter query.
        // Unix time also accounts for the other endpoint's sequential query.
        const age = Math.max(measuredAge + status.age_s!, nowMs / 1000 - verifiedAt);
        runtime = t("ballModelStatus.cachedMeasurement", {model: e.runtime_model?.toUpperCase() ?? model,
          time: new Date(verifiedAt * 1000).toLocaleTimeString(undefined, {hour12: false}), age: Math.ceil(age)});
      }
    }
    return t("ballModelStatus.configured", {endpoint: label, model, runtime, error: e.error ? ` · ${e.error}` : ""});
  });
  const ready = ["LOADED", "CONFIGURED"].includes(status.state) && !invalidCached;
  const summary = invalidCached ? t("ballModelStatus.verificationUnavailable")
    : hasCached && ready ? t("ballModelStatus.cachedSummary") : labels[status.state] ?? t("ballModelStatus.unknownVerificationState");
  return { value: status.configured_model ?? "", ready,
    text: `${summary} · ${parts.join("; ")}` };
}
