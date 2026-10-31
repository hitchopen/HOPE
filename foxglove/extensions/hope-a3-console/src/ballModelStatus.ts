import { Translator, defaultTranslator } from "./i18n";

export type BallModelStatus = {
  state: string;
  configured_model: string | null;
  age_s: number | null;
  endpoints: Record<string, { configured_model?: string | null; runtime_model?: string | null;
    runtime: string; mismatches?: string[]; error?: string }>;
};

export function ballModelDisplay(status: BallModelStatus | undefined, fieldFresh: boolean, t: Translator = defaultTranslator) {
  if (!fieldFresh || status == undefined || status.age_s == null || !Number.isFinite(status.age_s) || status.age_s > 45) {
    return { value: "", text: t("ballModelStatus.waitingForFreshParameterVerificationFromBothEndpoints"), ready: false };
  }
  const labels: Record<string, string> = { LOADED: t("ballModelStatus.bothEndpointsLoadedAndMatching"), CONFIGURED: t("ballModelStatus.bothEndpointsConfiguredConsistentlyNotAllRunning"),
    MISMATCH: t("ballModelStatus.parameterMismatchNormalStartupBlocked"), UNAVAILABLE: t("ballModelStatus.verificationUnavailable"), CHECKING: t("ballModelStatus.verifying") };
  const parts = [["hdu", "Planner"], ["laptop", t("ballModelStatus.ballBridge")]].map(([key, label]) => {
    const e = status.endpoints[key!];
    if (e == undefined) { return t("ballModelStatus.noReceipt", {endpoint: label}); }
    const model = e.configured_model?.toUpperCase() ?? t("ballModelStatus.unknown");
    const runtime = e.runtime === "VERIFIED" ? t("ballModelStatus.loaded", {model: e.runtime_model?.toUpperCase() ?? model})
      : e.runtime === "NOT_RUNNING" ? t("ballModelStatus.notRunning") : e.runtime === "MISMATCH"
        ? t("ballModelStatus.runtimeParameterMismatch", {model: e.runtime_model?.toUpperCase() ?? t("ballModelStatus.unknownModel")}) : t("ballModelStatus.cannotRead");
    return t("ballModelStatus.configured", {endpoint: label, model, runtime, error: e.error ? ` · ${e.error}` : ""});
  });
  const ready = ["LOADED", "CONFIGURED"].includes(status.state);
  return { value: status.configured_model ?? "", ready,
    text: `${labels[status.state] ?? t("ballModelStatus.unknownVerificationState")} · ${parts.join("; ")}` };
}
