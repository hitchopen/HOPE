import { ReactElement, useEffect, useState } from "react";

import { useI18n } from "./LanguageContext";
import { HitSpeedScaleState, parseHitSpeedScale } from "./hitSpeedScale";
import { errorText, LocalText } from "./i18n";

export function HitSpeedScaleControls({ state, available, kernelMode, onApply }: {
  state?: HitSpeedScaleState; available: boolean; kernelMode: boolean;
  onApply: (scale: number) => Promise<void>;
}): ReactElement {
  const t = useI18n();
  const [draft, setDraft] = useState("1");
  const [edited, setEdited] = useState(false);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<LocalText>("");
  useEffect(() => {
    if (!edited && state != undefined) { setDraft(String(state.scale)); }
  }, [state, edited]);
  const scale = parseHitSpeedScale(draft);
  const apply = async () => {
    if (scale == undefined) { return; }
    setPending(true); setError("");
    try { await onApply(scale); setEdited(false); }
    catch (reason) { setError(errorText(reason)); }
    finally { setPending(false); }
  };
  return <div className="plane-mode-controls hit-speed-scale-controls">
    <label>{t("hitSpeedScaleControls.hitSpeedScale")} <input aria-label={t("hitSpeedScaleControls.hitSpeedScale")} type="number" min="0" step="any"
        value={draft} disabled={!available || pending}
        onChange={(event) => { setDraft(event.target.value); setEdited(true); setError(""); }} />
    </label>
    <button type="button" disabled={!available || pending || scale == undefined || state == undefined || scale === state.scale}
      onClick={() => void apply()}>{t("hitSpeedScaleControls.applyScale")}</button>
    <div>{t("hitSpeedScaleControls.default1TargetSpeedScaleChangesApplyTo")}{state?.persistent ? t("hitSpeedScaleControls.persistenceEnabledRestoredAutomaticallyAfterRestart") : t("hitSpeedScaleControls.scalePersistenceIsNotEnabledOnThisRobot")}</div>
    <div>{kernelMode ? t("hitSpeedScaleControls.kernelModeDoesNotUsePlannerThisOption") : state == undefined
      ? t("hitSpeedScaleControls.waitingForPlannerStatus") : t("hitSpeedScaleControls.currentSettingLastSentTarget", {current: state.scale, last: state.lastCommand ?? t("hitSpeedScaleControls.notSentYet")})}</div>
    {edited && scale == undefined && <div role="alert">{t("hitSpeedScaleControls.enterAFiniteNumberGreaterThanOrEqual")}</div>}
    {pending && <div role="status">{t("hitSpeedScaleControls.waitingForPlannerConfirmation")}</div>}
    {t.text(error) !== "" && <div role="alert">{t.text(error)}</div>}
  </div>;
}
