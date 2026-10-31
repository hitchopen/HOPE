import { ReactElement, useState } from "react";

import { useI18n } from "./LanguageContext";
import { errorText, LocalText } from "./i18n";
import { PlaneMode, PlaneModeState, PLANE_MODES, PLANE_MODE_LABELS } from "./planeMode";

export function PlaneModeControls({ state, available, onApply }: {
  state?: PlaneModeState; available: boolean; onApply: (mode: PlaneMode) => Promise<void>;
}): ReactElement {
  const t = useI18n();
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<LocalText>("");
  const change = async (mode: PlaneMode) => {
    setPending(true); setError("");
    try { await onApply(mode); }
    catch (reason) { setError(errorText(reason)); }
    finally { setPending(false); }
  };
  const metres = (value?: number) => value == undefined ? "—" : `${value.toFixed(3)} m`;
  return <div className="plane-mode-controls">
    <label>{t("planeModeControls.hitPlaneNextBall")} <select aria-label={t("planeModeControls.hitPlaneMode")} disabled={!available || pending || state == undefined}
        value={state?.requested ?? ""} onChange={(event) => void change(event.target.value as PlaneMode)}>
        {state == undefined && <option value="">{t("hitSpeedScaleControls.waitingForPlannerStatus")}</option>}
        {PLANE_MODES.map((mode) => <option key={mode} value={mode}>{t.text(PLANE_MODE_LABELS[mode])}</option>)}
      </select>
    </label>
    <div>{t("planeModeControls.switchAfterStartTheCurrentTargetIsUnchanged")}</div>
    {pending && <div role="status">{t("hitSpeedScaleControls.waitingForPlannerConfirmation")}</div>}
    {state != undefined && <>
      <div>{t("planeModeControls.lastSentBall")}{state.flight > 0 ? state.flight : "—"}：{state.lastCommand == undefined ? t("planeModeControls.noBallSentYet") : t.text(PLANE_MODE_LABELS[state.lastCommand])}</div>
      <div>{t("planeModeControls.referenceX")} {metres(state.referenceX)} {t("planeModeControls.planeX")} {metres(state.planeX)} {t("planeModeControls.policyOffset")} {metres(state.offset)}</div>
      {state.requested !== state.lastCommand && <div>{t("planeModeControls.theSelectedModeAppliesToSubsequentNewBalls")}</div>}
      {state.requested !== "fixed" && !state.supported && <div>{t("planeModeControls.waitingForARunnerSessionSupportingDynamicPlanes")}</div>}
      {state.requested === "stable_per_ball" && <div>{t("planeModeControls.for200MsContinuouslyWaitingHorizontalSpeed3")}{state.stable ? t("planeModeControls.postureCriteriaCurrentlyMet") : t("planeModeControls.otherwiseThePreviousPlaneIsRetained")}</div>}
    </>}
    {t.text(error) !== "" && <div role="alert">{t.text(error)}</div>}
  </div>;
}
