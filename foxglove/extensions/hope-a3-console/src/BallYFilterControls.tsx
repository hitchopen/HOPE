import { ReactElement, useEffect, useState } from "react";

import { useI18n } from "./LanguageContext";
import { BallYFilterState, DEFAULT_BALL_Y_FILTER, parseBallYBounds } from "./ballYFilter";
import { BallYFilterDraft, ballYFilterDraftEdited, readBallYFilterDraft } from "./ballYFilterPreview";
import { message as msg, errorText, LocalText } from "./i18n";

const displayY = (value: number) => String(Number(value.toFixed(6)));

export function BallYFilterControls({ state, available, kernelMode, onApply, initialState, onPreview }: {
  state: BallYFilterState | undefined; available: boolean; kernelMode: boolean;
  onApply: (config: typeof DEFAULT_BALL_Y_FILTER) => Promise<void>;
  initialState?: unknown;
  onPreview: (draft: BallYFilterDraft, options: { kernelMode: boolean; edited: boolean }) => void;
}): ReactElement {
  const t = useI18n();
  const [initial] = useState(() => readBallYFilterDraft(initialState));
  const [enabled, setEnabled] = useState(initial.enabled);
  const [min, setMin] = useState(initial.min);
  const [max, setMax] = useState(initial.max);
  const [dirty, setDirty] = useState(() => ballYFilterDraftEdited(initialState));
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<LocalText>("");
  const [previewError, setPreviewError] = useState<LocalText>("");
  useEffect(() => {
    try { onPreview({ enabled, min, max }, { kernelMode, edited: dirty }); setPreviewError(""); }
    catch (cause) { setPreviewError(msg("ballYFilterControls.localPreviewUpdateFailed", {error: errorText(cause)})); }
  }, [enabled, min, max, kernelMode, dirty, onPreview]);
  useEffect(() => {
    if (!dirty && state != undefined) {
      setEnabled(state.enabled); setMin(displayY(state.min)); setMax(displayY(state.max));
    }
  }, [dirty, state]);
  const bounds = parseBallYBounds(min, max);
  const edit = () => { setDirty(true); setError(""); };
  const apply = async () => {
    if (bounds == undefined || !available || pending) { return; }
    setPending(true); setError("");
    try { await onApply({ enabled, ...bounds }); setDirty(false); }
    catch (cause) { setError(errorText(cause)); }
    finally { setPending(false); }
  };
  return <div className="ball-y-filter" aria-label={t("ballYFilterControls.plannerYFilter")}>
    <label className="ball-y-filter-toggle">
      <input type="checkbox" checked={enabled} disabled={pending || kernelMode}
        onChange={(event) => { edit(); setEnabled(event.target.checked); }} />
      {t("ballYFilterControls.ppp3Ppp4YFilterDraftApplyToActivate")} <span className={state?.enabled === true ? "filter-on" : "filter-off"}>
        {state == undefined ? t("ballYFilterControls.waitingForPlannerFeedbackActiveStateUnknown") : state.enabled ? t("ballYFilterControls.plannerFilterEnabled") : t("ballYFilterControls.plannerFilterDisabled")}
      </span>
    </label>
    <div className="ball-y-filter-inputs">
      <label>{t("ballYFilterControls.leftBoundaryYM")}<input type="number" step="0.01" value={max} disabled={pending || kernelMode}
        aria-label={t("ballYFilterControls.leftBoundaryYM")} onChange={(event) => { edit(); setMax(event.target.value); }} /></label>
      <label>{t("ballYFilterControls.rightBoundaryYM")}<input type="number" step="0.01" value={min} disabled={pending || kernelMode}
        aria-label={t("ballYFilterControls.rightBoundaryYM")} onChange={(event) => { edit(); setMin(event.target.value); }} /></label>
    </div>
    <div className="ball-y-filter-actions">
      <button type="button" disabled={pending || !available || bounds == undefined || !dirty}
        onClick={() => void apply()}>{pending ? t("ballYFilterControls.applying") : t("ballYFilterControls.applyToPlanner")}</button>
      <button type="button" disabled={pending || kernelMode} onClick={() => {
        edit(); setEnabled(DEFAULT_BALL_Y_FILTER.enabled);
        setMin(displayY(DEFAULT_BALL_Y_FILTER.min)); setMax(displayY(DEFAULT_BALL_Y_FILTER.max));
      }}>{t("ballYFilterControls.restoreDefaults")}</button>
    </div>
    <small role="status">
      {t.text(previewError) || t.text(error) || (bounds == undefined ? t("ballYFilterControls.enterValidNumbersRightBoundaryYMustBe") : kernelMode
        ? t("ballYFilterControls.kernelModeDoesNotUseThePlannerFilter") : !available
          ? t("ballYFilterControls.localPreviewNeedsNoHduConnectToPlanner") : dirty
            ? t("ballYFilterControls.theEnteredRangeIsPreviewedOnTheLeft")
            : t("ballYFilterControls.plannerConfirmed", {persistence: state?.persistent === true ? t("ballYFilterControls.settingsSaved") : t("ballYFilterControls.currentSession")}))}
    </small>
    <small>{t("ballYFilterControls.localPreview")}{!enabled ? t("ballYFilterControls.hidden") : bounds == undefined || kernelMode ? t("ballYFilterControls.unavailable") : t("ballYFilterControls.leftRightM", {left: bounds.max, right: bounds.min})} {t("ballYFilterControls.yDecreasesToTheRight")}</small>
    {state != undefined && <small>{t("ballYFilterControls.activePlannerFilter")}{state.enabled ? t("ballYFilterControls.leftRightM", {left: state.max.toFixed(5), right: state.min.toFixed(5)}) : t("ballYFilterControls.off")}</small>}
  </div>;
}
