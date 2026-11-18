import { ReactElement, useEffect, useState } from "react";

import { useI18n } from "./LanguageContext";
import { FlightTimeState, parseFlightTime } from "./flightTime";
import { errorText, LocalText } from "./i18n";

export function FlightTimeControls({ state, available, kernelMode, onApply }: {
  state?: FlightTimeState; available: boolean; kernelMode: boolean;
  onApply: (seconds: number) => Promise<void>;
}): ReactElement {
  const t = useI18n();
  const [draft, setDraft] = useState("0.5");
  const [edited, setEdited] = useState(false);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<LocalText>("");
  useEffect(() => {
    if (!edited && state != undefined) { setDraft(String(state.seconds)); }
  }, [state, edited]);
  const seconds = parseFlightTime(draft);
  const valid = seconds != undefined && state != undefined && seconds <= state.maxSeconds;
  const apply = async () => {
    if (!valid) { return; }
    setPending(true); setError("");
    try { await onApply(seconds); setEdited(false); }
    catch (reason) { setError(errorText(reason)); }
    finally { setPending(false); }
  };
  return <div className="plane-mode-controls flight-time-controls">
    <label>{t("flightTime.label")} <input aria-label={t("flightTime.label")} type="number" min="0" max={state?.maxSeconds} step="0.01"
      value={draft} disabled={!available || pending}
      onChange={(event) => { setDraft(event.target.value); setEdited(true); setError(""); }} /></label>
    <button type="button" disabled={!available || pending || !valid ||
      (seconds === state.forehand && seconds === state.backhand)} onClick={() => void apply()}>{t("flightTime.apply")}</button>
    <div>{t("flightTime.help")}</div>
    <div>{state?.persistent === true ? t("hitSpeedScaleControls.persistenceEnabledRestoredAutomaticallyAfterRestart") : t("flightTime.notPersistent")}</div>
    <div>{kernelMode ? t("hitSpeedScaleControls.kernelModeDoesNotUsePlannerThisOption") : state == undefined
      ? t("hitSpeedScaleControls.waitingForPlannerStatus") : t("flightTime.current", {
        fh: state.forehand, bh: state.backhand, requested: state.lastRequested ?? t("hitSpeedScaleControls.notSentYet"),
        actual: state.lastCommand ?? t("hitSpeedScaleControls.notSentYet"), max: state.maxSeconds,
      })}</div>
    {edited && !valid && <div role="alert">{t("flightTime.invalid")}</div>}
    {pending && <div role="status">{t("hitSpeedScaleControls.waitingForPlannerConfirmation")}</div>}
    {t.text(error) !== "" && <div role="alert">{t.text(error)}</div>}
  </div>;
}
