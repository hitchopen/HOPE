import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useRef, useState } from "react";

import { useI18n } from "./LanguageContext";
import {
  DOCK_CONFIG_TOPIC, DOCK_TUNING_FIELDS, DockTuning, DockTuningEcho, dockTuningResult,
  dockTuningValid, encodeDockTuning,
} from "./dockTuning";
import { FLOAT64_ARRAY_DATATYPES } from "./operatorHeartbeat";

/** Reposition limits. Applies on the next walk; the Runner saves dock.yaml. */
export function DockTuningCard({ context, echo, boot, blocked, embedded = false }: {
  context: PanelExtensionContext;
  embedded?: boolean;
  echo?: DockTuningEcho;
  boot?: number;
  blocked?: string;
}): ReactElement {
  const t = useI18n();
  const [draft, setDraft] = useState<DockTuning | undefined>();
  const [edited, setEdited] = useState(false);
  const [sent, setSent] = useState<{ seq: number; at: number } | undefined>();
  const [, tick] = useState(0);
  const advertised = useRef(false);
  const echoKey = echo == undefined ? "" : JSON.stringify(echo.values);
  useEffect(() => {
    if (!edited && echoKey !== "") { setDraft(JSON.parse(echoKey) as DockTuning); }
  }, [echoKey, edited]);
  useEffect(() => {
    if (sent == undefined) { return; }
    const timer = window.setInterval(() => { tick((n) => n + 1); }, 250);
    return () => { window.clearInterval(timer); };
  }, [sent]);
  useEffect(() => () => {
    if (advertised.current) {
      try { context.unadvertise?.(DOCK_CONFIG_TOPIC); } catch { /* already disconnected */ }
    }
  }, [context]);

  const replied = sent != undefined && echo?.seq === sent.seq;
  const timedOut = sent != undefined && !replied && Date.now() - sent.at > 3_000;
  const waiting = sent != undefined && !replied && !timedOut;
  const valid = draft != undefined && dockTuningValid(draft);
  const canPublish = context.advertise != undefined && context.publish != undefined;
  const disabledReason = !canPublish ? t("dockTuning.noPublish")
    : echo == undefined || boot == undefined ? t("dockTuning.noTelemetry") : blocked;

  const apply = () => {
    if (draft == undefined || !valid || boot == undefined || disabledReason != undefined) { return; }
    try {
      if (!advertised.current) {
        context.advertise?.(DOCK_CONFIG_TOPIC, "std_msgs/Float64MultiArray", { datatypes: FLOAT64_ARRAY_DATATYPES });
        advertised.current = true;
      }
      const seq = Date.now();
      context.publish?.(DOCK_CONFIG_TOPIC, { layout: { dim: [], data_offset: 0 }, data: encodeDockTuning(boot, seq, draft) });
      setSent({ seq, at: Date.now() });
      setEdited(false);
    } catch {
      advertised.current = false;
      setSent({ seq: -1, at: 0 });
    }
  };
  const status = sent == undefined ? undefined
    : waiting ? t("dockTuning.waiting")
      : timedOut ? t("dockTuning.noReply")
        : t(`dockTuning.result.${dockTuningResult(echo?.result ?? 0)}`);
  const summary = echo == undefined ? "—"
    : `(${echo.values.targetX.toFixed(2)}, ${echo.values.targetY.toFixed(2)}, ${echo.values.targetYawDeg.toFixed(0)}°) · ±${echo.values.tolCm.toFixed(0)} cm`;

  // Built into the Walk stand card only the target position is tunable; every
  // other limit keeps the Runner's current value (sent back unchanged).
  const shown = embedded ? DOCK_TUNING_FIELDS.filter(({ key }) => key === "targetX" || key === "targetY")
    : DOCK_TUNING_FIELDS;
  const fields = (
    <div className="plane-mode-controls dock-tuning-grid">
      {shown.map(({ key, min, max, step }) => (
        <label key={key}>
          {t(`dockTuning.field.${key}`)}
          <input type="number" min={min} max={max} step={step}
            aria-label={t(`dockTuning.field.${key}`)}
            value={draft?.[key] ?? ""} disabled={draft == undefined || waiting}
            onChange={(event) => {
              const value = Number(event.target.value);
              setDraft((current) => current == undefined ? current : { ...current, [key]: value });
              setEdited(true);
            }} />
        </label>
      ))}
    </div>
  );
  const actions = (
    <div className="c-rowcard">
      <button type="button" className="c-btn c-sm"
        disabled={!valid || waiting || disabledReason != undefined || !edited}
        onClick={apply}>{t("dockTuning.apply")}</button>
      <button type="button" className="c-ghost" disabled={!edited || echo == undefined}
        onClick={() => { setDraft(echo?.values); setEdited(false); }}>{t("dockTuning.revert")}</button>
      <span className="c-hint">{disabledReason ?? (draft != undefined && !valid ? t("dockTuning.outOfRange") : status ?? t("dockTuning.hint"))}</span>
    </div>
  );
  // Inside the Walk stand dropdown: target x / y fields and Apply only.
  if (embedded) {
    return <div className="dock-tuning-inline">{fields}{actions}</div>;
  }
  return (
    <details className="c-card c-tune">
      <summary>
        <b>{t("dockTuning.title")}</b>
        <span className="c-tune-vals"><span>{summary}</span></span>
      </summary>
      <div className="c-tune-body">{fields}{actions}</div>
    </details>
  );
}
