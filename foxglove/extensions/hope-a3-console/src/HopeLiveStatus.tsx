import { withBallYPreview, BallYFilterDraft } from "./ballYFilterPreview";
import { createTranslator, readLocale } from "./i18n";
import { PanelExtensionContext } from "@foxglove/extension";
import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { BallPlayState, currentBallPlayState, readLocalBallPlayState, readLocalControlKey, saveLocalControlKey } from "./ballPlayState";
import { ballStatusDisplay, freshValue, poseDetails, StatusTone, TimedValue } from "./liveStatus";
import "./liveStatus.css";

const topics = {
  runner: "/hope/runner/mode", alive: "/hope/runner/alive",
  base: "/hope/base/fresh", pose: "/hope/pelvis/pose", side: "/hope/lifecycle/config/table_side",
} as const;

function StatusCard({label,title,detail,tone}: {label:string;title:string;detail:string;tone:StatusTone}) {
  return <div className={`hope-live-card ${tone}`}>
    <div className="hope-live-label"><i />{label}</div>
    <strong>{title}</strong><span className="hope-live-detail">{detail}</span>
  </div>;
}

function HopeLiveStatus({context}: {context:PanelExtensionContext}) {
  const t = createTranslator(readLocale(context.initialState));
  const values = useRef<Record<string,TimedValue>>({});
  const previewApplied = useRef("");
  const [keyError,setKeyError] = useState(false);
  const [now,setNow] = useState(Date.now());
  const [done,setDone] = useState<(() => void)>();
  const [ball,setBall] = useState<{state:BallPlayState;receivedAt:number}>();
  useEffect(() => {
    context.onRender = (state,renderDone) => {
      if (state.didSeek) values.current = {};
      const preview = state.variables?.get("hopeLocalPreview") as (BallYFilterDraft & {kernelMode:boolean}) | undefined;
      if (preview && typeof preview.enabled === "boolean" && typeof preview.min === "string" && typeof preview.max === "string" && typeof preview.kernelMode === "boolean") {
        const identity = JSON.stringify(preview);
        if (identity !== previewApplied.current) {
          previewApplied.current = identity;
          context.layout.addPanel({position:"sibling",type:"3D",updateIfExists:true,
            getState:existing => withBallYPreview(existing,preview,{kernelMode:preview.kernelMode},t)});
        }
      }
      const receivedAt = Date.now();
      for (const event of state.currentFrame ?? []) {
        const message = event.message as {data?:unknown};
        values.current[event.topic] = {value:event.topic===topics.pose?event.message:message.data,receivedAt};
      }
      setNow(receivedAt);setDone(() => renderDone);
    };
    context.watch("currentFrame");context.watch("didSeek");context.watch("variables");
    context.subscribe(Object.values(topics).map(topic => ({topic})));
    let closed = false, pending:AbortController | undefined;
    const poll = async () => {
      setNow(Date.now());
      if (pending) return;
      const controller = new AbortController();pending=controller;
      const timeout = window.setTimeout(() => controller.abort(),300);
      try {
        const state = await readLocalBallPlayState(window.fetch.bind(window),controller.signal,readLocalControlKey());
        if (!closed) setBall(state ? {state,receivedAt:Date.now()} : undefined);
      } catch { if(!closed)setBall(undefined); }
      finally {window.clearTimeout(timeout);pending=undefined;}
    };
    void poll();const timer=window.setInterval(() => {void poll();},200);
    return () => {closed=true;pending?.abort();window.clearInterval(timer);context.onRender=undefined;context.unsubscribeAll();};
  },[context]);
  useEffect(() => {done?.();},[done]);
  const state=currentBallPlayState(ball?.state,ball?.receivedAt,now);
  const ballDisplay=ballStatusDisplay(state,t);
  const runner=freshValue(values.current[topics.runner],now);
  const alive=freshValue(values.current[topics.alive],now);
  const runnerKnown=typeof runner==="string" && runner!=="UNKNOWN" && alive===true;
  const runnerNames:Record<string,string>={PASSIVE:t("liveStatus.passive"),PD_STAND:t("liveStatus.stand"),MOTION:t("liveStatus.motion"),SERVE:t("liveStatus.serve"),TELEOP:t("liveStatus.teleop")};
  const base=freshValue(values.current[topics.base],now);
  const pose=poseDetails(freshValue(values.current[topics.pose],now),now,t);
  const side=freshValue(values.current[topics.side],now,5000);
  return <main className="hope-live-status">
    <div className="hope-live-grid">
      <StatusCard label={t("liveStatus.ball")} {...ballDisplay} />
      <StatusCard label={t("liveStatus.robot")} title={runnerKnown?(runnerNames[runner as string]??String(runner)):t("liveStatus.noUpdate")}
        detail={runnerKnown?t("liveStatus.runnerLive"):t("liveStatus.waitRunner")} tone={runnerKnown?"neutral":"unavailable"} />
      <StatusCard label={t("liveStatus.mocap",{side:side==="P1"||side==="P2"?` · ${side}`:""})}
        title={base===true?t("liveStatus.poseGood"):base===false?t("liveStatus.poseBad"):t("liveStatus.noUpdate")}
        detail={base===true?t("liveStatus.poseData"):t("liveStatus.waitPose")} tone={base===true?"good":"unavailable"} />
    </div>
    <details className="hope-live-diagnostics">
      <summary>{t("liveStatus.details")}</summary>
      <div>{pose??t("liveStatus.noPose")}</div>
      <div>{state?`${state.state} · ${state.reason} · ${state.forwarding?t("liveStatus.gateOn"):t("liveStatus.gateOff")}`:t("liveStatus.connectionHint")}</div>
      {!state && <label>{t("hopeA3Console.ballPairingFile")} <input type="file" onChange={event => {
        const file=event.target.files?.[0];event.target.value="";if(!file)return;
        if(file.size>128){setKeyError(true);return;}
        void file.text().then(value => {saveLocalControlKey(value);setKeyError(false);}).catch(() => setKeyError(true));
      }} /></label>}
      {keyError && <div>{t("hopeA3Console.ballPairingFailed")}</div>}
      {state?.forecast_advisory && <div>{t("liveStatus.forecast",{detail:state.forecast_advisory})}</div>}
    </details>
  </main>;
}

export function initHopeLiveStatus(context:PanelExtensionContext):() => void {
  const root=createRoot(context.panelElement);root.render(<HopeLiveStatus context={context} />);
  return () => root.unmount();
}
