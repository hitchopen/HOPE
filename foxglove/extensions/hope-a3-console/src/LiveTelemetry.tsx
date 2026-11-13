import { PanelExtensionContext } from "@foxglove/extension";
import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { BallYFilterDraft, withBallYPreview } from "./ballYFilterPreview";
import { createTranslator, readLocale } from "./i18n";
import {
  LiveTelemetrySnapshot,
  LiveVec,
  liveSourceFresh,
  liveRunnerLabel,
  liveTelemetryFresh,
  parseLiveTelemetry,
} from "./liveTelemetry";
import "./replayTelemetry.css";
import "./liveTelemetry.css";

const TOPIC = "/hope/live/telemetry";
const stateNames: Record<string, string> = {
  SERVE: "我方发球",
  RALLY: "我方回球",
  OPPONENT_SERVE: "对方发球",
  OPPONENT_RALLY: "对方回球",
  INVALID: "无有效球",
};
const fmt = (v: unknown, digits = 3): string =>
  typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
const xyz = (v: LiveVec | null | undefined, digits = 3): string =>
  Array.isArray(v) && v.length === 3 ? v.map((n) => fmt(n, digits)).join("  ") : "—  —  —";
const hand = (v: number | undefined): string => (v === 1 ? "正手" : v === -1 ? "反手" : "—");

function LiveTelemetry({ context }: { context: PanelExtensionContext }) {
  const [sample, setSample] = useState<{ value: LiveTelemetrySnapshot; receivedAt: number }>();
  const [now, setNow] = useState(Date.now());
  const pendingDone = useRef<() => void>();
  const [revision, setRevision] = useState(0);
  useEffect(() => {
    context.setDefaultPanelTitle("实时球状态 · XYZ / RPY");
    context.onRender = (state, done) => {
      if (state.didSeek === true) {
        setSample(undefined);
      }
      for (const event of state.currentFrame ?? []) {
        if (event.topic !== TOPIC) {
          continue;
        }
        const value = parseLiveTelemetry((event.message as { data?: unknown }).data);
        setSample(value ? { value, receivedAt: Date.now() } : undefined);
      }
      setNow(Date.now());
      pendingDone.current = done;
      setRevision((v) => v + 1);
    };
    context.watch("currentFrame");
    context.watch("didSeek");
    context.subscribe([{ topic: TOPIC }]);
    const timer = window.setInterval(() => {
      setNow(Date.now());
    }, 200);
    return () => {
      window.clearInterval(timer);
      context.onRender = undefined;
      context.unsubscribeAll();
    };
  }, [context]);
  useEffect(() => {
    const done = pendingDone.current;
    pendingDone.current = undefined;
    done?.();
  }, [revision]);
  const fresh = liveTelemetryFresh(sample?.value, sample?.receivedAt ?? 0, now);
  const value = fresh ? sample?.value : undefined;
  const ballState = liveSourceFresh(value?.play_state_age_ms) ? value?.play_state : undefined;
  const robot = value?.robot_available === true && liveSourceFresh(value.robot_age_ms);
  const cell = (label: string, number: string, tone = "") => (
    <div className={`hope-replay-cell ${tone}`}>
      <span>{label}</span>
      <strong>{number}</strong>
    </div>
  );
  return (
    <main className="hope-replay-metrics">
      <header className="hope-live-state-strip">
        <strong
          className={`hope-live-badge ${!ballState ? "unknown" : ballState.state === "INVALID" ? "blocked" : "active"}`}
        >
          球：{ballState ? (stateNames[ballState.state] ?? ballState.state) : "状态未知"}
        </strong>
        <strong className="hope-live-badge runner">Runner：{liveRunnerLabel(value)}</strong>
        <strong
          className={`hope-live-badge ${ballState?.forwarding === true ? "active" : "blocked"}`}
        >
          {ballState ? (ballState.forwarding ? "转发中" : "转发暂停") : "转发未知"}
        </strong>
        <span
          className={
            !fresh || !ballState || ballState.observation_missing
              ? "observation missing"
              : "observation"
          }
        >
          {!fresh
            ? "连接中断"
            : !ballState
              ? "球状态未更新"
              : ballState.observation_missing
                ? "观测缺失 · 状态保留"
                : "观测正常"}
        </span>
        {ballState?.serve_armed === true && <span>等待发球</span>}
        {ballState?.receive_armed === true && <span>等待接球</span>}
        {ballState?.state === "INVALID" && <span className="reason">{ballState.reason}</span>}
        <span className="shot">
          {value ? `第 ${value.generation} 球 · ${value.frozen ? "已留点" : "待过面"}` : "第 — 球"}
        </span>
      </header>
      <div className="hope-replay-grid">
        {cell("机器人 XYZ · m", xyz(robot ? value.robot_xyz : null))}
        {cell("姿态 R / P / Y · °", xyz(robot ? value.robot_rpy_deg : null, 1))}
        {cell("球 XYZ · m", xyz(value?.ball_xyz))}
        {cell("实际拍速 · m/s", fmt(robot ? value.racket_speed : null, 2))}
        {cell("目标 XYZ · m", xyz(value?.target_xyz), "target")}
        {cell("过平面球 XYZ · m", xyz(value?.frozen?.ball_xyz), "ball")}
        {cell("击球留影 XYZ · m", xyz(value?.policy_frozen?.racket_xyz), "actual")}
        {cell(
          "实际 − 目标 · mm",
          xyz(
            value?.policy_frozen?.delta_xyz?.map((v) => v * 1000),
            1,
          ),
        )}
      </div>
      <footer>
        <span className="target">● 蓝：目标</span>
        <span className="actual">● 红：击球时实际拍</span>
        <span className="ball">● 橙：过平面球</span>
        <span>
          目标 {hand(value?.target_swing_sign)} · Runner 标记 {hand(value?.policy_swing_sign)}
        </span>
        <span>{robot ? `关节 ${fmt(value.robot_age_ms, 0)} ms` : "实际关节回传缺失"}</span>
        <span>
          {value?.policy_frozen
            ? `拍留影 · 采样差 ${fmt(value.policy_frozen.sample_age_ms, 1)} ms`
            : "实际拍留影：待数据"}
        </span>
      </footer>
    </main>
  );
}

function LiveViewPreview({ context }: { context: PanelExtensionContext }) {
  const applied = useRef("");
  const pendingDone = useRef<() => void>();
  const [revision, setRevision] = useState(0);
  useEffect(() => {
    const t = createTranslator(readLocale(context.initialState));
    context.onRender = (state, done) => {
      const preview = state.variables?.get("hopeLocalPreview") as
        (BallYFilterDraft & { kernelMode: boolean }) | undefined;
      if (
        preview &&
        typeof preview.enabled === "boolean" &&
        typeof preview.min === "string" &&
        typeof preview.max === "string" &&
        typeof preview.kernelMode === "boolean"
      ) {
        const identity = JSON.stringify(preview);
        if (identity !== applied.current) {
          applied.current = identity;
          context.layout.addPanel({
            position: "sibling",
            type: "3D",
            updateIfExists: true,
            getState: (existing) =>
              withBallYPreview(existing, preview, { kernelMode: preview.kernelMode }, t),
          });
        }
      }
      pendingDone.current = done;
      setRevision((v) => v + 1);
    };
    context.watch("variables");
    return () => {
      context.onRender = undefined;
    };
  }, [context]);
  useEffect(() => {
    const done = pendingDone.current;
    pendingDone.current = undefined;
    done?.();
  }, [revision]);
  return null;
}
export function initLiveTelemetry(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<LiveTelemetry context={context} />);
  return () => {
    root.unmount();
  };
}
export function initLiveViewPreview(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<LiveViewPreview context={context} />);
  return () => {
    root.unmount();
  };
}
