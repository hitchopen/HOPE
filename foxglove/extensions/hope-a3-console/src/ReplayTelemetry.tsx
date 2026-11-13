import { PanelExtensionContext } from "@foxglove/extension";
import { useCallback, useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import "./replayTelemetry.css";

type Vec = readonly number[];
type Telemetry = {
  timestamp_s: number;
  shot: number;
  target_swing_sign?: number;
  policy_swing_sign?: number;
  play_state?: {
    state: string;
    reason: string;
    observation_missing: boolean;
    forwarding: boolean;
    serve_armed?: boolean;
    receive_armed?: boolean;
  };
  robot_xyz: Vec;
  robot_rpy_deg: Vec;
  ball_xyz?: Vec | null;
  racket_xyz: Vec;
  racket_speed: number;
  target_xyz?: Vec | null;
  frozen?: {
    timestamp_s: number;
    ball_xyz: Vec;
  } | null;
  policy_frozen?: {
    timestamp_s: number;
    racket_xyz: Vec;
    delta_xyz: Vec;
    sample_age_ms: number;
  } | null;
};
const TOPIC = "/hope/replay/telemetry";
const RESET = "hopeReplayCameraReset";
const stateNames: Record<string, string> = {
  SERVE: "我方发球",
  RALLY: "我方回球",
  OPPONENT_SERVE: "对方发球",
  OPPONENT_RALLY: "对方回球",
  INVALID: "无有效球",
};
const handName = (sign: number | undefined): string =>
  sign === 1 ? "正手" : sign === -1 ? "反手" : "—";
const fmt = (value: unknown, digits = 3): string =>
  typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—";
const xyz = (value: Vec | null | undefined, digits = 3): string =>
  value?.length === 3 ? value.map((v) => fmt(v, digits)).join("  ") : "—  —  —";
function readTelemetry(raw: unknown): Telemetry | undefined {
  if (typeof raw !== "string") {
    return undefined;
  }
  try {
    const x = JSON.parse(raw) as Telemetry;
    return Number.isFinite(x.timestamp_s) &&
      Number.isInteger(x.shot) &&
      Array.isArray(x.robot_xyz) &&
      Array.isArray(x.robot_rpy_deg)
      ? x
      : undefined;
  } catch {
    return undefined;
  }
}
function ReplayTelemetry({ context }: { context: PanelExtensionContext }) {
  const latest = useRef<Telemetry>();
  const [value, setValue] = useState<Telemetry>();
  const pendingDone = useRef<() => void>();
  const [renderRevision, setRenderRevision] = useState(0);
  const [resetSerial, setResetSerial] = useState(0);
  useEffect(() => {
    context.setDefaultPanelTitle("回放数值 · XYZ / RPY");
    context.onRender = (state, renderDone) => {
      if (state.didSeek === true) {
        latest.current = undefined;
      }
      for (const event of state.currentFrame ?? []) {
        if (event.topic === TOPIC) {
          latest.current = readTelemetry((event.message as { data?: unknown }).data);
        }
      }
      const clock = state.currentTime
        ? state.currentTime.sec + state.currentTime.nsec / 1e9
        : undefined;
      const candidate = latest.current;
      setValue(
        candidate &&
          (clock == undefined ||
            (clock >= candidate.timestamp_s - 0.001 && clock - candidate.timestamp_s < 0.25))
          ? candidate
          : undefined,
      );
      pendingDone.current = renderDone;
      setRenderRevision((revision) => revision + 1);
    };
    context.watch("currentFrame");
    context.watch("currentTime");
    context.watch("didSeek");
    context.subscribe([{ topic: TOPIC }]);
    return () => {
      context.onRender = undefined;
      context.unsubscribeAll();
    };
  }, [context]);
  useEffect(() => {
    const done = pendingDone.current;
    pendingDone.current = undefined;
    done?.();
  }, [renderRevision]);
  const cell = (label: string, number: string, className = "") => (
    <div className={`hope-replay-cell ${className}`}>
      <span>{label}</span>
      <strong>{number}</strong>
    </div>
  );
  const resetAll = () => {
    const serial = resetSerial + 1;
    setResetSerial(serial);
    context.setVariable(RESET, `${Date.now()}-${serial}`);
  };
  return (
    <main className="hope-replay-metrics">
      <header className="hope-replay-state">
        <strong>
          球：
          {value?.play_state ? (stateNames[value.play_state.state] ?? value.play_state.state) : "—"}
        </strong>
        <span className="muted">回放重算</span>
        <span>
          {value?.play_state?.serve_armed === true
            ? "等待发球"
            : value?.play_state?.receive_armed === true
              ? "等待接球"
              : ""}
        </span>
        <span>
          {value?.play_state
            ? value.play_state.observation_missing
              ? "观测缺失 · 状态保留"
              : "观测正常"
            : "状态数据缺失"}
        </span>
        <span>{value?.play_state?.forwarding === true ? "转发中" : "转发暂停"}</span>
        {value?.play_state?.state === "INVALID" && (
          <span className="reason">{value.play_state.reason}</span>
        )}
        <button onClick={resetAll}>↺ 重置三视</button>
      </header>
      <div className="hope-replay-grid">
        {cell("机器人 XYZ · m", xyz(value?.robot_xyz))}
        {cell("姿态 R / P / Y · °", xyz(value?.robot_rpy_deg, 1))}
        {cell("球 XYZ · m", xyz(value?.ball_xyz))}
        {cell("实际拍速 · m/s", fmt(value?.racket_speed, 2))}
        {cell("目标 XYZ · m", xyz(value?.target_xyz), "target")}
        {cell("过平面球 XYZ · m", xyz(value?.frozen?.ball_xyz), "ball")}
        {cell("击球留影 XYZ · m", xyz(value?.policy_frozen?.racket_xyz), "actual")}
        {cell(
          "实际 − 目标 · mm",
          xyz(
            value?.policy_frozen?.delta_xyz.map((v) => v * 1000),
            1,
          ),
        )}
      </div>
      <footer>
        <span>历史 #{value?.shot ?? "—"}</span>
        <span>
          目标 {handName(value?.target_swing_sign)} · Runner 标记{" "}
          {handName(value?.policy_swing_sign)}
        </span>
        <span className="target">● 蓝：目标</span>
        <span className="actual">● 红：击球时实际拍</span>
        <span className="ball">● 橙：过平面球</span>
        <span>
          {value?.frozen
            ? `留点 ${new Date(value.frozen.timestamp_s * 1000).toISOString().slice(11, 23)} UTC`
            : "等待过平面"}
        </span>
        <span>
          {value?.policy_frozen
            ? `拍留影 ${new Date(value.policy_frozen.timestamp_s * 1000).toISOString().slice(11, 23)} UTC · 采样差 ${fmt(value.policy_frozen.sample_age_ms, 1)} ms`
            : "等待击球时刻"}
        </span>
      </footer>
    </main>
  );
}

type CameraConfig = { view?: string; cameraState?: Record<string, unknown> };
function ReplayViewControls({ context }: { context: PanelExtensionContext }) {
  const config = context.initialState as CameraConfig | undefined;
  const applied = useRef<unknown>();
  const pendingDone = useRef<() => void>();
  const [renderRevision, setRenderRevision] = useState(0);
  const reset = useCallback(() => {
    if (!config?.cameraState) {
      return;
    }
    context.layout.addPanel({
      position: "sibling",
      type: "3D",
      updateIfExists: true,
      getState: (existing) => ({
        ...(existing as Record<string, unknown> | undefined),
        cameraState: { ...config.cameraState },
      }),
    });
  }, [context, config?.cameraState]);
  useEffect(() => {
    context.setDefaultPanelTitle(config?.view ?? "视角");
    context.onRender = (state, renderDone) => {
      const token = state.variables?.get(RESET);
      if (token != undefined && token !== applied.current) {
        applied.current = token;
        reset();
      }
      pendingDone.current = renderDone;
      setRenderRevision((revision) => revision + 1);
    };
    context.watch("variables");
    return () => {
      context.onRender = undefined;
    };
  }, [context, config?.view, reset]);
  useEffect(() => {
    const done = pendingDone.current;
    pendingDone.current = undefined;
    done?.();
  }, [renderRevision]);
  // The zero-height sibling stays mounted to route the single global reset.
  return null;
}
export function initReplayTelemetry(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ReplayTelemetry context={context} />);
  return () => {
    root.unmount();
  };
}
export function initReplayViewControls(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ReplayViewControls context={context} />);
  return () => {
    root.unmount();
  };
}
