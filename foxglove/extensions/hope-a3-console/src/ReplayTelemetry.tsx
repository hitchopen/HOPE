import { PanelExtensionContext } from "@foxglove/extension";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { createTranslator, readLocale, Translator } from "./i18n";
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
const stateNames = (t: Translator): Record<string, string> => ({
  SERVE: t("telemetry.ourServe"),
  RALLY: t("telemetry.ourRally"),
  OPPONENT_SERVE: t("telemetry.theirServe"),
  OPPONENT_RALLY: t("telemetry.theirRally"),
  INVALID: t("telemetry.invalid"),
});
const handName = (sign: number | undefined, t: Translator): string =>
  sign === 1 ? t("telemetry.forehand") : sign === -1 ? t("telemetry.backhand") : "—";
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
  const t = useMemo(() => createTranslator(readLocale(context.initialState)), [context]);
  const latest = useRef<Telemetry>();
  const [value, setValue] = useState<Telemetry>();
  const pendingDone = useRef<() => void>();
  const [renderRevision, setRenderRevision] = useState(0);
  const [resetSerial, setResetSerial] = useState(0);
  useEffect(() => {
    context.setDefaultPanelTitle(t("telemetry.replayTitle"));
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
  }, [context, t]);
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
          {t("telemetry.ballPrefix")}
          {value?.play_state ? (stateNames(t)[value.play_state.state] ?? value.play_state.state) : "—"}
        </strong>
        <span className="muted">{t("telemetry.recomputed")}</span>
        <span>
          {value?.play_state?.serve_armed === true
            ? t("telemetry.waitServe")
            : value?.play_state?.receive_armed === true
              ? t("telemetry.waitReceive")
              : ""}
        </span>
        <span>
          {value?.play_state
            ? value.play_state.observation_missing
              ? t("telemetry.missingHeld")
              : t("telemetry.observed")
            : t("telemetry.stateMissing")}
        </span>
        <span>{value?.play_state?.forwarding === true ? t("telemetry.forwarding") : t("telemetry.forwardPaused")}</span>
        {value?.play_state?.state === "INVALID" && (
          <span className="reason">{value.play_state.reason}</span>
        )}
        <button onClick={resetAll}>{t("telemetry.resetViews")}</button>
      </header>
      <div className="hope-replay-grid">
        {cell(t("telemetry.robotXYZ"), xyz(value?.robot_xyz))}
        {cell(t("telemetry.robotRPY"), xyz(value?.robot_rpy_deg, 1))}
        {cell(t("telemetry.ballXYZ"), xyz(value?.ball_xyz))}
        {cell(t("telemetry.racketSpeed"), fmt(value?.racket_speed, 2))}
        {cell(t("telemetry.targetXYZ"), xyz(value?.target_xyz), "target")}
        {cell(t("telemetry.planeXYZ"), xyz(value?.frozen?.ball_xyz), "ball")}
        {cell(t("telemetry.contactXYZ"), xyz(value?.policy_frozen?.racket_xyz), "actual")}
        {cell(
          t("telemetry.delta"),
          xyz(
            value?.policy_frozen?.delta_xyz.map((v) => v * 1000),
            1,
          ),
        )}
      </div>
      <footer>
        <span>{t("telemetry.history", {shot: value?.shot ?? "—"})}</span>
        <span>
          {t("telemetry.hands", {target: handName(value?.target_swing_sign, t), runner: handName(value?.policy_swing_sign, t)})}
        </span>
        <span className="target">{t("telemetry.targetLegend")}</span>
        <span className="actual">{t("telemetry.racketLegend")}</span>
        <span className="ball">{t("telemetry.ballLegend")}</span>
        <span>
          {value?.frozen
            ? t("telemetry.frozenAt", {time: new Date(value.frozen.timestamp_s * 1000).toISOString().slice(11, 23)})
            : t("telemetry.waitPlane")}
        </span>
        <span>
          {value?.policy_frozen
            ? t("telemetry.contactAt", {time: new Date(value.policy_frozen.timestamp_s * 1000).toISOString().slice(11, 23), age: fmt(value.policy_frozen.sample_age_ms, 1)})
            : t("telemetry.waitContact")}
        </span>
      </footer>
    </main>
  );
}

type CameraConfig = { view?: string; cameraState?: Record<string, unknown> };
function ReplayViewControls({ context }: { context: PanelExtensionContext }) {
  const t = useMemo(() => createTranslator(readLocale(context.initialState)), [context]);
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
    context.setDefaultPanelTitle(config?.view ?? t("telemetry.view"));
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
  }, [context, config?.view, reset, t]);
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
