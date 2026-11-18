import { PanelExtensionContext } from "@foxglove/extension";
import { useEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { BallYFilterDraft, withBallYPreview } from "./ballYFilterPreview";
import { createTranslator, readLocale, Translator } from "./i18n";
import {
  LiveTelemetrySnapshot,
  LiveVec,
  liveSourceFresh,
  liveRunnerLabel,
  liveShotEvents,
  liveTelemetryFresh,
  parseLiveTelemetry,
} from "./liveTelemetry";
import "./replayTelemetry.css";
import "./liveTelemetry.css";

const TOPIC = "/hope/live/telemetry";
const stateNames = (t: Translator): Record<string, string> => ({
  SERVE: t("telemetry.ourServe"),
  RALLY: t("telemetry.ourRally"),
  OPPONENT_SERVE: t("telemetry.theirServe"),
  OPPONENT_RALLY: t("telemetry.theirRally"),
  INVALID: t("telemetry.invalid"),
});
const fmt = (v: unknown, digits = 3): string =>
  typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
const xyz = (v: LiveVec | null | undefined, digits = 3): string =>
  Array.isArray(v) && v.length === 3 ? v.map((n) => fmt(n, digits)).join("  ") : "—  —  —";
const hand = (v: number | undefined, t: Translator): string => (v === 1 ? t("telemetry.forehand") : v === -1 ? t("telemetry.backhand") : "—");

type PreviewDraft = BallYFilterDraft & { kernelMode: boolean };

/**
 * Apply the Console's local y-filter preview (variable hopeLocalPreview) to the
 * sibling 3D panel. Display only: never publishes ROS messages.
 */
function applyPreviewVariable(
  context: PanelExtensionContext,
  variables: ReadonlyMap<string, unknown> | undefined,
  applied: { current: string },
  t: Translator,
): void {
  const preview = variables?.get("hopeLocalPreview") as PreviewDraft | undefined;
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
}

function LiveTelemetry({ context }: { context: PanelExtensionContext }) {
  const t = useMemo(() => createTranslator(readLocale(context.initialState)), [context]);
  const [sample, setSample] = useState<{ value: LiveTelemetrySnapshot; receivedAt: number }>();
  const [now, setNow] = useState(Date.now());
  const pendingDone = useRef<() => void>();
  const [revision, setRevision] = useState(0);
  const previewApplied = useRef("");
  useEffect(() => {
    context.setDefaultPanelTitle(t("telemetry.liveTitle"));
    // Opt-in (panel config ownsPreview: true) for layouts where this panel, not
    // HOPE Live Status, is the 3D panel's split sibling.
    const ownsPreview =
      (context.initialState as { ownsPreview?: unknown } | undefined)?.ownsPreview === true;
    const previewT = createTranslator(readLocale(context.initialState));
    context.onRender = (state, done) => {
      if (ownsPreview) {
        applyPreviewVariable(context, state.variables, previewApplied, previewT);
      }
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
    if (ownsPreview) {
      context.watch("variables");
    }
    context.subscribe([{ topic: TOPIC }]);
    const timer = window.setInterval(() => {
      setNow(Date.now());
    }, 200);
    return () => {
      window.clearInterval(timer);
      context.onRender = undefined;
      context.unsubscribeAll();
    };
  }, [context, t]);
  useEffect(() => {
    const done = pendingDone.current;
    pendingDone.current = undefined;
    done?.();
  }, [revision]);
  const fresh = liveTelemetryFresh(sample?.value, sample?.receivedAt ?? 0, now);
  const value = fresh ? sample?.value : undefined;
  const ballState = liveSourceFresh(value?.play_state_age_ms) ? value?.play_state : undefined;
  const robot = value?.robot_available === true && liveSourceFresh(value.robot_age_ms);
  // Pelvis XYZ / RPY needs only mocap; racket values still need Runner joints.
  const pose = robot || (value?.base_available === true && liveSourceFresh(value.base_age_ms));
  const events = liveShotEvents(value);
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
          {t("telemetry.ballPrefix")}{ballState ? (stateNames(t)[ballState.state] ?? ballState.state) : t("telemetry.unknown")}
        </strong>
        <strong className="hope-live-badge runner">{t("telemetry.runnerPrefix")}{liveRunnerLabel(value, t)}</strong>
        <strong
          className={`hope-live-badge ${ballState?.forwarding === true ? "active" : "blocked"}`}
        >
          {ballState ? (ballState.forwarding ? t("telemetry.forwarding") : t("telemetry.forwardPaused")) : t("telemetry.forwardUnknown")}
        </strong>
        <span
          className={
            !fresh || !ballState || ballState.observation_missing
              ? "observation missing"
              : "observation"
          }
        >
          {!fresh
            ? t("telemetry.disconnected")
            : !ballState
              ? t("telemetry.stateStale")
              : ballState.observation_missing
                ? t("telemetry.missingHeld")
                : t("telemetry.observed")}
        </span>
        {ballState?.serve_armed === true && <span>{t("telemetry.waitServe")}</span>}
        {ballState?.receive_armed === true && <span>{t("telemetry.waitReceive")}</span>}
        {ballState?.state === "INVALID" && <span className="reason">{ballState.reason}</span>}
        <span className="shot">
          {value ? t("telemetry.shot", {shot: value.generation, state: value.frozen ? t("telemetry.frozen") : t("telemetry.pendingPlane")}) : t("telemetry.shotUnknown")}
        </span>
      </header>
      <div className="hope-live-events" role="group" aria-label={t("liveTelemetry.events")}>
        <span className={`hope-live-event target ${events.target ? "on" : ""}`}>
          {t("liveTelemetry.eventTarget")}
        </span>
        <span className={`hope-live-event plane ${events.plane ? "on" : ""}`}>
          {t("liveTelemetry.eventPlane")}
        </span>
        <span className={`hope-live-event strike ${events.strike ? "on" : ""}`}>
          {events.deltaMm != undefined
            ? t("liveTelemetry.eventStrikeDelta", { mm: events.deltaMm.toFixed(0) })
            : t("liveTelemetry.eventStrike")}
        </span>
      </div>
      <div className="hope-replay-grid">
        {cell(t("telemetry.robotXYZ"), xyz(pose ? value?.robot_xyz : null))}
        {cell(t("telemetry.robotRPY"), xyz(pose ? value?.robot_rpy_deg : null, 1))}
        {cell(t("telemetry.ballXYZ"), xyz(value?.ball_xyz))}
        {cell(t("telemetry.racketSpeed"), fmt(robot ? value.racket_speed : null, 2))}
        {cell(t("telemetry.targetXYZ"), xyz(value?.target_xyz), "target")}
        {cell(t("telemetry.planeXYZ"), xyz(value?.frozen?.ball_xyz), "ball")}
        {cell(t("telemetry.contactXYZ"), xyz(value?.policy_frozen?.racket_xyz), "actual")}
        {cell(
          t("telemetry.delta"),
          xyz(
            value?.policy_frozen?.delta_xyz?.map((v) => v * 1000),
            1,
          ),
        )}
      </div>
      <footer>
        <span className="target">{t("telemetry.targetLegend")}</span>
        <span className="actual">{t("telemetry.racketLegend")}</span>
        <span className="ball">{t("telemetry.ballLegend")}</span>
        <span>
          {t("telemetry.hands", {target: hand(value?.target_swing_sign, t), runner: hand(value?.policy_swing_sign, t)})}
        </span>
        <span>
          {robot
            ? t("telemetry.jointsAge", {age: fmt(value.robot_age_ms, 0)})
            : pose
              ? t("telemetry.poseFromMocap", {age: fmt(value?.base_age_ms, 0)})
              : t("telemetry.jointsMissing")}
        </span>
        <span>
          {value?.policy_frozen
            ? t("telemetry.sampleAge", {age: fmt(value.policy_frozen.sample_age_ms, 1)})
            : t("telemetry.contactPending")}
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
      applyPreviewVariable(context, state.variables, applied, t);
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
