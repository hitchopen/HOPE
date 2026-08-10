import { MessageEvent, PanelExtensionContext, Topic } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import "@fontsource/ibm-plex-mono/400.css";
import "@fontsource/ibm-plex-mono/500.css";
import "@fontsource/ibm-plex-mono/600.css";
import "@fontsource/ibm-plex-sans/400.css";
import "@fontsource/ibm-plex-sans/500.css";
import "@fontsource/space-grotesk/600.css";
import "@fontsource/space-grotesk/700.css";
import "./styles.css";

const TOPICS = {
  ntpOffset: "/hope/ntp/offset_ms",
  ntpDispersion: "/hope/ntp/root_dispersion_ms",
  ntpGate: "/hope/ntp/gate_pass",
  latency: "/hope/clock/message_latency_ms",
  timestampFresh: "/hope/clock/message_fresh",
  cpu: "/hope/system/cpu_load_percent",
  agibotPm: "/hope/vendor/agibot_pm_active",
  tfReady: "/hope/vendor/tf_ready",
  estopReady: "/hope/safety/estop_ready",
  hduActive: "/hope/v17/system/hdu_active",
  mduActive: "/hope/v17/system/mdu_active",
  markers: "/hope/v17/mocap/p1_marker_count",
  markersFresh: "/hope/v17/mocap/p1_marker_fresh",
  runnerAlive: "/hope/v17/runner/alive",
  runnerMode: "/hope/v17/runner/mode",
  runnerFault: "/hope/v17/runner/command_fault_latched",
  localRole: "/hope/v17/runner/local_role",
  roleChangeAllowed: "/hope/v17/runner/role_change_allowed",
  serveCapability: "/hope/v17/runner/serve_capability",
  serveState: "/hope/v17/runner/serve_state",
  standing: "/hope/v17/runner/standing",
  ready: "/hope/v17/runner/ready",
  readyToServe: "/hope/v17/runner/is_ready_to_serve",
  serving: "/hope/v17/runner/serving",
  lastAction: "/hope/v17/runner/last_action",
  lastResult: "/hope/v17/runner/last_action_result",
  lastReason: "/hope/v17/runner/last_action_reason",
  xHitSuccess: "/hope/v17/x_hit/success",
  xHitStatus: "/hope/v17/x_hit/status",
} as const;

const SERVICES = {
  estop: "/hope/safety/trigger_estop",
  setServer: "/hope/v17/runner/set_server",
  setReceiver: "/hope/v17/runner/set_receiver",
  stand: "/hope/v17/runner/enter_pd_stand",
  calibration: "/hope/v17/refresh_x_hit",
  ready: "/hope/v17/runner/enter_motion",
  readyToServe: "/hope/v17/runner/ready_to_serve",
  serve: "/hope/v17/runner/serve",
  passive: "/hope/v17/runner/emergency_passive",
} as const;

type ServiceKey = keyof typeof SERVICES;
type TopicName = (typeof TOPICS)[keyof typeof TOPICS];

type CpuSample = { at: number; value: number };

type Snapshot = {
  ntpOffsetMs?: number;
  ntpDispersionMs?: number;
  ntpPass?: boolean;
  latencyMs?: number;
  timestampFresh?: boolean;
  cpuPercent?: number;
  cpuSamples: CpuSample[];
  agibotPm?: boolean;
  tfReady?: boolean;
  estopReady?: boolean;
  hduActive?: boolean;
  mduActive?: boolean;
  markerCount?: number;
  markersFresh?: boolean;
  runnerAlive?: boolean;
  runnerMode?: string;
  runnerFault?: boolean;
  localRole?: string;
  roleChangeAllowed?: boolean;
  serveCapability?: string;
  serveState?: string;
  standing?: boolean;
  ready?: boolean;
  readyToServe?: boolean;
  serving?: boolean;
  lastAction?: string;
  lastResult?: string;
  lastReason?: string;
  xHitSuccess?: boolean;
  xHitStatus?: string;
  availableTopics: Set<string>;
  lastReceived: Partial<Record<TopicName, number>>;
};

const INITIAL_SNAPSHOT: Snapshot = {
  cpuSamples: [],
  availableTopics: new Set<string>(),
  lastReceived: {},
};

type ScalarMessage = { data?: unknown };
type TriggerResponse = { success?: unknown; message?: unknown };

function scalar(message: unknown): unknown {
  if (typeof message !== "object" || message == undefined || !("data" in message)) {
    return undefined;
  }
  return (message as ScalarMessage).data;
}

function numberValue(message: unknown): number | undefined {
  const value = scalar(message);
  return typeof value === "number" && Number.isFinite(value) ? value : undefined;
}

function boolValue(message: unknown): boolean | undefined {
  const value = scalar(message);
  return typeof value === "boolean" ? value : undefined;
}

function stringValue(message: unknown): string | undefined {
  const value = scalar(message);
  return typeof value === "string" ? value : undefined;
}

function applyMessage(next: Snapshot, event: MessageEvent, receivedAt: number): void {
  const topic = event.topic as TopicName;
  if (!(Object.values(TOPICS) as string[]).includes(topic)) {
    return;
  }
  next.lastReceived[topic] = receivedAt;
  switch (topic) {
    case TOPICS.ntpOffset:
      next.ntpOffsetMs = numberValue(event.message);
      break;
    case TOPICS.ntpDispersion:
      next.ntpDispersionMs = numberValue(event.message);
      break;
    case TOPICS.ntpGate:
      next.ntpPass = boolValue(event.message);
      break;
    case TOPICS.latency:
      next.latencyMs = numberValue(event.message);
      break;
    case TOPICS.timestampFresh:
      next.timestampFresh = boolValue(event.message);
      break;
    case TOPICS.cpu: {
      const value = numberValue(event.message);
      next.cpuPercent = value;
      if (value != undefined) {
        next.cpuSamples.push({ at: receivedAt, value: Math.max(0, Math.min(100, value)) });
        next.cpuSamples = next.cpuSamples
          .filter((sample) => receivedAt - sample.at <= 120_000)
          .slice(-240);
      }
      break;
    }
    case TOPICS.agibotPm:
      next.agibotPm = boolValue(event.message);
      break;
    case TOPICS.tfReady:
      next.tfReady = boolValue(event.message);
      break;
    case TOPICS.estopReady:
      next.estopReady = boolValue(event.message);
      break;
    case TOPICS.hduActive:
      next.hduActive = boolValue(event.message);
      break;
    case TOPICS.mduActive:
      next.mduActive = boolValue(event.message);
      break;
    case TOPICS.markers:
      next.markerCount = numberValue(event.message);
      break;
    case TOPICS.markersFresh:
      next.markersFresh = boolValue(event.message);
      break;
    case TOPICS.runnerAlive:
      next.runnerAlive = boolValue(event.message);
      break;
    case TOPICS.runnerMode:
      next.runnerMode = stringValue(event.message);
      break;
    case TOPICS.runnerFault:
      next.runnerFault = boolValue(event.message);
      break;
    case TOPICS.localRole:
      next.localRole = stringValue(event.message);
      break;
    case TOPICS.roleChangeAllowed:
      next.roleChangeAllowed = boolValue(event.message);
      break;
    case TOPICS.serveCapability:
      next.serveCapability = stringValue(event.message);
      break;
    case TOPICS.serveState:
      next.serveState = stringValue(event.message);
      break;
    case TOPICS.standing:
      next.standing = boolValue(event.message);
      break;
    case TOPICS.ready:
      next.ready = boolValue(event.message);
      break;
    case TOPICS.readyToServe:
      next.readyToServe = boolValue(event.message);
      break;
    case TOPICS.serving:
      next.serving = boolValue(event.message);
      break;
    case TOPICS.lastAction:
      next.lastAction = stringValue(event.message);
      break;
    case TOPICS.lastResult:
      next.lastResult = stringValue(event.message);
      break;
    case TOPICS.lastReason:
      next.lastReason = stringValue(event.message);
      break;
    case TOPICS.xHitSuccess:
      next.xHitSuccess = boolValue(event.message);
      break;
    case TOPICS.xHitStatus:
      next.xHitStatus = stringValue(event.message);
      break;
  }
}

function triggerResponse(value: unknown): { success: boolean; message: string } {
  if (typeof value !== "object" || value == undefined) {
    return { success: false, message: "service returned no structured response" };
  }
  const response = value as TriggerResponse;
  return {
    success: response.success === true,
    message: typeof response.message === "string" ? response.message : "service returned no message",
  };
}

async function timeoutAfter(milliseconds: number): Promise<never> {
  return await new Promise((_, reject) => {
    window.setTimeout(() => {
      reject(new Error(`service timeout after ${milliseconds / 1000} s`));
    }, milliseconds);
  });
}

function isFresh(snapshot: Snapshot, topic: TopicName, now: number, maxAgeMs: number): boolean {
  const received = snapshot.lastReceived[topic];
  return received != undefined && now - received <= maxAgeMs;
}

function GateChip({ label, value, detail }: { label: string; value?: boolean; detail?: string }): ReactElement {
  const state = value == undefined ? "unknown" : value ? "ok" : "attention";
  return (
    <div className={`gate gate-${state}`} title={detail}>
      <span className="gate-dot" />
      {label}{value === false ? " · CHECK" : ""}
    </div>
  );
}

function ProcessTile({ name, topic, value }: { name: string; topic: string; value?: boolean }): ReactElement {
  const state = value == undefined ? "unknown" : value ? "running" : "stopped";
  return (
    <div className={`process-tile process-${state}`}>
      <div className="process-state"><span className="process-dot" />{state === "unknown" ? "NO DATA" : state.toUpperCase()}</div>
      <div className="process-name">{name}</div>
      <div className="topic-path">{topic}</div>
    </div>
  );
}

type ActionButtonProps = {
  label: string;
  detail: string;
  disabled: boolean;
  busy: boolean;
  completed?: boolean;
  next?: boolean;
  wide?: boolean;
  danger?: boolean;
  onClick: () => void;
};

function ActionButton(props: ActionButtonProps): ReactElement {
  const classes = [
    "action-button",
    props.completed === true ? "action-completed" : "",
    props.next === true ? "action-next" : "",
    props.wide === true ? "action-wide" : "",
    props.danger === true ? "action-danger" : "",
  ].filter(Boolean).join(" ");
  return (
    <button className={classes} type="button" disabled={props.disabled || props.busy} onClick={props.onClick}>
      <span>{props.busy ? "Working…" : props.label}</span>
      <small>{props.detail}</small>
    </button>
  );
}

function HopeA3Console({ context }: { context: PanelExtensionContext }): ReactElement {
  const latest = useRef<Snapshot>(INITIAL_SNAPSHOT);
  const [snapshot, setSnapshot] = useState<Snapshot>(INITIAL_SNAPSHOT);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [now, setNow] = useState(Date.now());
  const [busy, setBusy] = useState<Partial<Record<ServiceKey, boolean>>>({});
  const [notice, setNotice] = useState("Waiting for authoritative Runner state");

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      const receivedAt = Date.now();
      const next: Snapshot = {
        ...latest.current,
        cpuSamples: [...latest.current.cpuSamples],
        availableTopics: new Set(latest.current.availableTopics),
        lastReceived: { ...latest.current.lastReceived },
      };
      if (renderState.topics != undefined) {
        next.availableTopics = new Set(renderState.topics.map((topic: Topic) => topic.name));
      }
      for (const event of renderState.currentFrame ?? []) {
        applyMessage(next, event, receivedAt);
      }
      latest.current = next;
      setSnapshot(next);
      setRenderDone(() => done);
    };
    context.watch("topics");
    context.watch("currentFrame");
    context.subscribe(Object.values(TOPICS).map((topic) => ({ topic })));
    return () => {
      context.onRender = undefined;
      context.unsubscribeAll();
    };
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      setNow(Date.now());
    }, 250);
    return () => {
      window.clearInterval(timer);
    };
  }, []);

  const invoke = useCallback(async (key: ServiceKey) => {
    if (context.callService == undefined || busy[key] === true) {
      setNotice("Current data source does not expose service calls");
      return;
    }
    setBusy((current) => ({ ...current, [key]: true }));
    setNotice(`${SERVICES[key]} requested…`);
    try {
      const raw = await Promise.race([context.callService(SERVICES[key], {}), timeoutAfter(3_000)]);
      const response = triggerResponse(raw);
      setNotice(`${response.success ? "ACCEPTED" : "REJECTED"} · ${response.message}`);
    } catch (error) {
      setNotice(`FAILED · ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      setBusy((current) => ({ ...current, [key]: false }));
    }
  }, [busy, context]);

  const ntpFresh = isFresh(snapshot, TOPICS.ntpOffset, now, 1_500);
  const latencyFresh = isFresh(snapshot, TOPICS.latency, now, 500) && snapshot.timestampFresh === true;
  const cpuFresh = isFresh(snapshot, TOPICS.cpu, now, 1_500);
  const hduFresh = isFresh(snapshot, TOPICS.hduActive, now, 1_500);
  const mduFresh = isFresh(snapshot, TOPICS.mduActive, now, 1_500);
  const pmFresh = isFresh(snapshot, TOPICS.agibotPm, now, 1_500);
  const runnerFresh = isFresh(snapshot, TOPICS.runnerAlive, now, 1_500) && snapshot.runnerAlive === true;
  const markerFresh = isFresh(snapshot, TOPICS.markers, now, 1_000) && snapshot.markersFresh === true;
  const estopUsable = isFresh(snapshot, TOPICS.estopReady, now, 1_000) && snapshot.estopReady === true;
  const runnerUsable = runnerFresh && snapshot.runnerFault !== true;
  const serveAvailable = snapshot.serveCapability === "AVAILABLE";

  const nextStep = snapshot.standing !== true
    ? "stand"
    : snapshot.localRole === "SERVER"
      ? snapshot.readyToServe !== true && snapshot.serving !== true
        ? "readyToServe"
        : snapshot.readyToServe === true
            ? "serve"
            : "none"
      : snapshot.xHitSuccess !== true
        ? "calibration"
        : snapshot.ready !== true
          ? "ready"
          : "none";

  const cpuPoints = useMemo(() => {
    if (snapshot.cpuSamples.length === 0) {
      return "";
    }
    const start = now - 120_000;
    return snapshot.cpuSamples.map((sample) => {
      const x = Math.max(0, Math.min(600, ((sample.at - start) / 120_000) * 600));
      const y = 150 - (Math.max(0, Math.min(100, sample.value)) / 100) * 140;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    }).join(" ");
  }, [now, snapshot.cpuSamples]);

  const lastAction = snapshot.lastAction != undefined && snapshot.lastAction !== "NONE"
    ? `${snapshot.lastAction} → ${snapshot.lastResult ?? "?"} / ${snapshot.lastReason ?? "?"}`
    : notice;

  return (
    <div className="hope-console">
      <div className="source-row">
        <div className="source-block">
          <div className="eyebrow">A3 SOURCE</div>
          <div className="source-value">
            <span>{context.dataSourceProfile?.toUpperCase() ?? "ROS 2"} · CONTROL :8766</span>
            <span className={`source-dot ${hduFresh && snapshot.hduActive === true ? "source-live" : ""}`} />
          </div>
          <div className="helper">change the host through the Foxglove connection dialog</div>
        </div>
        <button className="estop-button" type="button" disabled={!estopUsable || busy.estop === true} onClick={() => void invoke("estop")}>
          {busy.estop === true ? "ASSERTING…" : "E-STOP"}
          <small>ASSERT ONLY · NO RESET</small>
        </button>
      </div>

      <div className="gate-row">
        <GateChip label="NTP GATE" value={ntpFresh ? snapshot.ntpPass : undefined} />
        <GateChip label="TIMESTAMP" value={latencyFresh ? true : undefined} />
        <GateChip label="TF READY" value={isFresh(snapshot, TOPICS.tfReady, now, 1_000) ? snapshot.tfReady : undefined} />
        <GateChip label="E-STOP BACKEND" value={estopUsable ? true : undefined} />
        <GateChip label={`MARKERS ${markerFresh ? Math.round(snapshot.markerCount ?? 0) : "—"}/10`} value={markerFresh ? snapshot.markerCount === 10 : undefined} />
      </div>

      <div className="metric-grid">
        <div className="metric-card">
          <div className="eyebrow">NTP WORLD-CLOCK OFFSET</div>
          <div className={`metric-value ${ntpFresh ? "" : "stale"}`}><span>{ntpFresh && snapshot.ntpOffsetMs != undefined ? snapshot.ntpOffsetMs.toFixed(2) : "—"}</span><small>ms</small></div>
          <div className="topic-path">{TOPICS.ntpOffset}{snapshot.ntpDispersionMs != undefined ? ` · disp ${snapshot.ntpDispersionMs.toFixed(2)}` : ""}{ntpFresh ? "" : " · stale"}</div>
        </div>
        <div className="metric-card">
          <div className="eyebrow">ROS 2 MESSAGE LATENCY</div>
          <div className={`metric-value ${latencyFresh ? "" : "stale"}`}><span>{latencyFresh && snapshot.latencyMs != undefined ? snapshot.latencyMs.toFixed(1) : "—"}</span><small>ms</small></div>
          <div className="topic-path">{TOPICS.latency}{latencyFresh ? "" : " · stale"}</div>
        </div>
      </div>

      <div className="process-grid">
        <ProcessTile name="agibot_pm" topic={TOPICS.agibotPm} value={pmFresh ? snapshot.agibotPm : undefined} />
        <ProcessTile name="HDU" topic={TOPICS.hduActive} value={hduFresh ? snapshot.hduActive : undefined} />
        <ProcessTile name="MDU Runner" topic={TOPICS.mduActive} value={mduFresh ? snapshot.mduActive : undefined} />
      </div>

      <div className="cpu-card">
        <div className="cpu-header">
          <span className="eyebrow">A3 CPU LOAD</span>
          <span className={`cpu-value ${cpuFresh ? "" : "stale"}`}>{cpuFresh && snapshot.cpuPercent != undefined ? snapshot.cpuPercent.toFixed(0) : "—"}<small>%</small></span>
          <span className="topic-path cpu-topic">{TOPICS.cpu} · 0–100 %, 120 s</span>
        </div>
        <svg className="cpu-plot" viewBox="0 0 600 160" preserveAspectRatio="none" aria-label="CPU load over 120 seconds">
          <line x1="0" y1="40" x2="600" y2="40" />
          <line x1="0" y1="80" x2="600" y2="80" />
          <line x1="0" y1="120" x2="600" y2="120" />
          {cpuPoints.length > 0 && <polyline className="cpu-line" points={cpuPoints} />}
        </svg>
      </div>

      <div className="sequence-card">
        <div className="sequence-header">
          <div><span className="eyebrow">RUNNER SEQUENCE</span><span className="sequence-status">{runnerFresh ? `${snapshot.runnerMode ?? "UNKNOWN"} · ${snapshot.serveState ?? "UNAVAILABLE"}` : "NO FRESH RUNNER STATE"}</span></div>
          <div className="role-controls">
            <span>OUR ROLE: {snapshot.localRole ?? "UNASSIGNED"}</span>
            <button type="button" disabled={snapshot.roleChangeAllowed !== true || busy.setServer === true} onClick={() => void invoke("setServer")}>SERVER</button>
            <button type="button" disabled={snapshot.roleChangeAllowed !== true || busy.setReceiver === true} onClick={() => void invoke("setReceiver")}>RECEIVER</button>
          </div>
        </div>
        <div className="action-grid">
          <ActionButton label="Stand" detail={snapshot.standing === true ? "DONE · PD_STAND" : "same as keyboard s"} disabled={!runnerUsable} busy={busy.stand === true} completed={snapshot.standing === true} next={nextStep === "stand"} onClick={() => void invoke("stand")} />
          <ActionButton label="Calibration" detail={snapshot.xHitSuccess === true ? "DONE · x_hit refreshed" : snapshot.standing === true ? "refresh x_hit" : "LOCKED · stand first"} disabled={!runnerUsable || snapshot.standing !== true} busy={busy.calibration === true} completed={snapshot.xHitSuccess === true} next={nextStep === "calibration"} onClick={() => void invoke("calibration")} />
          <ActionButton label="Ready" detail={snapshot.ready === true ? "DONE · MOTION" : "same as keyboard m"} disabled={!runnerUsable} busy={busy.ready === true} completed={snapshot.ready === true} next={nextStep === "ready"} onClick={() => void invoke("ready")} />
          <ActionButton label="Ready to Serve" detail={!serveAvailable ? "UNAVAILABLE · launch Runner with --serve" : snapshot.readyToServe === true ? "DONE · ball on palm" : "start serve pre-position"} disabled={!runnerUsable || !serveAvailable} busy={busy.readyToServe === true} completed={snapshot.readyToServe === true} next={nextStep === "readyToServe"} onClick={() => void invoke("readyToServe")} />
          <ActionButton label="Serve" detail={snapshot.serving === true ? "SERVING" : snapshot.readyToServe === true ? "confirm ball on palm" : "LOCKED UNTIL AWAIT_BALL_ON_PALM"} disabled={!runnerUsable || snapshot.readyToServe !== true} busy={busy.serve === true} completed={snapshot.serving === true} next={nextStep === "serve"} wide onClick={() => void invoke("serve")} />
        </div>
        <div className="sequence-footer">
          <span title={snapshot.xHitStatus}>{lastAction}</span>
          <ActionButton label="Runner Passive" detail="ZERO GAINS · robot loses support" disabled={!runnerFresh} busy={busy.passive === true} danger onClick={() => void invoke("passive")} />
        </div>
      </div>
    </div>
  );
}

export function initHopeA3Console(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HopeA3Console context={context} />);
  return () => {
    root.unmount();
  };
}
