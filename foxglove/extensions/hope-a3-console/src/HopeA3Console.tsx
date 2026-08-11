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
  cpuTopProcess: "/hope/system/cpu_top_process",
  agibotPm: "/hope/vendor/agibot_pm_active",
  tfReady: "/hope/vendor/tf_ready",
  estopReady: "/hope/safety/estop_ready",
  estopFullReady: "/hope/safety/estop_full_ready",
  estopLatched: "/hope/safety/estop_latched",
  estopText: "/hope/safety/estop_text",
  hduActive: "/hope/system/hdu_active",
  mduActive: "/hope/system/mdu_active",
  markers: "/hope/mocap/p1_marker_count",
  markersFresh: "/hope/mocap/p1_marker_fresh",
  baseFresh: "/hope/base/fresh",
  runnerAlive: "/hope/runner/alive",
  runnerMode: "/hope/runner/mode",
  runnerFault: "/hope/runner/command_fault_latched",
  localRole: "/hope/runner/local_role",
  roleChangeAllowed: "/hope/runner/role_change_allowed",
  serveCapability: "/hope/runner/serve_capability",
  serveState: "/hope/runner/serve_state",
  standing: "/hope/runner/standing",
  ready: "/hope/runner/ready",
  readyToServe: "/hope/runner/is_ready_to_serve",
  serving: "/hope/runner/serving",
  lastAction: "/hope/runner/last_action",
  lastResult: "/hope/runner/last_action_result",
  lastReason: "/hope/runner/last_action_reason",
  xHitSuccess: "/hope/x_hit/success",
  xHitStatus: "/hope/x_hit/status",
  calibrationSuccess: "/hope/calibration/success",
  calibrationStatus: "/hope/calibration/status",
  lifecycleState: "/hope/lifecycle/state",
  lifecycleStep: "/hope/lifecycle/step",
  lifecycleSession: "/hope/lifecycle/session_id",
  lifecycleResult: "/hope/lifecycle/last_result",
  lifecycleBusy: "/hope/lifecycle/busy",
  lifecycleConfigRevision: "/hope/lifecycle/config/revision",
  laptopWifiIp: "/hope/lifecycle/config/laptop_wifi_ip",
  hduWifiIp: "/hope/lifecycle/config/hdu_wifi_ip",
  mduInternalIp: "/hope/lifecycle/config/mdu_internal_ip",
  motiveIp: "/hope/lifecycle/config/motive_ip",
} as const;

const SERVICES = {
  estop: "/hope/safety/trigger_estop",
  setServer: "/hope/runner/set_server",
  setReceiver: "/hope/runner/set_receiver",
  stand: "/hope/runner/enter_pd_stand",
  calibration: "/hope/calibrate",
  refreshXHit: "/hope/refresh_x_hit",
  ready: "/hope/runner/enter_motion",
  readyToServe: "/hope/runner/ready_to_serve",
  serve: "/hope/runner/serve",
  passive: "/hope/runner/emergency_passive",
  applyLifecycleConfig: "/hope/lifecycle/apply_config",
  startLifecycle: "/hope/lifecycle/start",
  killAllAndCollect: "/hope/lifecycle/kill_all_and_collect",
} as const;

type ServiceKey = keyof typeof SERVICES;
type TopicName = (typeof TOPICS)[keyof typeof TOPICS];

type CpuSample = { at: number; value: number };

const CONFIG_FIELDS = {
  laptop_wifi_ip: { label: "Laptop Wi-Fi", topic: TOPICS.laptopWifiIp },
  hdu_wifi_ip: { label: "HDU Wi-Fi", topic: TOPICS.hduWifiIp },
  mdu_internal_ip: { label: "MDU Internal", topic: TOPICS.mduInternalIp },
  motive_ip: { label: "Motive", topic: TOPICS.motiveIp },
} as const;

type ConfigField = keyof typeof CONFIG_FIELDS;
type LifecycleConfigDraft = Record<ConfigField, string>;

const EMPTY_CONFIG: LifecycleConfigDraft = {
  laptop_wifi_ip: "",
  hdu_wifi_ip: "",
  mdu_internal_ip: "",
  motive_ip: "",
};

type Snapshot = {
  ntpOffsetMs?: number;
  ntpDispersionMs?: number;
  ntpPass?: boolean;
  latencyMs?: number;
  timestampFresh?: boolean;
  cpuPercent?: number;
  cpuTopProcess?: string;
  cpuSamples: CpuSample[];
  agibotPm?: boolean;
  tfReady?: boolean;
  estopReady?: boolean;
  estopFullReady?: boolean;
  estopLatched?: boolean;
  estopText?: string;
  hduActive?: boolean;
  mduActive?: boolean;
  markerCount?: number;
  markersFresh?: boolean;
  baseFresh?: boolean;
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
  calibrationSuccess?: boolean;
  calibrationStatus?: string;
  lifecycleState?: string;
  lifecycleStep?: string;
  lifecycleSession?: string;
  lifecycleResult?: string;
  lifecycleBusy?: boolean;
  lifecycleConfigRevision?: number;
  lifecycleConfig: Partial<LifecycleConfigDraft>;
  availableTopics: Set<string>;
  lastReceived: Partial<Record<TopicName, number>>;
};

const INITIAL_SNAPSHOT: Snapshot = {
  cpuSamples: [],
  lifecycleConfig: {},
  availableTopics: new Set<string>(),
  lastReceived: {},
};

type ScalarMessage = { data?: unknown };
type TriggerResponse = { success?: unknown; message?: unknown };
type SetParametersResponse = {
  results?: Array<{ successful?: unknown; reason?: unknown }>;
};

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
    case TOPICS.cpuTopProcess:
      next.cpuTopProcess = stringValue(event.message);
      break;
    case TOPICS.agibotPm:
      next.agibotPm = boolValue(event.message);
      break;
    case TOPICS.tfReady:
      next.tfReady = boolValue(event.message);
      break;
    case TOPICS.estopReady:
      next.estopReady = boolValue(event.message);
      break;
    case TOPICS.estopFullReady:
      next.estopFullReady = boolValue(event.message);
      break;
    case TOPICS.estopLatched:
      next.estopLatched = boolValue(event.message);
      break;
    case TOPICS.estopText:
      next.estopText = stringValue(event.message);
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
    case TOPICS.baseFresh:
      next.baseFresh = boolValue(event.message);
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
    case TOPICS.calibrationSuccess:
      next.calibrationSuccess = boolValue(event.message);
      break;
    case TOPICS.calibrationStatus:
      next.calibrationStatus = stringValue(event.message);
      break;
    case TOPICS.lifecycleState:
      next.lifecycleState = stringValue(event.message);
      break;
    case TOPICS.lifecycleStep:
      next.lifecycleStep = stringValue(event.message);
      break;
    case TOPICS.lifecycleSession:
      next.lifecycleSession = stringValue(event.message);
      break;
    case TOPICS.lifecycleResult:
      next.lifecycleResult = stringValue(event.message);
      break;
    case TOPICS.lifecycleBusy:
      next.lifecycleBusy = boolValue(event.message);
      break;
    case TOPICS.lifecycleConfigRevision:
      next.lifecycleConfigRevision = numberValue(event.message);
      break;
    case TOPICS.laptopWifiIp:
      next.lifecycleConfig.laptop_wifi_ip = stringValue(event.message);
      break;
    case TOPICS.hduWifiIp:
      next.lifecycleConfig.hdu_wifi_ip = stringValue(event.message);
      break;
    case TOPICS.mduInternalIp:
      next.lifecycleConfig.mdu_internal_ip = stringValue(event.message);
      break;
    case TOPICS.motiveIp:
      next.lifecycleConfig.motive_ip = stringValue(event.message);
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
  const [configDraft, setConfigDraft] = useState<LifecycleConfigDraft>(EMPTY_CONFIG);
  const [configTouched, setConfigTouched] = useState(false);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      const receivedAt = Date.now();
      const next: Snapshot = {
        ...latest.current,
        cpuSamples: [...latest.current.cpuSamples],
        lifecycleConfig: { ...latest.current.lifecycleConfig },
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

  useEffect(() => {
    if (configTouched) {
      return;
    }
    const complete = (Object.keys(CONFIG_FIELDS) as ConfigField[]).every(
      (name) => snapshot.lifecycleConfig[name] != undefined,
    );
    if (complete) {
      setConfigDraft(snapshot.lifecycleConfig as LifecycleConfigDraft);
    }
  }, [configTouched, snapshot.lifecycleConfig]);

  const invoke = useCallback(async (key: ServiceKey) => {
    if (context.callService == undefined || busy[key] === true) {
      setNotice("Current data source does not expose service calls");
      return;
    }
    setBusy((current) => ({ ...current, [key]: true }));
    setNotice(`${SERVICES[key]} requested…`);
    try {
      const timeoutMs = key === "estop"
        ? 5_000
        : key === "calibration"
          ? 40_000
          : key === "refreshXHit"
            ? 7_000
            : 3_000;
      const raw = await Promise.race([context.callService(SERVICES[key], {}), timeoutAfter(timeoutMs)]);
      const response = triggerResponse(raw);
      setNotice(`${response.success ? "ACCEPTED" : "REJECTED"} · ${response.message}`);
    } catch (error) {
      setNotice(`FAILED · ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      setBusy((current) => ({ ...current, [key]: false }));
    }
  }, [busy, context]);

  const confirmLifecycleConfig = useCallback(async () => {
    const key: ServiceKey = "applyLifecycleConfig";
    if (context.callService == undefined || busy[key] === true) {
      setNotice("Current data source does not expose lifecycle configuration");
      return;
    }
    setBusy((current) => ({ ...current, [key]: true }));
    setNotice("Validating and confirming lifecycle configuration…");
    const parameters = (Object.keys(CONFIG_FIELDS) as ConfigField[]).map((name) => ({
      name,
      value: {
        type: 4,
        bool_value: false,
        integer_value: 0,
        double_value: 0,
        string_value: configDraft[name],
        byte_array_value: [],
        bool_array_value: [],
        integer_array_value: [],
        double_array_value: [],
        string_array_value: [],
      },
    }));
    try {
      const raw = await Promise.race([
        context.callService(SERVICES.applyLifecycleConfig, { parameters }),
        timeoutAfter(3_000),
      ]) as SetParametersResponse;
      const results = Array.isArray(raw.results) ? raw.results : [];
      const rejected = results.find((result) => result.successful !== true);
      if (results.length !== Object.keys(CONFIG_FIELDS).length || rejected != undefined) {
        const reason = typeof rejected?.reason === "string"
          ? rejected.reason
          : "invalid service response";
        setNotice(`CONFIG REJECTED · ${reason}`);
      } else {
        setConfigTouched(false);
        setNotice("CONFIG CONFIRMED · the next start will use these addresses");
      }
    } catch (error) {
      setNotice(`CONFIG FAILED · ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      setBusy((current) => ({ ...current, [key]: false }));
    }
  }, [busy, configDraft, context]);

  const startSystem = useCallback(async () => {
    const accepted = window.confirm(
      "Start STEP 0/1/2A/2B/4/5 now? Confirm the robot is physically supported and the hardware E-stop is reachable. The Runner will start in PASSIVE.",
    );
    if (accepted) {
      await invoke("startLifecycle");
    }
  }, [invoke]);

  const killAllAndCollect = useCallback(async () => {
    const accepted = window.confirm(
      "Immediately terminate all lifecycle-managed Runner, HAL, Planner, base relay and OptiTrack sessions, restore agibot_pm, then collect logs? The robot may lose active support immediately; physically support it and keep the physical E-stop reachable.",
    );
    if (accepted) {
      await invoke("killAllAndCollect");
    }
  }, [invoke]);

  const ntpFresh = isFresh(snapshot, TOPICS.ntpOffset, now, 1_500);
  const latencyFresh = isFresh(snapshot, TOPICS.latency, now, 500) && snapshot.timestampFresh === true;
  const cpuFresh = isFresh(snapshot, TOPICS.cpu, now, 1_500);
  const hduFresh = isFresh(snapshot, TOPICS.hduActive, now, 1_500);
  const mduFresh = isFresh(snapshot, TOPICS.mduActive, now, 1_500);
  const pmFresh = isFresh(snapshot, TOPICS.agibotPm, now, 1_500);
  const runnerFresh = isFresh(snapshot, TOPICS.runnerAlive, now, 1_500) && snapshot.runnerAlive === true;
  const markerFresh = isFresh(snapshot, TOPICS.markers, now, 1_000) && snapshot.markersFresh === true;
  const baseFresh = isFresh(snapshot, TOPICS.baseFresh, now, 1_000) && snapshot.baseFresh === true;
  const estopUsable = isFresh(snapshot, TOPICS.estopReady, now, 1_000) && snapshot.estopReady === true;
  const estopFullReady = isFresh(snapshot, TOPICS.estopFullReady, now, 1_000) && snapshot.estopFullReady === true;
  // Once observed true, keep the safety indication asserted through a bridge
  // outage. Only a later authoritative false after approved local recovery
  // clears it; stale telemetry must never make the panel look reset.
  const estopAsserted = snapshot.estopLatched === true;
  const runnerUsable = runnerFresh && snapshot.runnerFault !== true;
  const serveAvailable = snapshot.serveCapability === "AVAILABLE";
  const lifecycleFresh = isFresh(snapshot, TOPICS.lifecycleState, now, 1_500);
  const lifecycleState = lifecycleFresh ? snapshot.lifecycleState ?? "UNKNOWN" : "NO DATA";
  const lifecycleStopped = lifecycleState === "STOPPED" || lifecycleState === "CONFIG_ERROR";
  const lifecycleRunning = lifecycleState === "RUNNING" || lifecycleState === "FAILED";
  const lifecycleBusy = snapshot.lifecycleBusy === true;
  const configComplete = Object.values(configDraft).every((value) => value.trim().length > 0);

  const nextStep = snapshot.standing !== true
    ? "stand"
    : snapshot.localRole === "SERVER"
      ? snapshot.readyToServe !== true && snapshot.serving !== true
        ? "readyToServe"
        : snapshot.readyToServe === true
            ? "serve"
            : "none"
      : snapshot.calibrationSuccess !== true
        ? "calibration"
        : snapshot.xHitSuccess !== true
          ? "refreshXHit"
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
        <button
          className={`estop-button ${estopAsserted ? "estop-asserted" : ""}`}
          type="button"
          disabled={busy.estop === true}
          title={notice}
          onClick={() => void invoke("estop")}
        >
          {busy.estop === true ? "ASSERTING…" : estopAsserted ? "E-STOP ASSERTED" : "E-STOP"}
          <small>{estopAsserted ? "CLICK TO REASSERT · LOCAL RECOVERY REQUIRED" : estopFullReady ? "DUAL PATH · ASSERT ONLY · NO RESET" : estopUsable ? "PARTIAL SOFTWARE STOP · USE PHYSICAL E-STOP" : "BACKEND UNKNOWN · CLICK STILL ASSERTS"}</small>
        </button>
      </div>

      <div className="gate-row">
        <GateChip label="NTP · AUDIT" value={ntpFresh ? snapshot.ntpPass : undefined} />
        <GateChip label="TIMESTAMP · AUDIT" value={latencyFresh ? true : undefined} />
        <GateChip label="TF · AUDIT" value={isFresh(snapshot, TOPICS.tfReady, now, 1_000) ? snapshot.tfReady : undefined} />
        <GateChip label="E-STOP BACKEND · AUDIT" value={estopAsserted ? false : estopUsable ? estopFullReady : undefined} detail={snapshot.estopText} />
        <GateChip label={`MARKERS · AUDIT ${markerFresh ? Math.round(snapshot.markerCount ?? 0) : "—"}/10`} value={markerFresh ? snapshot.markerCount === 10 : undefined} />
      </div>

      <div className={`operator-notice ${estopAsserted ? "operator-notice-danger" : ""}`}>
        <span className="eyebrow">LAST UI REQUEST</span>
        <span>{notice}</span>
      </div>

      <div className="lifecycle-card">
        <div className="lifecycle-header">
          <div>
            <div className="eyebrow">SYSTEM LIFECYCLE · STEP 0/1/2A/2B/4/5</div>
            <div className="lifecycle-state">
              {lifecycleState} · {snapshot.lifecycleStep ?? "IDLE"}
            </div>
            <div className="topic-path">
              {snapshot.lifecycleSession != undefined && snapshot.lifecycleSession.length > 0
                ? snapshot.lifecycleSession
                : "no active session"} · config revision {snapshot.lifecycleConfigRevision ?? 0}
            </div>
          </div>
          <div className="lifecycle-result" title={snapshot.lifecycleResult}>
            {snapshot.lifecycleResult ?? "waiting for supervisor"}
          </div>
        </div>
        <div className="config-grid">
          {(Object.keys(CONFIG_FIELDS) as ConfigField[]).map((name) => (
            <label className="config-field" key={name}>
              <span>{CONFIG_FIELDS[name].label}</span>
              <input
                type="text"
                inputMode="decimal"
                spellCheck={false}
                value={configDraft[name]}
                placeholder="IPv4 address"
                onChange={(event) => {
                  setConfigTouched(true);
                  setConfigDraft((current) => ({ ...current, [name]: event.target.value }));
                }}
              />
            </label>
          ))}
        </div>
        <div className="lifecycle-actions">
          <button
            className="config-confirm"
            type="button"
            disabled={!configComplete || !lifecycleStopped || lifecycleBusy || busy.applyLifecycleConfig === true}
            onClick={() => void confirmLifecycleConfig()}
          >
            {busy.applyLifecycleConfig === true ? "CONFIRMING…" : "CONFIRM CONFIG"}
          </button>
          <button
            className="system-start"
            type="button"
            disabled={!lifecycleStopped || lifecycleBusy || (snapshot.lifecycleConfigRevision ?? 0) < 1 || configTouched || busy.startLifecycle === true}
            onClick={() => void startSystem()}
          >
            {busy.startLifecycle === true || (lifecycleBusy && lifecycleState === "STARTING") ? "STARTING…" : "START SYSTEM"}
          </button>
          <button
            className="system-stop"
            type="button"
            disabled={!lifecycleRunning || lifecycleBusy || busy.killAllAndCollect === true}
            onClick={() => void killAllAndCollect()}
          >
            {busy.killAllAndCollect === true || (lifecycleBusy && lifecycleState === "KILLING") ? "KILLING…" : "KILL ALL & COLLECT"}
          </button>
          <span className="config-helper">
            Inputs stay editable. Confirm is accepted only while stopped; changing HDU Wi-Fi also requires reconnecting Foxglove to the new :8766 address.
          </span>
        </div>
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
          <div className="cpu-diagnostics">
            <span className="topic-path">{TOPICS.cpu} · 0–100 %, 120 s</span>
            <span className="topic-path" title={snapshot.cpuTopProcess}>{snapshot.cpuTopProcess ?? "top process warming up"}</span>
          </div>
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
          <div className="sequence-summary">
            <span className="eyebrow">RUNNER SEQUENCE</span>
            <span className="sequence-status">{runnerFresh ? `${snapshot.runnerMode ?? "UNKNOWN"} · ${snapshot.serveState ?? "UNAVAILABLE"}` : "NO FRESH RUNNER STATE"}</span>
          </div>
          <div className="role-controls">
            <span className="role-label">OUR ROLE: {snapshot.localRole ?? "UNASSIGNED"}</span>
            <button type="button" disabled={snapshot.roleChangeAllowed !== true || busy.setServer === true} onClick={() => void invoke("setServer")}>SERVER</button>
            <button type="button" disabled={snapshot.roleChangeAllowed !== true || busy.setReceiver === true} onClick={() => void invoke("setReceiver")}>RECEIVER</button>
          </div>
        </div>
        <div className="action-grid">
          <ActionButton label="Stand" detail={snapshot.standing === true ? "DONE · PD_STAND" : "same as keyboard s"} disabled={!runnerUsable} busy={busy.stand === true} completed={snapshot.standing === true} next={nextStep === "stand"} onClick={() => void invoke("stand")} />
          <ActionButton label="Calibration" detail={snapshot.calibrationSuccess === true ? "DONE · world→pelvis JSON" : snapshot.standing === true ? snapshot.calibrationStatus ?? "recompute and persist world→pelvis" : "LOCKED · stand first"} disabled={!runnerUsable || snapshot.standing !== true} busy={busy.calibration === true} completed={snapshot.calibrationSuccess === true} next={nextStep === "calibration"} onClick={() => void invoke("calibration")} />
          <ActionButton label="Refresh x_hit" detail={snapshot.xHitSuccess === true ? snapshot.xHitStatus ?? "DONE · Planner acknowledged" : snapshot.standing === true ? snapshot.xHitStatus ?? "refresh Planner x_hit only" : "LOCKED · stand first"} disabled={!runnerUsable || snapshot.standing !== true} busy={busy.refreshXHit === true} completed={snapshot.xHitSuccess === true} next={nextStep === "refreshXHit"} onClick={() => void invoke("refreshXHit")} />
          <ActionButton label="Ready" detail={snapshot.ready === true ? "DONE · MOTION" : snapshot.standing !== true ? "LOCKED · stand first" : !baseFresh ? "LOCKED · wait for fresh Pelvis base" : "same as keyboard m"} disabled={!runnerUsable || snapshot.standing !== true || !baseFresh} busy={busy.ready === true} completed={snapshot.ready === true} next={nextStep === "ready"} onClick={() => void invoke("ready")} />
          <ActionButton label="Ready to Serve" detail={!serveAvailable ? "UNAVAILABLE · launch Runner with --serve" : snapshot.readyToServe === true ? "DONE · ball on palm" : "start serve pre-position"} disabled={!runnerUsable || !serveAvailable} busy={busy.readyToServe === true} completed={snapshot.readyToServe === true} next={nextStep === "readyToServe"} onClick={() => void invoke("readyToServe")} />
          <ActionButton label="Serve" detail={snapshot.serving === true ? "SERVING" : snapshot.readyToServe === true ? "confirm ball on palm" : "LOCKED UNTIL AWAIT_BALL_ON_PALM"} disabled={!runnerUsable || snapshot.readyToServe !== true} busy={busy.serve === true} completed={snapshot.serving === true} next={nextStep === "serve"} onClick={() => void invoke("serve")} />
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
