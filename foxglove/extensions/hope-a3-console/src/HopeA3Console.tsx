import { MessageEvent, PanelExtensionContext, Topic } from "@foxglove/extension";
import {
  ReactElement,
  useCallback,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { createRoot } from "react-dom/client";

import { decodeTeleopState, teleopEntryReason, xboxInputFresh } from "./teleopStatus";

import "@fontsource/ibm-plex-mono/400.css";
import "@fontsource/ibm-plex-mono/500.css";
import "@fontsource/ibm-plex-mono/600.css";
import "@fontsource/ibm-plex-sans/400.css";
import "@fontsource/ibm-plex-sans/500.css";
import "@fontsource/space-grotesk/600.css";
import "@fontsource/space-grotesk/700.css";
import "./styles.css";

const TOPICS = {
  fieldStatus: "/hope/field/status",
  ntpOffset: "/hope/ntp/offset_ms",
  ntpSkew: "/hope/ntp/skew_ppm",
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
  markerAsset: "/hope/mocap/marker_asset",
  markerExpected: "/hope/mocap/marker_expected_count",
  markers: "/hope/mocap/marker_count",
  markersFresh: "/hope/mocap/marker_fresh",
  baseFresh: "/hope/base/fresh",
  commandValid: "/hope/command/valid",
  commandCountdown: "/hope/command/hdu_wall_countdown_s",
  commandFlight: "/hope/command/flight_id",
  commandSummary: "/hope/command/summary",
  runnerAlive: "/hope/runner/alive",
  runnerMode: "/hope/runner/mode",
  xboxPreview: "/hope/runner/xbox_preview",
  teleopState: "/hope/runner/teleop_state_hdu_flat",
  runnerFault: "/hope/runner/command_fault_latched",
  localRole: "/hope/runner/local_role",
  roleChangeAllowed: "/hope/runner/role_change_allowed",
  serveCapability: "/hope/runner/serve_capability",
  serveState: "/hope/runner/serve_state",
  gripperState: "/hope/runner/gripper_state",
  serveCleanupRequired: "/hope/runner/serve_cleanup_required",
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
  calibrationProfile: "/hope/calibration/profile",
  lifecycleState: "/hope/lifecycle/state",
  lifecycleStep: "/hope/lifecycle/step",
  lifecycleSession: "/hope/lifecycle/session_id",
  lifecycleResult: "/hope/lifecycle/last_result",
  lifecycleBusy: "/hope/lifecycle/busy",
  lifecycleConfigRevision: "/hope/lifecycle/config/revision",
  timeCalibrationState: "/hope/lifecycle/time_calibration/state",
  timeCalibrationStep: "/hope/lifecycle/time_calibration/step",
  timeCalibrationResult: "/hope/lifecycle/time_calibration/result",
  timeCalibrationOperation: "/hope/lifecycle/time_calibration/operation_id",
  timeCalibrationBusy: "/hope/lifecycle/time_calibration/busy",
  laptopWifiIp: "/hope/lifecycle/config/laptop_wifi_ip",
  hduWifiIp: "/hope/lifecycle/config/hdu_wifi_ip",
  mduInternalIp: "/hope/lifecycle/config/mdu_internal_ip",
  motiveIp: "/hope/lifecycle/config/motive_ip",
  tableSide: "/hope/lifecycle/config/table_side",
} as const;

const SERVICES = {
  estop: "/hope/safety/trigger_estop",
  resetSoftwareEstop: "/hope/safety/reset_software_estop",
  setServer: "/hope/runner/set_server",
  setReceiver: "/hope/runner/set_receiver",
  stand: "/hope/runner/enter_pd_stand",
  calibrationStickers: "/hope/calibrate_stickers_v3",
  calibrationV2: "/hope/calibrate_v2",
  calibrationV3: "/hope/calibrate_v3",
  refreshXHit: "/hope/refresh_x_hit",
  ready: "/hope/runner/enter_motion",
  teleop: "/hope/runner/enter_teleop",
  prepareServe: "/hope/runner/prepare_serve",
  confirmBallLoaded: "/hope/runner/confirm_ball_loaded",
  readyToServe: "/hope/runner/ready_to_serve",
  openGripper: "/hope/runner/open_gripper",
  passive: "/hope/runner/emergency_passive",
  applyLifecycleConfig: "/hope/lifecycle/apply_config",
  startLifecycle: "/hope/lifecycle/start",
  killAllAndCollect: "/hope/lifecycle/kill_all_and_collect",
  timeCalibration: "/hope/lifecycle/time_calibration",
} as const;

type ServiceKey = keyof typeof SERVICES;
type TopicName = (typeof TOPICS)[keyof typeof TOPICS];

type FieldStatus = {
  mode: "OPTITRACK" | "KERNEL";
  calibration?: { name: string; sha256: string; profile: string; table_side: string } | null;
  calibration_sync: string;
  csv?: { name: string; sha256: string; frames: number; hz: number } | null;
  recording: { state: string; output: string; error: string };
  busy?: boolean;
  error?: string;
};

type CpuSample = { at: number; value: number };

const CONFIG_FIELDS = {
  laptop_wifi_ip: { label: "Laptop IP", topic: TOPICS.laptopWifiIp },
  hdu_wifi_ip: { label: "HDU IP", topic: TOPICS.hduWifiIp },
  mdu_internal_ip: { label: "MDU Internal", topic: TOPICS.mduInternalIp },
  motive_ip: { label: "Motive (optional telemetry)", topic: TOPICS.motiveIp },
  table_side: { label: "Our table side", topic: TOPICS.tableSide },
} as const;

type ConfigField = keyof typeof CONFIG_FIELDS;
const NETWORK_CONFIG_FIELDS: ConfigField[] = [
  "laptop_wifi_ip",
  "hdu_wifi_ip",
  "mdu_internal_ip",
  "motive_ip",
];
const RUNNER_NETWORK_CONFIG_FIELDS: ConfigField[] = [
  "laptop_wifi_ip",
  "hdu_wifi_ip",
  "mdu_internal_ip",
];
type LifecycleConfigDraft = Record<ConfigField, string>;

const EMPTY_CONFIG: LifecycleConfigDraft = {
  laptop_wifi_ip: "",
  hdu_wifi_ip: "",
  mdu_internal_ip: "",
  motive_ip: "",
  table_side: "",
};

type XboxPreview = {
  connected: boolean;
  device: string;
  enabled: boolean;
  lt: boolean;
  action_status?: string;
  estop_requested?: boolean;
  action_pending?: boolean;
  axes: number[];
  velocity: number[];
  state: string;
  source_wall_ms: number;
};

function xboxPreviewValue(message: unknown): XboxPreview | undefined {
  const value = stringValue(message);
  if (value == undefined) { return undefined; }
  try {
    const parsed: unknown = JSON.parse(value);
    if (parsed == undefined || typeof parsed !== "object") { return undefined; }
    const p = parsed as Record<string, unknown>;
    if (p.schema !== 1 || p.purpose !== "XBOX_INPUT" ||
      typeof p.connected !== "boolean" || typeof p.device !== "string" ||
      typeof p.enabled !== "boolean" || typeof p.lt !== "boolean" ||
      typeof p.state !== "string" || typeof p.source_wall_ms !== "number" ||
      !Number.isFinite(p.source_wall_ms) || !Array.isArray(p.axes) ||
      p.axes.length !== 3 || !p.axes.every((x: unknown) => typeof x === "number" && Number.isFinite(x)) ||
      !Array.isArray(p.velocity) ||
      p.velocity.length !== 3 || !p.velocity.every((x: unknown) => typeof x === "number" && Number.isFinite(x))) { return undefined; }
    return p as unknown as XboxPreview;
  } catch { return undefined; }
}

type Snapshot = {
  xboxPreview?: XboxPreview;
  teleopState?: number[];
  ntpOffsetMs?: number;
  ntpSkewPpm?: number;
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
  markerAsset?: string;
  markerCount?: number;
  markerExpected?: number;
  markersFresh?: boolean;
  baseFresh?: boolean;
  commandValid?: boolean;
  commandCountdownS?: number;
  commandFlight?: number;
  commandSummary?: string;
  runnerAlive?: boolean;
  runnerMode?: string;
  runnerFault?: boolean;
  localRole?: string;
  roleChangeAllowed?: boolean;
  serveCapability?: string;
  serveState?: string;
  gripperState?: string;
  serveCleanupRequired?: boolean;
  standing?: boolean;
  ready?: boolean;
  readyToServe?: boolean;
  serving?: boolean;
  lastAction?: string;
  lastResult?: string;
  lastReason?: string;
  xHitSuccess?: boolean;
  xHitStatus?: string;
  field?: FieldStatus;
  calibrationSuccess?: boolean;
  calibrationStatus?: string;
  calibrationProfile?: string;
  lifecycleState?: string;
  lifecycleStep?: string;
  lifecycleSession?: string;
  lifecycleResult?: string;
  lifecycleBusy?: boolean;
  lifecycleConfigRevision?: number;
  timeCalibrationState?: string;
  timeCalibrationStep?: string;
  timeCalibrationResult?: string;
  timeCalibrationOperation?: string;
  timeCalibrationBusy?: boolean;
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
    case TOPICS.fieldStatus:
      try {
        next.field = JSON.parse(stringValue(event.message) ?? "null") as FieldStatus;
      } catch {
        next.field = undefined;
      }
      break;
    case TOPICS.ntpOffset:
      next.ntpOffsetMs = numberValue(event.message);
      break;
    case TOPICS.ntpSkew:
      next.ntpSkewPpm = numberValue(event.message);
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
    case TOPICS.markerAsset:
      next.markerAsset = stringValue(event.message);
      break;
    case TOPICS.markerExpected:
      next.markerExpected = numberValue(event.message);
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
    case TOPICS.commandValid:
      next.commandValid = boolValue(event.message);
      break;
    case TOPICS.commandCountdown:
      next.commandCountdownS = numberValue(event.message);
      break;
    case TOPICS.commandFlight:
      next.commandFlight = numberValue(event.message);
      break;
    case TOPICS.commandSummary:
      next.commandSummary = stringValue(event.message);
      break;
    case TOPICS.runnerAlive:
      next.runnerAlive = boolValue(event.message);
      break;
    case TOPICS.runnerMode:
      next.runnerMode = stringValue(event.message);
      break;
    case TOPICS.xboxPreview:
      next.xboxPreview = xboxPreviewValue(event.message);
      break;
    case TOPICS.teleopState: {
      next.teleopState = decodeTeleopState(scalar(event.message));
      break;
    }
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
    case TOPICS.gripperState:
      next.gripperState = stringValue(event.message);
      break;
    case TOPICS.serveCleanupRequired:
      next.serveCleanupRequired = boolValue(event.message);
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
    case TOPICS.calibrationProfile:
      next.calibrationProfile = stringValue(event.message);
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
    case TOPICS.timeCalibrationState:
      next.timeCalibrationState = stringValue(event.message);
      break;
    case TOPICS.timeCalibrationStep:
      next.timeCalibrationStep = stringValue(event.message);
      break;
    case TOPICS.timeCalibrationResult:
      next.timeCalibrationResult = stringValue(event.message);
      break;
    case TOPICS.timeCalibrationOperation:
      next.timeCalibrationOperation = stringValue(event.message);
      break;
    case TOPICS.timeCalibrationBusy:
      next.timeCalibrationBusy = boolValue(event.message);
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
    case TOPICS.tableSide:
      next.lifecycleConfig.table_side = stringValue(event.message);
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
    message:
      typeof response.message === "string" ? response.message : "service returned no message",
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

function GateChip({
  label,
  value,
  detail,
}: {
  label: string;
  value?: boolean;
  detail?: string;
}): ReactElement {
  const state = value == undefined ? "unknown" : value ? "ok" : "attention";
  return (
    <div className={`gate gate-${state}`} title={detail}>
      <span className="gate-dot" />
      {label}
      {value === false ? " · CHECK" : ""}
    </div>
  );
}

function ProcessTile({
  name,
  topic,
  value,
}: {
  name: string;
  topic: string;
  value?: boolean;
}): ReactElement {
  const state = value == undefined ? "unknown" : value ? "running" : "stopped";
  return (
    <div className={`process-tile process-${state}`}>
      <div className="process-state">
        <span className="process-dot" />
        {state === "unknown" ? "NO DATA" : state.toUpperCase()}
      </div>
      <div className="process-name">{name}</div>
      <div className="topic-path">{topic}</div>
    </div>
  );
}

function selectedMarkerAsset(snapshot: Snapshot): string {
  const configuredSide = snapshot.lifecycleConfig.table_side?.trim();
  if (configuredSide === "P1" || configuredSide === "P2") {
    return `UCB_${configuredSide}`;
  }
  const reportedAsset = snapshot.markerAsset?.trim();
  return reportedAsset === "UCB_P1" || reportedAsset === "UCB_P2"
    ? reportedAsset
    : "SIDE UNSET";
}

function calibrationDetail(snapshot: Snapshot, profile: "V2" | "V3" | "STICKERS_V3"): string {
  const tableSide = selectedMarkerAsset(snapshot);
  if (snapshot.calibrationSuccess === true && snapshot.calibrationProfile === profile) {
    return `${tableSide} · DONE · ${profile} pants · world→pelvis JSON`;
  }
  if (snapshot.calibrationProfile === profile && snapshot.calibrationStatus != undefined) {
    return `${tableSide} · ${snapshot.calibrationStatus}`;
  }
  if (snapshot.calibrationSuccess === true && snapshot.calibrationProfile != undefined) {
    return `${tableSide} · ${snapshot.calibrationProfile} is active · click only after installing ${profile} pants`;
  }
  if (snapshot.runnerMode === "PD_STAND") {
    return profile === "V2"
      ? `${tableSide} · legacy V2 ten-point layout · replaces active receipt`
      : `${tableSide} · new V3 ten-point layout · replaces active receipt`;
  }
  return `${tableSide} · LOCKED · stand first`;
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
  ]
    .filter(Boolean)
    .join(" ");
  return (
    <button
      className={classes}
      type="button"
      disabled={props.disabled || props.busy}
      onClick={props.onClick}
    >
      <span>{props.busy ? "Working…" : props.label}</span>
      <small>{props.detail}</small>
    </button>
  );
}

function HopeA3Console({ context }: { context: PanelExtensionContext }): ReactElement {
  const latest = useRef<Snapshot>(INITIAL_SNAPSHOT);
  const [snapshot, setSnapshot] = useState<Snapshot>(INITIAL_SNAPSHOT);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [, refreshClock] = useState(0);
  // Samples can arrive between timer ticks. Compare them with the time of THIS
  // render, otherwise a new packet appears up to 250 ms in the future.
  const now = Date.now();
  const [busy, setBusy] = useState<Partial<Record<ServiceKey, boolean>>>({});
  const [notice, setNotice] = useState("Waiting for authoritative Runner state");
  const [configDraft, setConfigDraft] = useState<LifecycleConfigDraft>(EMPTY_CONFIG);
  const [configTouched, setConfigTouched] = useState(false);
  const [fieldBusy, setFieldBusy] = useState(false);
  const [recordBusy, setRecordBusy] = useState(false);
  const [recalibrating, setRecalibrating] = useState(false);
  const calibrationInput = useRef<HTMLInputElement>(null);
  const csvInput = useRef<HTMLInputElement>(null);

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
      refreshClock((tick) => tick + 1);
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

  const invoke = useCallback(
    async (key: ServiceKey) => {
      if (context.callService == undefined || busy[key] === true) {
        setNotice("Current data source does not expose service calls");
        return;
      }
      setBusy((current) => ({ ...current, [key]: true }));
      setNotice(`${SERVICES[key]} requested…`);
      try {
        const timeoutMs =
          key === "estop"
            ? 5_000
            : key === "calibrationV2" || key === "calibrationV3" || key === "calibrationStickers"
              ? 40_000
              : key === "refreshXHit"
                ? 7_000
                : key === "timeCalibration"
                  ? 10_000
                  : 3_000;
        const raw = await Promise.race([
          context.callService(SERVICES[key], {}),
          timeoutAfter(timeoutMs),
        ]);
        const response = triggerResponse(raw);
        setNotice(`${response.success ? "ACCEPTED" : "REJECTED"} · ${response.message}`);
      } catch (error) {
        setNotice(`FAILED · ${error instanceof Error ? error.message : String(error)}`);
      } finally {
        setBusy((current) => ({ ...current, [key]: false }));
      }
    },
    [busy, context],
  );

  const fieldCommand = useCallback(async (payload: Record<string, unknown>) => {
    const recording = payload.op === "record_on" || payload.op === "record_off";
    const setPending = recording ? setRecordBusy : setFieldBusy;
    setPending(true);
    try {
      if (context.callService == undefined) {throw new Error("Connect to the control bridge :8766");}
      const raw = (await context.callService("/hope/field/command", {
        parameters: [{ name: "request", value: {
          type: 4, string_value: JSON.stringify(payload), bool_value: false,
          integer_value: 0, double_value: 0, byte_array_value: [], bool_array_value: [],
          integer_array_value: [], double_array_value: [], string_array_value: [],
        } }],
      })) as SetParametersResponse;
      const result = raw.results?.[0];
      if (result?.successful !== true) {throw new Error(typeof result?.reason === "string" ? result.reason : "invalid response");}
      if (payload.op === "export_calibration") {
        const exported = JSON.parse(String(result.reason)) as { name: string; content: string };
        const url = URL.createObjectURL(new Blob([exported.content], { type: "application/json" }));
        const anchor = document.createElement("a");
        anchor.href = url;
        anchor.download = exported.name;
        anchor.click();
        window.setTimeout(() => { URL.revokeObjectURL(url); }, 1_000);
      }
      setNotice(`ACCEPTED · ${String(payload.op)} · follow the status shown below`);
    } catch (error) {
      setNotice(`REJECTED · ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      setPending(false);
    }
  }, [context]);

  const uploadAsset = useCallback(async (file: File | undefined, kind: "calibration" | "csv") => {
    if (file == undefined) {return;}
    const maximum = (kind === "csv" ? 8 : 1) * 1024 * 1024;
    if (file.size > maximum) {
      setNotice(`REJECTED · ${kind} exceeds ${maximum / 1024 / 1024} MiB`);
      return;
    }
    try {
      await fieldCommand({ op: kind === "csv" ? "upload_csv" : "load_calibration",
        name: file.name, content: await file.text() });
    } catch (error) {
      setNotice(`File read failed · ${String(error)}`);
    }
  }, [fieldCommand]);

  const invokeConfirmed = useCallback(
    async (key: ServiceKey, prompt: string) => {
      if (window.confirm(prompt)) {
        await invoke(key);
      }
    },
    [invoke],
  );

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
      const raw = (await Promise.race([
        context.callService(SERVICES.applyLifecycleConfig, { parameters }),
        timeoutAfter(3_000),
      ])) as SetParametersResponse;
      const results = Array.isArray(raw.results) ? raw.results : [];
      const rejected = results.find((result) => result.successful !== true);
      if (results.length !== Object.keys(CONFIG_FIELDS).length || rejected != undefined) {
        const reason =
          typeof rejected?.reason === "string" ? rejected.reason : "invalid service response";
        setNotice(`CONFIG REJECTED · ${reason}`);
      } else {
        setConfigTouched(false);
        setNotice(
          `CONFIG CONFIRMED · next start uses ${configDraft.table_side} and these addresses`,
        );
      }
    } catch (error) {
      setNotice(`CONFIG FAILED · ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      setBusy((current) => ({ ...current, [key]: false }));
    }
  }, [busy, configDraft, context]);

  const startSystem = useCallback(async () => {
    const accepted = window.confirm(
      snapshot.field?.mode === "KERNEL"
        ? "Start Kernel Mode with the selected CSV? Confirm the robot is physically supported and the hardware E-stop is reachable. Runner starts in PASSIVE; X/Ready and serve completion run the receive policy with local stance-foot odometry. No OptiTrack or Planner will be started."
        : "Start STEP 0/1/2A/2B/4/5 now? Confirm the robot is physically supported and the hardware E-stop is reachable. The Runner will start in PASSIVE.",
    );
    if (accepted) {
      await invoke("startLifecycle");
    }
  }, [invoke, snapshot.field?.mode]);

  const killAllAndCollect = useCallback(async () => {
    const accepted = window.confirm(
      snapshot.field?.mode === "KERNEL"
        ? "Stop the Kernel Mode Runner and HAL and restore agibot_pm? The robot may lose active support immediately; physically support it and keep the physical E-stop reachable. Diagnostic logs remain on the robot. Recording remains under its own switch."
        : "Immediately terminate all lifecycle-managed Runner, HAL, Planner and base-relay sessions, restore agibot_pm, then collect logs? The robot may lose active support immediately; physically support it and keep the physical E-stop reachable. Any separately managed OptiTrack session is left untouched.",
    );
    if (accepted) {
      await invoke("killAllAndCollect");
    }
  }, [invoke, snapshot.field?.mode]);

  const runTimeCalibration = useCallback(async () => {
    const accepted = window.confirm(
      "Run the controlled 10.4 HDU time calibration now? Continue only when the robot is physically supported, no Policy/Runner session is active, and the physical E-stop is reachable. If an earlier managed MDU stop left a valid same-boot receipt, its exact recorded baseline will be restored automatically before preflight; arbitrary stopped services are never started. MDU and HDU time consumers will then stop, HDU UTC will hard-step once, and services will recover in dependency order. Foxglove :8766 will disconnect temporarily.",
    );
    if (accepted) {
      await invoke("timeCalibration");
    }
  }, [invoke]);

  const ntpFresh = isFresh(snapshot, TOPICS.ntpOffset, now, 1_500);
  const ntpGateFresh = isFresh(snapshot, TOPICS.ntpGate, now, 1_500);
  const latencyFresh =
    isFresh(snapshot, TOPICS.latency, now, 500) && snapshot.timestampFresh === true;
  const cpuFresh = isFresh(snapshot, TOPICS.cpu, now, 1_500);
  const hduFresh = isFresh(snapshot, TOPICS.hduActive, now, 1_500);
  const mduFresh = isFresh(snapshot, TOPICS.mduActive, now, 1_500);
  const pmFresh = isFresh(snapshot, TOPICS.agibotPm, now, 1_500);
  const runnerFresh =
    isFresh(snapshot, TOPICS.runnerAlive, now, 1_500) && snapshot.runnerAlive === true;
  const markerFresh =
    isFresh(snapshot, TOPICS.markers, now, 1_000) &&
    snapshot.markersFresh === true;
  const markerAsset = selectedMarkerAsset(snapshot);
  const baseFresh = isFresh(snapshot, TOPICS.baseFresh, now, 1_000) && snapshot.baseFresh === true;
  const estopUsable =
    isFresh(snapshot, TOPICS.estopReady, now, 1_000) && snapshot.estopReady === true;
  const estopFullReady =
    isFresh(snapshot, TOPICS.estopFullReady, now, 1_000) && snapshot.estopFullReady === true;
  // Once observed true, keep the safety indication asserted through a bridge
  // outage. Only a later authoritative false after an explicit software reset
  // clears it; stale telemetry must never make the panel look reset.

  const runnerUsable = runnerFresh && snapshot.runnerFault !== true;
  // Use the same mode receipt for button admission and completion indicators.
  // The observer's separate standing/ready topics can arrive on another frame.
  const runnerStanding = runnerFresh && snapshot.runnerMode === "PD_STAND";
  const runnerReady = runnerFresh && snapshot.runnerMode === "MOTION";
  const xboxFresh = xboxInputFresh(snapshot.lastReceived[TOPICS.xboxPreview],
    snapshot.xboxPreview?.source_wall_ms, now);
  const estopAsserted = snapshot.estopLatched === true ||
    (xboxFresh && snapshot.xboxPreview?.estop_requested === true);
  const teleopFresh = isFresh(snapshot, TOPICS.teleopState, now, 1000);
  const teleop = teleopFresh ? snapshot.teleopState : undefined;
  const teleopBlocked = teleopEntryReason({ runnerFresh: runnerFresh &&
    isFresh(snapshot, TOPICS.runnerMode, now, 1_000) &&
    isFresh(snapshot, TOPICS.runnerFault, now, 1_000), runnerFault: snapshot.runnerFault !== false,
    mode: snapshot.runnerMode, telemetryFresh: teleopFresh, state: teleop, pending: busy.teleop === true,
    xboxFresh, xbox: snapshot.xboxPreview, estop: estopAsserted });
  const xboxVelocity = xboxFresh ? snapshot.xboxPreview?.velocity : undefined;
  const velocityText = (values: number[] | undefined) => values == undefined
    ? "vx — · vy — · yaw —"
    : `vx ${(values[0] ?? 0).toFixed(2)} m/s · vy ${(values[1] ?? 0).toFixed(2)} m/s · yaw ${(values[2] ?? 0).toFixed(2)} rad/s`;

  const serveAvailable = snapshot.serveCapability === "AVAILABLE";
  const lifecycleFresh = isFresh(snapshot, TOPICS.lifecycleState, now, 1_500);
  const lifecycleState = lifecycleFresh ? (snapshot.lifecycleState ?? "UNKNOWN") : "NO DATA";
  const lifecycleStopped = lifecycleState === "STOPPED" || lifecycleState === "CONFIG_ERROR";
  const lifecycleRunning = lifecycleState === "RUNNING" || lifecycleState === "FAILED";
  const lifecycleBusy = snapshot.lifecycleBusy === true;
  const fieldFresh = isFresh(snapshot, TOPICS.fieldStatus, now, 2_000);
  const field = snapshot.field;
  const kernelMode = field?.mode === "KERNEL";
  const assetEditable = fieldFresh && lifecycleState === "STOPPED" && !lifecycleBusy &&
    !fieldBusy && field?.busy !== true && !configTouched && (snapshot.lifecycleConfigRevision ?? 0) > 0;
  const recordingState = fieldFresh ? (field?.recording.state ?? "UNKNOWN") : "UNKNOWN";
  const recordingActive = ["ON", "STARTING", "STOPPING"].includes(recordingState);
  const savedCalibration = field?.calibration;
  const calibrationMatchesSide = savedCalibration?.table_side === snapshot.lifecycleConfig.table_side;
  const configComplete =
    RUNNER_NETWORK_CONFIG_FIELDS.every((name) => configDraft[name].trim().length > 0) &&
    ["P1", "P2"].includes(configDraft.table_side);
  const timeCalibrationFresh = isFresh(snapshot, TOPICS.timeCalibrationState, now, 1_500);
  const timeCalibrationState = timeCalibrationFresh
    ? (snapshot.timeCalibrationState ?? "UNKNOWN")
    : "NO DATA";
  const timeCalibrationBusy = snapshot.timeCalibrationBusy === true;
  const timeCalibrationLocked = ["RUNNING", "INTERRUPTED", "FAILED_SAFE_STOP", "COMPLETE"].includes(
    timeCalibrationState,
  );
  const timeCalibrationBlocksStart =
    timeCalibrationBusy ||
    ["RUNNING", "INTERRUPTED", "FAILED_SAFE_STOP"].includes(timeCalibrationState);
  const timeCalibrationReady =
    ntpFresh &&
    ntpGateFresh &&
    snapshot.ntpPass === false &&
    timeCalibrationFresh &&
    lifecycleStopped &&
    !lifecycleBusy &&
    (snapshot.lifecycleConfigRevision ?? 0) >= 1 &&
    !configTouched &&
    !timeCalibrationBusy &&
    !timeCalibrationLocked;
  const timeCalibrationTitle = !timeCalibrationFresh
    ? "Waiting for the time-calibration coordinator"
    : !ntpFresh || !ntpGateFresh
      ? "Waiting for a fresh NTP gate"
      : snapshot.ntpPass !== false
        ? "Available only when the fresh NTP gate is failing"
        : !lifecycleStopped || lifecycleBusy
          ? "Stop the managed lifecycle before calibrating time"
          : configTouched || (snapshot.lifecycleConfigRevision ?? 0) < 1
            ? "Confirm the three Runner addresses and table side before calibrating time"
            : timeCalibrationState === "FAILED_SAFE_STOP"
              ? "Manual recovery is required; do not repeat the hard-step this boot"
              : timeCalibrationState === "COMPLETE"
                ? "One hard-step has already completed in this maintenance cycle"
                : "Run receipt-safe MDU baseline recovery, then the fixed 10.4 sequence";

  const serveState = snapshot.serveState ?? "UNAVAILABLE";
  const commandFresh = isFresh(snapshot, TOPICS.commandSummary, now, 1_500);
  const swingInputReady =
    snapshot.runnerMode === "MOTION"
      ? commandFresh
        ? snapshot.commandValid === true &&
          snapshot.commandCountdownS != undefined &&
          snapshot.commandCountdownS > 0
        : undefined
      : undefined;
  const serveWorkflowActive = ![
    "UNAVAILABLE",
    "IDLE",
    "COMPLETE",
    "ABORTED",
    "FAULT",
  ].includes(serveState);
  const nextStep =
    snapshot.localRole === "SERVER" && serveState === "WAIT_READY_TO_SERVE"
      ? "readyToServe"
      : snapshot.localRole === "SERVER" && serveWorkflowActive
        ? "none"
        : snapshot.runnerMode === "MOTION"
          ? snapshot.localRole === "SERVER" && ["IDLE", "COMPLETE", "ABORTED"].includes(serveState)
            ? "prepareServe" : "none"
          : !runnerStanding
            ? "stand"
            : snapshot.localRole === "SERVER"
              ? serveState === "FAULT"
                ? "none"
                : "prepareServe"
              : snapshot.localRole === "RECEIVER" && !runnerReady
                ? "ready"
                : "none";

  const cpuPoints = useMemo(() => {
    if (snapshot.cpuSamples.length === 0) {
      return "";
    }
    const start = now - 120_000;
    return snapshot.cpuSamples
      .map((sample) => {
        const x = Math.max(0, Math.min(600, ((sample.at - start) / 120_000) * 600));
        const y = 150 - (Math.max(0, Math.min(100, sample.value)) / 100) * 140;
        return `${x.toFixed(1)},${y.toFixed(1)}`;
      })
      .join(" ");
  }, [now, snapshot.cpuSamples]);

  const lastAction =
    snapshot.lastAction != undefined && snapshot.lastAction !== "NONE"
      ? `${snapshot.lastAction} → ${snapshot.lastResult ?? "?"} / ${snapshot.lastReason ?? "?"}`
      : notice;

  return (
    <div className="hope-console">
      <div className="source-row">
        <div className="source-block">
          <div className="eyebrow">A3 SOURCE · 1.8.7</div>
          <div className="source-value">
            <span>{context.dataSourceProfile?.toUpperCase() ?? "ROS 2"} · CONTROL :8766</span>
            <span
              className={`source-dot ${hduFresh && snapshot.hduActive === true ? "source-live" : ""}`}
            />
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
          <small>
            {estopAsserted
              ? "CLICK TO REASSERT · RESET SOFTWARE E-STOP BELOW"
              : estopFullReady
                ? "DUAL PATH · STOPS RUNNER AND REQUESTS VENDOR STOP"
                : estopUsable
                  ? "PARTIAL SOFTWARE STOP · USE PHYSICAL E-STOP"
                  : "BACKEND UNKNOWN · CLICK STILL ASSERTS"}
          </small>
        </button>
      </div>

      <button
        type="button"
        disabled={busy.estop === true || busy.resetSoftwareEstop === true}
        onClick={() => void invoke("resetSoftwareEstop")}
        title="Clears the software latch only. Does not release hardware/vendor emergency stop or start motion."
      >
        {busy.resetSoftwareEstop === true ? "RESETTING…" : "Reset Software E-stop"}
      </button>

      <div className="gate-row">
        <GateChip
          label="NTP · AUDIT"
          value={ntpFresh && ntpGateFresh ? snapshot.ntpPass : undefined}
        />
        <GateChip label="TIMESTAMP · AUDIT" value={latencyFresh ? true : undefined} />
        <GateChip
          label="TF · AUDIT"
          value={isFresh(snapshot, TOPICS.tfReady, now, 1_000) ? snapshot.tfReady : undefined}
        />
        <GateChip
          label="E-STOP BACKEND · AUDIT"
          value={estopAsserted ? false : estopUsable ? estopFullReady : undefined}
          detail={snapshot.estopText}
        />
        <GateChip
          label={`${markerAsset} MARKERS · AUDIT ${markerFresh ? Math.round(snapshot.markerCount ?? 0) : "—"}/${snapshot.markerExpected ?? 24}`}
          value={markerFresh ? snapshot.markerCount === (snapshot.markerExpected ?? 24) : undefined}
          detail={`selected marker asset: ${markerAsset}`}
        />
        <GateChip
          label={`SWING INPUT · AUDIT ${snapshot.runnerMode === "MOTION" ? `F${snapshot.commandFlight ?? "—"} · ${snapshot.commandCountdownS != undefined ? `${snapshot.commandCountdownS.toFixed(2)}s` : "NO COMMAND"}` : "WAIT FOR MOTION"}`}
          value={swingInputReady}
          detail={snapshot.commandSummary ?? "Ball visibility alone is not a swing command"}
        />
      </div>

      <div className={`operator-notice ${estopAsserted ? "operator-notice-danger" : ""}`}>
        <span className="eyebrow">LAST UI REQUEST</span>
        <span>{notice}</span>
      </div>

      <div className="field-toolbar">
        <div>
          <span className="eyebrow">OPERATING MODE · SELECT WHILE STOPPED</span>
          <div className="field-buttons" role="group" aria-label="Operating mode">
            <button type="button" aria-pressed={field?.mode === "OPTITRACK"}
              disabled={!assetEditable} onClick={() => void fieldCommand({ op: "set_mode", mode: "OPTITRACK" })}>
              OptiTrack · Normal play
            </button>
            <button type="button" aria-pressed={kernelMode}
              disabled={!assetEditable} onClick={() => void fieldCommand({ op: "set_mode", mode: "KERNEL" })}>
              Kernel Mode
            </button>
          </div>
          <p className="helper">{!fieldFresh ? "Waiting for field controls · update the HDU backend if unavailable" :
            kernelMode ? "No OptiTrack · IMU + stance-foot odometry · estimated local position/velocity. Serve hands off after contact; X enters receive policy." :
            "Normal play · start OptiTrack separately when needed. Mode selection starts no processes."}</p>
        </div>
        <div className={`record-control ${recordingActive ? "record-active" : ""}`}>
          <span className="eyebrow">RECORDING · {recordingState}</span>
          <button type="button" role="switch" aria-checked={recordingActive}
            disabled={!fieldFresh || recordBusy || recordingState === "STOPPING"}
            onClick={() => void fieldCommand({ op: recordingActive ? "record_off" : "record_on" })}>
            {recordBusy ? "Updating…" : recordingActive ? "Record OFF · Stop" : "Record ON · Start"}
          </button>
          <span className="helper">Default OFF · manual MCAP recording</span>
          <span className="asset-path">{field?.recording.output}</span>
          <span className="helper">{field?.recording.error}</span>
        </div>
      </div>

      <div className="field-assets">
        <section className="field-card">
          <span className="eyebrow">VENUE CALIBRATION</span>
          <strong>{savedCalibration ? calibrationMatchesSide ? "SAVED" : "TABLE SIDE MISMATCH" : "NO CALIBRATION LOADED"}</strong>
          <span className="asset-path">{savedCalibration?.name ?? "Load a venue JSON, or explicitly recalibrate"}</span>
          <span className="helper">{savedCalibration ? `${savedCalibration.profile.toUpperCase()} · ${savedCalibration.table_side} · SHA ${savedCalibration.sha256.slice(0, 12)}` : ""}</span>
          <span className="helper">File: {field?.calibration_sync ?? "checking"} · live pose: {calibrationMatchesSide && baseFresh && snapshot.calibrationSuccess === true ? "VALID" : "unavailable / awaiting matching receipt"}</span>
          <div className="field-buttons">
            <button type="button" disabled={!assetEditable} onClick={() => calibrationInput.current?.click()}>Load JSON</button>
            <button type="button" disabled={!fieldFresh || fieldBusy || savedCalibration == undefined}
              onClick={() => void fieldCommand({ op: "export_calibration" })}>Export JSON</button>
            <button type="button" aria-expanded={recalibrating} onClick={() => { setRecalibrating((value) => !value); }}>Recalibrate</button>
          </div>
          <input ref={calibrationInput} type="file" accept=".json,application/json" hidden
            onChange={(event) => { void uploadAsset(event.target.files?.[0], "calibration"); event.target.value = ""; }} />
          <p className="helper">Saved files are reused after restart. Load while stopped. Recalibrate only when the venue, marker mounting or rigid-body setup changes.</p>
          {recalibrating && <div className="recalibration-actions">
            <p className="helper">OptiTrack mode · start the mocap stream · Stand · select the matching marker layout. Opening this section starts nothing.</p>
          <ActionButton
            label={`Cali 24 stickers · ${markerAsset}`}
            detail={calibrationDetail(snapshot, "STICKERS_V3")}
            disabled={
              kernelMode || !runnerUsable || !runnerStanding || busy.calibrationV3 === true
            }
            busy={busy.calibrationStickers === true}
            completed={
              snapshot.calibrationSuccess === true && snapshot.calibrationProfile === "STICKERS_V3"
            }
            next={false}
            onClick={() => void invokeConfirmed("calibrationStickers", "Replace the active venue calibration with a new 24-sticker capture? The previous approved JSON remains in the venue archive. Keep the robot in PD_STAND during capture.")}
          />
          <ActionButton
            label={`Cali V2 · ${markerAsset}`}
            detail={calibrationDetail(snapshot, "V2")}
            disabled={
              kernelMode || !runnerUsable || !runnerStanding || busy.calibrationV3 === true
            }
            busy={busy.calibrationV2 === true}
            completed={
              snapshot.calibrationSuccess === true && snapshot.calibrationProfile === "V2"
            }
            next={false}
            onClick={() => void invokeConfirmed("calibrationV2", "Replace the active venue calibration with a new V2 capture? The previous approved JSON remains in the venue archive. Keep the robot in PD_STAND during capture.")}
          />
          <ActionButton
            label={`Cali V3 · ${markerAsset}`}
            detail={calibrationDetail(snapshot, "V3")}
            disabled={
              kernelMode || !runnerUsable || !runnerStanding || busy.calibrationV2 === true
            }
            busy={busy.calibrationV3 === true}
            completed={
              snapshot.calibrationSuccess === true && snapshot.calibrationProfile === "V3"
            }
            next={false}
            onClick={() => void invokeConfirmed("calibrationV3", "Replace the active venue calibration with a new V3 capture? The previous approved JSON remains in the venue archive. Keep the robot in PD_STAND during capture.")}
          />
          </div>}
        </section>
        <section className="field-card">
          <span className="eyebrow">SERVE CSV</span>
          <strong>{field?.csv?.name ?? "serve025 · forwardhit_v4.csv (default)"}</strong>
          <span className="helper">{field?.csv ? `Validated · ${field.csv.frames} frames · ${field.csv.hz} Hz · SHA ${field.csv.sha256.slice(0, 12)}` : "468 frames · 100 Hz · SDK 31-joint format"}</span>
          <div className="field-buttons">
            <button type="button" disabled={!assetEditable} onClick={() => csvInput.current?.click()}>Upload CSV</button>
            <button type="button" disabled={!assetEditable || field?.csv == undefined}
              onClick={() => void fieldCommand({ op: "use_default_csv" })}>Use built-in CSV</button>
          </div>
          <input ref={csvInput} type="file" accept=".csv,text/csv" hidden
            onChange={(event) => { void uploadAsset(event.target.files?.[0], "csv"); event.target.value = ""; }} />
          <p className="helper">Select Kernel Mode, upload while stopped, then Run → SERVER → Stand → Start to Serve → load ball and clear hands → Serve → slowly lower arms → Stand → Start to Serve again.</p>
          <span className="helper">Uploads use the Runner CSV validator and never start motion. Normal play retains its built-in serve CSV.</span>
        </section>
      </div>
      {field?.error && <div className="operator-notice">{field.error}</div>}

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
                : "no active session"}{" "}
              · config revision {snapshot.lifecycleConfigRevision ?? 0}
            </div>
          </div>
          <div className="lifecycle-result" title={snapshot.lifecycleResult}>
            {snapshot.lifecycleResult ?? "waiting for supervisor"}
          </div>
        </div>
        <div className="config-grid">
          {NETWORK_CONFIG_FIELDS.map((name) => (
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
        <div className="table-side-selector">
          <div>
            <div className="eyebrow">OUR TABLE SIDE · LOCKED INTO NEXT START</div>
            <div className="topic-path">
              START SYSTEM does not contact Motive · when optional OptiTrack is running, P1
              uses Motive world directly and P2 rotates UCB_P2 poses 180° about the table centre
            </div>
          </div>
          {(["P1", "P2"] as const).map((side) => (
            <button
              key={side}
              className={configDraft.table_side === side ? "side-selected" : ""}
              type="button"
              disabled={!lifecycleStopped || lifecycleBusy}
              onClick={() => {
                setConfigTouched(true);
                setConfigDraft((current) => ({ ...current, table_side: side }));
              }}
            >
              {side === "P1" ? "P1 · DIRECT" : "P2 · ROTATE 180°"}
            </button>
          ))}
        </div>
        <div className="lifecycle-actions">
          <button
            className="config-confirm"
            type="button"
            disabled={
              !configComplete ||
              !lifecycleStopped ||
              lifecycleBusy ||
              timeCalibrationBlocksStart ||
              busy.applyLifecycleConfig === true
            }
            onClick={() => void confirmLifecycleConfig()}
          >
            {busy.applyLifecycleConfig === true ? "CONFIRMING…" : "CONFIRM CONFIG"}
          </button>
          <button
            className="system-start"
            type="button"
            disabled={
              !lifecycleStopped ||
              lifecycleBusy ||
              timeCalibrationBlocksStart ||
              (snapshot.lifecycleConfigRevision ?? 0) < 1 ||
              configTouched ||
              fieldBusy || field?.busy === true || !fieldFresh ||
              busy.startLifecycle === true
            }
            onClick={() => void startSystem()}
          >
            {busy.startLifecycle === true || (lifecycleBusy && lifecycleState === "STARTING")
              ? "STARTING…"
              : "START SYSTEM"}
          </button>
          <button
            className="system-stop"
            type="button"
            disabled={!lifecycleRunning || lifecycleBusy || busy.killAllAndCollect === true}
            onClick={() => void killAllAndCollect()}
          >
            {busy.killAllAndCollect === true || (lifecycleBusy && lifecycleState === "KILLING")
              ? "KILLING…"
              : "KILL ALL & COLLECT"}
          </button>
          <span className="config-helper">
            Inputs stay editable. Confirm is accepted only while stopped; changing the HDU IP also
            requires reconnecting Foxglove to the new :8766 address.
          </span>
        </div>
        <div
          className={`time-calibration-row time-calibration-${timeCalibrationState.toLowerCase().replace(/[_ ]/g, "-")}`}
        >
          <div className="time-calibration-copy">
            <div className="eyebrow">TIME CALIBRATION · RUNBOOK 10.4 · RECEIPT-SAFE</div>
            <div className="time-calibration-state">
              {timeCalibrationState} · {snapshot.timeCalibrationStep ?? "IDLE"}
            </div>
            <div className="topic-path" title={snapshot.timeCalibrationResult}>
              {snapshot.timeCalibrationOperation != undefined &&
              snapshot.timeCalibrationOperation.length > 0
                ? `${snapshot.timeCalibrationOperation} · `
                : ""}
              {snapshot.timeCalibrationResult ?? "waiting for calibration coordinator"}
            </div>
          </div>
          <button
            className="time-calibration-button"
            type="button"
            disabled={!timeCalibrationReady || busy.timeCalibration === true}
            title={timeCalibrationTitle}
            onClick={() => void runTimeCalibration()}
          >
            {busy.timeCalibration === true || timeCalibrationBusy
              ? "CALIBRATING…"
              : "TIME CALIBRATION"}
          </button>
        </div>
      </div>

      <section className="diagnostics-section" aria-label="Diagnostics">
        <div className="diagnostics-content">
      <div className="metric-grid">
        <div className="metric-card">
          <div className="eyebrow">NTP WORLD-CLOCK OFFSET</div>
          <div className={`metric-value ${ntpFresh ? "" : "stale"}`}>
            <span>
              {ntpFresh && snapshot.ntpOffsetMs != undefined
                ? snapshot.ntpOffsetMs.toFixed(2)
                : "—"}
            </span>
            <small>ms</small>
          </div>
          <div className="topic-path">
            {TOPICS.ntpOffset}
            {snapshot.ntpDispersionMs != undefined
              ? ` · disp ${snapshot.ntpDispersionMs.toFixed(2)}`
              : ""}
            {snapshot.ntpSkewPpm != undefined
              ? ` · skew ${snapshot.ntpSkewPpm.toFixed(2)} ppm audit-only`
              : ""}
            {ntpFresh ? "" : " · stale"}
          </div>
        </div>
        <div className="metric-card">
          <div className="eyebrow">ROS 2 MESSAGE LATENCY</div>
          <div className={`metric-value ${latencyFresh ? "" : "stale"}`}>
            <span>
              {latencyFresh && snapshot.latencyMs != undefined
                ? snapshot.latencyMs.toFixed(1)
                : "—"}
            </span>
            <small>ms</small>
          </div>
          <div className="topic-path">
            {TOPICS.latency}
            {latencyFresh ? "" : " · stale"}
          </div>
        </div>
      </div>

      <div className="process-grid">
        <ProcessTile
          name="agibot_pm"
          topic={TOPICS.agibotPm}
          value={pmFresh ? snapshot.agibotPm : undefined}
        />
        <ProcessTile
          name="HDU"
          topic={TOPICS.hduActive}
          value={hduFresh ? snapshot.hduActive : undefined}
        />
        <ProcessTile
          name="MDU Runner"
          topic={TOPICS.mduActive}
          value={mduFresh ? snapshot.mduActive : undefined}
        />
      </div>

      <div className="cpu-card">
        <div className="cpu-header">
          <span className="eyebrow">A3 CPU LOAD</span>
          <span className={`cpu-value ${cpuFresh ? "" : "stale"}`}>
            {cpuFresh && snapshot.cpuPercent != undefined ? snapshot.cpuPercent.toFixed(0) : "—"}
            <small>%</small>
          </span>
          <div className="cpu-diagnostics">
            <span className="topic-path">{TOPICS.cpu} · 0–100 %, 120 s</span>
            <span className="topic-path" title={snapshot.cpuTopProcess}>
              {snapshot.cpuTopProcess ?? "top process warming up"}
            </span>
          </div>
        </div>
        <svg
          className="cpu-plot"
          viewBox="0 0 600 160"
          preserveAspectRatio="none"
          aria-label="CPU load over 120 seconds"
        >
          <line x1="0" y1="40" x2="600" y2="40" />
          <line x1="0" y1="80" x2="600" y2="80" />
          <line x1="0" y1="120" x2="600" y2="120" />
          {cpuPoints.length > 0 && <polyline className="cpu-line" points={cpuPoints} />}
        </svg>
      </div>

        </div>
      </section>

      <div className="sequence-card teleop-card">
        <span className="eyebrow">XBOX · BODY-FRAME LOCOMOTION</span>
        <div className="teleop-details">
          <span>Controller</span><span title={snapshot.xboxPreview?.device}>{snapshot.xboxPreview?.device === "" ? "No controller detected" : (snapshot.xboxPreview?.device ?? "No controller detected")}</span>
          <span>Input</span><span>{!xboxFresh ? "NO FRESH XBOX INPUT" : snapshot.xboxPreview?.connected !== true
            ? "DISCONNECTED" : `LIVE · LT ${snapshot.xboxPreview.lt ? "HELD" : "RELEASED"} · ${snapshot.xboxPreview.state}`}</span>
          <span>Sticks</span><span>{velocityText(xboxVelocity)}</span>
          <span>Runner</span><span>{!runnerFresh ? "NO FRESH RUNNER STATUS" : teleop == undefined
            ? "WAITING FOR TELEOP TELEMETRY" : teleop[2] !== 1 ? "TELEOP POLICY NOT LOADED"
              : `${snapshot.runnerMode ?? "UNKNOWN"} · ${["OFF", "ENTERING", "ACTIVE", "STOPPING"][teleop[3] ?? 0] ?? "UNKNOWN"} · enable ${teleop[9] === 1 ? "ON" : "OFF"} · transition ${((teleop[5] ?? 0) * 100).toFixed(0)}%`}</span>
          <span>Command</span><span>{velocityText(teleop?.slice(6, 9))}</span>
          <span>Mode keys</span><span title={snapshot.xboxPreview?.action_status}>{xboxFresh ? (snapshot.xboxPreview?.action_status ?? "A prepare · B serve · X receive · Y Teleop") : "Waiting for Xbox input"}</span>
          <span>Input age</span><span>{teleop == undefined || (teleop[4] ?? 1e9) >= 1e9
            ? "No input received by Runner" : `${Math.min(9999, (teleop[4] ?? 9999) * 1000).toFixed(0)} ms`}</span>
        </div>
        <div className="role-controls">
          <button type="button" disabled={teleopBlocked != undefined} title={teleopBlocked}
            onClick={() => void invoke("teleop")}>Enter Teleop</button>
          <button type="button" disabled={!runnerFresh || snapshot.runnerMode !== "TELEOP" || busy.stand === true}
            onClick={() => void invoke("stand")}>Stop and Return to Stand</button>
        </div>
        <div className="teleop-entry-status" title={teleopBlocked ?? "Ready to request Teleop; Runner validates current input."}>
          {teleopBlocked ?? "Ready to request Teleop; Runner validates current input."}
        </div>
        <div className="sequence-status">Wait for ACTIVE, center the sticks, then release and hold LT again. Use the left stick to move forward, backward, or sideways, and the right stick to turn. Releasing LT or losing input slows the robot to a stop. Returning to Stand waits for the feet to settle. A: SERVER + Start to Serve. B: Serve, then automatic Stand in Kernel Mode or Ready in normal play. X: RECEIVER + Ready. Y: Teleop from Stand only; presses in other modes are ignored. Press face buttons once, without LT. LB + RB together: E-STOP.</div>
      </div>

      <div className="sequence-card">
        <div className="sequence-header">
          <div className="sequence-summary">
            <span className="eyebrow">{kernelMode ? "KERNEL MODE · TWO-STEP SERVE" : "OPTITRACK · NORMAL RUNNER SEQUENCE"}</span>
            <span className="sequence-status">
              {runnerFresh
                ? `${snapshot.runnerMode ?? "UNKNOWN"} · ${serveState} · gripper ${snapshot.gripperState ?? "UNAVAILABLE"}${snapshot.serveCleanupRequired === true ? " · CLEANUP REQUIRED" : ""}`
                : "NO FRESH RUNNER STATE"}
            </span>
          </div>
          <div className="role-controls">
            <span className="role-label">OUR ROLE: {snapshot.localRole ?? "UNASSIGNED"}</span>
            <button
              type="button"
              disabled={snapshot.roleChangeAllowed !== true || busy.setServer === true}
              onClick={() => void invoke("setServer")}
            >
              SERVER
            </button>
            <button
              type="button"
              disabled={snapshot.roleChangeAllowed !== true || busy.setReceiver === true}
              onClick={() => void invoke("setReceiver")}
            >
              RECEIVER
            </button>
          </div>
        </div>
        <div className="action-grid">
          <ActionButton
            label="Stand"
            detail={
              snapshot.runnerMode === "TELEOP"
                ? "STOP AND SETTLE · waits for the feet, then enters Stand"
                : serveWorkflowActive
                ? "PHASE-AWARE ABORT · recovery stays in this Runner"
                : runnerStanding
                  ? "DONE · PD_STAND"
                  : "same as keyboard s"
            }
            disabled={!runnerUsable}
            busy={busy.stand === true}
            completed={runnerStanding}
            next={nextStep === "stand"}
            onClick={() => void invoke("stand")}
          />
          {!kernelMode && <ActionButton
            label="Refresh x_hit"
            detail={
              snapshot.xHitSuccess === true
                ? (snapshot.xHitStatus ?? "DONE · Planner acknowledged")
                : runnerStanding
                  ? (snapshot.xHitStatus ?? "refresh Planner x_hit only")
                  : "LOCKED · stand first"
            }
            disabled={!runnerUsable || !runnerStanding}
            busy={busy.refreshXHit === true}
            completed={snapshot.xHitSuccess === true}
            next={false}
            onClick={() => void invoke("refreshXHit")}
          />}
          <ActionButton
            label="Ready"
            detail={
              runnerReady
                ? kernelMode ? "READY · receive policy test · Start to Serve begins the next cycle" : "READY · receiving · Start to Serve begins the next cycle"
                : snapshot.localRole !== "RECEIVER"
                ? kernelMode ? "SERVER lowers arms and returns to Stand after Serve" : "SERVER enters Ready automatically after Serve completes"
                : !runnerStanding
                  ? "LOCKED · stand first"
                  : kernelMode
                    ? "ENTER RECEIVE POLICY · no OptiTrack required · IMU-local test"
                    : baseFresh
                    ? "ENTER MOTION · fresh OptiTrack base is available"
                    : "Normal Play uses OptiTrack; Kernel Mode permits an IMU-local policy test without mocap"
            }
            disabled={
              !runnerUsable ||
              snapshot.localRole !== "RECEIVER" ||
              !runnerStanding
            }
            busy={busy.ready === true}
            completed={runnerReady}
            next={nextStep === "ready"}
            onClick={() => void invoke("ready")}
          />
          <ActionButton
            label="Start to Serve · Raise hand"
            detail={
              !serveAvailable
                ? "UNAVAILABLE · launch Runner with --serve"
                : snapshot.localRole !== "SERVER"
                  ? "SERVER ONLY"
                  : serveWorkflowActive
                    ? `${serveState} · same Runner owns q_des`
                    : "smooth Stand / Ready → loading pose; gripper closes automatically"
            }
            disabled={
              !runnerUsable ||
              !serveAvailable ||
              snapshot.localRole !== "SERVER" ||
              !["PD_STAND", "MOTION"].includes(snapshot.runnerMode ?? "") ||
              !["IDLE", "COMPLETE", "ABORTED"].includes(serveState)
            }
            busy={busy.prepareServe === true}
            completed={serveWorkflowActive}
            next={nextStep === "prepareServe"}
            onClick={() =>
              void invokeConfirmed(
                "prepareServe",
                "Prepare serve025 now? Confirm our role is SERVER, the robot is supported and clear, and the physical E-stop is reachable. The same Runner transitions all 31 joints from its last delivered Stand / Ready command to CSV frame 0 and immediately commands GRAB. Place the ball only after the transition has stopped.",
              )
            }
          />
          <ActionButton
            label="Serve · Play"
            detail={
              serveState === "WAIT_READY_TO_SERVE"
                ? `LOAD BALL, CLEAR HANDS · starts selected full31 timeline at 100 Hz · gripper ${snapshot.gripperState ?? "UNAVAILABLE"} is non-gating`
                : snapshot.serving === true
                  ? `PLAYING · ${serveState}`
                  : kernelMode && serveState === "COMPLETE" && snapshot.runnerMode === "PD_STAND"
                    ? "STAND · arms lowered · Start to Serve begins the next cycle"
                  : serveState === "COMPLETE" && snapshot.runnerMode === "MOTION"
                    ? "READY · receiving · Start to Serve begins the next cycle"
                  : "LOCKED UNTIL WAIT_READY_TO_SERVE"
            }
            disabled={
              !runnerUsable ||
              snapshot.localRole !== "SERVER" ||
              serveState !== "WAIT_READY_TO_SERVE"
            }
            busy={busy.readyToServe === true}
            completed={snapshot.serving === true || serveState === "COMPLETE"}
            next={nextStep === "readyToServe"}
            onClick={() =>
              void invokeConfirmed(
                "readyToServe",
                kernelMode ? "Test the selected CSV now? Confirm the ball is loaded, hands are clear and the physical E-stop is reachable. After the stroke, Runner slowly lowers the arms over 2.5 seconds and returns to Stand." : "Start the serve motion now? Confirm all people are outside the loading and swing zones, the opponent is ready, and the physical E-stop is reachable. Gripper state is not checked; this action starts frame 0 immediately. A return ball can engage the receive policy after the post-contact handoff finishes and Runner reports MOTION.",
              )
            }
          />
          <ActionButton
            label="Open gripper"
            detail={
              serveState === "CLEANUP_OPENING"
                ? "REQUESTED · OPEN publish in progress"
                : snapshot.serveCleanupRequired === true
                  ? "RETAINED-BALL TELEMETRY · does not gate any action"
                  : `MANUAL COMMAND · gripper ${snapshot.gripperState ?? "UNAVAILABLE"}`
            }
            disabled={
              !runnerUsable ||
              snapshot.localRole !== "SERVER" ||
              snapshot.runnerMode !== "PD_STAND"
            }
            busy={busy.openGripper === true}
            completed={snapshot.gripperState === "OPEN"}
            next={false}
            onClick={() =>
              void invokeConfirmed(
                "openGripper",
                "Open the gripper for cleanup? Catch or remove the retained ball first: it may drop. Confirm the robot is in PD_STAND, the area is attended, and hands are clear of the claw.",
              )
            }
          />
        </div>
        <div className="sequence-footer">
          <span title={snapshot.xHitStatus}>{lastAction}</span>
          <ActionButton
            label="Runner Passive"
            detail="ZERO GAINS · robot loses support"
            disabled={!runnerFresh}
            busy={busy.passive === true}
            danger
            onClick={() => void invoke("passive")}
          />
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
