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

import { BallYFilterControls } from "./BallYFilterControls";
import { HitSpeedScaleControls } from "./HitSpeedScaleControls";
import { LanguageContext, useI18n } from "./LanguageContext";
import { PlaneModeControls } from "./PlaneModeControls";
import { BallModelStatus, ballModelDisplay } from "./ballModelStatus";
import { BALL_Y_FILTER_SERVICE, BallYFilterState, ballYFilterRequest, decodeBallYFilter } from "./ballYFilter";
import { BallYFilterDraft, withBallYPreview } from "./ballYFilterPreview";
import { HIT_SPEED_SCALE_SERVICE, HitSpeedScaleState, decodeHitSpeedScale, hitSpeedScaleRequest } from "./hitSpeedScale";
import { message as msg, LocalizedError, errorText, LocalText, Translator, Locale, readLocale, createTranslator } from "./i18n";
import { mergeConfirmedConfig } from "./lifecycleConfig";
import { MocapSafety, decodeMocapSafety, mocapSafetyText } from "./mocapSafety";
import { MOCAP_LABELS, mocapSetupCommand } from "./mocapSetup";
import { OPERATOR_CHALLENGE, OperatorHeartbeat } from "./operatorHeartbeat";
import { PLANE_MODE_SERVICE, PlaneMode, PlaneModeState, decodePlaneMode, planeModeRequest } from "./planeMode";
import { PlayerPosition, decodePlayerPosition, playerSideAvailable } from "./playerPositions";
import { reconcileRobotTarget, robotConnectionUrl, switchRobot } from "./robotConnection";
import { decodeTeleopState, teleopEntryReason, xboxInputFresh } from "./teleopStatus";
import { timeCalibrationControls } from "./timeCalibration";

import "@fontsource/ibm-plex-mono/400.css";
import "@fontsource/ibm-plex-mono/500.css";
import "@fontsource/ibm-plex-mono/600.css";
import "@fontsource/ibm-plex-sans/400.css";
import "@fontsource/ibm-plex-sans/500.css";
import "@fontsource/space-grotesk/600.css";
import "@fontsource/space-grotesk/700.css";
import "./styles.css";

const TOPICS = {
  plannerDiagnostics: "/planner/diagnostics",
  fieldStatus: "/hope/field/status",
  playerP1: "/hope/mocap/players/UCB_P1",
  playerP2: "/hope/mocap/players/UCB_P2",
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
  markers: "/hope/mocap/marker_count",
  markerExpectedCount: "/hope/mocap/marker_expected_count",
  markersFresh: "/hope/mocap/marker_fresh",
  assetMatched: "/hope/mocap/asset_match",
  assetText: "/hope/mocap/asset_text",
  baseFresh: "/hope/base/fresh",
  commandValid: "/hope/command/valid",
  commandCountdown: "/hope/command/hdu_wall_countdown_s",
  commandFlight: "/hope/command/flight_id",
  commandSummary: "/hope/command/summary",
  runnerAlive: "/hope/runner/alive",
  runnerBoot: "/hope/runner/boot_id",
  mocapSafety: "/hope/runner/mocap_safety_hdu_flat",
  operatorChallenge: OPERATOR_CHALLENGE,
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
  gripperProfile: "/hope/lifecycle/config/gripper_profile",
  pantsProfile: "/hope/lifecycle/config/pants_profile",
  mocapSource: "/hope/lifecycle/config/mocap_source",
  vrpnIp: "/hope/lifecycle/config/vrpn_ip",
  vrpnPort: "/hope/lifecycle/config/vrpn_port",
  vrpnUnits: "/hope/lifecycle/config/vrpn_units",
  operatorProfile: "/hope/lifecycle/config/operator_profile",
} as const;

const SERVICES = {
  estop: "/hope/safety/trigger_estop",
  resetSoftwareEstop: "/hope/safety/reset_software_estop",
  setServer: "/hope/runner/set_server",
  setReceiver: "/hope/runner/set_receiver",
  stand: "/hope/runner/enter_pd_stand",
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

type PolicyModel = "model14600" | "model11999" | "model12600" | "model17400" | "model12000";
type FieldStatus = {
  player_positions?: { P1: boolean; P2: boolean };
  ball_model_status?: BallModelStatus;
  policy_model?: PolicyModel | null;
  policy_models?: Partial<Record<PolicyModel, { available: boolean; reason: string; receive_only?: boolean }>>;
  policy_models_age_s?: number | null;
  policy_models_config_revision?: number | null;
  ball_model?: "berkeley" | "ppq002" | "ppq004" | null;
  ball_model_verification?: { model: string; planner: string; packetizer: string };
  mode: "OPTITRACK" | "KERNEL";
  calibration?: { name: string; sha256: string; profile: string; table_side: string } | null;
  calibration_sync: string;
  csv?: { name: string; sha256: string; frames: number; hz: number;
    segments?: { version: number; branch_frame: number; policy_ready_s: number; policy_blend_s: number;
      policy_sha256: string; clips: Record<string, { frames: number; duration_s: number }> } } | null;
  recording: { state: string; output: string; error: string };
  busy?: boolean;
  error?: string;
};

type CpuSample = { at: number; value: number };

const CONFIG_FIELDS = {
  laptop_wifi_ip: { label: msg("hopeA3Console.laptopIp"), topic: TOPICS.laptopWifiIp },
  hdu_wifi_ip: { label: msg("hopeA3Console.hduIp"), topic: TOPICS.hduWifiIp },
  mdu_internal_ip: { label: msg("hopeA3Console.mduInternal"), topic: TOPICS.mduInternalIp },
  motive_ip: { label: msg("hopeA3Console.motiveServerIpNatnet"), topic: TOPICS.motiveIp },
  vrpn_ip: { label: msg("hopeA3Console.chingmuServerIpVrpn"), topic: TOPICS.vrpnIp },
  mocap_source: { label: msg("hopeA3Console.mocapSystem"), topic: TOPICS.mocapSource },
  vrpn_port: { label: msg("hopeA3Console.vrpnPort"), topic: TOPICS.vrpnPort },
  vrpn_units: { label: msg("hopeA3Console.vrpnUnits"), topic: TOPICS.vrpnUnits },
  table_side: { label: msg("hopeA3Console.ourTableSide"), topic: TOPICS.tableSide },
  gripper_profile: { label: msg("hopeA3Console.gripper"), topic: TOPICS.gripperProfile },
  pants_profile: { label: msg("hopeA3Console.markerPants"), topic: TOPICS.pantsProfile },
  operator_profile: { label: msg("hopeA3Console.operatorComputer"), topic: TOPICS.operatorProfile },
} as const;

type ConfigField = keyof typeof CONFIG_FIELDS;
const NETWORK_CONFIG_FIELDS: ConfigField[] = [
  "laptop_wifi_ip",
  "hdu_wifi_ip",
  "mdu_internal_ip",
  "motive_ip",
  "vrpn_ip",
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
  gripper_profile: "",
  pants_profile: "",
  mocap_source: "",
  vrpn_ip: "",
  vrpn_port: "",
  vrpn_units: "",
  operator_profile: "",
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
  mocapSafety?: MocapSafety;
  runnerBoot?: number;
  playerP1?: PlayerPosition;
  playerP2?: PlayerPosition;
  hitSpeedScale?: HitSpeedScaleState;
  ballYFilter?: BallYFilterState;
  planeMode?: PlaneModeState;
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
  markerExpectedCount?: number;
  assetMatched?: boolean;
  assetText?: string;
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
  field?: FieldStatus;
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
  if (typeof value === "bigint") {
    // ROS 2 Int64/UInt64 may decode as bigint. Never round an identity to Number.
    const decoded = Number(value);
    return Number.isSafeInteger(decoded) ? decoded : undefined;
  }
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
    case TOPICS.mocapSafety:
      next.mocapSafety = decodeMocapSafety(scalar(event.message));
      break;
    case TOPICS.runnerBoot: {
      const value = numberValue(event.message);
      next.runnerBoot = value != undefined && Number.isSafeInteger(value) &&
        value > 0 && value <= 2**52 ? value : undefined;
      break;
    }
    case TOPICS.playerP1:
      next.playerP1 = decodePlayerPosition(event.message, receivedAt, next.playerP1);
      break;
    case TOPICS.playerP2:
      next.playerP2 = decodePlayerPosition(event.message, receivedAt, next.playerP2);
      break;
    case TOPICS.plannerDiagnostics:
      next.ballYFilter = decodeBallYFilter(event.message);
      next.planeMode = decodePlaneMode(event.message);
      next.hitSpeedScale = decodeHitSpeedScale(event.message);
      break;
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
    case TOPICS.markers:
      next.markerCount = numberValue(event.message);
      break;
    case TOPICS.markerExpectedCount:
      next.markerExpectedCount = numberValue(event.message);
      break;
    case TOPICS.markersFresh:
      next.markersFresh = boolValue(event.message);
      break;
    case TOPICS.assetMatched:
      next.assetMatched = boolValue(event.message);
      break;
    case TOPICS.assetText:
      next.assetText = stringValue(event.message);
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
    case TOPICS.mocapSource:
      next.lifecycleConfig.mocap_source = stringValue(event.message);
      break;
    case TOPICS.vrpnIp:
      next.lifecycleConfig.vrpn_ip = stringValue(event.message);
      break;
    case TOPICS.vrpnPort:
      next.lifecycleConfig.vrpn_port = stringValue(event.message);
      break;
    case TOPICS.vrpnUnits:
      next.lifecycleConfig.vrpn_units = stringValue(event.message);
      break;
    case TOPICS.pantsProfile:
      next.lifecycleConfig.pants_profile = stringValue(event.message);
      break;
    case TOPICS.gripperProfile:
      next.lifecycleConfig.gripper_profile = stringValue(event.message);
      break;
    case TOPICS.tableSide:
      next.lifecycleConfig.table_side = stringValue(event.message);
      break;
    case TOPICS.operatorProfile:
      next.lifecycleConfig.operator_profile = stringValue(event.message);
      break;
  }
}

function triggerResponse(value: unknown): { success: boolean; message: LocalText } {
  if (typeof value !== "object" || value == undefined) {
    return { success: false, message: msg("hopeA3Console.serviceReturnedNoStructuredResponse") };
  }
  const response = value as TriggerResponse;
  return {
    success: response.success === true,
    message:
      typeof response.message === "string" ? response.message : msg("hopeA3Console.serviceReturnedNoMessage"),
  };
}

async function timeoutAfter(milliseconds: number): Promise<never> {
  return await new Promise((_, reject) => {
    window.setTimeout(() => {
      reject(new LocalizedError(msg("hopeA3Console.serviceTimeoutAfterS", {seconds: milliseconds / 1000})));
    }, milliseconds);
  });
}

function isFresh(snapshot: Snapshot, topic: TopicName, now: number, maxAgeMs: number): boolean {
  const received = snapshot.lastReceived[topic];
  return received != undefined && now - received <= maxAgeMs;
}

type Confirmation = {
  key: ServiceKey;
  prompt: LocalText;
  host?: string;
  session?: string;
  boot?: number;
};

function confirmationBlocked(pending: Confirmation, current: Snapshot, now: number): LocalText | undefined {
  if (pending.host !== current.lifecycleConfig.hdu_wifi_ip ||
      pending.session !== current.lifecycleSession || pending.boot !== current.runnerBoot) {
    return msg("hopeA3Console.confirmationIdentityChanged");
  }
  const key = pending.key;
  if (key !== "prepareServe" && key !== "readyToServe" && key !== "openGripper") { return; }
  const required = [TOPICS.runnerAlive, TOPICS.runnerBoot, TOPICS.runnerFault,
    TOPICS.runnerMode, TOPICS.localRole, TOPICS.serveCapability, TOPICS.serveState];
  if (!current.lifecycleSession || current.runnerBoot == undefined || current.runnerAlive !== true ||
      required.some(topic => !isFresh(current, topic, now, 1_000))) {
    return msg("hopeA3Console.confirmationStateUnavailable");
  }
  if (current.runnerFault !== false || current.localRole !== "SERVER" ||
      (key === "openGripper" ? current.runnerMode !== "PD_STAND" :
        current.serveCapability !== "AVAILABLE" ||
        (key === "prepareServe"
          ? !["PD_STAND", "MOTION"].includes(current.runnerMode ?? "") ||
            !["IDLE", "COMPLETE", "ABORTED"].includes(current.serveState ?? "")
          : current.serveState !== "WAIT_READY_TO_SERVE"))) {
    return msg("hopeA3Console.confirmationStateChanged");
  }
  if (key === "openGripper") { return; }
  const safety = current.mocapSafety;
  if (!isFresh(current, TOPICS.mocapSafety, now, 1_000) || safety?.boot !== current.runnerBoot) {
    return msg("hopeA3Console.confirmationStateUnavailable");
  }
  if (safety.operatorRequired !== false && (safety.uiFresh !== true || safety.computerFresh !== true)) {
    return msg("hopeA3Console.confirmationHeartbeatUnavailable");
  }
  return undefined;
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
  const t = useI18n();
  const state = value == undefined ? "unknown" : value ? "ok" : "attention";
  return (
    <div className={`gate gate-${state}`} title={detail}>
      <span className="gate-dot" />
      {label}
      {value === false ? t("hopeA3Console.check") : ""}
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
  const t = useI18n();
  const state = value == undefined ? "unknown" : value ? "running" : "stopped";
  return (
    <div className={`process-tile process-${state}`}>
      <div className="process-state">
        <span className="process-dot" />
        {state === "unknown" ? t("hopeA3Console.noData") : state.toUpperCase()}
      </div>
      <div className="process-name">{name}</div>
      <div className="topic-path">{topic}</div>
    </div>
  );
}

function selectedMarkerAsset(snapshot: Snapshot, t: Translator): string {
  const configuredSide = snapshot.lifecycleConfig.table_side?.trim();
  if (configuredSide === "P1" || configuredSide === "P2") {
    return `UCB_${configuredSide}`;
  }
  const reportedAsset = snapshot.markerAsset?.trim();
  return reportedAsset === "UCB_P1" || reportedAsset === "UCB_P2"
    ? reportedAsset
    : t("hopeA3Console.sideUnset");
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
  const t = useI18n();
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
      <span>{props.busy ? t("hopeA3Console.working") : props.label}</span>
      <small>{props.detail}</small>
    </button>
  );
}

function HopeA3Console({ context }: { context: PanelExtensionContext }): ReactElement {
  const [locale, setLocale] = useState<Locale>(readLocale(context.initialState));
  const t = useMemo(() => createTranslator(locale), [locale]);
  const latest = useRef<Snapshot>(INITIAL_SNAPSHOT);
  const [snapshot, setSnapshot] = useState<Snapshot>(INITIAL_SNAPSHOT);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [, refreshClock] = useState(0);
  // Samples can arrive between timer ticks. Compare them with the time of THIS
  // render, otherwise a new packet appears up to 250 ms in the future.
  const now = Date.now();
  const [busy, setBusy] = useState<Partial<Record<ServiceKey, boolean>>>({});
  const [notice, setNotice] = useState<LocalText>(msg("hopeA3Console.waitingForAuthoritativeRunnerState"));
  const [confirmation, setConfirmation] = useState<Confirmation | undefined>();
  const confirmationRef = useRef<Confirmation>();
  const confirmationCancel = useRef<HTMLButtonElement>(null);
  useEffect(() => {
    if (confirmation != undefined) { confirmationCancel.current?.focus(); }
  }, [confirmation]);
  const [configDraft, setConfigDraft] = useState<LifecycleConfigDraft>(EMPTY_CONFIG);
  const [configTouched, setConfigTouched] = useState(false);
  const savedPanel = useRef((context.initialState ?? {}) as Record<string, unknown>);
  const [robotTarget, setRobotTarget] = useState(typeof savedPanel.current.robotTarget === "string" ? savedPanel.current.robotTarget : "");
  const [targetEdited, setTargetEdited] = useState(false);
  // A layout can outlive its connection. Never restore an in-flight switch.
  const [pendingTarget, setPendingTarget] = useState<string>("");
  const configHost = snapshot.lifecycleConfig.hdu_wifi_ip;
  const configHostFresh = isFresh(snapshot, TOPICS.hduWifiIp, now, 1_500);
  const previousHost = useRef<string>();
  useEffect(() => {
    if (!configHostFresh || !configHost) { return; }
    if (previousHost.current && previousHost.current !== configHost) {
      setConfigTouched(false);
      setConfigDraft({ ...EMPTY_CONFIG, ...snapshot.lifecycleConfig });
    }
    previousHost.current = configHost;
    const resolved = reconcileRobotTarget(robotTarget, targetEdited, pendingTarget, configHost);
    setRobotTarget(resolved.target);
    setPendingTarget(resolved.pending);
    setTargetEdited(resolved.edited);
    if (!resolved.pending && savedPanel.current.robotTarget !== configHost) {
      savedPanel.current = { ...savedPanel.current, robotTarget: configHost };
      context.saveState(savedPanel.current);
    }
  }, [configHost, configHostFresh, pendingTarget, robotTarget, targetEdited, snapshot.lifecycleConfig, context]);
  const connectRobot = () => {
    try {
      switchRobot(robotTarget.trim(), (ip) => {
        setPendingTarget(ip);
        setNotice(msg("hopeA3Console.opening8766WaitingForTheNewRobotControls", {ip}));
      }, (url) => { window.open(url, "_blank", "noopener,noreferrer"); });
    } catch (error) { setNotice(errorText(error)); }
  };
  let validRobotTarget = false;
  try { robotConnectionUrl(robotTarget.trim()); validRobotTarget = true; } catch { /* incomplete input */ }

  const [fieldBusy, setFieldBusy] = useState(false);
  const [recordBusy, setRecordBusy] = useState(false);
  const csvInput = useRef<HTMLInputElement>(null);
  const previewBallYFilter = useCallback((draft: BallYFilterDraft, options: { kernelMode: boolean; edited: boolean }) => {
    context.layout.addPanel({ position: "sibling", type: "3D", updateIfExists: true,
      getState: (existing) => withBallYPreview(existing, draft, options, t) });
    savedPanel.current = { ...savedPanel.current,
      ballYFilterDraft: draft, ballYFilterDraftEdited: options.edited,
      ballYFilterRobot: latest.current.lifecycleConfig.hdu_wifi_ip };
    context.saveState(savedPanel.current);
  }, [context, t]);

  useLayoutEffect(() => {
    const heartbeat = new OperatorHeartbeat(context);
    context.onRender = (renderState, done) => {
      const receivedAt = Date.now();
      const hostMessage = renderState.currentFrame?.find((event) => event.topic === TOPICS.hduWifiIp);
      const incomingHost = hostMessage ? stringValue(hostMessage.message) : undefined;
      if (renderState.didSeek || (incomingHost !== undefined && latest.current.lifecycleConfig.hdu_wifi_ip !== undefined &&
          incomingHost !== latest.current.lifecycleConfig.hdu_wifi_ip)) {
        confirmationRef.current = undefined;
        setConfirmation(undefined);
        latest.current = INITIAL_SNAPSHOT;
        setConfigTouched(false);
        setConfigDraft(EMPTY_CONFIG);
      }
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
      for (const event of renderState.currentFrame ?? []) {
        if (event.topic === OPERATOR_CHALLENGE && !renderState.didSeek) {
          heartbeat.echo(scalar(event.message), next.runnerBoot, next.lifecycleSession);
        }
      }
      latest.current = next;
      setSnapshot(next);
      setRenderDone(() => done);
    };
    context.watch("didSeek");
    context.watch("topics");
    context.watch("currentFrame");
    context.subscribe(Object.values(TOPICS).map((topic) => ({ topic })));
    return () => {
      heartbeat.close();
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
    setConfigDraft((current) => mergeConfirmedConfig(current, snapshot.lifecycleConfig, configTouched));
  }, [configTouched, snapshot.lifecycleConfig]);

  const invoke = useCallback(
    async (key: ServiceKey) => {
      if (context.callService == undefined || busy[key] === true) {
        setNotice(msg("hopeA3Console.currentDataSourceDoesNotExposeServiceCalls"));
        return;
      }
      setBusy((current) => ({ ...current, [key]: true }));
      setNotice(msg("hopeA3Console.requested", {service: SERVICES[key]}));
      try {
        const timeoutMs =
          key === "estop"
            ? 5_000
            : key === "timeCalibration"
              ? 10_000
              : 3_000;
        const raw = await Promise.race([
          context.callService(SERVICES[key], {}),
          timeoutAfter(timeoutMs),
        ]);
        const response = triggerResponse(raw);
        setNotice(msg("hopeA3Console.requestResult", {status: response.success ? "ACCEPTED" : "REJECTED", detail: response.message}));
      } catch (error) {
        setNotice(msg("hopeA3Console.failed", {error: errorText(error)}));
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
      if (context.callService == undefined) {throw new LocalizedError(msg("hopeA3Console.connectToTheControlBridge8766"));}
      const raw = (await context.callService("/hope/field/command", {
        parameters: [{ name: "request", value: {
          type: 4, string_value: JSON.stringify(payload), bool_value: false,
          integer_value: 0, double_value: 0, byte_array_value: [], bool_array_value: [],
          integer_array_value: [], double_array_value: [], string_array_value: [],
        } }],
      })) as SetParametersResponse;
      const result = raw.results?.[0];
      if (result?.successful !== true) {throw new LocalizedError(typeof result?.reason === "string" ? result.reason : msg("hopeA3Console.invalidResponse"));}
      setNotice(msg("hopeA3Console.acceptedFollowTheStatusShownBelow", {operation: String(payload.op)}));
    } catch (error) {
      setNotice(msg("hopeA3Console.rejected", {error: errorText(error)}));
    } finally {
      setPending(false);
    }
  }, [context]);

  const applyHitSpeedScale = useCallback(async (scale: number) => {
    if (context.callService == undefined) { throw new LocalizedError(msg("hopeA3Console.connectToControlPort8766")); }
    const requestedAt = Date.now();
    const raw = await Promise.race([
      context.callService(HIT_SPEED_SCALE_SERVICE, hitSpeedScaleRequest(scale)), timeoutAfter(3_000),
    ]) as { result?: { successful?: boolean; reason?: string } };
    if (raw.result?.successful !== true) { throw new LocalizedError(raw.result?.reason ?? msg("hopeA3Console.plannerDidNotConfirmTheChange")); }
    const deadline = Date.now() + 2_500;
    while (Date.now() < deadline) {
      const current = latest.current;
      if ((current.lastReceived[TOPICS.plannerDiagnostics] ?? 0) >= requestedAt &&
          isFresh(current, TOPICS.plannerDiagnostics, Date.now(), 2_500) && current.hitSpeedScale?.scale === scale) {
        setNotice(msg("hopeA3Console.hitSpeedScaleConfirmedAppliesToSubsequentNew", {scale}));
        return;
      }
      await new Promise((resolve) => window.setTimeout(resolve, 100));
    }
    throw new LocalizedError(msg("hopeA3Console.scaleFeedbackTimeout"));
  }, [context]);

  const applyPlaneMode = useCallback(async (mode: PlaneMode) => {
    if (context.callService == undefined) { throw new LocalizedError(msg("hopeA3Console.connectToControlPort8766")); }
    const requestedAt = Date.now();
    const raw = await Promise.race([
      context.callService(PLANE_MODE_SERVICE, planeModeRequest(mode)), timeoutAfter(3_000),
    ]) as { result?: { successful?: boolean; reason?: string } };
    if (raw.result?.successful !== true) { throw new LocalizedError(raw.result?.reason ?? msg("hopeA3Console.plannerDidNotConfirmTheChange")); }
    const deadline = Date.now() + 2_500;
    while (Date.now() < deadline) {
      const current = latest.current;
      if ((current.lastReceived[TOPICS.plannerDiagnostics] ?? 0) >= requestedAt &&
          isFresh(current, TOPICS.plannerDiagnostics, Date.now(), 2_500) && current.planeMode?.requested === mode) {
        setNotice(msg("hopeA3Console.planeModeConfirmedForTheNextBallThe"));
        return;
      }
      await new Promise((resolve) => window.setTimeout(resolve, 100));
    }
    throw new LocalizedError(msg("hopeA3Console.planeFeedbackTimeout"));
  }, [context]);

  const applyBallYFilter = useCallback(async (config: { enabled: boolean; min: number; max: number }) => {
    const { enabled, min, max } = config;
    if (context.callService == undefined) { throw new LocalizedError(msg("hopeA3Console.connectToControlPort8766")); }
    const requestedAt = Date.now();
    const raw = await Promise.race([
      context.callService(BALL_Y_FILTER_SERVICE, ballYFilterRequest(config)),
      timeoutAfter(3_000),
    ]) as { result?: { successful?: boolean; reason?: string } };
    if (raw.result?.successful !== true) { throw new LocalizedError(raw.result?.reason ?? msg("hopeA3Console.plannerDidNotConfirmTheChange")); }
    const deadline = Date.now() + 2_500;
    while (Date.now() < deadline) {
      const current = latest.current;
      const state = current.ballYFilter;
      if ((current.lastReceived[TOPICS.plannerDiagnostics] ?? 0) >= requestedAt &&
          isFresh(current, TOPICS.plannerDiagnostics, Date.now(), 2_500) &&
          state?.enabled === enabled && state.min === min && state.max === max) {
        setNotice(msg("hopeA3Console.appliedYFilterAndTableMarkersUpdated"));
        return;
      }
      await new Promise((resolve) => window.setTimeout(resolve, 100));
    }
    throw new LocalizedError(msg("hopeA3Console.requestConfirmedButFeedbackTimedOutCheckThe"));
  }, [context]);

  const uploadAsset = useCallback(async (file: File | undefined) => {
    if (file == undefined) {return;}
    const maximum = 8 * 1024 * 1024;
    if (file.size > maximum) {
      setNotice(msg("hopeA3Console.rejectedCsvExceedsMib", {limit: maximum / 1024 / 1024}));
      return;
    }
    try {
      await fieldCommand({ op: "upload_csv",
        name: file.name, content: await file.text() });
    } catch (error) {
      setNotice(msg("hopeA3Console.fileReadFailed", {error: errorText(error)}));
    }
  }, [fieldCommand]);

  const invokeConfirmed = useCallback(
    (key: ServiceKey, prompt: LocalText) => {
      const current = latest.current;
      const pending = { key, prompt, host: current.lifecycleConfig.hdu_wifi_ip,
        session: current.lifecycleSession, boot: current.runnerBoot };
      confirmationRef.current = pending;
      setConfirmation(pending);
    },
    [],
  );

  const cancelConfirmation = () => {
    confirmationRef.current = undefined;
    setConfirmation(undefined);
  };
  const submitConfirmation = () => {
    // Re-read at the click, not the last React render. Consume synchronously so
    // double clicks or callbacks retained across a connection change cannot send twice.
    if (confirmation == undefined || confirmationRef.current !== confirmation || pendingTarget) { return; }
    const blocked = confirmationBlocked(confirmation, latest.current, Date.now());
    if (blocked != undefined) { setNotice(blocked); return; }
    const key = confirmation.key;
    cancelConfirmation();
    void invoke(key);
  };

  const confirmLifecycleConfig = useCallback(async () => {
    // A robot address is a connection target, never an edit to the old robot.
    if (configDraft.hdu_wifi_ip !== latest.current.lifecycleConfig.hdu_wifi_ip) {
      setNotice(msg("hopeA3Console.robotIpChangedUseConfirmAtTheTop"));
      return;
    }
    const key: ServiceKey = "applyLifecycleConfig";
    if (context.callService == undefined || busy[key] === true) {
      setNotice(msg("hopeA3Console.currentDataSourceDoesNotExposeLifecycleConfiguration"));
      return;
    }
    setBusy((current) => ({ ...current, [key]: true }));
    setNotice(msg("hopeA3Console.validatingAndConfirmingLifecycleConfiguration"));
    const parameters = (Object.keys(CONFIG_FIELDS) as ConfigField[])
      .filter((name) => name !== "operator_profile" || snapshot.lifecycleConfig.operator_profile != undefined)
      .map((name) => ({
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
        timeoutAfter(60_000),
      ])) as SetParametersResponse;
      const results = Array.isArray(raw.results) ? raw.results : [];
      const rejected = results.find((result) => result.successful !== true);
      if (results.length !== parameters.length || rejected != undefined) {
        const reason =
          typeof rejected?.reason === "string" ? rejected.reason : msg("hopeA3Console.invalidServiceResponse");
        setNotice(msg("hopeA3Console.configRejected", {reason}));
      } else {
        setConfigTouched(false);
        setNotice(
          msg("hopeA3Console.configConfirmed", {operator: configDraft.operator_profile ? msg("hopeA3Console.operator", {platform: configDraft.operator_profile.toUpperCase()}) : "", mocap: MOCAP_LABELS[configDraft.mocap_source] ?? "UNKNOWN", side: configDraft.table_side, gripper: configDraft.gripper_profile, pants: configDraft.pants_profile.toUpperCase()}),
        );
      }
    } catch (error) {
      setNotice(msg("hopeA3Console.configFailed", {error: errorText(error)}));
    } finally {
      setBusy((current) => ({ ...current, [key]: false }));
    }
  }, [busy, configDraft, context, snapshot.lifecycleConfig.operator_profile]);

  const startSystem = useCallback(() => {
    invokeConfirmed("startLifecycle",
      snapshot.lifecycleState === "FAILED"
        ? msg("hopeA3Console.cleanUpTheFailedSessionAndRetryStart")
        : snapshot.field?.mode === "KERNEL"
        ? msg("hopeA3Console.startKernelModeWithTheSelectedCsvConfirm")
        : msg("hopeA3Console.startStep012A2B45"),
    );
  }, [invokeConfirmed, snapshot.field?.mode, snapshot.lifecycleState]);

  const killAllAndCollect = useCallback(() => {
    invokeConfirmed("killAllAndCollect",
      snapshot.field?.mode === "KERNEL"
        ? msg("hopeA3Console.stopTheKernelModeRunnerAndHalAnd")
        : msg("hopeA3Console.immediatelyTerminateAllLifecycleManagedRunnerHalPlanner"),
    );
  }, [invokeConfirmed, snapshot.field?.mode]);

  const runTimeCalibration = useCallback(() => {
    invokeConfirmed("timeCalibration",
      msg("hopeA3Console.runTheControlled104HduTimeCalibration"),
    );
  }, [invokeConfirmed]);

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
    isFresh(snapshot, TOPICS.markerExpectedCount, now, 1_000) &&
    snapshot.markersFresh === true;
  const markerAsset = selectedMarkerAsset(snapshot, t);
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
    xboxFresh, xbox: snapshot.xboxPreview, estop: estopAsserted }, t);
  const xboxVelocity = xboxFresh ? snapshot.xboxPreview?.velocity : undefined;
  const velocityText = (values: number[] | undefined) => values == undefined
    ? "vx — · vy — · yaw —"
    : t("hopeA3Console.vxMSVyMSYawRad", {vx: (values[0] ?? 0).toFixed(2), vy: (values[1] ?? 0).toFixed(2), yaw: (values[2] ?? 0).toFixed(2)});

  const serveAvailable = snapshot.serveCapability === "AVAILABLE";
  const lifecycleFresh = isFresh(snapshot, TOPICS.lifecycleState, now, 1_500);
  const lifecycleState = lifecycleFresh ? (snapshot.lifecycleState ?? "UNKNOWN") : t("hopeA3Console.noData");
  const lifecycleStopped = lifecycleState === "STOPPED" || lifecycleState === "CONFIG_ERROR";
  const lifecycleRunning = lifecycleState === "RUNNING" || lifecycleState === "FAILED";
  const lifecycleBusy = snapshot.lifecycleBusy === true;
  const fieldFresh = isFresh(snapshot, TOPICS.fieldStatus, now, 2_000);
  const field = snapshot.field;
  const kernelMode = field?.mode === "KERNEL";
  const ballDisplay = ballModelDisplay(field?.ball_model_status, fieldFresh, t);
  const assetEditable = fieldFresh && lifecycleState === "STOPPED" && !lifecycleBusy &&
    !fieldBusy && field?.busy !== true && !configTouched && (snapshot.lifecycleConfigRevision ?? 0) > 0;
  const policyModelsFresh = fieldFresh && typeof field?.policy_models_age_s === "number" &&
    field.policy_models_age_s >= 0 && field.policy_models_age_s <= 30 &&
    field.policy_models_config_revision === snapshot.lifecycleConfigRevision;
  const policyUnavailableReason = (model: PolicyModel): string => {
    if (!assetEditable) return t("hopeA3Console.confirmConfigurationAndStopTheLifecycleBeforeSelecting");
    if (kernelMode && model !== "model14600") return t("hopeA3Console.selectOptitrackModeToUseThisPolicy");
    if (!policyModelsFresh) return t("hopeA3Console.waitingForInstalledModelVerification");
    const availability = field?.policy_models?.[model];
    return availability?.available === true ? "" : availability?.reason || t("hopeA3Console.modelIsNotInstalledOnBothBoards");
  };
  const recordingState = fieldFresh ? (field?.recording.state ?? "UNKNOWN") : "UNKNOWN";
  const recordingActive = ["ON", "STARTING", "STOPPING", "ERROR"].includes(recordingState);
  const assetFresh = isFresh(snapshot, TOPICS.assetMatched, now, 1_000);
  const confirmedSource = snapshot.lifecycleConfig.mocap_source ?? "";
  const vrpnSelected = configDraft.mocap_source === "vrpn";
  const setupCommand = mocapSetupCommand(snapshot.lifecycleConfig, snapshot.lifecycleSession);
  const configComplete =
    RUNNER_NETWORK_CONFIG_FIELDS.every((name) => configDraft[name].trim().length > 0) &&
    ["P1", "P2"].includes(configDraft.table_side) &&
    (snapshot.lifecycleConfig.operator_profile == undefined || ["linux", "mac"].includes(configDraft.operator_profile)) &&
    ["legacy", "new_20261015"].includes(configDraft.gripper_profile) &&
    ["v2", "v4"].includes(configDraft.pants_profile) &&
    ["none", "motive", "vrpn"].includes(configDraft.mocap_source) &&
    (configDraft.mocap_source !== "motive" || configDraft.motive_ip.trim().length > 0) &&
    (!vrpnSelected || (configDraft.vrpn_ip.trim().length > 0 &&
      /^[1-9][0-9]{0,4}$/.test(configDraft.vrpn_port) && Number(configDraft.vrpn_port) <= 65535)) &&
    ["mm", "m"].includes(configDraft.vrpn_units);
  const timeCalibrationFresh = isFresh(snapshot, TOPICS.timeCalibrationState, now, 1_500);
  const timeCalibrationState = timeCalibrationFresh
    ? (snapshot.timeCalibrationState ?? "UNKNOWN")
    : t("hopeA3Console.noData");
  const timeCalibrationBusy = snapshot.timeCalibrationBusy === true;
  const { ready: timeCalibrationReady, title: timeCalibrationTitle, blocksStart: timeCalibrationBlocksStart } =
    timeCalibrationControls({
      state: timeCalibrationState,
      fresh: timeCalibrationFresh,
      busy: timeCalibrationBusy,
      ntpFresh: ntpFresh && ntpGateFresh,
      ntpPass: snapshot.ntpPass,
      lifecycleStopped: lifecycleState === "STOPPED",
      lifecycleBusy: lifecycleBusy || !isFresh(snapshot, TOPICS.lifecycleBusy, now, 1_500),
      configConfirmed: (snapshot.lifecycleConfigRevision ?? 0) >= 1 && !configTouched,
    }, t);

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
      : t.text(notice);
  const confirmationReason = confirmation == undefined ? undefined :
    confirmationBlocked(confirmation, snapshot, now);

  return (
    <LanguageContext.Provider value={t}>
    <div className="hope-console">
      {confirmation != undefined && (
        <section className="action-confirmation" role="dialog" aria-label={t("hopeA3Console.confirmationTitle")}
          onKeyDown={event => { if (event.key === "Escape") { cancelConfirmation(); } }}>
          <strong>{t("hopeA3Console.confirmationTitle")}</strong>
          <p>{t.text(confirmation.prompt)}</p>
          {confirmationReason != undefined && <p role="status">{t.text(confirmationReason)}</p>}
          <div className="role-controls">
            <button ref={confirmationCancel} type="button" onClick={cancelConfirmation}>{t("hopeA3Console.cancelAction")}</button>
            <button type="button" disabled={confirmationReason != undefined || Boolean(pendingTarget) || busy[confirmation.key] === true}
              onClick={submitConfirmation}>{t("hopeA3Console.confirmAction")}</button>
          </div>
        </section>
      )}
      <div className="language-switch" role="group" aria-label={t("language.label")}>
        {(["zh", "en"] as const).map((language) => <button key={language} type="button"
          aria-pressed={locale === language} onClick={() => {
            setLocale(language);
            savedPanel.current = { ...savedPanel.current, language };
            context.saveState(savedPanel.current);
          }}>{t(language === "zh" ? "language.chinese" : "language.english")}</button>)}
      </div>
      <div className="robot-connection">
        <label>{t("hopeA3Console.connectionTargetIp")} <input aria-label={t("hopeA3Console.robotConnectionIp")} value={robotTarget}
          onChange={(event) => { setRobotTarget(event.target.value); setTargetEdited(true); }} /></label>
        <button type="button" disabled={!validRobotTarget || Object.values(busy).some(Boolean) || fieldBusy || recordBusy}
          onClick={connectRobot}>{t("hopeA3Console.confirmConnection")}</button>
        <div role="status">{pendingTarget ? t("hopeA3Console.switchingTo8766ControlsInThisPanelAre", {ip: pendingTarget}) :
          lifecycleFresh && configHostFresh && configHost ? t("hopeA3Console.addressReportedByTheCurrentRobot", {ip: configHost, state: snapshot.lifecycleState}) :
          lifecycleFresh ? t("hopeA3Console.liveStatusReceivedFromRobotAddressIsUnset", {state: snapshot.lifecycleState}) :
          t("hopeA3Console.noLiveRobotStatusTheTargetDoesNot")}</div>
        {pendingTarget && <button type="button" disabled={!configHostFresh || !configHost} onClick={() => { setPendingTarget(""); setRobotTarget(configHost ?? ""); setTargetEdited(false); savedPanel.current = { ...savedPanel.current, robotTarget: configHost }; context.saveState(savedPanel.current); }}>
          {t("hopeA3Console.cancelSwitchAndContinueWithTheCurrentRobot")} {configHost}
        </button>}
      </div>
      <fieldset className="robot-controls" disabled={Boolean(pendingTarget)}>
      <div className="source-row">
        <div className="source-block">
          <div className="eyebrow">{t("hopeA3Console.version", { version: "1.9.1" })}</div>
          <div className="source-value">
            <span>{context.dataSourceProfile?.toUpperCase() ?? t("hopeA3Console.ros2")} {t("hopeA3Console.control8766")}</span>
            <span
              className={`source-dot ${hduFresh && snapshot.hduActive === true ? "source-live" : ""}`}
            />
          </div>
          <div className="helper">{t("hopeA3Console.useTheRobotIpAndConfirmAboveTo")}</div>
        </div>
        <button
          className={`estop-button ${estopAsserted ? "estop-asserted" : ""}`}
          type="button"
          disabled={busy.estop === true}
          title={t.text(notice)}
          onClick={() => void invoke("estop")}
        >
          {busy.estop === true ? t("hopeA3Console.asserting") : estopAsserted ? t("hopeA3Console.eStopAsserted") : t("hopeA3Console.eStop")}
          <small>
            {estopAsserted
              ? t("hopeA3Console.clickToReassertResetSoftwareEStopBelow")
              : estopFullReady
                ? t("hopeA3Console.dualPathStopsRunnerAndRequestsVendorStop")
                : estopUsable
                  ? t("hopeA3Console.partialSoftwareStopUsePhysicalEStop")
                  : t("hopeA3Console.backendUnknownClickStillAsserts")}
          </small>
        </button>
      </div>

      <button
        type="button"
        disabled={busy.estop === true || busy.resetSoftwareEstop === true}
        onClick={() => void invoke("resetSoftwareEstop")}
        title={t("hopeA3Console.clearsTheSoftwareLatchOnlyDoesNotRelease")}
      >
        {busy.resetSoftwareEstop === true ? t("hopeA3Console.resetting") : t("hopeA3Console.resetSoftwareEStop")}
      </button>

      <div className="gate-row">
        <GateChip
          label={t("hopeA3Console.ntpAudit")}
          value={ntpFresh && ntpGateFresh ? snapshot.ntpPass : undefined}
        />
        <GateChip label={t("hopeA3Console.timestampAudit")} value={latencyFresh ? true : undefined} />
        <GateChip
          label={t("hopeA3Console.tfAudit")}
          value={isFresh(snapshot, TOPICS.tfReady, now, 1_000) ? snapshot.tfReady : undefined}
        />
        <GateChip
          label={t("hopeA3Console.eStopBackendAudit")}
          value={estopAsserted ? false : estopUsable ? estopFullReady : undefined}
          detail={snapshot.estopText}
        />
        <GateChip
          label={confirmedSource === "vrpn" ? t("hopeA3Console.vrpnLiveMarkerCountNA") : t("hopeA3Console.markersAudit", {asset: markerAsset, actual: markerFresh ? Math.round(snapshot.markerCount ?? 0) : "\u2014", expected: markerFresh && (snapshot.markerExpectedCount ?? 0) > 0 ? Math.round(snapshot.markerExpectedCount ?? 0) : "\u2014"})}
          value={confirmedSource === "motive" && markerFresh && (snapshot.markerExpectedCount ?? 0) > 0 ? snapshot.markerCount === snapshot.markerExpectedCount : undefined}
          detail={confirmedSource === "vrpn" ? t("hopeA3Console.vrpnStreamsPosesOnlyLiveMarkerCountsAre") : t("hopeA3Console.selectedMarkerAsset", {asset: markerAsset})}
        />
        <GateChip
          label={t("hopeA3Console.swingInputAudit", {status: snapshot.runnerMode === "MOTION" ? `F${snapshot.commandFlight ?? "\u2014"} · ${snapshot.commandCountdownS != undefined ? `${snapshot.commandCountdownS.toFixed(2)}s` : t("hopeA3Console.noCommand")}` : t("hopeA3Console.waitForMotion")})}
          value={swingInputReady}
          detail={snapshot.commandSummary ?? t("hopeA3Console.ballVisibilityAloneIsNotASwingCommand")}
        />
      </div>

      <div className={`operator-notice ${estopAsserted ? "operator-notice-danger" : ""}`}>
        <span className="eyebrow">{t("hopeA3Console.lastUiRequest")}</span>
        <span>{t.text(notice)}</span>
      </div>

      <div className="field-toolbar">
        <div>
          <span className="eyebrow">{t("hopeA3Console.operatingModeSelectWhileStopped")}</span>
          <div className="field-buttons" role="group" aria-label={t("hopeA3Console.operatingMode")}>
            <button type="button" aria-pressed={field?.mode === "OPTITRACK"}
              disabled={!assetEditable} onClick={() => void fieldCommand({ op: "set_mode", mode: "OPTITRACK" })}>
              {t("hopeA3Console.normalPlay")} </button>
            <button type="button" aria-pressed={kernelMode}
              disabled={!assetEditable} onClick={() => void fieldCommand({ op: "set_mode", mode: "KERNEL" })}>
              {t("hopeA3Console.kernelMode")} </button>
          </div>
          <p className="helper">{!fieldFresh ? t("hopeA3Console.waitingForFieldControlsUpdateTheHduBackend") :
            kernelMode ? t("hopeA3Console.noMocapImuStanceFootOdometryEstimatedLocal") :
            t("hopeA3Console.normalPlayStartsTheLaptopDataPipelineFor")}</p>
        </div>
        <div className={`record-control ${recordingActive ? "record-active" : ""}`}>
          <span className="eyebrow">{t("hopeA3Console.recording")} {recordingState}</span>
          <button type="button" role="switch" aria-checked={recordingActive}
            disabled={!fieldFresh || recordBusy || recordingState === "STOPPING"}
            onClick={() => void fieldCommand({ op: recordingActive ? "record_off" : "record_on" })}>
            {recordBusy ? t("hopeA3Console.updating") : recordingActive ? t("hopeA3Console.recordOffStop") : t("hopeA3Console.recordOnStart")}
          </button>
          <span className="helper">{field?.mode === "KERNEL" ? t("hopeA3Console.defaultOffRobotDataKernelHasNoPlanner") : t("hopeA3Console.defaultOffRobotPolicyPlannerAndMocapData")}</span>
          <span className="asset-path">{field?.recording.output}</span>
          <span className="helper">{field?.recording.error}</span>
        </div>
      </div>

      <div className="field-assets">
        <section className="field-card">
          <span className="eyebrow">{t("hopeA3Console.receivePolicy")}</span>
          <strong>{field?.policy_model === "model12000" ? t("hopeA3Console.12000V2312000ServeReceive") : field?.policy_model === "model17400" ? (field.policy_models?.model17400?.receive_only === true ? t("hopeA3Console.17400ReceiveOnly") : t("hopeA3Console.17400V2317400ServeReceive")) : field?.policy_model === "model12600" ? t("hopeA3Console.12600V2312600ServeReceive") : field?.policy_model === "model11999" ? t("hopeA3Console.11999ReceiveOnly") : field?.policy_model === "model14600" ? t("hopeA3Console.14600ServeReceive") : t("hopeA3Console.noPolicySelected")}</strong>
          <div className="field-buttons">
            <button type="button" disabled={!!policyUnavailableReason("model14600")} title={policyUnavailableReason("model14600")}
              aria-pressed={field?.policy_model === "model14600"}
              onClick={() => void fieldCommand({op: "set_policy_model", model: "model14600"})}>14600</button>
            <button type="button" disabled={!!policyUnavailableReason("model11999")} title={policyUnavailableReason("model11999")}
              aria-pressed={field?.policy_model === "model11999"}
              onClick={() => void fieldCommand({op: "set_policy_model", model: "model11999"})}>{t("hopeA3Console.11999ReceiveOnly")}</button>
            <button type="button" disabled={!!policyUnavailableReason("model12600")} title={policyUnavailableReason("model12600")}
              aria-pressed={field?.policy_model === "model12600"}
              onClick={() => void fieldCommand({op: "set_policy_model", model: "model12600"})}>{t("hopeA3Console.12600ServeReceive")}</button>
            <button type="button" disabled={!!policyUnavailableReason("model17400")} title={policyUnavailableReason("model17400")}
              aria-pressed={field?.policy_model === "model17400"}
              onClick={() => void fieldCommand({op: "set_policy_model", model: "model17400"})}>{t("hopeA3Console.17400ServeReceive")}</button>
            <button type="button" disabled={!!policyUnavailableReason("model12000")} title={policyUnavailableReason("model12000")}
              aria-pressed={field?.policy_model === "model12000"}
              onClick={() => void fieldCommand({op: "set_policy_model", model: "model12000"})}>{t("hopeA3Console.12000ServeReceive")}</button>
          </div>
          <p className="helper">{t("hopeA3Console.switchWhileStoppedThenStartToLoadEach")}</p>
        </section>
        <section className="field-card">
          <span className="eyebrow">{t("hopeA3Console.mocapPelvisAsset")}</span>
          <strong>{confirmedSource === "none" ? t("hopeA3Console.noMocapSelected") : confirmedSource === "vrpn" ? t("hopeA3Console.vrpnPoseOnly") : assetFresh ? snapshot.assetMatched === true ? t("hopeA3Console.assetAccepted") : t("hopeA3Console.checkMocapAsset") : t("hopeA3Console.waitingForMocap")}</strong>
          <span className="helper">{t("hopeA3Console.confirmed")}{t.text(MOCAP_LABELS[confirmedSource] ?? msg("hopeA3Console.waitingForConfiguration"))}</span>
          <span className="asset-path">{markerAsset}</span>
          <span className="helper">{confirmedSource === "none" ? t("hopeA3Console.runnerOnlyNoMocapPipeline") : assetFresh ? snapshot.assetText : t("hopeA3Console.importTheSelectedPelvisAlignedAssetAndEnable", {asset: markerAsset})}</span>
          <span className="helper">{t("hopeA3Console.livePelvis")} {baseFresh ? t("hopeA3Console.available") : t("hopeA3Console.waiting")} {t("hopeA3Console.selectedRobot")} {markerAsset}</span>
          {confirmedSource === "vrpn" && <p className="helper">{t("hopeA3Console.vrpnStreamsPosesOnlyMarkerCountsAndImported")}</p>}
          {confirmedSource === "motive" && <p className="helper">{t("hopeA3Console.motiveNatnetUsesLiveAssetsVisibleMarkerCounts")}</p>}
        </section>
        <section className="field-card">
          <span className="eyebrow">{t("hopeA3Console.serveCsv")}</span>
          <strong>{field?.csv?.name ?? t("hopeA3Console.a3pOp3Serve025V23Branch110Model14600CsvDefault")}</strong>
          <span className="helper">{field?.csv ? t("hopeA3Console.validatedFramesHzSha", {frames: field.csv.frames, hz: field.csv.hz, hash: field.csv.sha256.slice(0, 12)}) : t("hopeA3Console.468Frames100HzSdk31JointFormat")}</span>
          {field?.csv?.segments && <span className="helper">
            {t("hopeA3Console.csvTimeline", { branch: (field.csv.segments.branch_frame / field.csv.hz).toFixed(2), handoff: field.csv.segments.clips.SERVE_TO_POLICY?.duration_s.toFixed(2) })} </span>}
          <div className="field-buttons">
            <button type="button" disabled={!assetEditable} onClick={() => csvInput.current?.click()}>{t("hopeA3Console.uploadCsv")}</button>
            <button type="button" disabled={!assetEditable || field?.csv == undefined}
              onClick={() => void fieldCommand({ op: "use_default_csv" })}>{t("hopeA3Console.useBuiltInCsv")}</button>
          </div>
          <input ref={csvInput} type="file" accept=".csv,text/csv" hidden
            onChange={(event) => { void uploadAsset(event.target.files?.[0]); event.target.value = ""; }} />
          <p className="helper">{t("hopeA3Console.selectKernelModeUploadWhileStoppedThenRun")}</p>
          <span className="helper">{t("hopeA3Console.uploadsUseTheRunnerCsvValidatorAndNever")}</span>
        </section>
      </div>
      {field?.error && <div className="operator-notice">{field.error}</div>}

      <div className="lifecycle-card">
        <div className="lifecycle-header">
          <div>
            <div className="eyebrow">{t("hopeA3Console.systemLifecycleStep012A2B4")}</div>
            <div className="lifecycle-state">
              {lifecycleState} · {snapshot.lifecycleStep ?? t("hopeA3Console.idle")}
            </div>
            <div className="topic-path">
              {snapshot.lifecycleSession != undefined && snapshot.lifecycleSession.length > 0
                ? snapshot.lifecycleSession
                : t("hopeA3Console.noActiveSession")}{" "}
              {t("hopeA3Console.configRevision")} {snapshot.lifecycleConfigRevision ?? 0}
            </div>
          </div>
          <div className="lifecycle-result" title={snapshot.lifecycleResult}>
            {snapshot.lifecycleResult ?? t("hopeA3Console.waitingForSupervisor")}
          </div>
        </div>
        <label className="table-side-selector">
          <div>
            <div className="eyebrow">{t("hopeA3Console.mocapHeading")}</div>
            <div className="topic-path">{t("hopeA3Console.selectWhileStoppedThenConfirmConfigOnlyThe")}</div>
          </div>
          <select aria-label={t("hopeA3Console.mocapInput")} value={configDraft.mocap_source}
            disabled={!lifecycleStopped || lifecycleBusy}
            onChange={(event) => {
              setConfigTouched(true);
              setConfigDraft((current) => ({ ...current, mocap_source: event.target.value }));
            }}>
            <option value="" disabled>{t("hopeA3Console.selectAMocapSystem")}</option>
            {Object.entries(MOCAP_LABELS).map(([value, label]) => <option key={value} value={value}>{t.text(label)}</option>)}
          </select>
        </label>
        {snapshot.lifecycleConfig.operator_profile != undefined && <div className="table-side-selector" role="group" aria-label={t("hopeA3Console.operatorComputer")}>
          <div>
            <div className="eyebrow">{t("hopeA3Console.operatorComputer2")}</div>
            <div className="helper">{t("hopeA3Console.chooseTheComputerForThisSessionThenConfirm")}</div>
          </div>
          {(["linux", "mac"] as const).map((profile) => (
            <button key={profile} type="button"
              className={configDraft.operator_profile === profile ? "side-selected" : ""}
              aria-pressed={configDraft.operator_profile === profile}
              disabled={!lifecycleStopped || lifecycleBusy}
              onClick={() => {
                setConfigTouched(true);
                setConfigDraft((current) => ({ ...current, operator_profile: profile }));
              }}>
              {profile === "mac" ? t("hopeA3Console.mac") : t("hopeA3Console.linux")}
            </button>
          ))}
        </div>}
        <div className="config-grid">
          {NETWORK_CONFIG_FIELDS.filter((name) => name !== "hdu_wifi_ip").filter((name) =>
            name === "motive_ip" ? configDraft.mocap_source === "motive" :
            name === "vrpn_ip" ? vrpnSelected : true).map((name) => (
            <label className="config-field" key={name}>
              <span>{t.text(CONFIG_FIELDS[name].label)}</span>
              <input
                type="text"
                inputMode="decimal"
                spellCheck={false}
                disabled={!lifecycleStopped || lifecycleBusy}
                value={configDraft[name]}
                placeholder={t("hopeA3Console.ipv4Address")}
                onChange={(event) => {
                  setConfigTouched(true);
                  setConfigDraft((current) => ({ ...current, [name]: event.target.value }));
                }}
              />
            </label>
          ))}
          {vrpnSelected && <>
            <label className="config-field"><span>{t("hopeA3Console.vrpnPort")}</span>
              <input aria-label={t("hopeA3Console.vrpnPort")} inputMode="numeric" value={configDraft.vrpn_port}
                disabled={!lifecycleStopped || lifecycleBusy}
                onChange={(event) => { setConfigTouched(true); setConfigDraft((current) => ({ ...current, vrpn_port: event.target.value })); }} />
            </label>
            <label className="config-field"><span>{t("hopeA3Console.vrpnOutputUnits")}</span>
              <select aria-label={t("hopeA3Console.vrpnUnits")} value={configDraft.vrpn_units}
                disabled={!lifecycleStopped || lifecycleBusy}
                onChange={(event) => { setConfigTouched(true); setConfigDraft((current) => ({ ...current, vrpn_units: event.target.value })); }}>
                <option value="mm">{t("hopeA3Console.millimetersMm0001M")}</option>
                <option value="m">{t("hopeA3Console.metersM10")}</option>
              </select>
            </label>
          </>}
        </div>
        <label className="table-side-selector">
          <div>
            <div className="eyebrow">{t("hopeA3Console.ballPhysics")}</div>
            <div className="topic-path">{t("hopeA3Console.selectWhileStoppedSavedToPlannerAndBall")}</div>
          </div>
          <select aria-label={t("hopeA3Console.ballPhysicsModel")} value={ballDisplay.value}
            disabled={!assetEditable || fieldBusy || kernelMode || field?.ball_model_status == undefined}
            onChange={(event) => void fieldCommand({ op: "set_ball_model", model: event.target.value })}>
            <option value="" disabled>{t("hopeA3Console.bothEndpointsHaveNotConfirmedTheSameModel")}</option>
            <option value="berkeley">{t("hopeA3Console.berkeleyBallOriginalModel20261010")}</option>
            <option value="ppq002">{t("hopeA3Console.ourOneStarBallPpq002")}</option>
            <option value="ppq004">{t("hopeA3Console.ourThreeStarBallPpq004UnifiedModel2026")}</option>
          </select>
        </label>
        <p className="helper" role="status">{ballDisplay.text}</p>
        <p className="helper">{t("hopeA3Console.afterSwitchingRestartLaptopMocapBallBridgeThen")}</p>
        <div className="mocap-setup">
          {configDraft.mocap_source === "motive" && <p className="helper">{t("hopeA3Console.motiveSetupImportThePelvisAlignedMotiveAsset")}</p>}
          {vrpnSelected && <>
            <p className="helper">{t("hopeA3Console.chingmuSetup", { pants: configDraft.pants_profile, side: configDraft.table_side })}</p>
            <p className="helper">{t("hopeA3Console.defaultStartupAcceptsPosesOnlyPublishingAValid")}</p>
          </>}
          {configDraft.mocap_source === "none" && <p className="helper">{t("hopeA3Console.noMocapRunnerOnlyTestingWithoutLiveBall")}</p>}
          <p className="helper">{t("hopeA3Console.confirmConfigSavesSettingsAndReconnectsTheSelected")}</p>
          <p className="helper">{t("hopeA3Console.connectionStatusIsBasedOnLiveDataConnected")}</p>
          <p className="helper" role="status">{busy.applyLifecycleConfig ? t("hopeA3Console.savingAndConnectingMocap") : snapshot.lifecycleResult?.includes("MOCAP_FAILED") ? t("hopeA3Console.mocapConnectionFailedConfigurationSavedCheckTheAddress") : configTouched ? t("hopeA3Console.configurationChangedNotAppliedYet") : confirmedSource === "none" ? t("hopeA3Console.mocapDisabled") :
            baseFresh && (confirmedSource === "vrpn" || markerFresh) ? t("hopeA3Console.mocapDataConnectedLiveDataReceived") : t("hopeA3Console.mocapConnectionUnconfirmedWaitingForLiveData")}</p>
          {lifecycleFresh && setupCommand && <details><summary>{t("hopeA3Console.manualStartupCommandFallback")}</summary><textarea aria-label={t("hopeA3Console.confirmedMocapStartupCommand")} readOnly rows={5} value={setupCommand} /></details>}
        </div>
        <div className="table-side-selector">
          <div>
            <div className="eyebrow">{t("hopeA3Console.ourTableSideLockedIntoNextStart")}</div>
            <div className="topic-path">
              {t("hopeA3Console.bothMocapSystemsUseUcbP1UcbP2")} {!kernelMode && fieldFresh && field?.player_positions?.[configDraft.table_side as "P1" | "P2"] !== true &&
                t("hopeA3Console.peerNotVerified")}
            </div>
          </div>
          {(["P1", "P2"] as const).map((side) => (
            <button
              key={side}
              className={configDraft.table_side === side ? "side-selected" : ""}
              type="button"
              disabled={!lifecycleStopped || lifecycleBusy || (!kernelMode &&
                !playerSideAvailable(side, [snapshot.playerP1, snapshot.playerP2], now))}
              onClick={() => {
                setConfigTouched(true);
                setConfigDraft((current) => ({ ...current, table_side: side }));
              }}
            >
              {side === "P1" ? t("hopeA3Console.ucbP1Direct") : t("hopeA3Console.ucbP2Rotate180")}
            </button>
          ))}
        </div>
        <label className="table-side-selector">
          <div>
            <div className="eyebrow">{t("hopeA3Console.gripper2")}</div>
            <div className="topic-path">{t("hopeA3Console.manualOpenCloseAndServeReleaseChangesTake")}</div>
          </div>
          <select
            aria-label={t("hopeA3Console.gripperProfile")}
            value={configDraft.gripper_profile}
            disabled={!lifecycleStopped || lifecycleBusy}
            onChange={(event) => {
              setConfigTouched(true);
              setConfigDraft((current) => ({ ...current, gripper_profile: event.target.value }));
            }}
          >
            <option value="" disabled>{t("hopeA3Console.selectAGripper")}</option>
            <option value="legacy">{t("hopeA3Console.oldGripperClose800Open2000")}</option>
            <option value="new_20261015">{t("hopeA3Console.newGripperClose850Open1500")}</option>
          </select>
        </label>
        {vrpnSelected && <label className="table-side-selector">
          <div>
            <div className="eyebrow">{t("hopeA3Console.markerPants2")}</div>
            <div className="topic-path">{t("hopeA3Console.selectWhileStoppedThenConfirmConfigVrpnSwitches")}</div>
          </div>
          <select aria-label={t("hopeA3Console.markerPantsProfile")} value={configDraft.pants_profile}
            disabled={!lifecycleStopped || lifecycleBusy}
            onChange={(event) => {
              setConfigTouched(true);
              setConfigDraft((current) => ({ ...current, pants_profile: event.target.value }));
            }}>
            <option value="" disabled>{t("hopeA3Console.selectPants")}</option>
            <option value="v2">{t("hopeA3Console.v2OldPants10Markers")}</option>
            <option value="v4">{t("hopeA3Console.v412MarkersEachForP1P2")}</option>
          </select>
        </label>}
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
            {busy.applyLifecycleConfig === true ? t("hopeA3Console.confirming") : t("hopeA3Console.confirmConfig")}
          </button>
          <button
            className="system-start"
            type="button"
            disabled={
              !(lifecycleStopped || lifecycleState === "FAILED") ||
              lifecycleBusy ||
              timeCalibrationBlocksStart ||
              (snapshot.lifecycleConfigRevision ?? 0) < 1 ||
              configTouched ||
              fieldBusy || field?.busy === true || !fieldFresh ||
              (!kernelMode && lifecycleState !== "FAILED" && !ballDisplay.ready) ||
              (!kernelMode && lifecycleState !== "FAILED" &&
                (!playerSideAvailable(configDraft.table_side as "P1" | "P2",
                  [snapshot.playerP1, snapshot.playerP2], now) ||
                  field?.player_positions?.[configDraft.table_side as "P1" | "P2"] !== true)) ||
              busy.startLifecycle === true
            }
            onClick={startSystem}
          >
            {busy.startLifecycle === true || (lifecycleBusy && lifecycleState === "STARTING")
              ? t("hopeA3Console.starting")
              : lifecycleBusy && lifecycleState === "KILLING" ? t("hopeA3Console.cleaning")
              : lifecycleState === "FAILED" ? t("hopeA3Console.retryStartCleanUpAutomatically") : t("hopeA3Console.startSystem")}
          </button>
          <button
            className="system-stop"
            type="button"
            disabled={!lifecycleRunning || lifecycleBusy || busy.killAllAndCollect === true}
            onClick={killAllAndCollect}
          >
            {busy.killAllAndCollect === true || (lifecycleBusy && lifecycleState === "KILLING")
              ? t("hopeA3Console.killing")
              : t("hopeA3Console.killAllCollect")}
          </button>
          <span className="config-helper">
            {t("hopeA3Console.editAndConfirmConfigurationWhileStoppedChangingThe")} </span>
        </div>
        <div
          className={`time-calibration-row time-calibration-${timeCalibrationState.toLowerCase().replace(/[_ ]/g, "-")}`}
        >
          <div className="time-calibration-copy">
            <div className="eyebrow">{t("hopeA3Console.timeCalibrationRunbook104ReceiptSafe")}</div>
            <div className="time-calibration-state">
              {timeCalibrationState} · {snapshot.timeCalibrationStep ?? t("hopeA3Console.idle")}
            </div>
            <div className="topic-path" title={snapshot.timeCalibrationResult}>
              {snapshot.timeCalibrationOperation != undefined &&
              snapshot.timeCalibrationOperation.length > 0
                ? `${snapshot.timeCalibrationOperation} · `
                : ""}
              {snapshot.timeCalibrationResult ?? t("hopeA3Console.waitingForCalibrationCoordinator")}
            </div>
          </div>
          <button
            className="time-calibration-button"
            type="button"
            disabled={!timeCalibrationReady || busy.timeCalibration === true}
            title={timeCalibrationTitle}
            onClick={runTimeCalibration}
          >
            {busy.timeCalibration === true || timeCalibrationBusy
              ? t("hopeA3Console.calibrating")
              : t("hopeA3Console.timeCalibration")}
          </button>
        </div>
      </div>

      <section className="diagnostics-section" aria-label={t("hopeA3Console.diagnostics")}>
        <div className="diagnostics-content">
      <div className="metric-grid">
        <div className="metric-card">
          <div className="eyebrow">{t("hopeA3Console.ntpWorldClockOffset")}</div>
          <div className={`metric-value ${ntpFresh ? "" : "stale"}`}>
            <span>
              {ntpFresh && snapshot.ntpOffsetMs != undefined
                ? snapshot.ntpOffsetMs.toFixed(2)
                : "—"}
            </span>
            <small>{t("hopeA3Console.ms")}</small>
          </div>
          <div className="topic-path">
            {TOPICS.ntpOffset}
            {snapshot.ntpDispersionMs != undefined
              ? t("hopeA3Console.disp", {dispersion: snapshot.ntpDispersionMs.toFixed(2)})
              : ""}
            {snapshot.ntpSkewPpm != undefined
              ? t("hopeA3Console.skewPpmAuditOnly", {skew: snapshot.ntpSkewPpm.toFixed(2)})
              : ""}
            {ntpFresh ? "" : t("hopeA3Console.stale")}
          </div>
        </div>
        <div className="metric-card">
          <div className="eyebrow">{t("hopeA3Console.ros2MessageLatency")}</div>
          <div className={`metric-value ${latencyFresh ? "" : "stale"}`}>
            <span>
              {latencyFresh && snapshot.latencyMs != undefined
                ? snapshot.latencyMs.toFixed(1)
                : "—"}
            </span>
            <small>{t("hopeA3Console.ms")}</small>
          </div>
          <div className="topic-path">
            {TOPICS.latency}
            {latencyFresh ? "" : t("hopeA3Console.stale")}
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
          name={t("hopeA3Console.runnerProcess")}
          topic={TOPICS.mduActive}
          value={mduFresh ? snapshot.mduActive : undefined}
        />
      </div>

      <div className="cpu-card">
        <div className="cpu-header">
          <span className="eyebrow">{t("hopeA3Console.a3CpuLoad")}</span>
          <span className={`cpu-value ${cpuFresh ? "" : "stale"}`}>
            {cpuFresh && snapshot.cpuPercent != undefined ? snapshot.cpuPercent.toFixed(0) : "—"}
            <small>%</small>
          </span>
          <div className="cpu-diagnostics">
            <span className="topic-path">{TOPICS.cpu} {t("hopeA3Console.0100120S")}</span>
            <span className="topic-path" title={snapshot.cpuTopProcess}>
              {snapshot.cpuTopProcess ?? t("hopeA3Console.topProcessWarmingUp")}
            </span>
          </div>
        </div>
        <svg
          className="cpu-plot"
          viewBox="0 0 600 160"
          preserveAspectRatio="none"
          aria-label={t("hopeA3Console.cpuLoadOver120Seconds")}
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
        <span className="eyebrow">{t("hopeA3Console.xboxBodyFrameLocomotion")}</span>
        <div className="teleop-details">
          <span>{t("hopeA3Console.controller")}</span><span title={snapshot.xboxPreview?.device}>{snapshot.xboxPreview?.device === "" ? t("hopeA3Console.noControllerDetected") : (snapshot.xboxPreview?.device ?? t("hopeA3Console.noControllerDetected"))}</span>
          <span>{t("hopeA3Console.input")}</span><span>{!xboxFresh ? t("hopeA3Console.noFreshXboxInput") : snapshot.xboxPreview?.connected !== true
            ? t("hopeA3Console.disconnected") : t("hopeA3Console.liveLt", {lt: snapshot.xboxPreview.lt ? t("hopeA3Console.held") : t("hopeA3Console.released"), source: snapshot.xboxPreview.state})}</span>
          <span>{t("hopeA3Console.sticks")}</span><span>{velocityText(xboxVelocity)}</span>
          <span>{t("hopeA3Console.runner")}</span><span>{!runnerFresh ? t("hopeA3Console.noFreshRunnerStatus") : teleop == undefined
            ? t("hopeA3Console.waitingForTeleopTelemetry") : teleop[2] !== 1 ? t("hopeA3Console.teleopPolicyNotLoaded")
              : t("hopeA3Console.enableTransition", {mode: snapshot.runnerMode ?? t("hopeA3Console.unknown"), phase: [t("hopeA3Console.off"), t("hopeA3Console.entering"), t("hopeA3Console.active"), t("hopeA3Console.stopping")][teleop[3] ?? 0] ?? t("hopeA3Console.unknown"), enabled: teleop[9] === 1 ? t("hopeA3Console.on") : t("hopeA3Console.off"), progress: ((teleop[5] ?? 0) * 100).toFixed(0)})}</span>
          <span>{t("hopeA3Console.command")}</span><span>{velocityText(teleop?.slice(6, 9))}</span>
          <span>{t("hopeA3Console.modeKeys")}</span><span title={snapshot.xboxPreview?.action_status}>{xboxFresh ? (snapshot.xboxPreview?.action_status ?? t("hopeA3Console.aPrepareBServeXReceiveYTeleop")) : t("hopeA3Console.waitingForXboxInput")}</span>
          <span>{t("hopeA3Console.inputAge")}</span><span>{teleop == undefined || (teleop[4] ?? 1e9) >= 1e9
            ? t("hopeA3Console.noInputReceivedByRunner") : `${Math.min(9999, (teleop[4] ?? 9999) * 1000).toFixed(0)} ms`}</span>
        </div>
        <div className="role-controls">
          <button type="button" disabled={teleopBlocked != undefined} title={teleopBlocked}
            onClick={() => void invoke("teleop")}>{t("hopeA3Console.enterTeleop")}</button>
          <button type="button" disabled={!runnerFresh || snapshot.runnerMode !== "TELEOP" || busy.stand === true}
            onClick={() => void invoke("stand")}>{t("hopeA3Console.stopAndReturnToStand")}</button>
        </div>
        <div className="teleop-entry-status" title={teleopBlocked ?? t("hopeA3Console.readyToRequestTeleopRunnerValidatesCurrentInput")}>
          {teleopBlocked ?? t("hopeA3Console.readyToRequestTeleopRunnerValidatesCurrentInput")}
        </div>
        <div className="sequence-status">{t("hopeA3Console.waitForActiveCenterTheSticksThenRelease")}</div>
      </div>

      <div className="sequence-card">
        <div className="sequence-header">
          <div className="sequence-summary">
            <span className="eyebrow">{kernelMode ? t("hopeA3Console.kernelModeTwoStepServe") : t("hopeA3Console.optitrackNormalRunnerSequence")}</span>
            <span className="sequence-status">
              {runnerFresh
                ? t("hopeA3Console.runnerSummary", {mode: snapshot.runnerMode ?? t("hopeA3Console.unknown"), serve: serveState, gripper: snapshot.gripperState ?? t("hopeA3Console.unavailable"), cleanup: snapshot.serveCleanupRequired === true ? t("hopeA3Console.cleanupRequired") : ""})
                : t("hopeA3Console.noFreshRunnerState")}
            </span>
          </div>
          <div className="role-controls">
            <span className="role-label">{t("hopeA3Console.ourRole")} {snapshot.localRole ?? t("hopeA3Console.unassigned")}</span>
            <button
              type="button"
              disabled={snapshot.roleChangeAllowed !== true || busy.setServer === true}
              onClick={() => void invoke("setServer")}
            >
              {t("hopeA3Console.server")} </button>
            <button
              type="button"
              disabled={snapshot.roleChangeAllowed !== true || busy.setReceiver === true}
              onClick={() => void invoke("setReceiver")}
            >
              {t("hopeA3Console.receiver")} </button>
          </div>
        </div>
        <div className="action-grid">
          <ActionButton
            label={t("hopeA3Console.stand")}
            detail={
              snapshot.runnerMode === "TELEOP"
                ? t("hopeA3Console.stopAndSettleWaitsForTheFeetThen")
                : serveWorkflowActive
                ? t("hopeA3Console.phaseAwareAbortRecoveryStaysInThisRunner")
                : runnerStanding
                  ? t("hopeA3Console.donePdStand")
                  : t("hopeA3Console.sameAsKeyboardS")
            }
            disabled={!runnerUsable}
            busy={busy.stand === true}
            completed={runnerStanding}
            next={nextStep === "stand"}
            onClick={() => void invoke("stand")}
          />
          <div className="ready-controls">
          <ActionButton
            label={t("hopeA3Console.ready")}
            detail={
              runnerReady
                ? kernelMode ? t("hopeA3Console.readyReceivePolicyTestStartToServeBegins") : t("hopeA3Console.readyReceivingStartToServeBeginsTheNext")
                : snapshot.localRole !== "RECEIVER"
                ? kernelMode ? t("hopeA3Console.serverLowersArmsAndReturnsToStandAfter") : t("hopeA3Console.serverEntersReadyAutomaticallyAfterServeCompletes")
                : !runnerStanding
                  ? t("hopeA3Console.lockedStandFirst")
                  : kernelMode
                    ? t("hopeA3Console.enterReceivePolicyNoMocapRequiredImuLocal")
                    : baseFresh
                    ? t("hopeA3Console.enterMotionFreshMocapBaseIsAvailable")
                    : t("hopeA3Console.normalPlayUsesTheSelectedMocapSetupKernel")
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
          <HitSpeedScaleControls key={configHost}
            state={isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) ? snapshot.hitSpeedScale : undefined}
            available={!kernelMode && context.callService != undefined &&
              isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) && snapshot.hitSpeedScale?.configurable === true}
            kernelMode={kernelMode} onApply={applyHitSpeedScale}
          />
          <PlaneModeControls key={configHost}
            state={isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) ? snapshot.planeMode : undefined}
            available={!kernelMode && context.callService != undefined &&
              isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) && snapshot.planeMode?.configurable === true}
            onApply={applyPlaneMode}
          />
          <BallYFilterControls
            state={isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) ? snapshot.ballYFilter : undefined}
            available={!kernelMode && context.callService != undefined &&
              isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) && snapshot.ballYFilter?.configurable === true}
            kernelMode={kernelMode} onApply={applyBallYFilter}
            key={configHost}
            initialState={savedPanel.current.ballYFilterRobot === configHost ? savedPanel.current : undefined} onPreview={previewBallYFilter}
          />
          </div>
          <ActionButton
            label={t("hopeA3Console.startToServeRaiseHand")}
            detail={
              !serveAvailable
                ? t("hopeA3Console.unavailableLaunchRunnerWithServe")
                : snapshot.localRole !== "SERVER"
                  ? t("hopeA3Console.serverOnly")
                  : serveWorkflowActive
                    ? t("hopeA3Console.sameRunnerOwnsQDes", {state: serveState})
                    : t("hopeA3Console.smoothStandReadyLoadingPoseGripperClosesAutomatically")
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
            onClick={() => {
              invokeConfirmed(
                "prepareServe",
                msg("hopeA3Console.prepareServeConfirm"),
              );
            }}
          />
          <ActionButton
            label={t("hopeA3Console.servePlay")}
            detail={
              serveState === "WAIT_READY_TO_SERVE"
                ? t("hopeA3Console.loadBallClearHandsStartsSelectedFull31Timeline", {gripper: snapshot.gripperState ?? t("hopeA3Console.unavailable")})
                : snapshot.serving === true
                  ? t("hopeA3Console.playing", { state: serveState })
                  : kernelMode && serveState === "COMPLETE" && snapshot.runnerMode === "PD_STAND"
                    ? t("hopeA3Console.standArmsLoweredStartToServeBeginsThe")
                  : serveState === "COMPLETE" && snapshot.runnerMode === "MOTION"
                    ? t("hopeA3Console.readyReceivingStartToServeBeginsTheNext")
                  : t("hopeA3Console.lockedUntilWaitReadyToServe")
            }
            disabled={
              !runnerUsable ||
              snapshot.localRole !== "SERVER" ||
              serveState !== "WAIT_READY_TO_SERVE"
            }
            busy={busy.readyToServe === true}
            completed={snapshot.serving === true || serveState === "COMPLETE"}
            next={nextStep === "readyToServe"}
            onClick={() => {
              invokeConfirmed(
                "readyToServe",
                kernelMode ? msg("hopeA3Console.testCsvConfirm") : msg("hopeA3Console.serveConfirm"),
              );
            }}
          />
          <ActionButton
            label={t("hopeA3Console.openGripper")}
            detail={
              serveState === "CLEANUP_OPENING"
                ? t("hopeA3Console.requestedOpenPublishInProgress")
                : snapshot.serveCleanupRequired === true
                  ? t("hopeA3Console.retainedBallTelemetryDoesNotGateAnyAction")
                  : t("hopeA3Console.manualCommandGripper", {gripper: snapshot.gripperState ?? t("hopeA3Console.unavailable")})
            }
            disabled={
              !runnerUsable ||
              snapshot.localRole !== "SERVER" ||
              snapshot.runnerMode !== "PD_STAND"
            }
            busy={busy.openGripper === true}
            completed={snapshot.gripperState === "OPEN"}
            next={false}
            onClick={() => {
              invokeConfirmed(
                "openGripper",
                msg("hopeA3Console.openGripperConfirm"),
              );
            }}
          />
        </div>
        <div className="sequence-footer">
          <span>{lastAction}</span>
          <span>{mocapSafetyText(
            snapshot.runnerAlive === true && isFresh(snapshot, TOPICS.runnerAlive, now, 1_000) &&
            isFresh(snapshot, TOPICS.runnerBoot, now, 1_000) &&
            isFresh(snapshot, TOPICS.mocapSafety, now, 1_000) &&
            snapshot.mocapSafety?.boot === snapshot.runnerBoot
              ? snapshot.mocapSafety : undefined, t)}</span>
          <ActionButton
            label={t("hopeA3Console.runnerPassive")}
            detail={t("hopeA3Console.zeroGainsRobotLosesSupport")}
            disabled={!runnerFresh}
            busy={busy.passive === true}
            danger
            onClick={() => void invoke("passive")}
          />
        </div>
      </div>
      </fieldset>
    </div>
    </LanguageContext.Provider>
  );
}

export function initHopeA3Console(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HopeA3Console context={context} />);
  return () => {
    root.unmount();
  };
}
