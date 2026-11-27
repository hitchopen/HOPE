import { version as extensionVersion } from "../package.json";
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
import { DockTuningCard } from "./DockTuningCard";
import { FlightTimeControls } from "./FlightTimeControls";
import { HitSpeedScaleControls } from "./HitSpeedScaleControls";
import { LanguageContext, useI18n } from "./LanguageContext";
import { PlaneModeControls } from "./PlaneModeControls";
import { BallModelStatus, ballModelDisplay } from "./ballModelStatus";
import { BallPlayState, readLocalBallPlayState, currentBallPlayState, localIntentForAction, sendLocalBallIntent, readLocalControlKey, saveLocalControlKey } from "./ballPlayState";
import { BALL_Y_FILTER_SERVICE, BallYFilterState, ballYFilterRequest, decodeBallYFilter } from "./ballYFilter";
import { BallYFilterDraft, withBallYPreview } from "./ballYFilterPreview";
import { confirmedRobotHost, configForSave, configUnchangedReply, sameMocapBinding, ConfigRuntime, RuntimeConfigValues, decodeConfigRuntime, configRuntimeReady, savedConfigObserved } from "./configRuntime";
import { decodeDockState, dockPhaseName, dockRepositionReason, formatDockStatus } from "./dockStatus";
import { decodeDockTuning } from "./dockTuning";
import { FLIGHT_TIME_SERVICE, FlightTimeState, decodeFlightTime, flightTimeRequest } from "./flightTime";
import { HIT_SPEED_SCALE_SERVICE, HitSpeedScaleState, decodeHitSpeedScale, hitSpeedScaleRequest } from "./hitSpeedScale";
import { message as msg, LocalizedError, errorText, LocalText, Translator, Locale, readLocale, createTranslator } from "./i18n";
import { LifecycleButtonTone, lifecycleButtons } from "./lifecycleButtons";
import { mergeConfirmedConfig } from "./lifecycleConfig";
import { lifecycleResultView } from "./lifecycleResult";
import { MocapSafety, decodeMocapSafety, mocapSafetyText, pelvisReady } from "./mocapSafety";
import { MOCAP_LABELS, mocapSetupCommand } from "./mocapSetup";
import { PLANE_MODE_LABELS, PLANE_MODE_SERVICE, PlaneMode, PlaneModeState, decodePlaneMode, planeModeRequest } from "./planeMode";
import { PlayerPosition, decodePlayerPosition, selectedPlayerAvailable } from "./playerPositions";
import { MocapRecovery, StartCheckId, StartEvidence, mocapRecoveryText, startBlockReason, startChecks, mocapConnectionView } from "./mocapRecovery";
import { reconcileRobotTarget, robotConnectionUrl, confirmRobotConnection } from "./robotConnection";
import { KeyboardWalk } from "./keyboardWalk";
import { sessionFingerprint } from "./operatorHeartbeat";
import { bindRunnerShortcuts, RUNNER_SHORTCUT_KEYS, RunnerShortcut } from "./runnerShortcuts";
import { decodeTeleopState, teleopEntryReason, xboxInputFresh } from "./teleopStatus";
import { timeCalibrationControls } from "./timeCalibration";
import { walkStandPhase } from "./walkStandStatus";

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
  baseSummary: "/hope/base/summary",
  commandValid: "/hope/command/valid",
  commandCountdown: "/hope/command/hdu_wall_countdown_s",
  commandFlight: "/hope/command/flight_id",
  commandSummary: "/hope/command/summary",
  runnerAlive: "/hope/runner/alive",
  runnerBoot: "/hope/runner/boot_id",
  mocapSafety: "/hope/runner/mocap_safety_hdu_flat",
  runnerMode: "/hope/runner/mode",
  xboxPreview: "/hope/runner/xbox_preview",
  teleopState: "/hope/runner/teleop_state_hdu_flat",
  dockState: "/hope/runner/dock_state_hdu_flat",
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
  configRuntime: "/hope/lifecycle/config/status",
  worldConfirmed: "/hope/lifecycle/config/vrpn_world_frame_confirmed",
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
  rigidStand: "/hope/runner/enter_rigid_pd_stand",
  restoreArms: "/hope/runner/restore_stand_arms",
  walkStand: "/hope/runner/enter_walk_stand",
  dock: "/hope/runner/enter_dock",
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

type PolicyModel = "model137000" | "model54000" | "model400center3" | "model400" | "model14600" | "model11999" | "model12600" | "model17400" | "model12000" | "model13800" | "modelfusion" | "model13800astra" | "model14200" | "model14200fusion" | "model14200best" | "model15200body" | "model19000" | "model26500" | "model23700";
type FieldStatus = {
  mocap_recovery?: MocapRecovery;
  player_positions?: { P1: boolean; P2: boolean; players_x_m?: Record<string, number> };
  ball_model_status?: BallModelStatus;
  policy_model?: PolicyModel | null;
  policy_models?: Partial<Record<PolicyModel, { available: boolean; reason: string; receive_only?: boolean;
    serve_csv?: { name: string; sha256: string } | null }>>;
  policy_models_age_s?: number | null;
  policy_models_config_revision?: number | null;
  ball_model?: "berkeley" | "ppq002" | "ppq004" | null;
  ball_model_verification?: { model: string; planner: string; packetizer: string };
  mode: "OPTITRACK" | "KERNEL";
  calibration?: { name: string; sha256: string; profile: string; table_side: string } | null;
  calibration_sync: string;
  csv?: { name: string; sha256: string; frames: number; hz: number;
    segments?: { version: number; branch_frame: number; policy_ready_s: number | null; policy_blend_s: number;
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
  vrpn_world_frame_confirmed: { label: msg("configRuntime.worldConfirmed"), topic: TOPICS.worldConfirmed },
} as const;

type ConfigField = keyof typeof CONFIG_FIELDS;
const RUNNER_NETWORK_CONFIG_FIELDS: ConfigField[] = [
  "laptop_wifi_ip",
  "hdu_wifi_ip",
  "mdu_internal_ip",
];
type LifecycleConfigDraft = Record<ConfigField, string>;
type ParameterValue = {
  type: number; bool_value: boolean; integer_value: number; double_value: number; string_value: string;
  byte_array_value: number[]; bool_array_value: boolean[]; integer_array_value: number[];
  double_array_value: number[]; string_array_value: string[];
};

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
  vrpn_world_frame_confirmed: "false",
};

type XboxPreview = {
  connected: boolean;
  device: string;
  enabled: boolean;
  lt: boolean;
  keyboard_walk?: string;
  lt_value?: number;
  buttons?: string[];
  lb?: boolean;
  rb?: boolean;
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
  flightTime?: FlightTimeState;
  ballYFilter?: BallYFilterState;
  planeMode?: PlaneModeState;
  xboxPreview?: XboxPreview;
  teleopState?: number[];
  dockState?: number[];
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
  baseSummary?: string;
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
  configRuntime?: ConfigRuntime;
  configRuntimeSeen?: boolean;
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
  if (topic === TOPICS.configRuntime) {
    next.configRuntimeSeen = true;
    next.configRuntime = decodeConfigRuntime(scalar(event.message));
    if (next.configRuntime) {
      next.lifecycleConfig = { ...next.configRuntime.values };
      next.lifecycleConfigRevision = next.configRuntime.revision;
      next.lastReceived[TOPICS.hduWifiIp] = receivedAt;
    }
    next.lastReceived[topic] = receivedAt;
    return;
  }
  if (next.configRuntimeSeen && topic.startsWith("/hope/lifecycle/config/")) { return; }
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
      next.flightTime = decodeFlightTime(event.message);
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
    case TOPICS.baseSummary:
      next.baseSummary = stringValue(event.message);
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
    case TOPICS.dockState: {
      next.dockState = decodeDockState(scalar(event.message));
      break;
    }
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

function playPelvisReady(current: Snapshot, now: number): boolean {
  // Only a fresh receipt from this Runner can exempt a pelvis-independent mode.
  const received = current.lastReceived[TOPICS.mocapSafety];
  const safety = current.mocapSafety;
  if (received == undefined || now < received || now - received >= 1_000 ||
      safety == undefined || safety.boot !== current.runnerBoot) { return false; }
  // HELD is the Runner's bounded jitter allowance; it becomes false at expiry.
  return !safety.required || ((safety.fresh || safety.held === true) && pelvisReady({
    baseFresh: current.baseFresh, tfReady: current.tfReady,
    baseReceived: current.lastReceived[TOPICS.baseFresh],
    tfReceived: current.lastReceived[TOPICS.tfReady],
  }, now));
}

function confirmationBlocked(pending: Omit<Confirmation, "prompt">, current: Snapshot, now: number): LocalText | undefined {
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
      (key === "openGripper" ? !isStanding(current.runnerMode) :
        current.serveCapability !== "AVAILABLE" ||
        (key === "prepareServe"
          ? !["PD_STAND", "WALK_STAND", "MOTION"].includes(current.runnerMode ?? "") ||
            !["IDLE", "COMPLETE", "ABORTED"].includes(current.serveState ?? "")
          : current.serveState !== "WAIT_READY_TO_SERVE"))) {
    return msg("hopeA3Console.confirmationStateChanged");
  }
  if (key === "openGripper") { return; }
  if (!playPelvisReady(current, now)) { return msg("mocapSafety.readyNeedsPelvis"); }
  const safety = current.mocapSafety;
  if (!isFresh(current, TOPICS.mocapSafety, now, 1_000) || safety?.boot !== current.runnerBoot) {
    return msg("hopeA3Console.confirmationStateUnavailable");
  }
  return undefined;
}

// 2.1: Health merged into Play as the collapsible System health card (operator request 2026-11-17).
type ConsoleTab = "setup" | "play";
// How long a robot-reported "configuration not applied" stays displayed and gating START (see configApplied).
const CONFIG_APPLIED_HOLD_MS = 15_000;
/** Laptop mocap receiver process kinds reported by the HDU configuration audit. */
const MOCAP_RECEIVER_KINDS = new Set(["motive", "motive_relay", "vrpn", "vrpn_relay", "base"]);
type Tone = "ok" | "bad" | "warn" | "idle" | "busy" | "kern" | "acc";

const toneOf = ({ value }: { value?: boolean }): Tone => value == undefined ? "idle" : value ? "ok" : "bad";

/** Header health chip: true → ok, false → bad, undefined → idle. Details live in the tooltip. */
function HealthChip({ label, value, detail, onOpen, warn = false }: {
  label: string; value?: boolean; detail?: string; onOpen: () => void; warn?: boolean;
}): ReactElement {
  const tone: Tone = warn ? "warn" : toneOf({ value });
  return (
    <button type="button" className={`c-hchip c-${tone}`} title={detail} onClick={onOpen}>
      <span className={`c-dot c-${tone}`} />
      <span className="c-hchip-label">{label}</span>
    </button>
  );
}

function ProcessPill({ name, topic, value, optional = false }: {
  name: string; topic: string; value?: boolean; optional?: boolean;
}): ReactElement {
  const t = useI18n();
  // optional: a stopped process is expected (e.g. the MDU Runner while the lifecycle is not RUNNING).
  const tone: Tone = optional && value === false ? "idle" : toneOf({ value });
  return (
    <span className={`c-pill c-${tone}`} title={topic}>
      <span className={`c-dot c-${tone}`} />
      {name}
      <small>{value == undefined ? t("hopeA3Console.noData") : value ? t("console2.processRunning") : t("console2.processStopped")}</small>
    </span>
  );
}

const SHORTCUT_HINT: Record<RunnerShortcut, string> = { stand: "I", ready: "R", serve: "S", dock: "D", restoreArms: "A", passive: "← →" };

type NodeId = "passive" | "stand" | "receive" | "raise" | "serve" | "reposition";
// State-graph geometry in SVG viewBox units; nodes are positioned in % of the same box.
const GRAPH_W = 260;
const NODE_HW = 31;
const NODE_HH = 15.6;
const NODE_POS: Record<NodeId, { x: number; y: number }> = {
  passive: { x: 33, y: 28 }, stand: { x: 104, y: 28 }, receive: { x: 226, y: 28 },
  raise: { x: 150, y: 78 }, serve: { x: 226, y: 78 },
  // Normal play only: Kernel has no mocap base pose and uses this corner for auto -> stand.
  reposition: { x: 50, y: 78 },
};
const graphHeight = ({ kernelMode }: { kernelMode: boolean }): number => kernelMode ? 108 : 100;

type GraphNodeProps = {
  id: NodeId;
  label: string;
  sub: string;
  kernelMode: boolean;
  current: boolean;
  disabled: boolean;
  busy: boolean;
  reason?: string;
  tone?: "danger" | "serve";
  shortcut?: RunnerShortcut;
  onClick: () => void;
};

/** One clickable Runner state. `disabled`/`busy` are the unchanged action gates. */
function GraphNode(props: GraphNodeProps): ReactElement {
  const t = useI18n();
  const blocked = props.disabled || props.busy;
  const position = NODE_POS[props.id];
  const height = graphHeight({ kernelMode: props.kernelMode });
  const classes = [
    "c-node",
    props.current ? "c-node-cur" : blocked ? "c-node-off" : "c-node-avail",
    props.tone != undefined ? `c-node-${props.tone}` : "",
  ].filter(Boolean).join(" ");
  return (
    <button
      type="button"
      className={classes}
      style={{ left: `${(position.x / GRAPH_W) * 100}%`, top: `${(position.y / height) * 100}%`,
        height: `${((NODE_HH * 2) / height) * 100}%` }}
      disabled={blocked}
      title={props.disabled ? props.reason : undefined}
      data-runner-shortcut={props.shortcut}
      aria-keyshortcuts={props.shortcut == undefined ? undefined : RUNNER_SHORTCUT_KEYS[props.shortcut]}
      onClick={props.onClick}
    >
      {props.current && <span className="c-node-now">{t("console2.now")}</span>}
      <b>{props.busy ? t("hopeA3Console.working") : props.label}</b>
      <small>{props.disabled && !props.current && props.reason != undefined ? props.reason : props.sub}</small>
      {/* Key labels stay visible (dimmed while the action is blocked) so the mapping is always readable. */}
      {props.shortcut != undefined && <span className={`c-kbd ${blocked ? "c-kbd-off" : ""}`}>{SHORTCUT_HINT[props.shortcut]}</span>}
    </button>
  );
}

type EdgeState = "" | "live" | "livew";

function GraphEdges({ kernelMode, edge, autoLive, receiveLabel }: {
  kernelMode: boolean;
  edge: (from: NodeId, to: NodeId) => EdgeState;
  autoLive: boolean;
  receiveLabel: string;
}): ReactElement {
  const t = useI18n();
  const h = graphHeight({ kernelMode });
  const p = NODE_POS;
  const sx = kernelMode ? p.stand.x + 12 : p.stand.x;
  const edges: { key: string; state: EdgeState; d: string; label?: string; lx?: number; ly?: number; anchor?: "middle" | "end" }[] = [
    { key: "passive-stand", state: edge("passive", "stand"), d: `M ${p.passive.x + NODE_HW} ${p.passive.y} L ${p.stand.x - NODE_HW - 1} ${p.stand.y}` },
    { key: "stand-receive", state: edge("stand", "receive"), d: `M ${p.stand.x + NODE_HW} ${p.stand.y} L ${p.receive.x - NODE_HW - 1} ${p.receive.y}`,
      label: receiveLabel, lx: (p.stand.x + p.receive.x) / 2, ly: p.stand.y - 5, anchor: "middle" },
    { key: "stand-raise", state: edge("stand", "raise"), d: `M ${sx} ${p.stand.y + NODE_HH} L ${sx} ${p.raise.y} L ${p.raise.x - NODE_HW - 1} ${p.raise.y}` },
    ...(kernelMode ? [] : [{ key: "stand-reposition", state: edge("stand", "reposition"),
      d: `M ${p.stand.x - 14} ${p.stand.y + NODE_HH} L ${p.stand.x - 14} ${p.reposition.y} L ${p.reposition.x + NODE_HW + 1} ${p.reposition.y}` }]),
    { key: "raise-serve", state: edge("raise", "serve"), d: `M ${p.raise.x + NODE_HW} ${p.raise.y} L ${p.serve.x - NODE_HW - 1} ${p.serve.y}` },
    kernelMode
      ? { key: "auto", state: autoLive ? "livew" : "", d: `M ${p.serve.x} ${p.serve.y + NODE_HH} L ${p.serve.x} ${h - 6} L ${p.stand.x - 14} ${h - 6} L ${p.stand.x - 14} ${p.stand.y + NODE_HH + 1}`,
        label: t("console2.edgeAutoStand"), lx: p.stand.x - 18, ly: 76, anchor: "end" }
      : { key: "auto", state: autoLive ? "livew" : "", d: `M ${p.serve.x} ${p.serve.y - NODE_HH} L ${p.serve.x} ${p.receive.y + NODE_HH + 1}`,
        label: t("console2.edgeAuto"), lx: p.serve.x - 4, ly: 55, anchor: "end" },
  ];
  return (
    <svg viewBox={`0 0 ${GRAPH_W} ${h}`} aria-hidden="true">
      <defs>
        {(["", "live", "livew"] as const).map((state) => (
          <marker key={state} id={`hope-c2-arrow-${state || "idle"}`} viewBox="0 0 6 6" refX="5" refY="3"
            markerWidth="4.5" markerHeight="4.5" orient="auto">
            <path className={`c-arrow ${state}`} d="M0,0 L6,3 L0,6 z" />
          </marker>
        ))}
      </defs>
      {edges.map((e) => (
        <g key={e.key}>
          <path className={`c-edge ${e.state}`} d={e.d} markerEnd={`url(#hope-c2-arrow-${e.state || "idle"})`} />
          {e.label != undefined && <text className={`c-elbl ${e.state}`} x={e.lx} y={e.ly} textAnchor={e.anchor}>{e.label}</text>}
        </g>
      ))}
    </svg>
  );
}

function GripperAction({ label, detail, disabled, busy, onClick }: {
  label: string; detail: string; disabled: boolean; busy: boolean; onClick: () => void;
}): ReactElement {
  const t = useI18n();
  return (
    <button type="button" className="c-btn c-sm" disabled={disabled || busy} title={detail} onClick={onClick}>
      {busy ? t("hopeA3Console.working") : label}
    </button>
  );
}

// Serve025 event frames enforced by the Runner validator
// (agi/.../pp_serve025_fullbody_timeline.hpp: kServe025ReleaseFrame / kServe025ContactFrame).
const SERVE_RELEASE_FRAME = 48;
const SERVE_CONTACT_FRAME = 78;

/** Display-only CSV timeline from the validated segment metadata in /hope/field/status. */
function CsvTimeline({ csv, kernelMode }: { csv: FieldStatus["csv"]; kernelMode: boolean }): ReactElement | null {
  const t = useI18n();
  const segments = csv?.segments;
  if (csv == undefined || segments == undefined || !(csv.hz > 0)) { return null; }
  const branch = segments.branch_frame / csv.hz;
  const tail = (kernelMode ? segments.clips.SERVE_TO_STAND : segments.clips.SERVE_TO_POLICY)?.duration_s ?? 0;
  const span = branch + tail;
  if (!(span > 0)) { return null; }
  const pct = (seconds: number) => `${Math.max(0, Math.min(100, (seconds / span) * 100)).toFixed(2)}%`;
  const release = SERVE_RELEASE_FRAME / csv.hz;
  const contact = SERVE_CONTACT_FRAME / csv.hz;
  const ready = !kernelMode && segments.policy_ready_s != undefined ? branch + segments.policy_ready_s : undefined;
  return (
    <div className="c-tl" title={t("hopeA3Console.csvTimeline", { branch: branch.toFixed(2),
      handoff: segments.clips.SERVE_TO_POLICY?.duration_s.toFixed(2) })}>
      <div className="c-tlbar">
        <div className="c-tlseg c-tl-s" style={{ width: pct(branch) }}>{t("console2.csvSegmentServe")}</div>
        <div className={`c-tlseg ${kernelMode ? "c-tl-b" : "c-tl-c"}`} style={{ width: pct(tail) }}>
          {kernelMode ? t("console2.csvSegmentReturn") : t("console2.csvSegmentHandoff")}</div>
        <div className="c-tlev" style={{ left: pct(release) }} title={t("console2.csvRelease")} />
        <div className="c-tlev" style={{ left: pct(contact) }} title={t("console2.csvContact")} />
        {ready != undefined && <div className="c-tlev c-tlev-ready" style={{ left: pct(ready) }} title={t("console2.csvPolicyReady")} />}
      </div>
      <div className="c-tlaxis">
        <span>{t("console2.seconds", { value: "0" })}</span>
        <span style={{ left: pct(release) }}>{t("console2.csvReleaseAt", { value: release.toFixed(2) })}</span>
        <span style={{ left: pct(contact) }} className="c-tlaxis-low">{t("console2.csvContactAt", { value: contact.toFixed(2) })}</span>
        <span style={{ left: "100%" }}>{t("console2.seconds", { value: span.toFixed(2) })}</span>
      </div>
    </div>
  );
}

type PolicyRow = { model: PolicyModel; name: string; label?: LocalText; note?: LocalText; serve: boolean;
  /** Kept in the catalog but not shown in the dropdown. */
  hidden?: boolean };
const POLICY_ROWS: PolicyRow[] = [
  { model: "model137000", name: "FT48 137000", label: msg("hopeA3Console.model137000"), serve: false },
  { model: "model54000", name: "54000", label: msg("hopeA3Console.model54000"), serve: false },
  { model: "model23700", name: "23700 Mobile", label: msg("hopeA3Console.model23700"), serve: false },
  { model: "model400center3", name: "400 Center3", label: msg("hopeA3Console.model400center3"), serve: true },
  { model: "model400", name: "400", label: msg("hopeA3Console.model400ServeReceive"), serve: true },
  { model: "model14200best", name: "14200 Best", label: msg("hopeA3Console.model14200bestServeReceive"), serve: true },
  { model: "model14200", name: "14200", label: msg("hopeA3Console.model14200ServeReceive"), serve: true },
  { model: "model13800astra", name: "13800 Astra", label: msg("hopeA3Console.astraV23ServeReceive"), serve: true },
  { model: "model13800", name: "13800", label: msg("hopeA3Console.13800V2313800ServeReceive"), serve: true },
  { model: "model12000", name: "12000", label: msg("hopeA3Console.12000V2312000ServeReceive"), serve: true },
  { model: "model17400", name: "17400", label: msg("hopeA3Console.17400V2317400ServeReceive"), serve: true },
  { model: "model12600", name: "12600", label: msg("hopeA3Console.12600V2312600ServeReceive"), serve: true },
  { model: "model14600", name: "14600", label: msg("hopeA3Console.14600ServeReceive"), serve: true },
];
const VISIBLE_POLICY_ROWS: PolicyRow[] = POLICY_ROWS.filter((row) => row.hidden !== true);
const OFFERED_MODELS: ReadonlySet<PolicyModel> = new Set<PolicyModel>(VISIBLE_POLICY_ROWS.map((row) => row.model));
const KERNEL_MODELS: ReadonlySet<PolicyModel> = new Set<PolicyModel>(["model14600", "model14200best"]);
// Display names for whatever the robot reports, including models no longer offered in the list.
const POLICY_NAMES: Partial<Record<PolicyModel, string>> = {
  model14600: "14600", model11999: "11999", model12600: "12600", model17400: "17400", model12000: "12000",
  model13800: "13800", modelfusion: "Fusion", model13800astra: "13800 Astra", model14200: "14200",
  model14200fusion: "14200 Fusion", model15200body: "15200 BODY", model14200best: "14200 Best", model19000: "19000 GRU", model26500: "26500",
  model137000: "FT48 137000", model54000: "54000", model23700: "23700 Mobile", model400center3: "400 Center3", model400: "400",
};
// Hashes from model14200best/manifest.json; the matched serve CSV identifies which bundle is installed.
const BEST_14200_VARIANTS = [
  { variant: "best-v1", policy: "5344f9a220b953ca4cf4c5c57b536be6f04b5a88d13c0ac19b3f3c00c70adba2",
    csv: "14c75e8e2f17234e26d2498c7b75da9b2f2b8aad48f8e31659901c56fa856630" },
  { variant: "earlier-staged", policy: "95e63b544763ee52e9d6dfa45e0584e4205479a09eba2ba31cf2572e9e9be7e8",
    csv: "fa37dd8ffc89390163d30cda29ebea47c4e56db4317ceafcc4d33fcd971ff4c9" },
] as const;
// The installed bundle's own serve CSV (policy catalog) is authoritative; the active serve CSV is a fallback.
function best14200Variant(catalogCsvSha: string | undefined, csv: FieldStatus["csv"] | undefined): string | undefined {
  return BEST_14200_VARIANTS.find((v) => (catalogCsvSha != undefined && v.csv === catalogCsvSha) ||
    (csv != undefined && (v.csv === csv.sha256 || v.policy === csv.segments?.policy_sha256)))?.variant;
}

function formatElapsed(ms: number): string {
  const total = Math.max(0, Math.floor(ms / 1000));
  const hours = Math.floor(total / 3600);
  const pad = (value: number) => String(value).padStart(2, "0");
  return hours > 0
    ? `${hours}:${pad(Math.floor((total % 3600) / 60))}:${pad(total % 60)}`
    : `${pad(Math.floor(total / 60))}:${pad(total % 60)}`;
}

/** Session ids carry their UTC start time (e.g. model21800_20261113T090712Z). */
function sessionStartMs(session: string | undefined): number | undefined {
  const match = /(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})Z$/.exec(session ?? "");
  if (match == undefined) { return undefined; }
  const [, y, mo, d, h, mi, s] = match.map(Number);
  return Date.UTC(y ?? 0, (mo ?? 1) - 1, d ?? 1, h ?? 0, mi ?? 0, s ?? 0);
}

/**
 * The 1.9 action-button gates, moved verbatim into one function so the render and
 * the post-role-switch re-check evaluate exactly the same conditions.
 */
// The operator Stand is WALK_STAND (HumanLike); PD_STAND is the stiff Init and the
// Runner's internal handoff before Ready/Serve. Both count as standing.
const STANDING_MODES = ["PD_STAND", "WALK_STAND"];
const isStanding = (mode: string | undefined): boolean => STANDING_MODES.includes(mode ?? "");

function actionGates(s: Snapshot, now: number) {
  const runnerFresh = isFresh(s, TOPICS.runnerAlive, now, 1_500) && s.runnerAlive === true;
  const runnerUsable = runnerFresh && s.runnerFault !== true;
  const runnerStanding = runnerFresh && isStanding(s.runnerMode);
  const serveState = s.serveState ?? "UNAVAILABLE";
  const serveAvailable = s.serveCapability === "AVAILABLE";
  // Ready, Start to Serve and Serve fail closed without a fresh pelvis from this Runner.
  const pelvisOk = playPelvisReady(s, now);
  // Ready / Start to Serve without their role term; the role term is applied below.
  const readyExceptRole = !runnerUsable || !runnerStanding || !pelvisOk;
  const prepareServeExceptRole = !runnerUsable || !serveAvailable || !pelvisOk ||
    !["PD_STAND", "WALK_STAND", "MOTION"].includes(s.runnerMode ?? "") ||
    !["IDLE", "COMPLETE", "ABORTED"].includes(serveState);
  return {
    runnerFresh, runnerUsable, runnerStanding, serveState, serveAvailable, pelvisOk,
    readyExceptRole, prepareServeExceptRole,
    standBlocked: !runnerUsable,
    readyBlocked: readyExceptRole || s.localRole !== "RECEIVER",
    prepareServeBlocked: prepareServeExceptRole || s.localRole !== "SERVER",
    readyToServeBlocked: !runnerUsable || !pelvisOk || s.localRole !== "SERVER" || serveState !== "WAIT_READY_TO_SERVE",
    openGripperBlocked: !runnerUsable || s.localRole !== "SERVER" || !isStanding(s.runnerMode),
    passiveBlocked: !runnerFresh,
    teleopStandBlocked: !runnerFresh || s.runnerMode !== "TELEOP",
  };
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

function HopeA3Console({ context }: { context: PanelExtensionContext }): ReactElement {
  const [locale, setLocale] = useState<Locale>(readLocale(context.initialState));
  const t = useMemo(() => createTranslator(locale), [locale]);
  const latest = useRef<Snapshot>(INITIAL_SNAPSHOT);
  const [snapshot, setSnapshot] = useState<Snapshot>(INITIAL_SNAPSHOT);
  const [localBall, setLocalBall] = useState<{ state: BallPlayState; receivedAt: number }>();
  const [localBallKey, setLocalBallKey] = useState(readLocalControlKey());
  const [localBallPairFailed, setLocalBallPairFailed] = useState(false);
  const [localBallIntentFailed, setLocalBallIntentFailed] = useState(false);
  const localIntentQueue = useRef<Promise<void>>(Promise.resolve());
  const applyLocalButton = useCallback((key: ServiceKey) => {
    const action = localIntentForAction(key);
    if (action == undefined) return;
    localIntentQueue.current = localIntentQueue.current.then(async () => {
      const controller = new AbortController();
      const timeout = window.setTimeout(() => controller.abort(), 500);
      try {
        await sendLocalBallIntent(window.fetch.bind(window), action, controller.signal, localBallKey);
        setLocalBallIntentFailed(false);
      } catch { setLocalBallIntentFailed(true); }
      finally { window.clearTimeout(timeout); }
    });
  }, [localBallKey]);
  useEffect(() => {
    let closed = false;
    let pending: AbortController | undefined;
    const poll = async () => {
      if (pending != undefined) return;
      const controller = new AbortController(); pending = controller;
      const timeout = window.setTimeout(() => controller.abort(), 300);
      try {
        const state = await readLocalBallPlayState(window.fetch.bind(window), controller.signal, localBallKey);
        if (!closed) setLocalBall(state == undefined ? undefined : { state, receivedAt: Date.now() });
      } catch { if (!closed) setLocalBall(undefined); }
      finally { window.clearTimeout(timeout); pending = undefined; }
    };
    void poll();
    const timer = window.setInterval(() => { void poll(); }, 100);
    return () => { closed = true; window.clearInterval(timer); pending?.abort(); };
  }, [localBallKey]);

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [, refreshClock] = useState(0);
  // Samples can arrive between timer ticks. Compare them with the time of THIS
  // render, otherwise a new packet appears up to 250 ms in the future.
  const now = Date.now();
  const [busy, setBusy] = useState<Partial<Record<ServiceKey, boolean>>>({});
  const inFlightServices = useRef(new Set<ServiceKey>());
  const [notice, setNotice] = useState<LocalText>(msg("hopeA3Console.waitingForAuthoritativeRunnerState"));
  const [confirmation, setConfirmation] = useState<Confirmation | undefined>();
  const confirmationRef = useRef<Confirmation>();
  const confirmationCancel = useRef<HTMLButtonElement>(null);
  useEffect(() => {
    if (confirmation != undefined) { confirmationCancel.current?.focus(); }
  }, [confirmation]);
  const [configDraft, setConfigDraft] = useState<LifecycleConfigDraft>(EMPTY_CONFIG);
  const [configTouched, setConfigTouched] = useState(false);
  const [awaitingConfig, setAwaitingConfig] = useState<{ revision: number; values: RuntimeConfigValues }>();
  const savedPanel = useRef((context.initialState ?? {}) as Record<string, unknown>);
  const [robotTarget, setRobotTarget] = useState(typeof savedPanel.current.robotTarget === "string" ? savedPanel.current.robotTarget : "");
  const [targetEdited, setTargetEdited] = useState(false);
  // A layout can outlive its connection. Never restore an in-flight switch.
  const [pendingTarget, setPendingTarget] = useState<string>("");
  // Read by the asynchronous role-switch follow-up, which outlives its render.
  const pendingTargetRef = useRef(pendingTarget);
  pendingTargetRef.current = pendingTarget;
  const [tab, setTab] = useState<ConsoleTab>();
  const [diagOpen, setDiagOpen] = useState(false);
  const [policyOpen, setPolicyOpen] = useState(false);
  const [policyFilter, setPolicyFilter] = useState("");
  const [switchOpen, setSwitchOpen] = useState(false);
  const [noticeLog, setNoticeLog] = useState<{ id: number; at: number; text: LocalText }[]>([]);
  // Any robot command other than a role change supersedes a pending role-switch follow-up.
  const actionEpoch = useRef(0);
  const dockReturn = useRef<AbortController>();
  const [returnPending, setReturnPending] = useState(false);
  const roleSwitchInFlight = useRef(false);
  const [roleSwitchFor, setRoleSwitchFor] = useState<"ready" | "prepareServe">();
  const snapshotWaiters = useRef(new Set<() => void>());
  const rootRef = useRef<HTMLDivElement>(null);
  // The robot address is the one bound to the live robot identity (machine_id/boot_id), never a typed IP.
  const configHost = confirmedRobotHost(snapshot.configRuntime);
  const configHostFresh = snapshot.configRuntimeSeen
    ? snapshot.configRuntime != undefined && isFresh(snapshot, TOPICS.configRuntime, now, 1_500)
    : false;
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
      const current = latest.current;
      confirmRobotConnection(robotTarget.trim(), confirmedRobotHost(current.configRuntime),
        current.configRuntime != undefined && isFresh(current, TOPICS.configRuntime, Date.now(), 1_500),
        (ip) => {
          setPendingTarget("");
          setTargetEdited(false);
          savedPanel.current = { ...savedPanel.current, robotTarget: ip };
          context.saveState(savedPanel.current);
          setNotice(msg("connection.confirmedInPlace", { ip }));
        }, (ip) => {
          dockReturn.current?.abort();
          setPendingTarget(ip);
          setNotice(msg("hopeA3Console.opening8766WaitingForTheNewRobotControls", {ip}));
        }, (url) => { window.open(url, "_blank", "noopener,noreferrer"); });
    } catch (error) { setNotice(errorText(error)); }
  };
  let validRobotTarget = false;
  try { robotConnectionUrl(robotTarget.trim()); validRobotTarget = true; } catch { /* incomplete input */ }

  const [fieldBusy, setFieldBusy] = useState(false);
  const [mocapRequesting, setMocapRequesting] = useState(false);
  const [recordBusy, setRecordBusy] = useState(false);
  const csvInput = useRef<HTMLInputElement>(null);
  const previewBallYFilter = useCallback((draft: BallYFilterDraft, options: { kernelMode: boolean; edited: boolean }) => {
    if ((context.initialState as { liveStatusOwnsPreview?: boolean } | undefined)?.liveStatusOwnsPreview) {
      context.setVariable("hopeLocalPreview", { ...draft, kernelMode: options.kernelMode });
    } else {
      context.layout.addPanel({ position: "sibling", type: "3D", updateIfExists: true,
        getState: (existing) => withBallYPreview(existing, draft, options, t) });
    }
    savedPanel.current = { ...savedPanel.current,
      ballYFilterDraft: draft, ballYFilterDraftEdited: options.edited,
      ballYFilterRobot: latest.current.lifecycleConfig.hdu_wifi_ip };
    context.saveState(savedPanel.current);
  }, [context, t]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      const receivedAt = Date.now();
      const hostMessage = renderState.currentFrame?.find((event) => event.topic === TOPICS.hduWifiIp);
      const incomingHost = hostMessage ? stringValue(hostMessage.message) : undefined;
      const identityMessage = renderState.currentFrame?.find((event) => event.topic === TOPICS.configRuntime);
      const incomingIdentity = identityMessage ? decodeConfigRuntime(scalar(identityMessage.message))?.robotIdentity : undefined;
      const priorIdentity = latest.current.configRuntime?.robotIdentity;
      const identityChanged = incomingIdentity != undefined && priorIdentity != undefined &&
        (incomingIdentity.machine_id !== priorIdentity.machine_id || incomingIdentity.boot_id !== priorIdentity.boot_id);
      if (renderState.didSeek || identityChanged || (incomingHost !== undefined && latest.current.lifecycleConfig.hdu_wifi_ip !== undefined &&
          incomingHost !== latest.current.lifecycleConfig.hdu_wifi_ip)) {
        confirmationRef.current = undefined;
        setConfirmation(undefined);
        dockReturn.current?.abort();
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
      latest.current = next;
      setSnapshot(next);
      setRenderDone(() => done);
      for (const waiter of Array.from(snapshotWaiters.current)) { waiter(); }
    };
    context.watch("didSeek");
    context.watch("topics");
    context.watch("currentFrame");
    context.subscribe(Object.values(TOPICS).map((topic) => ({ topic })));
    return () => {
      dockReturn.current?.abort();
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
    if (awaitingConfig && savedConfigObserved(snapshot.configRuntime, awaitingConfig)) {
      // A delayed save acknowledgement must not discard edits made meanwhile.
      setConfigTouched(!Object.keys(awaitingConfig.values).every((key) =>
        configDraft[key as ConfigField] === awaitingConfig.values[key as ConfigField]));
      setAwaitingConfig(undefined);
    }
  }, [awaitingConfig, snapshot.configRuntime, configDraft]);

  useEffect(() => {
    setConfigDraft((current) => mergeConfirmedConfig(current, snapshot.lifecycleConfig, configTouched));
  }, [configTouched, snapshot.lifecycleConfig]);

  // Resolves true only when the service explicitly accepted the request.
  const invoke = useCallback(
    async (key: ServiceKey, cycle?: AbortController): Promise<boolean> => {
      if (cycle != undefined && (cycle.signal.aborted || dockReturn.current !== cycle)) { return false; }
      if (cycle == undefined && key !== "restoreArms") { dockReturn.current?.abort(); }
      if (busy[key] === true || inFlightServices.current.has(key)) { return false; }
      if (["ready", "prepareServe", "readyToServe"].includes(key) && !playPelvisReady(latest.current, Date.now())) {
        setNotice(msg("mocapSafety.readyNeedsPelvis"));
        return false;
      }
      if (key !== "setServer" && key !== "setReceiver" && key !== "restoreArms") { actionEpoch.current += 1; }
      // Local button intent and the existing robot request are independent.
      // Neither rejection nor acknowledgement from A3 changes the classifier.
      applyLocalButton(key);
      if (context.callService == undefined) {
        setNotice(msg("hopeA3Console.currentDataSourceDoesNotExposeServiceCalls"));
        return false;
      }
      inFlightServices.current.add(key);
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
        return response.success;
      } catch (error) {
        setNotice(msg("hopeA3Console.failed", {error: errorText(error)}));
        return false;
      } finally {
        inFlightServices.current.delete(key);
        setBusy((current) => ({ ...current, [key]: false }));
      }
    },
    [busy, context, applyLocalButton],
  );

  const fieldCommand = useCallback(async (payload: Record<string, unknown>): Promise<boolean> => {
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
      return true;
    } catch (error) {
      setNotice(msg("hopeA3Console.rejected", {error: errorText(error)}));
      return false;
    } finally {
      setPending(false);
    }
  }, [context]);

  const applyFlightTime = useCallback(async (scale: number) => {
    if (context.callService == undefined) { throw new LocalizedError(msg("hopeA3Console.connectToControlPort8766")); }
    const requestedAt = Date.now();
    const raw = await Promise.race([
      context.callService(FLIGHT_TIME_SERVICE, flightTimeRequest(scale)), timeoutAfter(3_000),
    ]) as { result?: { successful?: boolean; reason?: string } };
    if (raw.result?.successful !== true) { throw new LocalizedError(raw.result?.reason ?? msg("hopeA3Console.plannerDidNotConfirmTheChange")); }
    const deadline = Date.now() + 2_500;
    while (Date.now() < deadline) {
      const current = latest.current;
      if ((current.lastReceived[TOPICS.plannerDiagnostics] ?? 0) >= requestedAt &&
          isFresh(current, TOPICS.plannerDiagnostics, Date.now(), 2_500) && current.flightTime?.seconds === scale &&
          current.flightTime.forehand === scale && current.flightTime.backhand === scale) {
        setNotice(msg("flightTime.confirmed", {scale}));
        return;
      }
      await new Promise((resolve) => window.setTimeout(resolve, 100));
    }
    throw new LocalizedError(msg("flightTime.timeout"));
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

  const invokeServe = (key: "prepareServe" | "readyToServe") => {
    if (pendingTarget) { return; }
    // Serve is immediate; keep the existing state/identity checks at the click.
    const blocked = confirmationBlocked({ key, host: snapshot.lifecycleConfig.hdu_wifi_ip,
      session: snapshot.lifecycleSession, boot: snapshot.runnerBoot }, latest.current, Date.now());
    if (blocked != undefined) { setNotice(blocked); return; }
    void invoke(key);
  };

  /** Resolves true once a frame received after `since` reports `role`; false after 2 s. */
  const waitForRole = async (role: string, since: number): Promise<boolean> => await new Promise((resolve) => {
    const reached = () => latest.current.localRole === role &&
      (latest.current.lastReceived[TOPICS.localRole] ?? 0) >= since;
    const waiters = snapshotWaiters.current;
    const state = { timer: 0 };
    const onSnapshot = (): void => {
      if (!reached()) { return; }
      waiters.delete(onSnapshot);
      window.clearTimeout(state.timer);
      resolve(true);
    };
    state.timer = window.setTimeout(() => { waiters.delete(onSnapshot); resolve(false); }, 2_000);
    waiters.add(onSnapshot);
    onSnapshot();
  });

  // 2.0 behavior change: Receive / Raise first switch our role when the Runner
  // allows it, wait for the new role to be reported, then re-check the unchanged
  // action gate. Nothing is sent if the switch fails, times out or is superseded.
  const switchRoleThen = async (role: "RECEIVER" | "SERVER", target: "ready" | "prepareServe",
    send: () => void): Promise<void> => {
    if (pendingTargetRef.current) { return; }
    if (roleSwitchInFlight.current) { setNotice(msg("console2.roleSwitchBusy")); return; }
    const start = latest.current;
    const blocked = (s: Snapshot): boolean => {
      const gates = actionGates(s, Date.now());
      return target === "ready" ? gates.readyBlocked : gates.prepareServeBlocked;
    };
    if (start.localRole === role) {
      // The role changed after the last render: no switch needed, same gate as a direct click.
      if (!blocked(start)) { send(); }
      return;
    }
    if (start.roleChangeAllowed !== true) { setNotice(msg("console2.roleChangeNotAllowed", { role })); return; }
    roleSwitchInFlight.current = true;
    setRoleSwitchFor(target);
    const epoch = actionEpoch.current;
    const identity = { host: start.lifecycleConfig.hdu_wifi_ip, session: start.lifecycleSession, boot: start.runnerBoot };
    const since = Date.now();
    try {
      // invoke already reports a rejection or failure with the Runner's reason.
      if (!(await invoke(role === "RECEIVER" ? "setReceiver" : "setServer"))) { return; }
      if (!(await waitForRole(role, since))) { setNotice(msg("console2.roleChangeTimeout", { role })); return; }
      const current = latest.current;
      if (actionEpoch.current !== epoch || pendingTargetRef.current || current.estopLatched === true ||
          current.lifecycleConfig.hdu_wifi_ip !== identity.host || current.lifecycleSession !== identity.session ||
          current.runnerBoot !== identity.boot) {
        setNotice(msg("console2.roleChangeSuperseded", { role }));
        return;
      }
      if (blocked(current)) {
        setNotice(msg("console2.roleChangedActionBlocked", { role }));
        return;
      }
      send();
    } finally {
      roleSwitchInFlight.current = false;
      setRoleSwitchFor(undefined);
    }
  };

  const submitLifecycleConfig = useCallback(async (requestedValues: RuntimeConfigValues): Promise<boolean> => {
    const key: ServiceKey = "applyLifecycleConfig";
    if (context.callService == undefined || busy[key] === true) {
      setNotice(msg("hopeA3Console.currentDataSourceDoesNotExposeLifecycleConfiguration"));
      return false;
    }
    const requestedRevision = latest.current.lifecycleConfigRevision ?? 0;
    setConfigDraft(requestedValues);
    setBusy((current) => ({ ...current, [key]: true }));
    setNotice(msg("hopeA3Console.validatingAndConfirmingLifecycleConfiguration"));
    const parameters: { name: string; value: ParameterValue }[] = (Object.keys(CONFIG_FIELDS) as ConfigField[])
      .filter((name) => name !== "operator_profile" || snapshot.lifecycleConfig.operator_profile != undefined)
      .map((name) => ({
      name,
      value: {
        type: 4,
        bool_value: false,
        integer_value: 0,
        double_value: 0,
        string_value: requestedValues[name],
        byte_array_value: [],
        bool_array_value: [],
        integer_array_value: [],
        double_array_value: [],
        string_array_value: [],
      },
    }));
    const identity = latest.current.configRuntime?.robotIdentity;
    const parameterTemplate = parameters[0];
    if (identity != undefined && parameterTemplate != undefined && isFresh(latest.current, TOPICS.configRuntime, Date.now(), 1_500)) {
      parameters.push({ name: "connection_identity", value: { ...parameterTemplate.value,
        string_value: JSON.stringify({ machine_id: identity.machine_id, boot_id: identity.boot_id,
          revision: requestedRevision }) } });
    }
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
        return false;
      } else if (configUnchangedReply(results)) {
        // Same values as saved: the robot kept the revision and only re-checked (and, if needed,
        // restarted) the runtime. There is no new revision to wait for.
        setConfigTouched(false);
        setAwaitingConfig(undefined);
        setNotice(msg("console2.configUnchangedRechecked"));
        return true;
      } else {
        setConfigTouched(true);
        setAwaitingConfig({ revision: requestedRevision, values: requestedValues });
        setNotice(msg("configRuntime.savedPending"));
        return true;
      }
    } catch (error) {
      setNotice(msg("hopeA3Console.configFailed", {error: errorText(error)}));
      return false;
    } finally {
      setBusy((current) => ({ ...current, [key]: false }));
    }
  }, [busy, context, snapshot.lifecycleConfig.operator_profile]);

  const confirmLifecycleConfig = useCallback(async (): Promise<boolean> => {
    // Address changes (network migration) are validated by the robot against the live
    // control connection and identity sent with the request (connection_identity).
    return await submitLifecycleConfig(configForSave(configDraft, latest.current.lifecycleConfig));
  }, [configDraft, submitLifecycleConfig]);


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
  // runnerFresh / runnerUsable / runnerStanding and every action gate come from actionGates().
  const gates = actionGates(snapshot, now);
  const { runnerFresh, runnerStanding } = gates;
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

  // Use the same mode receipt for button admission and completion indicators.
  // The observer's separate standing/ready topics can arrive on another frame.
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
  // 2 s: Wi-Fi bursts must not toggle Reposition between enabled and disabled.
  const dockFresh = isFresh(snapshot, TOPICS.dockState, now, 2_000);
  const dockState = dockFresh ? snapshot.dockState : undefined;
  function repositionBlocked(s: Snapshot, at: number): string | undefined {
    const fromReceive = s.runnerMode === "MOTION";
    if (fromReceive && (!playPelvisReady(s, at) || !s.lifecycleSession || s.runnerBoot == undefined)) {
      return t("mocapSafety.readyNeedsPelvis");
    }
    return dockRepositionReason({
      runnerFresh: s.runnerAlive === true && [TOPICS.runnerAlive, TOPICS.runnerMode, TOPICS.runnerFault]
        .every(topic => isFresh(s, topic, at, 1_000)),
      runnerFault: s.runnerFault !== false, mode: s.runnerMode,
      telemetryFresh: isFresh(s, TOPICS.dockState, at, 2_000) && s.dockState != undefined && s.dockState[1] === s.runnerBoot,
      baseFresh: s.baseFresh === true && isFresh(s, TOPICS.baseFresh, at, 1_000),
      serving: s.serving === true, pending: inFlightServices.current.has("dock"),
      estop: s.estopLatched === true || (xboxInputFresh(s.lastReceived[TOPICS.xboxPreview], s.xboxPreview?.source_wall_ms, at)
        && s.xboxPreview?.estop_requested === true),
    }, t);
  }
  const dockBlocked = returnPending ? t("console2.returnToReceive") : repositionBlocked(snapshot, now);

  async function requestReposition(): Promise<void> {
    const start = latest.current;
    if (dockReturn.current || pendingTargetRef.current || start.field?.mode === "KERNEL" ||
        repositionBlocked(start, Date.now()) != undefined) { return; }
    if (start.runnerMode !== "MOTION") { await invoke("dock"); return; }

    const cycle = new AbortController();
    dockReturn.current = cycle;
    setReturnPending(true);
    // Monitor the same Runner throughout, including while service acknowledgements are in flight.
    const safe = () => {
      const s = latest.current, at = Date.now();
      return !cycle.signal.aborted && !pendingTargetRef.current && s.runnerBoot === start.runnerBoot &&
        s.lifecycleSession === start.lifecycleSession && s.lifecycleConfig.hdu_wifi_ip === start.lifecycleConfig.hdu_wifi_ip &&
        s.field?.mode !== "KERNEL" && s.runnerAlive === true && s.runnerFault === false &&
        s.estopLatched !== true && !(xboxInputFresh(s.lastReceived[TOPICS.xboxPreview], s.xboxPreview?.source_wall_ms, at) &&
          s.xboxPreview?.estop_requested === true) && s.serving !== true && s.baseFresh === true &&
        s.dockState?.[1] === start.runnerBoot && playPelvisReady(s, at) &&
        (!s.mocapSafety?.operatorRequired || (s.mocapSafety.uiFresh === true && s.mocapSafety.computerFresh === true)) &&
        ["MOTION", "PD_STAND", "WALK_STAND", "DOCK"].includes(s.runnerMode ?? "") &&
        [TOPICS.runnerAlive, TOPICS.runnerMode, TOPICS.runnerBoot, TOPICS.runnerFault, TOPICS.localRole, TOPICS.baseFresh]
          .every(topic => isFresh(s, topic, at, 1_000)) && isFresh(s, TOPICS.dockState, at, 2_000);
    };
    const guard = () => { if (!safe()) { cycle.abort(); } };
    const watchers = snapshotWaiters.current;
    watchers.add(guard);
    const watchdog = window.setInterval(guard, 200);
    // Subscribe before sending so a fast mode transition can precede its service reply.
    const step = async (key: ServiceKey, acceptedState: (s: Snapshot) => boolean, deadline: number) => {
      if (!safe()) { return false; }
      let finish!: (ok: boolean) => void;
      const observed = new Promise<boolean>(resolve => { finish = resolve; });
      const check = () => { if (safe() && acceptedState(latest.current)) { finish(true); } };
      const cancel = () => finish(false);
      watchers.add(check);
      cycle.signal.addEventListener("abort", cancel);
      const timeout = window.setTimeout(cancel, deadline);
      try {
        return await invoke(key, cycle) && await observed && safe();
      } finally {
        watchers.delete(check);
        cycle.signal.removeEventListener("abort", cancel);
        window.clearTimeout(timeout);
      }
    };
    let requestedReady = false;
    try {
      if (!await step("stand", s => isStanding(s.runnerMode), 5_000)) { return; }
      if (!isStanding(latest.current.runnerMode) || repositionBlocked(latest.current, Date.now()) != undefined) { return; }
      let sawDockMode = false;
      let sawRunningResult = false;
      let sawDockAction = false;
      let leftDock = false;
      const doneInStand = (s: Snapshot) => sawDockMode && sawRunningResult && s.dockState?.[2] === 5 &&
        s.dockState[9] === 2 && s.dockState[10] === 0 && s.runnerMode === "WALK_STAND";
      const completed = (s: Snapshot) => {
        // Mode and dock progress arrive on independent topics and can cross at either edge.
        if (s.runnerMode === "DOCK") { sawDockMode = true; }
        if (s.dockState?.[9] === 1 && s.dockState !== start.dockState) { sawRunningResult = true; }
        if (s.runnerMode === "DOCK" && s.lastAction === "ENTER_DOCK") { sawDockAction = true; }
        if ((sawDockAction && s.lastAction !== "ENTER_DOCK" && s.lastAction !== "RESTORE_STAND_ARMS") || s.runnerMode === "MOTION" ||
            (sawRunningResult && s.dockState?.[9] === 3) ||
            (leftDock && s.runnerMode === "DOCK")) {
          cycle.abort();
          return false;
        }
        if (sawDockMode && s.runnerMode === "WALK_STAND") { leftDock = true; }
        return doneInStand(s);
      };
      if (!await step("dock", completed, 60_000) || !doneInStand(latest.current)) { return; }
      if (latest.current.localRole !== "RECEIVER") {
        if (latest.current.roleChangeAllowed !== true ||
            !await step("setReceiver", s => s.localRole === "RECEIVER", 2_000)) { return; }
      }
      if (safe() && doneInStand(latest.current) && !actionGates(latest.current, Date.now()).readyBlocked) {
        requestedReady = await step("ready", s => s.runnerMode === "MOTION" && s.localRole === "RECEIVER", 8_000);
      }
    } finally {
      window.clearInterval(watchdog);
      watchers.delete(guard);
      if (dockReturn.current === cycle) {
        dockReturn.current = undefined;
        setReturnPending(false);
        if (!requestedReady && !cycle.signal.aborted) { setNotice(msg("console2.returnIncomplete")); }
      }
    }
  }
  const xboxVelocity = xboxFresh ? snapshot.xboxPreview?.velocity : undefined;
  const velocityText = (values: number[] | undefined) => values == undefined
    ? "vx — · vy — · yaw —"
    : t("hopeA3Console.vxMSVyMSYawRad", {vx: (values[0] ?? 0).toFixed(2), vy: (values[1] ?? 0).toFixed(2), yaw: (values[2] ?? 0).toFixed(2)});

  const lifecycleFresh = isFresh(snapshot, TOPICS.lifecycleState, now, 1_500);
  const lifecycleState = lifecycleFresh ? (snapshot.lifecycleState ?? "UNKNOWN") : t("hopeA3Console.noData");
  const lifecycleStopped = lifecycleState === "STOPPED" || lifecycleState === "CONFIG_ERROR";
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
  // 14200 Best runs in both modes, so switching mode is a single set_mode (no policy swap).
  const switchMode = (mode: "OPTITRACK" | "KERNEL") => { void fieldCommand({ op: "set_mode", mode }); };
  const policyUnavailableReason = (model: PolicyModel): string => {
    if (!assetEditable) return t("hopeA3Console.confirmConfigurationAndStopTheLifecycleBeforeSelecting");
    if (kernelMode && !KERNEL_MODELS.has(model)) return t("hopeA3Console.selectOptitrackModeToUseThisPolicy");
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
  const runtimeFresh = snapshot.configRuntime != undefined && isFresh(snapshot, TOPICS.configRuntime, now, 1_500);
  // A robot audit that flaps (e.g. a crash-looping laptop mocap process: ~3.5 s "not applied" every ~12 s) must
  // read as one steady "not applied", not a flashing bar, chip and START button. Once the robot reports
  // applied=false, hold it for CONFIG_APPLIED_HOLD_MS. Missing or stale data does not start the hold.
  const lastNotApplied = useRef<{ at: number; reason: string; receiverDown: boolean }>();
  if (runtimeFresh && snapshot.configRuntime?.applied === false) {
    // Every HDU consumer matches and only laptop mocap receiver processes are missing: the saved
    // configuration is fine, the receiver is not running (e.g. Motive unreachable / NIC unplugged).
    const runtime = snapshot.configRuntime;
    const receiverDown = runtime.services.length > 0 && runtime.services.every((row) => row.matches) &&
      runtime.laptopMissing.length > 0 && runtime.laptopMissing.every((kind) => MOCAP_RECEIVER_KINDS.has(kind));
    lastNotApplied.current = { at: now, reason: runtime.reason, receiverDown };
  }
  const notAppliedHeld = lastNotApplied.current != undefined && now - lastNotApplied.current.at < CONFIG_APPLIED_HOLD_MS;
  const configApplied = configRuntimeReady(snapshot.configRuntime, snapshot.lastReceived[TOPICS.configRuntime], now)
    && !notAppliedHeld && !configTouched && awaitingConfig == undefined;
  // Every HDU service runs the saved configuration (laptop mocap receiver excluded). The robot admits a
  // Kernel START and time calibration on this alone: neither uses mocap, so an unplugged Motive must not
  // block them (operator request 2026-11-18).
  const hduConfigApplied = runtimeFresh && snapshot.configRuntime?.hduApplied === true &&
    !configTouched && awaitingConfig == undefined;
  const kernelConfigApplied = kernelMode && hduConfigApplied;
  const configReady = configApplied || kernelConfigApplied;
  const runtimeReason = runtimeFresh ? snapshot.configRuntime?.reason : t("configRuntime.unverified");
  const mocapBindingUnchanged = sameMocapBinding(configDraft, snapshot.lifecycleConfig);
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
      configConfirmed: configApplied || hduConfigApplied,
    }, t);

  // P1/P2 select the pants body (UCB_P1 / UCB_P2), not a table end; ready when that body is tracked.
  const selectedPlayerReady = selectedPlayerAvailable(configDraft.table_side,
    [snapshot.playerP1, snapshot.playerP2], now) &&
    field?.player_positions?.players_x_m?.[`UCB_${configDraft.table_side}`] != undefined;
  const startEvidence: StartEvidence = {
    lifecycleState, lifecycleBusy: lifecycleBusy || busy.startLifecycle === true,
    calibrationBusy: timeCalibrationBlocksStart, configApplied: configReady, configTouched,
    mocapDown: !kernelMode && !configApplied && hduConfigApplied && lastNotApplied.current?.receiverDown === true,
    fieldBusy: fieldBusy || field?.busy === true, fieldFresh, kernelMode,
    ballReady: ballDisplay.ready, playerReady: selectedPlayerReady,
    side: configDraft.table_side,
  };
  const startBlocked = startBlockReason(startEvidence, t);
  // Display-only, one chip per startBlockReason condition (see startChecks).
  const startCheckList = startChecks(startEvidence);
  const mocapRecovery = fieldFresh ? field?.mocap_recovery : undefined;

  const serveState = gates.serveState;
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

  useEffect(() => {
    setNoticeLog((log) => [{ id: (log[0]?.id ?? 0) + 1, at: Date.now(), text: notice }, ...log].slice(0, 30));
  }, [notice]);
  useEffect(() => {
    // Choose the first tab once from live lifecycle state; never switch tabs under the operator later.
    if (tab == undefined && lifecycleFresh) { setTab(snapshot.lifecycleState === "RUNNING" ? "play" : "setup"); }
  }, [tab, lifecycleFresh, snapshot.lifecycleState]);
  const activeTab: ConsoleTab = tab ?? (lifecycleState === "RUNNING" ? "play" : "setup");
  const switching = Boolean(pendingTarget);
  const rigidStandShortcut = useRef<() => void>(() => {});
  rigidStandShortcut.current = () => {
    const current = latest.current;
    if (pendingTargetRef.current || estopAsserted || current.estopLatched === true ||
        actionGates(current, Date.now()).standBlocked || inFlightServices.current.has("stand") ||
        current.runnerMode === "SERVE" || current.serving === true) return;
    void invoke("rigidStand");
  };
  useEffect(() => {
    const walk = new KeyboardWalk(context, () => {
      const s = latest.current;
      const now = Date.now();
      const boot = s.runnerBoot;
      if (pendingTargetRef.current || dockReturn.current || inFlightServices.current.size > 0 ||
          !context.callService || !boot || !s.lifecycleSession || s.lifecycleState !== "RUNNING" ||
          s.lifecycleBusy !== false || s.runnerAlive !== true || s.runnerFault !== false ||
          s.estopLatched !== false || s.xboxPreview?.estop_requested || !s.xboxPreview?.keyboard_walk ||
          s.xboxPreview.lt || s.xboxPreview.action_pending ||
          !xboxInputFresh(s.lastReceived[TOPICS.xboxPreview], s.xboxPreview.source_wall_ms, now) ||
          ![TOPICS.runnerBoot, TOPICS.runnerAlive, TOPICS.runnerFault, TOPICS.runnerMode, TOPICS.teleopState,
            TOPICS.lifecycleSession, TOPICS.lifecycleState, TOPICS.lifecycleBusy, TOPICS.estopLatched]
            .every(topic => isFresh(s, topic, now, 1200)) ||
          s.teleopState?.[1] !== boot || s.teleopState[2] !== 1 || s.teleopState[3] !== 2 ||
          (s.runnerMode !== "WALK_STAND" && s.runnerMode !== "TELEOP")) { return undefined; }
      return { boot, fingerprint: sessionFingerprint(s.lifecycleSession), standing: s.runnerMode === "WALK_STAND" };
    });
    return bindRunnerShortcuts(context.panelElement, () => rigidStandShortcut.current(), walk);
  }, [context, context.panelElement]);

  const openTab = (next: ConsoleTab, card?: string) => {
    setTab(next);
    if (card != undefined) {
      window.setTimeout(() => {
        rootRef.current?.querySelector(`[data-card="${card}"]`)?.scrollIntoView({ block: "center", behavior: "smooth" });
      }, 0);
    }
  };
  const openDiag = () => {
    setDiagOpen(true);
    openTab("play", "diag");
  };
  const clock = (at: number) => new Date(at).toLocaleTimeString(undefined, { hour12: false });

  // ---- header ----
  const ntpValue = ntpFresh && ntpGateFresh ? snapshot.ntpPass : undefined;
  const tfValue = isFresh(snapshot, TOPICS.tfReady, now, 1_000) ? snapshot.tfReady : undefined;
  const estopBackendValue = estopAsserted ? false : estopUsable ? estopFullReady : undefined;
  const markerValue = confirmedSource === "motive" && markerFresh && (snapshot.markerExpectedCount ?? 0) > 0
    ? snapshot.markerCount === snapshot.markerExpectedCount : undefined;
  const markerActual = markerFresh ? Math.round(snapshot.markerCount ?? 0) : "—";
  const markerExpected = markerFresh && (snapshot.markerExpectedCount ?? 0) > 0 ? Math.round(snapshot.markerExpectedCount ?? 0) : "—";
  const markerAudit = confirmedSource === "vrpn" ? t("hopeA3Console.vrpnLiveMarkerCountNA")
    : t("hopeA3Console.markersAudit", { asset: markerAsset, actual: markerActual, expected: markerExpected });
  const markerDetail = confirmedSource === "vrpn" ? t("hopeA3Console.vrpnStreamsPosesOnlyLiveMarkerCountsAre")
    : t("hopeA3Console.selectedMarkerAsset", { asset: markerAsset });
  const swingAudit = t("hopeA3Console.swingInputAudit", { status: snapshot.runnerMode === "MOTION"
    ? `F${snapshot.commandFlight ?? "—"} · ${snapshot.commandCountdownS != undefined ? `${snapshot.commandCountdownS.toFixed(2)}s` : t("hopeA3Console.noCommand")}`
    : t("hopeA3Console.waitForMotion") });
  const healthChips: { key: string; label: string; value?: boolean; detail: string }[] = [
    { key: "ntp", label: t("console2.healthNtp"), value: ntpValue,
      detail: `${t("hopeA3Console.ntpAudit")} · ${ntpFresh && snapshot.ntpOffsetMs != undefined ? `${snapshot.ntpOffsetMs.toFixed(2)} ms` : "—"}` },
    { key: "timestamp", label: t("console2.healthTimestamp"), value: latencyFresh ? true : undefined,
      detail: `${t("hopeA3Console.timestampAudit")} · ${latencyFresh && snapshot.latencyMs != undefined ? `${snapshot.latencyMs.toFixed(1)} ms` : "—"}` },
    { key: "tf", label: t("console2.healthTf"), value: tfValue, detail: t("hopeA3Console.tfAudit") },
    { key: "estop", label: t("console2.healthEstop"), value: estopBackendValue,
      detail: snapshot.estopText ?? t("hopeA3Console.eStopBackendAudit") },
    kernelMode
      ? { key: "mocap", label: t("console2.healthImuLocal"), value: undefined, detail: t("console2.healthImuLocalDetail") }
      : { key: "mocap", label: confirmedSource === "vrpn" ? t("console2.healthMocapPose")
        : confirmedSource === "none" ? t("console2.healthMocapNone")
          : t("console2.healthMarkers", { actual: markerActual, expected: markerExpected }),
        value: markerValue, detail: `${markerAudit} · ${markerDetail}` },
    kernelMode
      ? { key: "swing", label: t("console2.healthPlannerNone"), value: undefined, detail: t("console2.healthPlannerNoneDetail") }
      : { key: "swing", label: t("console2.healthSwing"), value: swingInputReady,
        detail: `${swingAudit} · ${snapshot.commandSummary ?? t("hopeA3Console.ballVisibilityAloneIsNotASwingCommand")}` },
  ];

  const lifecycleLabel = lifecycleState === "STOPPED" ? t("console2.lifeStopped")
    : lifecycleState === "STARTING" ? t("console2.lifeStarting")
      : lifecycleState === "RUNNING" ? t("console2.lifeRunning")
        : lifecycleState === "KILLING" ? t("console2.lifeKilling")
          : lifecycleState === "FAILED" ? t("console2.lifeFailed")
            : lifecycleState === "CONFIG_ERROR" ? t("console2.lifeConfigError") : lifecycleState;
  const lifecycleTone: Tone = lifecycleState === "RUNNING" ? (kernelMode ? "kern" : "ok")
    : lifecycleState === "STARTING" || lifecycleState === "KILLING" ? "warn"
      : lifecycleState === "FAILED" || lifecycleState === "CONFIG_ERROR" ? "bad" : "idle";
  const runnerLabel = !runnerFresh ? t("console2.runnerOffline")
    : serveState === "SERVING" || snapshot.serving === true ? t("console2.runnerServing", { state: serveState })
      : serveState === "WAIT_READY_TO_SERVE" ? t("console2.runnerRaised")
        : serveState === "PREPARING" ? t("console2.runnerRaising")
          : snapshot.runnerMode === "PD_STAND" ? t("console2.runnerStand")
            : snapshot.runnerMode === "MOTION" ? (kernelMode ? t("console2.runnerReceiveTest") : t("console2.runnerReceiving"))
              : snapshot.runnerMode === "TELEOP" ? t("console2.runnerTeleop")
              : snapshot.runnerMode === "WALK_STAND" ? t("console2.runnerWalkStand")
              : snapshot.runnerMode === "DOCK" ? t("console2.runnerDock")
                : snapshot.runnerMode ?? t("hopeA3Console.unknown");
  const runnerTone: Tone = !runnerFresh ? "idle"
    : snapshot.runnerFault === true || snapshot.runnerMode === "PASSIVE" || serveWorkflowActive ? "warn"
      : snapshot.runnerMode === "PD_STAND" ? "ok" : "acc";
  const runnerSummaryText = runnerFresh
    ? t("hopeA3Console.runnerSummary", { mode: snapshot.runnerMode ?? t("hopeA3Console.unknown"), serve: serveState,
      gripper: snapshot.gripperState ?? t("hopeA3Console.unavailable"),
      cleanup: snapshot.serveCleanupRequired === true ? t("hopeA3Console.cleanupRequired") : "" })
    : t("hopeA3Console.noFreshRunnerState");
  const bestCatalog = field?.policy_models?.model14200best;
  const bestVariant = bestCatalog?.available === true || field?.policy_model === "model14200best"
    ? best14200Variant(bestCatalog?.serve_csv?.sha256 ?? undefined, field?.policy_model === "model14200best" ? field.csv : undefined) : undefined;
  const speed104 = runtimeFresh && fieldFresh && field?.policy_models?.model23700?.available === true &&
    field.policy_models_config_revision === snapshot.configRuntime?.revision &&
    snapshot.configRuntime?.policyDigests?.model23700 === "1b4adc3494d67d48cb548d0b5fc3b118df9079082f2135d49e5bc815480b821c";
  const speed104Name = "23700 Best · speed104";
  const displayPolicyRows = speed104 ? VISIBLE_POLICY_ROWS.map(row => row.model === "model23700"
    ? { ...row, name: speed104Name, label: msg("hopeA3Console.model23700Best") } : row) : VISIBLE_POLICY_ROWS;
  const policyName = field?.policy_model === "model23700" && speed104 ? speed104Name : field?.policy_model ? POLICY_NAMES[field.policy_model] ?? field.policy_model : "—";
  const roleLabel = snapshot.localRole === "SERVER" ? t("console2.roleServer")
    : snapshot.localRole === "RECEIVER" ? t("console2.roleReceiver") : snapshot.localRole ?? t("hopeA3Console.unassigned");
  const connectionTone: Tone = hduFresh && snapshot.hduActive === true ? "ok" : lifecycleFresh ? "warn" : "bad";
  const shownIp = configHostFresh && configHost ? configHost : robotTarget || "—";
  const connectionStatus = pendingTarget ? t("hopeA3Console.switchingTo8766ControlsInThisPanelAre", {ip: pendingTarget}) :
    lifecycleFresh && configHostFresh && configHost ? t("hopeA3Console.addressReportedByTheCurrentRobot", {ip: configHost, state: snapshot.lifecycleState}) :
      lifecycleFresh ? t("hopeA3Console.liveStatusReceivedFromRobotAddressIsUnset", {state: snapshot.lifecycleState}) :
        t("hopeA3Console.noLiveRobotStatusTheTargetDoesNot");
  const estopSubtitle = estopAsserted ? t("console2.estopSubAsserted") : estopFullReady ? t("console2.estopSubFull")
    : estopUsable ? t("console2.estopSubPartial") : t("console2.estopSubUnknown");
  const estopTitle = estopAsserted ? t("hopeA3Console.clickToReassertResetSoftwareEStopBelow")
    : estopFullReady ? t("hopeA3Console.dualPathStopsRunnerAndRequestsVendorStop")
      : estopUsable ? t("hopeA3Console.partialSoftwareStopUsePhysicalEStop") : t("hopeA3Console.backendUnknownClickStillAsserts");
  const recordTitle = [
    `${t("hopeA3Console.recording")} ${recordingState}`,
    field?.mode === "KERNEL" ? t("hopeA3Console.defaultOffRobotDataKernelHasNoPlanner") : t("hopeA3Console.defaultOffRobotPolicyPlannerAndMocapData"),
    field?.recording.output ?? "", field?.recording.error ?? "",
  ].filter((line) => line !== "").join("\n");
  const setupBadge = lifecycleState !== "RUNNING" && startCheckList.some((check) => !check.ok) ? "bad" : undefined;
  const diagBadge = healthChips.some((chip) => chip.value === false) ? "bad" : undefined;
  const tabs: { id: ConsoleTab; label: string; badge?: string }[] = [
    { id: "setup", label: t("console2.tabSetup"), badge: setupBadge },
    { id: "play", label: kernelMode ? t("console2.tabKernel") : t("console2.tabPlay"), badge: diagBadge },
  ];

  // ---- setup ----
  const configLocked = !lifecycleStopped || lifecycleBusy;
  const dirtyFields = configTouched ? (Object.keys(CONFIG_FIELDS) as ConfigField[]).filter((name) =>
    configDraft[name] !== (snapshot.lifecycleConfig[name] ?? EMPTY_CONFIG[name])) : [];
  const editDraft = (name: ConfigField, value: string) => {
    setConfigTouched(true);
    setConfigDraft((current) => ({ ...current, [name]: value }));
  };
  // Former <select> controls only reported real changes; keep that for segmented buttons.
  const selectDraft = (name: ConfigField, value: string) => {
    if (configDraft[name] !== value) { editDraft(name, value); }
  };
  // Edits stay a draft until 确认配置: one confirmation is one saved revision (operator request
  // 2026-11-18; per-choice auto-save produced a revision and a mocap receiver restart per click).
  const configInput = (name: ConfigField, options: { short?: boolean; numeric?: boolean } = {}) => (
    <input
      type="text"
      className={`c-inp ${options.short === true ? "c-inp-short" : ""} ${dirtyFields.includes(name) ? "c-dirty" : ""}`}
      inputMode={options.numeric === true ? "numeric" : "decimal"}
      spellCheck={false}
      aria-label={t.text(CONFIG_FIELDS[name].label)}
      disabled={configLocked}
      value={configDraft[name]}
      placeholder={options.numeric === true ? undefined : t("hopeA3Console.ipv4Address")}
      onChange={(event) => { editDraft(name, event.target.value); }}
    />
  );
  const checkLabel = ({ id, ok }: { id: StartCheckId; ok: boolean }): string => {
    switch (id) {
      case "busy": return ok ? t("console2.checkIdle") : t("console2.checkBusy");
      case "state": return ok ? t("console2.checkStopped") : t("console2.checkState", { state: lifecycleLabel });
      case "calibration": return ok ? t("console2.checkCalibration") : t("console2.checkCalibrationBad");
      case "config": return ok ? t("console2.checkConfig") : t("console2.checkConfigBad");
      case "field": return ok ? t("console2.checkField") : t("console2.checkFieldBad");
      case "ball": return ok ? t("console2.checkBall") : t("console2.checkBallBad");
    }
  };
  const checkReason: Record<StartCheckId, string> = {
    busy: t("mocapRecovery.startBusy"), state: t("mocapRecovery.startState"),
    calibration: t("mocapRecovery.startCalibration"), config: t("mocapRecovery.startConfig"),
    field: t("mocapRecovery.startField"), ball: t("mocapRecovery.startBall"),
  };
  const sessionStart = sessionStartMs(snapshot.lifecycleSession);
  const sessionText = snapshot.lifecycleSession != undefined && snapshot.lifecycleSession.length > 0
    ? snapshot.lifecycleSession : t("hopeA3Console.noActiveSession");
  const lifecycleView = lifecycleState === "RUNNING" ? "running"
    : lifecycleState === "STARTING" || lifecycleState === "KILLING" ? "busy" : "stopped";
  // START (left) and STOP (right) always sit together at the bottom of the lifecycle card; the
  // enable/grey rules live in lifecycleButtons.ts (operator request 2026-11-14).
  const lifeButtons = lifecycleButtons({
    fresh: lifecycleFresh,
    state: snapshot.lifecycleState,
    lifecycleBusy,
    startPending: busy.startLifecycle === true,
    stopPending: busy.killAllAndCollect === true,
    startBlocked: startBlocked != undefined,
    kernel: kernelMode,
  });
  const lifeToneClass: Record<LifecycleButtonTone, string> = {
    go: "c-go", kern: "c-gok", stop: "c-stopbig", busy: "c-busybig", off: "c-offbig",
  };
  const lifeReason = (reason: typeof lifeButtons.start.reason): string | undefined =>
    reason == undefined ? undefined : reason === "startChecks" ? startBlocked : t(reason);
  const lifecycleButtonRow = (
    <div className="c-bigrow">
      <button className={`c-btn c-big ${lifeToneClass[lifeButtons.start.tone]}`} type="button"
        disabled={!lifeButtons.start.enabled} title={lifeReason(lifeButtons.start.reason)} onClick={startSystem}>
        {t(lifeButtons.start.label)}
      </button>
      <button className={`c-btn c-big ${lifeToneClass[lifeButtons.stop.tone]}`} type="button"
        disabled={!lifeButtons.stop.enabled}
        title={lifeReason(lifeButtons.stop.reason) ?? t("console2.killAndCollect")} onClick={killAllAndCollect}>
        {t(lifeButtons.stop.label)}
      </button>
    </div>
  );
  const timeCalibrationTone: Tone = timeCalibrationState === "COMPLETE" ? "ok"
    : timeCalibrationBusy || timeCalibrationState === "RUNNING" ? "busy"
      : timeCalibrationState === "FAILED_SAFE_STOP" || timeCalibrationState === "INTERRUPTED" ? "bad" : "idle";
  // Time calibration is a pre-START step, so it is a row of the System card (between status and checks).
  const timeCalRow = (
    <div className={`c-lsub time-calibration-${timeCalibrationState.toLowerCase().replace(/[_ ]/g, "-")}`} data-card="timecal">
      <span className={`c-dot c-${timeCalibrationTone}`} />
      <div className="c-grow">
        <div className="c-lsub-head">
          <b>{t("console2.timeCalibration")}</b>
          <span className="c-hint">{timeCalibrationState} · {snapshot.timeCalibrationStep ?? t("hopeA3Console.idle")}</span>
        </div>
        <div className="c-mono c-dim c-trunc" title={snapshot.timeCalibrationResult}>
          {snapshot.timeCalibrationOperation != undefined && snapshot.timeCalibrationOperation.length > 0
            ? `${snapshot.timeCalibrationOperation} · ` : ""}
          {snapshot.timeCalibrationResult ?? t("hopeA3Console.waitingForCalibrationCoordinator")}
        </div>
      </div>
      <button
        className="c-btn c-sm"
        type="button"
        disabled={!timeCalibrationReady || busy.timeCalibration === true}
        title={configApplied || hduConfigApplied ? timeCalibrationTitle : runtimeReason}
        onClick={runTimeCalibration}
      >
        {busy.timeCalibration === true || timeCalibrationBusy ? t("console2.calibrating") : t("console2.runCalibration")}
      </button>
    </div>
  );
  // Configuration is a permanent row of the System card (like time calibration): the state text and dot change,
  // the row and its Confirm button never appear/disappear (operator request 2026-11-17).
  const configPending = configTouched || !configReady || busy.applyLifecycleConfig === true;
  const configTone: Tone = busy.applyLifecycleConfig === true || awaitingConfig != undefined ? "busy"
    : configPending ? "warn" : "ok";
  const savedMocapTarget = snapshot.lifecycleConfig.mocap_source === "vrpn"
    ? `VRPN ${snapshot.lifecycleConfig.vrpn_ip ?? ""}` : `Motive ${snapshot.lifecycleConfig.motive_ip ?? ""}`;
  const receiverFailed = (snapshot.lifecycleResult ?? "").startsWith("CONFIG_SAVED_MOCAP_FAILED");
  const notAppliedText = notAppliedHeld && lastNotApplied.current != undefined
    ? (lastNotApplied.current.receiverDown || receiverFailed
      ? t("console2.mocapReceiverDown", { target: savedMocapTarget.trim() })
      : lastNotApplied.current.reason !== "" ? `${t("configRuntime.pending")} · ${lastNotApplied.current.reason}`
        : t("configRuntime.pending"))
    : t("configRuntime.pending");
  const configStateText = busy.applyLifecycleConfig === true ? t("console2.confirming")
    : awaitingConfig != undefined ? t("configRuntime.savedPending")
      : configTouched ? (dirtyFields.length > 0 ? t("console2.unconfirmedEdits", { count: dirtyFields.length })
        : t("hopeA3Console.configurationChangedNotAppliedYet"))
        : configApplied ? t("console2.configApplied")
          : configReady ? t("console2.configAppliedKernel") : notAppliedText;
  // Full audit detail (revision, raw robot reason) stays available on hover only.
  const configStateTitle = [configStateText, t("console2.configRevision", { revision: snapshot.lifecycleConfigRevision ?? 0 }),
    !configApplied && runtimeFresh ? snapshot.configRuntime?.reason : undefined,
    receiverFailed ? snapshot.lifecycleResult : undefined].filter((line) => line != undefined && line !== "").join("\n");
  const configRow = (
    <div className={`c-lsub c-cfgrow c-cfg-${configTone}`} data-card="config">
      <span className={`c-dot c-${configTone}`} />
      <div className="c-grow">
        <div className="c-lsub-head">
          <b>{t("console2.configTitle")}</b>
        </div>
        <div className="c-cfgtext c-trunc" title={configStateTitle}>{configStateText}</div>
      </div>
      <button
        className={`c-btn c-sm ${configPending ? "c-warnb" : ""}`}
        type="button"
        disabled={
          !configPending ||
          !configComplete ||
          !lifecycleStopped ||
          lifecycleBusy ||
          timeCalibrationBlocksStart ||
          busy.applyLifecycleConfig === true
        }
        title={!configPending ? t("console2.configNothingToConfirm")
          : !configComplete ? t("console2.configIncomplete") : runtimeReason}
        onClick={() => { void confirmLifecycleConfig().then((ok) => { if (ok) { openTab("play", "life"); } }); }}
      >
        {busy.applyLifecycleConfig === true ? t("console2.confirming") : t("console2.confirmConfig")}
      </button>
    </div>
  );
  const resultView = lifecycleResultView(snapshot.lifecycleResult, snapshot.configRuntime == undefined ? undefined : {
    applied: configRuntimeReady(snapshot.configRuntime, snapshot.lastReceived[TOPICS.configRuntime], now),
    revision: snapshot.configRuntime.revision,
  });
  // Config results are shown by the permanent configuration row; only other lifecycle results get a line.
  // STOP's automatic vendor repair could not bring the vendor stack back: say what to do in plain words.
  const pairedRebootRequired = resultView.kind === "raw" && resultView.text.includes("PAIRED_REBOOT_REQUIRED");
  const lifecycleResultLine = resultView.kind === "raw" ? (
    <>
      {pairedRebootRequired && <div className="c-bad-text">{t("console2.pairedRebootRequired")}</div>}
      <div className="c-mono c-dim c-trunc" title={resultView.text}>{resultView.text}</div>
    </>
  ) : undefined;
  const sideConfigured = configDraft.table_side === "P1" || configDraft.table_side === "P2";
  const lifecycleStatusLine = lifecycleView === "running" ? (
    <div className="c-lrow">
      <span className={`c-dot c-${kernelMode ? "kern" : "ok"}`} />
      <b>{kernelMode ? t("console2.kernelRunning") : t("console2.systemRunning")}</b>
      <span className="c-mono c-hint c-trunc" title={snapshot.lifecycleStep}>
        {sessionStart != undefined ? `${sessionText} · ${formatElapsed(now - sessionStart)}` : sessionText}
      </span>
    </div>
  ) : lifecycleView === "busy" ? (
    <div className="c-lrow">
      <span className="c-dot c-busy" />
      <b>{lifecycleState === "STARTING" ? t("console2.starting") : t("console2.stopping")}</b>
      <span className="c-mono c-hint c-trunc">{snapshot.lifecycleStep === "VENDOR_REPAIR" ? t("console2.vendorRepairing")
        : snapshot.lifecycleStep ?? t("hopeA3Console.idle")}</span>
    </div>
  ) : (
    <div className="c-lrow">
      <span className={`c-dot c-${lifecycleTone === "idle" && lifecycleFresh ? "warn" : lifecycleTone}`} />
      <b>{lifecycleLabel}</b>
    </div>
  );
  const lifeCardClass = lifecycleView === "running" ? `c-life-run ${kernelMode ? "c-life-k" : ""}`
    : lifecycleState === "FAILED" ? "c-life-failed" : "";
  // Start card: mocap connection with its own reconnect, under time calibration (operator request 2026-11-18).
  const savedMocapSource = snapshot.lifecycleConfig.mocap_source ?? "none";
  const mocapTarget = savedMocapSource === "none" ? "" : savedMocapTarget.trim();
  const livePlayers = fieldFresh ? field?.player_positions?.players_x_m ?? {} : {};
  const mocapView = mocapConnectionView({
    source: savedMocapSource, target: mocapTarget, recovery: mocapRecovery,
    anyTracked: Object.keys(livePlayers).length > 0,
    selectedTracked: livePlayers[`UCB_${snapshot.lifecycleConfig.table_side ?? ""}`] != undefined,
    kernelMode, requesting: mocapRequesting, calibrating: timeCalibrationBlocksStart || timeCalibrationBusy,
  }, t);
  const receiverMissing = runtimeFresh ? snapshot.configRuntime?.laptopMissing ?? [] : [];
  const mocapRow = (
    <div className="c-lsub" data-card="mocapconn">
      <span className={`c-dot c-${mocapView.tone}`} />
      <div className="c-grow">
        <div className="c-lsub-head">
          <b>{t("mocapConnection.title")}</b>
          {mocapTarget !== "" && <span className="c-hint">{mocapTarget}</span>}
        </div>
        <div className="c-cfgtext c-trunc" title={receiverMissing.length > 0
          ? `${mocapView.text}\n${t("mocapConnection.receiverMissing", { kinds: receiverMissing.join(", ") })}`
          : mocapView.text}>{mocapView.text}</div>
      </div>
      <button
        className="c-btn c-sm"
        type="button"
        disabled={!mocapView.canReconnect}
        title={mocapView.buttonTitle}
        onClick={() => {
          setMocapRequesting(true);
          void fieldCommand({ op: "reconnect_mocap" }).finally(() => { setMocapRequesting(false); });
        }}
      >
        {mocapView.tone === "busy" ? t("mocapRecovery.reconnecting") : t("mocapRecovery.reconnect")}
      </button>
    </div>
  );
  // Recording, moved from the diagnostics fold into the Start card (operator request 2026-11-18). Same switch
  // as the header REC pill, where the status is read.
  const recordingTone: Tone = recordingState === "ERROR" ? "bad" : recordingState === "ON" ? "ok"
    : recordingState === "STARTING" || recordingState === "STOPPING" ? "busy" : "idle";
  const recordingRow = (
    <div className="c-lsub c-recsub" data-card="recording">
      <span className={`c-dot c-${recordingTone}`} />
      <div className="c-grow">
        <div className="c-lsub-head">
          <b>{t("console2.recording")}</b>
          <span className="c-hint c-mono">{recordingState}</span>
        </div>
        <div className="c-diag-lines">
          <div className="c-hint">{field?.mode === "KERNEL" ? t("console2.recDescKernel") : t("console2.recDesc")}</div>
          {field?.recording.output != undefined && field.recording.output !== "" && <div className="c-mono c-dim c-trunc" title={field.recording.output}>{field.recording.output}</div>}
          {field?.recording.error != undefined && field.recording.error !== "" && <div className="c-bad-text">{field.recording.error}</div>}
        </div>
      </div>
      <div className="c-seg c-seg-sm" role="group" aria-label={t("console2.recording")}>
        <button type="button" aria-pressed={!recordingActive}
          disabled={!fieldFresh || recordBusy || !recordingActive || recordingState === "STOPPING"}
          onClick={() => void fieldCommand({ op: "record_off" })}>{t("console2.recSwitchOff")}</button>
        <button type="button" aria-pressed={recordingActive}
          disabled={!fieldFresh || recordBusy || recordingActive}
          onClick={() => void fieldCommand({ op: "record_on" })}>{t("console2.recSwitchOn")}</button>
      </div>
    </div>
  );
  // Play: time calibration, START and STOP (operator request 2026-11-17: start from the Play tab).
  const startCard = (
    <div className={`c-card c-life ${lifeCardClass}`} data-card="life">
      <div className="c-lbl">{t("console2.startTitle")}</div>
      {lifecycleStatusLine}
      {timeCalRow}
      {mocapRow}
      {recordingRow}
      {lifecycleButtonRow}
      {lifecycleResultLine}
    </div>
  );
  // Header status: health signals and start checks merged by subject (operator request 2026-11-17).
  // Display only: startBlockReason still gates START. Each merged chip lists its parts in the tooltip
  // and opens the first failing part.
  type HeaderChip = { key: string; label: string; value?: boolean; warn?: boolean; detail: string; open: () => void };
  const showStartChecks = lifecycleView !== "running";
  const healthOf = (key: string) => healthChips.find((chip) => chip.key === key);
  const checkOf = (id: StartCheckId) => (showStartChecks ? startCheckList.find((check) => check.id === id) : undefined);
  const mark = (state: { ok?: boolean }) => (state.ok == undefined ? "–" : state.ok ? "✓" : "✕");
  const headerChips: HeaderChip[] = [];
  {
    // System: idle + stopped
    const parts = [checkOf("busy"), checkOf("state")].filter((check) => check != undefined);
    const failing = parts.find((check) => !check.ok);
    if (parts.length > 0) {
      headerChips.push({ key: "system", label: failing != undefined ? checkLabel(failing) : t("console2.chipSystem"),
        value: failing == undefined, detail: parts.map((check) => `${mark(check)} ${checkLabel(check)}`).join("\n"),
        open: () => { openTab("play", "life"); } });
    }
  }
  {
    // Time: NTP gate + message timestamps + time calibration
    const ntp = healthOf("ntp");
    const stamp = healthOf("timestamp");
    const calibration = checkOf("calibration");
    const value = ntp?.value === false || calibration?.ok === false ? false
      : ntp?.value === true && stamp?.value === true ? true : undefined;
    headerChips.push({ key: "time",
      label: calibration != undefined && !calibration.ok ? checkLabel(calibration) : t("console2.chipTime"),
      value,
      detail: [
        ntp != undefined ? `${mark({ ok: ntp.value })} ${ntp.detail}` : undefined,
        stamp != undefined ? `${mark({ ok: stamp.value })} ${stamp.detail}` : undefined,
        calibration != undefined ? `${mark(calibration)} ${checkLabel(calibration)} · ${timeCalibrationState}` : undefined,
      ].filter((line) => line != undefined).join("\n"),
      open: () => { if (calibration != undefined && !calibration.ok) { openTab("play", "timecal"); } else { openDiag(); } } });
  }
  {
    // Configuration (has its own Confirm bar on Setup)
    const config = checkOf("config");
    if (config != undefined) {
      headerChips.push({ key: "config", label: checkLabel(config), value: config.ok,
        detail: config.ok ? configStateText : `${checkReason.config}\n${configStateText}`,
        open: () => { openTab("setup", "config"); } });
    }
  }
  {
    // Mocap: markers / pose + our table side seen
    const mocap = healthOf("mocap");
    const sideShown = showStartChecks && !kernelMode && sideConfigured;
    const sideUnseen = sideShown && !selectedPlayerReady;
    if (mocap != undefined) {
      headerChips.push({ key: "mocap",
        label: sideShown ? `${mocap.label} · ${configDraft.table_side}` : mocap.label,
        value: mocap.value, warn: sideUnseen && mocap.value !== false,
        detail: [mocap.detail, sideShown ? `${selectedPlayerReady ? "✓" : "⚠"} ${selectedPlayerReady
          ? t("console2.checkSideSeen", { side: configDraft.table_side })
          : t("console2.checkSideUnseen", { side: configDraft.table_side })} · ${t("console2.sideCheckTitle")}` : undefined]
          .filter((line) => line != undefined).join("\n"),
        open: () => { if (sideUnseen) { openTab("setup", "mocap"); } else { openTab("setup", "hw"); } } });
    }
  }
  {
    // Field: HDU field status + ball model verification (part of the same status)
    const parts = [checkOf("field"), checkOf("ball")].filter((check) => check != undefined);
    const failing = parts.find((check) => !check.ok);
    if (parts.length > 0) {
      headerChips.push({ key: "field", label: failing != undefined ? checkLabel(failing) : t("console2.chipField"),
        value: failing == undefined,
        detail: parts.map((check) => `${mark(check)} ${checkLabel(check)}${check.ok ? "" : ` · ${checkReason[check.id]}`}`).join("\n"),
        open: () => { if (failing?.id === "ball") { openTab("setup", "mocap"); } else { openDiag(); } } });
    }
  }
  for (const key of ["estop", "tf"]) {
    const chip = healthOf(key);
    if (chip != undefined) { headerChips.push({ ...chip, open: openDiag }); }
  }
  {
    // Swing input only matters while the system runs
    const swing = healthOf("swing");
    if (swing != undefined && (!showStartChecks || snapshot.runnerMode === "MOTION")) {
      headerChips.push({ ...swing, open: openDiag });
    }
  }
  const selectedPolicyLabel = field?.policy_model === "model23700" ? t(speed104 ? "hopeA3Console.model23700Best" : "hopeA3Console.model23700") : field?.policy_model === "model400center3" ? t("hopeA3Console.model400center3") : field?.policy_model === "model400" ? t("hopeA3Console.model400ServeReceive") : field?.policy_model === "model26500" ? t("hopeA3Console.model26500ReceiveOnly") : field?.policy_model === "model19000" ? t("hopeA3Console.model19000ReceiveOnly") : field?.policy_model === "model15200body" ? t("hopeA3Console.model15200bodyServeReceive") : field?.policy_model === "model14200" ? t("hopeA3Console.model14200ServeReceive") : field?.policy_model === "model14200fusion" ? t("hopeA3Console.model14200fusionServeReceive") : field?.policy_model === "model14200best" ? t("hopeA3Console.model14200bestServeReceive") : field?.policy_model === "model13800astra" ? t("hopeA3Console.astraV23ServeReceive") : field?.policy_model === "modelfusion" ? t("hopeA3Console.fusionV23ServeReceive") : field?.policy_model === "model13800" ? t("hopeA3Console.13800V2313800ServeReceive") : field?.policy_model === "model12000" ? t("hopeA3Console.12000V2312000ServeReceive") : field?.policy_model === "model17400" ? (field.policy_models?.model17400?.receive_only === true ? t("hopeA3Console.17400ReceiveOnly") : t("hopeA3Console.17400V2317400ServeReceive")) : field?.policy_model === "model12600" ? t("hopeA3Console.12600V2312600ServeReceive") : field?.policy_model === "model11999" ? t("hopeA3Console.11999ReceiveOnly") : field?.policy_model === "model14600" ? t("hopeA3Console.14600ServeReceive") : t("hopeA3Console.noPolicySelected");

  const csvStatus = field?.csv;
  const csvName = field?.csv?.name ?? t("hopeA3Console.a3pOp3Serve025V23Branch110Model14600CsvDefault");
  const csvMeta = field?.csv
    ? t("hopeA3Console.validatedFramesHzSha", {frames: field.csv.frames, hz: field.csv.hz, hash: field.csv.sha256.slice(0, 12)})
    : t("hopeA3Console.468Frames100HzSdk31JointFormat");
  const sideRow = (
    <>
      <div className="c-k" title={t("hopeA3Console.bothMocapSystemsUseUcbP1UcbP2")}>
        {t("console2.ourSide")}<i>{t("console2.lockedAtStart")}</i>
      </div>
      <div className="c-seg" role="group" aria-label={t("hopeA3Console.ourTableSide")}>
        {(["P1", "P2"] as const).map((side) => {
          const live = selectedPlayerAvailable(side, [snapshot.playerP1, snapshot.playerP2], now);
          const verified = field?.player_positions?.players_x_m?.[`UCB_${side}`] != undefined;
          return (
            <button
              key={side}
              type="button"
              aria-label={side === "P1" ? t("hopeA3Console.ucbP1Direct") : t("hopeA3Console.ucbP2Rotate180")}
              aria-pressed={configDraft.table_side === side}
              disabled={configLocked}
              onClick={() => {
                setConfigTouched(true);
                setConfigDraft((current) => ({ ...current, table_side: side }));
              }}
            >
              {side === "P1" ? t("console2.sideP1") : t("console2.sideP2")}
              {!kernelMode && (
                <small className={live && verified ? "c-verify-ok" : "c-verify-no"}
                  title={live && !verified && fieldFresh ? t("hopeA3Console.peerNotVerified") : undefined}>
                  {live ? (verified ? t("console2.sideSeen") : t("console2.sideUnverified")) : t("console2.sideNoPose")}
                </small>
              )}
            </button>
          );
        })}
      </div>
    </>
  );

  const mocapStatusText = busy.applyLifecycleConfig === true ? t("hopeA3Console.savingAndConnectingMocap")
    : snapshot.lifecycleResult?.includes("MOCAP_FAILED") === true ? t("hopeA3Console.mocapConnectionFailedConfigurationSavedCheckTheAddress")
      : configTouched ? t("hopeA3Console.configurationChangedNotAppliedYet")
        : confirmedSource === "none" ? t("hopeA3Console.mocapDisabled")
          : baseFresh && (confirmedSource === "vrpn" || markerFresh) ? t("hopeA3Console.mocapDataConnectedLiveDataReceived")
            : t("hopeA3Console.mocapConnectionUnconfirmedWaitingForLiveData");
  const mocapStatusTone: Tone = busy.applyLifecycleConfig === true ? "busy"
    : snapshot.lifecycleResult?.includes("MOCAP_FAILED") === true ? "bad"
      : configTouched ? "warn" : confirmedSource === "none" ? "idle"
        : baseFresh && (confirmedSource === "vrpn" || markerFresh) ? "ok" : "warn";
  const assetSummary = confirmedSource === "none" ? t("hopeA3Console.noMocapSelected")
    : confirmedSource === "vrpn" ? t("hopeA3Console.vrpnPoseOnly")
      : assetFresh ? snapshot.assetMatched === true ? t("hopeA3Console.assetAccepted") : t("hopeA3Console.checkMocapAsset")
        : t("hopeA3Console.waitingForMocap");
  const assetDetail = [
    `${t("hopeA3Console.confirmed")}${t.text(MOCAP_LABELS[confirmedSource] ?? msg("hopeA3Console.waitingForConfiguration"))}`,
    confirmedSource === "none" ? t("hopeA3Console.runnerOnlyNoMocapPipeline")
      : assetFresh ? snapshot.assetText ?? "" : t("hopeA3Console.importTheSelectedPelvisAlignedAssetAndEnable", {asset: markerAsset}),
    confirmedSource === "vrpn" ? t("hopeA3Console.vrpnStreamsPosesOnlyMarkerCountsAndImported")
      : confirmedSource === "motive" ? t("hopeA3Console.motiveNatnetUsesLiveAssetsVisibleMarkerCounts") : "",
  ].filter((line) => line !== "").join("\n");
  const mocapHelp = [
    configDraft.mocap_source === "motive" ? t("hopeA3Console.motiveSetupImportThePelvisAlignedMotiveAsset") : "",
    vrpnSelected ? t("hopeA3Console.chingmuSetup", { pants: configDraft.pants_profile, side: configDraft.table_side }) : "",
    vrpnSelected ? t("hopeA3Console.defaultStartupAcceptsPosesOnlyPublishingAValid") : "",
    configDraft.mocap_source === "none" ? t("hopeA3Console.noMocapRunnerOnlyTestingWithoutLiveBall") : "",
    t("hopeA3Console.selectWhileStoppedThenConfirmConfigOnlyThe"),
    t("hopeA3Console.confirmConfigSavesSettingsAndReconnectsTheSelected"),
    t("hopeA3Console.connectionStatusIsBasedOnLiveDataConnected"),
  ].filter((line) => line !== "").join("\n");
  const mocapSources: { value: string; label: string; sub: string }[] = [
    { value: "motive", label: t("console2.mocapMotive"), sub: t("console2.mocapMotiveSub") },
    { value: "vrpn", label: t("console2.mocapVrpn"), sub: t("console2.mocapVrpnSub") },
    { value: "none", label: t("console2.mocapNone"), sub: t("console2.mocapNoneSub") },
  ];
  const ballModels: { value: string; label: string; sub: string; title: string }[] = [
    // Berkeley (2026-10-10) is no longer offered (operator 2026-11-17); the robot side still accepts it.
    { value: "ppq002", label: "PPQ002", sub: "★", title: t("hopeA3Console.ourOneStarBallPpq002") },
    { value: "ppq004", label: "PPQ004", sub: "★★★", title: t("hopeA3Console.ourThreeStarBallPpq004UnifiedModel2026") },
  ];
  const ballModelLocked = !assetEditable || fieldBusy || kernelMode || field?.ball_model_status == undefined;
  const gripperProfiles: { value: string; label: string; sub: string; title: string }[] = [
    { value: "legacy", label: t("console2.gripperLegacy"), sub: "800 / 2000", title: t("hopeA3Console.oldGripperClose800Open2000") },
    { value: "new_20261015", label: t("console2.gripperNew"), sub: "850 / 1500", title: t("hopeA3Console.newGripperClose850Open1500") },
  ];
  // 2.1 Setup: what changes per session sits in one card (mode → serve → policy → side → ball);
  // mocap system, addresses, operator computer and gripper fold into "Mocap & hardware".
  const policyNote = (row: PolicyRow, reason: string): string =>
    reason !== "" && assetEditable && !(kernelMode && !KERNEL_MODELS.has(row.model)) ? reason
      : row.note != undefined ? t.text(row.note) : "";
  const selectedPolicyRow = displayPolicyRows.find((row) => row.model === field?.policy_model);
  const localBallLive = currentBallPlayState(localBall?.state, localBall?.receivedAt, now) != undefined;
  // Local control pairing ("本机控制授权") is a connection setting: top of Setup, not inside System health.
  const pairingCard = (
      <details className="c-more c-card c-pairing" data-card="pairing">
        <summary>
          {t("hopeA3Console.ballPairing")}
          <span className={localBallLive ? "c-ok-text" : "c-dim"}>{localBallLive ? t("console2.pairingLive") : t("console2.pairingOffline")}</span>
        </summary>
        <p className="c-hint">{t("hopeA3Console.ballPairingInstructions")}</p>
        <label className="c-file">{t("hopeA3Console.ballPairingFile")}
          <input type="file" onChange={(event) => {
            const file = event.target.files?.[0];
            event.target.value = "";
            if (file == undefined) { return; }
            if (file.size > 128) { setLocalBallPairFailed(true); return; }
            void file.text().then((text) => {
              setLocalBallKey(saveLocalControlKey(text)); setLocalBallPairFailed(false);
            }).catch(() => { setLocalBallPairFailed(true); });
          }} />
        </label>
        <div className="c-hint">{t(localBallKey ? "hopeA3Console.ballPairingLoaded" : "hopeA3Console.ballPairingMissing")}</div>
        {localBallPairFailed && <div className="c-bad-text" role="alert">{t("hopeA3Console.ballPairingFailed")}</div>}
      </details>
  );
  // Policy picker: one line for the current model; the list opens below it, scrolls past ~5 rows and gets a
  // filter once the catalog grows, so new models never widen or lengthen the Setup page.
  const policyRowView = (row: PolicyRow) => {
    const reason = policyUnavailableReason(row.model);
    const selected = field?.policy_model === row.model;
    const serves = row.model === "model17400" ? field?.policy_models?.model17400?.receive_only !== true : row.serve;
    const variant = row.model === "model14200best" && (reason === "" || selected)
      ? (bestVariant != undefined ? t("console2.policyVariant", { variant: bestVariant }) : t("console2.policyVariantUnknown")) : undefined;
    const note = variant ?? policyNote(row, reason);
    const live = selected && lifecycleState === "RUNNING";
    const state = live ? t("console2.policyLoaded")
      : kernelMode && !KERNEL_MODELS.has(row.model) ? t("console2.policyNotInKernel") : "";
    return { reason, selected, serves, note, state, live };
  };
  const policyCaps = ({ serves }: { serves: boolean }) => (
    <span className="c-caps">
      <span className={`c-cap ${serves ? "c-cap-s" : "c-cap-no"}`}>{t("console2.capServe")}</span>
      <span className="c-cap c-cap-r">{t("console2.capReceive")}</span>
    </span>
  );
  const policyQuery = policyFilter.trim().toLowerCase();
  const policyRows = displayPolicyRows
    .map((row, index) => ({ row, index, view: policyRowView(row) }))
    .filter(({ row, view }) => policyQuery === "" ||
      `${row.name} ${row.model} ${view.note}`.toLowerCase().includes(policyQuery))
    // Selectable models first, then the ones this robot cannot load; catalog order otherwise.
    .sort((x, y) => {
      const unavailable = Number(x.view.reason !== "") - Number(y.view.reason !== "");
      return unavailable !== 0 ? unavailable : x.index - y.index;
    });
  const currentPolicy = selectedPolicyRow == undefined ? undefined : policyRowView(selectedPolicyRow);
  const policyPicker = (
    <div className="c-pick">
      <button type="button" className="c-pick-cur" aria-haspopup="listbox" aria-expanded={policyOpen}
        title={selectedPolicyLabel}
        onClick={() => { setPolicyOpen((open) => !open); }}
        onKeyDown={(event) => { if (event.key === "Escape") { setPolicyOpen(false); } }}>
        <span className="c-pn">{selectedPolicyRow?.name ?? (field?.policy_model != undefined
          ? POLICY_NAMES[field.policy_model] ?? field.policy_model : t("console2.policyNone"))}</span>
        {currentPolicy != undefined ? policyCaps(currentPolicy) : <span />}
        <span className="c-pnote">{currentPolicy?.note ?? ""}</span>
        <span className={`c-pstate ${currentPolicy?.live === true ? "c-live" : ""}`}>{currentPolicy?.state ?? ""}</span>
        <span className="c-caret" aria-hidden="true">{"▾"}</span>
      </button>
      <div className="c-pick-list" role="listbox" aria-label={t("hopeA3Console.receivePolicy")} hidden={!policyOpen}
        onKeyDown={(event) => { if (event.key === "Escape") { setPolicyOpen(false); } }}>
        {VISIBLE_POLICY_ROWS.length > 6 && (
          <input className="c-inp c-pick-filter" type="search" value={policyFilter} spellCheck={false}
            placeholder={t("console2.policyFilter", { count: VISIBLE_POLICY_ROWS.length })}
            aria-label={t("console2.policyFilter", { count: VISIBLE_POLICY_ROWS.length })}
            onChange={(event) => { setPolicyFilter(event.target.value); }} />
        )}
        {policyRows.map(({ row, view }) => (
          <button key={row.model} type="button" role="option" className="c-pol" disabled={view.reason !== ""}
            title={view.reason !== "" ? view.reason : undefined}
            aria-label={row.label == undefined ? row.name : t.text(row.label)}
            aria-selected={view.selected} aria-pressed={view.selected}
            onClick={() => { setPolicyOpen(false); void fieldCommand({op: "set_policy_model", model: row.model}); }}>
            <span className="c-radio" />
            <span className="c-pn">{row.name}</span>
            {policyCaps(view)}
            <span className="c-pnote">{view.note}</span>
            <span className={`c-pstate ${view.live ? "c-live" : ""}`}>{view.state}</span>
          </button>
        ))}
        {policyRows.length === 0 && <div className="c-hint">{t("console2.policyNoMatch")}</div>}
      </div>
    </div>
  );
  const sessionCard = (
    <div className={`c-card c-session ${kernelMode ? "c-kcard" : ""}`} data-card="mocap">
      <div className="c-lbl" title={t("hopeA3Console.switchWhileStoppedThenStartToLoadEach")}>
        {t("console2.sessionSettings")}
        {!assetEditable && <span className="c-r" title={t("hopeA3Console.confirmConfigurationAndStopTheLifecycleBeforeSelecting")}>{t("console2.lockedHint")}</span>}
      </div>
      <div className="c-setgrid">
        <div className="c-k" data-card="mode">{t("console2.runMode")}</div>
        <div className="c-seg" role="group" aria-label={t("hopeA3Console.operatingMode")}>
          <button type="button" aria-pressed={field?.mode === "OPTITRACK"} disabled={!assetEditable}
            title={t("hopeA3Console.normalPlayStartsTheLaptopDataPipelineFor")}
            onClick={() => { switchMode("OPTITRACK"); }}>
            {t("console2.modeNormal")}<small>{t("console2.modeNormalSub")}</small>
          </button>
          <button type="button" className="c-kopt" aria-pressed={kernelMode} disabled={!assetEditable}
            title={t("hopeA3Console.noMocapImuStanceFootOdometryEstimatedLocal")}
            onClick={() => { switchMode("KERNEL"); }}>
            {t("console2.modeKernel")}<small>{t("console2.modeKernelSub")}</small>
          </button>
        </div>
        <div className="c-k" data-card="csv"
          title={`${t("hopeA3Console.uploadsUseTheRunnerCsvValidatorAndNever")}\n${t("hopeA3Console.selectKernelModeUploadWhileStoppedThenRun")}`}>
          {t("console2.serveCsv")}
        </div>
        <div className="c-filebox c-filebox-row">
          <span className="c-n" title={csvName}>{csvName}</span>
          <span className="c-m c-mono c-trunc" title={csvMeta}>{csvMeta}</span>
          <button className="c-btn c-sm" type="button" disabled={!assetEditable} onClick={() => csvInput.current?.click()}>{t("console2.upload")}</button>
          <button className="c-btn c-sm" type="button" disabled={!assetEditable || field?.csv == undefined}
            onClick={() => void fieldCommand({ op: "use_default_csv" })}>{t("console2.useDefault")}</button>
        </div>
        {kernelMode && <><div className="c-k" /><div className="c-grow"><CsvTimeline csv={csvStatus} kernelMode={kernelMode} /></div></>}
        <div className="c-k" data-card="policy" title={selectedPolicyLabel}>{t("console2.policy")}</div>
        {policyPicker}
        {sideRow}
        {kernelMode ? <>
          <div className="c-k">{t("console2.localization")}</div>
          <div className="c-status c-kern" title={t("hopeA3Console.noMocapImuStanceFootOdometryEstimatedLocal")}>
            <span className="c-dot c-kern" />
            <b>{t("console2.imuOdometry")}</b>
            <span>{t("console2.noMocapNoPlanner")}</span>
          </div>
        </> : <>
          <div className="c-k" title={t("hopeA3Console.selectWhileStoppedSavedToPlannerAndBall")}>{t("console2.ballModel")}</div>
          <div className="c-seg" role="group" aria-label={t("hopeA3Console.ballPhysicsModel")}
            title={t("hopeA3Console.afterSwitchingRestartLaptopMocapBallBridgeThen")}>
            {ballModels.map((model) => (
              <button key={model.value} type="button" aria-pressed={ballDisplay.value === model.value}
                disabled={ballModelLocked} title={model.title}
                onClick={() => { if (ballDisplay.value !== model.value) { void fieldCommand({ op: "set_ball_model", model: model.value }); } }}>
                {model.label}<small>{model.sub}</small>
              </button>
            ))}
          </div>
          <div className="c-k" />
          <div className={`c-status c-${ballDisplay.ready ? "ok" : "warn"}`} title={ballDisplay.text}>
            <span className={`c-dot c-${ballDisplay.ready ? "ok" : "warn"}`} />
            <span>{ballDisplay.text}</span>
          </div>
        </>}
      </div>
      <input ref={csvInput} type="file" accept=".csv,text/csv" hidden
        onChange={(event) => { void uploadAsset(event.target.files?.[0]); event.target.value = ""; }} />
      {!fieldFresh && <div className="c-why c-warn">{t("hopeA3Console.waitingForFieldControlsUpdateTheHduBackend")}</div>}
      {field?.policy_model != undefined && !OFFERED_MODELS.has(field.policy_model) &&
        <div className="c-why c-warn">{t("console2.policyOtherSelected", { name: POLICY_NAMES[field.policy_model] ?? field.policy_model })}</div>}
    </div>
  );
  // Mocap & hardware: every original control stays visible; pairs share a row (label | control | label | control)
  // and the two mocap status lines are one line, so the card is about half its old height.
  const hwCard = (
    <div className="c-card" data-card="hw">
      <div className="c-lbl" title={`${t("hopeA3Console.editAndConfirmConfigurationWhileStoppedChangingThe")}\n${mocapHelp}`}>
        {kernelMode ? t("console2.hardware") : t("console2.rig")}
      </div>
      <div className="c-grid4">
        {!kernelMode && <>
          <div className="c-k">{t("console2.mocapSystem")}</div>
          <div className="c-seg c-span3" role="group" aria-label={t("hopeA3Console.mocapInput")}>
            {mocapSources.map((source) => (
              <button key={source.value} type="button" aria-pressed={configDraft.mocap_source === source.value}
                disabled={configLocked} onClick={() => { selectDraft("mocap_source", source.value); }}>
                {source.label}<small>{source.sub}</small>
              </button>
            ))}
          </div>
          {configDraft.mocap_source === "motive" && <>
            <div className="c-k">{t("console2.motiveIp")}</div>
            <div className="c-inrow c-span3">{configInput("motive_ip")}</div>
          </>}
          {vrpnSelected && <>
            <div className="c-k">{t("console2.vrpnIp")}</div>
            <div className="c-inrow c-span3">{configInput("vrpn_ip")}{configInput("vrpn_port", { short: true, numeric: true })}</div>
            <div className="c-k">{t("console2.vrpnUnits")}</div>
            <div className="c-seg c-seg-sm" role="group" aria-label={t("hopeA3Console.vrpnUnits")} title={t("hopeA3Console.vrpnOutputUnits")}>
              {(["mm", "m"] as const).map((unit) => (
                <button key={unit} type="button" aria-pressed={configDraft.vrpn_units === unit} disabled={configLocked}
                  title={unit === "mm" ? t("hopeA3Console.millimetersMm0001M") : t("hopeA3Console.metersM10")}
                  onClick={() => { selectDraft("vrpn_units", unit); }}>{unit}</button>
              ))}
            </div>
            <div className="c-k">{t("console2.markerPants")}</div>
            <div className="c-seg" role="group" aria-label={t("hopeA3Console.markerPantsProfile")}
              title={t("hopeA3Console.selectWhileStoppedThenConfirmConfigVrpnSwitches")}>
              {(["v2", "v4"] as const).map((pants) => (
                <button key={pants} type="button" aria-pressed={configDraft.pants_profile === pants} disabled={configLocked}
                  title={pants === "v2" ? t("hopeA3Console.v2OldPants10Markers") : t("hopeA3Console.v412MarkersEachForP1P2")}
                  onClick={() => { selectDraft("pants_profile", pants); }}>
                  {pants.toUpperCase()}<small>{pants === "v2" ? t("console2.pantsV2Sub") : t("console2.pantsV4Sub")}</small>
                </button>
              ))}
            </div>
          </>}
          <div className="c-k" />
          <div className={`c-status c-span3 c-${mocapStatusTone}`} title={confirmedSource !== "none" ? assetDetail : undefined}>
            <span className={`c-dot c-${mocapStatusTone}`} />
            <b>{mocapStatusText}</b>
            {confirmedSource !== "none" && <span>{assetSummary} · {t("console2.assetLine", { asset: markerAsset,
              pelvis: baseFresh ? t("hopeA3Console.available") : t("hopeA3Console.waiting") })}</span>}
          </div>
          {vrpnSelected && <>
            <div className="c-k" />
            <div className="c-attest c-span3">
              <div className="c-hint">{t("configRuntime.attestationBinding")}</div>
              {(["vrpn_world_frame_confirmed"] as const).map((key) => (
                <label key={key}>
                  <input type="checkbox" checked={mocapBindingUnchanged && configDraft[key] === "true"}
                    disabled={!lifecycleStopped || lifecycleBusy || !mocapBindingUnchanged || !runtimeFresh}
                    onChange={(event) => {
                      setConfigTouched(true);
                      setConfigDraft((current) => ({ ...current, [key]: event.target.checked ? "true" : "false" }));
                          }} />
                  {t.text(CONFIG_FIELDS[key].label)}
                </label>
              ))}
            </div>
            <div className="c-k" />
            <div className="c-inrow c-span3">
              <span className="c-hint c-grow" role="status">{mocapRecoveryText(mocapRecovery, t)}</span>
              <button type="button" className="c-btn c-sm"
                disabled={mocapRecovery?.manual_allowed !== true || !lifecycleStopped || lifecycleBusy ||
                  fieldBusy || field?.busy === true || !(configApplied || hduConfigApplied) || configTouched ||
                  timeCalibrationBlocksStart}
                onClick={() => void fieldCommand({ op: "reconnect_mocap" })}>
                {mocapRecovery?.state === "RECONNECTING" ? t("mocapRecovery.reconnecting") : t("mocapRecovery.reconnect")}
              </button>
            </div>
          </>}
        </>}
        <div className="c-gsub">{t("console2.groupNetwork")}</div>
        <div className="c-k">{t.text(CONFIG_FIELDS.hdu_wifi_ip.label)}</div>
        <div className="c-inrow">{configInput("hdu_wifi_ip")}</div>
        <div className="c-k">{t.text(CONFIG_FIELDS.laptop_wifi_ip.label)}</div>
        <div className="c-inrow">{configInput("laptop_wifi_ip")}</div>
        <div className="c-k">{t.text(CONFIG_FIELDS.mdu_internal_ip.label)}</div>
        <div className="c-inrow">{configInput("mdu_internal_ip")}</div>
        <div className="c-inrow c-span2 c-migrate">
          <button type="button" className="c-btn c-sm" title={t("connection.repairHint")}
            disabled={!lifecycleStopped || lifecycleBusy || !configHostFresh ||
            !snapshot.configRuntime?.robotIdentity?.connection || fieldBusy || recordBusy}
            onClick={() => {
              const pair = snapshot.configRuntime?.robotIdentity?.connection;
              if (pair) { setConfigTouched(true); setConfigDraft((current) => ({ ...current,
                hdu_wifi_ip: pair.hdu_ip, laptop_wifi_ip: pair.laptop_ip })); }
            }}>{t("connection.useObservedPair")}</button>
        </div>
        <div className="c-k" />
        <div className="c-hint c-span3" title={t("connection.repairHint")}>{t("connection.repairHintShort")}</div>
        <div className="c-gsub">{t("console2.devices")}</div>
        {snapshot.lifecycleConfig.operator_profile != undefined && <>
          <div className="c-k" title={t("hopeA3Console.chooseTheComputerForThisSessionThenConfirm")}>{t("console2.operatorComputer")}</div>
          <div className="c-seg c-seg-sm" role="group" aria-label={t("hopeA3Console.operatorComputer")}>
            {(["linux", "mac"] as const).map((profile) => (
              <button key={profile} type="button"
                aria-pressed={configDraft.operator_profile === profile}
                disabled={configLocked}
                onClick={() => {
                  setConfigTouched(true);
                  setConfigDraft((current) => ({ ...current, operator_profile: profile }));
                  }}>
                {profile === "mac" ? t("hopeA3Console.mac") : t("hopeA3Console.linux")}
              </button>
            ))}
          </div>
        </>}
        <div className="c-k" title={t("hopeA3Console.manualOpenCloseAndServeReleaseChangesTake")}>{t("console2.gripper")}</div>
        <div className="c-seg" role="group" aria-label={t("hopeA3Console.gripperProfile")}>
          {gripperProfiles.map((profile) => (
            <button key={profile.value} type="button" aria-pressed={configDraft.gripper_profile === profile.value}
              disabled={configLocked} title={profile.title}
              onClick={() => { selectDraft("gripper_profile", profile.value); }}>
              {profile.label}<small>{profile.sub}</small>
            </button>
          ))}
        </div>
      </div>
      {!kernelMode && lifecycleFresh && setupCommand && <details className="c-more">
        <summary>{t("hopeA3Console.manualStartupCommandFallback")}</summary>
        <textarea aria-label={t("hopeA3Console.confirmedMocapStartupCommand")} readOnly rows={5} value={setupCommand} />
      </details>}
    </div>
  );

  // ---- play ----
  const plannerFresh = isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500);
  const canSwitchRole = (role: "RECEIVER" | "SERVER") => snapshot.roleChangeAllowed === true &&
    busy[role === "RECEIVER" ? "setReceiver" : "setServer"] !== true;
  // Ready / Start to Serve keep every 1.9 condition; only "our role is X" also admits
  // "the Runner allows switching to X", which the click performs first.
  const receiveRoleOk = snapshot.localRole === "RECEIVER" || canSwitchRole("RECEIVER");
  const raiseRoleOk = snapshot.localRole === "SERVER" || canSwitchRole("SERVER");
  const receiveDisabled = gates.readyExceptRole || !receiveRoleOk;
  const raiseDisabled = gates.prepareServeExceptRole || !raiseRoleOk;
  const receiveBusy = busy.ready === true || roleSwitchFor === "ready";
  const raiseBusy = busy.prepareServe === true || roleSwitchFor === "prepareServe";
  const offlineReason = !runnerFresh ? t("console2.reasonRunnerOffline")
    : snapshot.runnerFault === true ? t("console2.reasonRunnerFault") : undefined;
  const roleReason = (role: "RECEIVER" | "SERVER") =>
    busy[role === "RECEIVER" ? "setReceiver" : "setServer"] === true ? t("console2.reasonRoleSwitching") : t("console2.reasonRoleLocked");
  const teleopActive = runnerFresh && snapshot.runnerMode === "TELEOP";
  const walkStandActive = runnerFresh && snapshot.runnerMode === "WALK_STAND";
  const dockActive = runnerFresh && snapshot.runnerMode === "DOCK";
  const currentNode: NodeId | undefined = !runnerFresh ? undefined
    : snapshot.runnerMode === "PASSIVE" ? "passive"
      : serveState === "SERVING" || snapshot.serving === true ? "serve"
        : serveState === "PREPARING" || serveState === "WAIT_READY_TO_SERVE" ? "raise"
          // MOTION only shows Ready as reached while the pelvis is fresh (fails closed with the Runner).
          : snapshot.runnerMode === "MOTION" ? (gates.pelvisOk ? "receive" : undefined)
            : snapshot.runnerMode === "DOCK" ? "reposition"
            : snapshot.runnerMode === "PD_STAND" || snapshot.runnerMode === "WALK_STAND" ? "stand"
              : snapshot.runnerMode === "SERVE" ? "serve" : undefined;
  const graphNodes: GraphNodeProps[] = [
    { id: "passive", label: t("console2.nodePassive"), sub: t("console2.subZeroGains"), kernelMode,
      current: currentNode === "passive", disabled: gates.passiveBlocked, busy: busy.passive === true,
      reason: !runnerFresh ? t("console2.reasonRunnerOffline") : undefined, tone: "danger", shortcut: "passive",
      onClick: () => void invoke("passive") },
    { id: "stand", label: t("console2.nodeStand"), sub: teleopActive ? t("console2.subTeleopExit") : walkStandActive ? t("console2.subWalkStand")
      : dockActive ? t("console2.subDockExit")
        : runnerFresh && snapshot.runnerMode === "PD_STAND" ? t("console2.subPdStandToWalk") : t("console2.subInit"), kernelMode,
      current: currentNode === "stand", disabled: gates.standBlocked, busy: busy.stand === true,
      reason: offlineReason, shortcut: "stand", onClick: () => void invoke("stand") },
    { id: "receive",
      label: kernelMode ? (currentNode === "receive" ? t("console2.nodeReceiveTestNow") : t("console2.nodeReceiveTest"))
        : currentNode === "receive" ? t("console2.nodeReceiveNow") : t("console2.nodeReceive"),
      sub: !receiveDisabled && snapshot.localRole !== "RECEIVER" ? t("console2.subSwitchToReceiver") : kernelMode ? "IMU · MOTION" : "MOTION",
      kernelMode, current: currentNode === "receive", disabled: receiveDisabled, busy: receiveBusy,
      reason: offlineReason ?? (!runnerStanding ? t("console2.reasonStandFirst") : !receiveRoleOk ? roleReason("RECEIVER") : undefined),
      shortcut: "ready",
      onClick: () => {
        if (snapshot.localRole === "RECEIVER") { void invoke("ready"); return; }
        void switchRoleThen("RECEIVER", "ready", () => { void invoke("ready"); });
      } },
    { id: "raise", label: t("console2.nodeRaise"),
      sub: !raiseDisabled && snapshot.localRole !== "SERVER" ? t("console2.subSwitchToServer") : "WAIT_READY", kernelMode,
      current: currentNode === "raise", disabled: raiseDisabled, busy: raiseBusy,
      reason: offlineReason ?? (!gates.serveAvailable ? t("console2.reasonServeUnavailable")
        : !raiseRoleOk ? roleReason("SERVER")
          : !["PD_STAND", "WALK_STAND", "MOTION"].includes(snapshot.runnerMode ?? "") ? t("console2.reasonStandFirst")
            : t("console2.reasonServeState", { state: serveState })),
      tone: "serve", shortcut: "serve",
      onClick: () => {
        if (snapshot.localRole === "SERVER") { invokeServe("prepareServe"); return; }
        void switchRoleThen("SERVER", "prepareServe", () => { invokeServe("prepareServe"); });
      } },
    { id: "serve",
      label: currentNode === "serve" ? t("console2.nodeServing") : kernelMode ? t("console2.nodeTestServe") : t("console2.nodeServe"),
      sub: kernelMode ? "CSV" : "SERVING", kernelMode,
      current: currentNode === "serve", disabled: gates.readyToServeBlocked, busy: busy.readyToServe === true,
      reason: offlineReason ?? (snapshot.localRole !== "SERVER" ? t("console2.reasonServerOnly") : t("console2.reasonRaiseFirst")),
      tone: "serve", shortcut: "serve", onClick: () => { invokeServe("readyToServe"); } },
  ];
  if (!kernelMode) {
    const repositionReason = offlineReason
      ?? (!dockFresh ? t("console2.reasonNoReposition")
        : snapshot.serving === true ? t("console2.reasonServing")
          : !["PD_STAND", "WALK_STAND", "TELEOP", "MOTION"].includes(snapshot.runnerMode ?? "") ? t("console2.reasonStandFirst")
            : !baseFresh ? t("console2.reasonWaitMocap") : undefined);
    graphNodes.push({ id: "reposition",
      label: dockActive ? t("console2.nodeRepositionNow") : t("console2.reposition"),
      sub: dockActive ? `${dockPhaseName(dockState) ?? "…"} · ${(dockState?.[7] ?? 0).toFixed(2)} m`
        : returnPending || snapshot.runnerMode === "MOTION" ? t("console2.returnToReceive") : t("console2.subReposition"),
      kernelMode, current: currentNode === "reposition",
      disabled: dockActive || dockBlocked != undefined, busy: busy.dock === true,
      reason: dockActive ? undefined : repositionReason ?? dockBlocked, shortcut: "dock",
      onClick: () => { void requestReposition(); } });
  }
  const nodeOpen = Object.fromEntries(graphNodes.map((node) => [node.id, !node.disabled && !node.busy])) as Record<NodeId, boolean>;
  const edgeFrom: NodeId | undefined = currentNode ?? (teleopActive ? "stand" : undefined);
  const edgeState = (from: NodeId, to: NodeId): EdgeState =>
    edgeFrom === from && nodeOpen[to] ? (to === "raise" || to === "serve" ? "livew" : "live") : "";
  const mocapSafetyLine = mocapSafetyText(
    snapshot.runnerAlive === true && isFresh(snapshot, TOPICS.runnerAlive, now, 1_000) &&
    isFresh(snapshot, TOPICS.runnerBoot, now, 1_000) &&
    isFresh(snapshot, TOPICS.mocapSafety, now, 1_000) &&
    snapshot.mocapSafety?.boot === snapshot.runnerBoot
      ? snapshot.mocapSafety : undefined, t);
  const gripperDetail = serveState === "CLEANUP_OPENING"
    ? t("hopeA3Console.requestedOpenPublishInProgress")
    : snapshot.serveCleanupRequired === true
      ? t("hopeA3Console.retainedBallTelemetryDoesNotGateAnyAction")
      : t("hopeA3Console.manualCommandGripper", {gripper: snapshot.gripperState ?? t("hopeA3Console.unavailable")});
  const teleopPhase = teleop == undefined ? undefined
    : [t("hopeA3Console.off"), t("hopeA3Console.entering"), t("hopeA3Console.active"), t("hopeA3Console.stopping")][teleop[3] ?? 0];
  const controllerName = snapshot.xboxPreview?.device === "" ? t("hopeA3Console.noControllerDetected")
    : (snapshot.xboxPreview?.device ?? t("hopeA3Console.noControllerDetected"));
  const controllerLive = xboxFresh && snapshot.xboxPreview?.connected === true;
  const teleopLine = !xboxFresh ? t("hopeA3Console.noFreshXboxInput")
    : snapshot.xboxPreview?.connected !== true ? t("hopeA3Console.disconnected")
      : teleopActive ? velocityText(teleop?.slice(6, 9))
        : teleopBlocked == undefined ? t("console2.pressY") : t("console2.pressYBlocked", { reason: teleopBlocked });
  const teleopDetails = [
    `${t("hopeA3Console.controller")}: ${controllerName}`,
    `${t("hopeA3Console.input")}: ${!xboxFresh ? t("hopeA3Console.noFreshXboxInput") : snapshot.xboxPreview?.connected !== true
      ? t("hopeA3Console.disconnected") : t("hopeA3Console.liveLt", {lt: snapshot.xboxPreview.lt ? t("hopeA3Console.held") : t("hopeA3Console.released"), source: snapshot.xboxPreview.state})}`,
    `${t("hopeA3Console.sticks")}: ${velocityText(xboxVelocity)}`,
    `${t("hopeA3Console.runner")}: ${!runnerFresh ? t("hopeA3Console.noFreshRunnerStatus") : teleop == undefined
      ? t("hopeA3Console.waitingForTeleopTelemetry") : teleop[2] !== 1 ? t("hopeA3Console.teleopPolicyNotLoaded")
        : t("hopeA3Console.enableTransition", {mode: snapshot.runnerMode ?? t("hopeA3Console.unknown"), phase: teleopPhase ?? t("hopeA3Console.unknown"), enabled: teleop[9] === 1 ? t("hopeA3Console.on") : t("hopeA3Console.off"), progress: ((teleop[5] ?? 0) * 100).toFixed(0)})}`,
    `${t("hopeA3Console.command")}: ${velocityText(teleop?.slice(6, 9))}`,
    `${t("hopeA3Console.modeKeys")}: ${xboxFresh ? (snapshot.xboxPreview?.action_status ?? t("hopeA3Console.aPrepareBServeXReceiveYTeleop")) : t("hopeA3Console.waitingForXboxInput")}`,
    `${t("hopeA3Console.inputAge")}: ${teleop == undefined || (teleop[4] ?? 1e9) >= 1e9
      ? t("hopeA3Console.noInputReceivedByRunner") : `${Math.min(9999, (teleop[4] ?? 9999) * 1000).toFixed(0)} ms`}`,
    teleopBlocked ?? t("hopeA3Console.readyToRequestTeleopRunnerValidatesCurrentInput"),
  ].join("\n");
  const teleBar = (
    <div className={`c-card c-tele ${teleopActive ? "c-tele-on" : ""}`} title={t("hopeA3Console.waitForActiveCenterTheSticksThenRelease")}>
      <span className={`c-dot c-${!controllerLive ? "bad" : teleopActive ? "ok" : "idle"}`} />
      <div className="c-tele-text">
        <b>
          {t("console2.teleop")}
          <span className="c-tele-dev">{` · ${controllerLive ? controllerName : "—"}`}</span>
          {teleopActive && <span className="c-accent">{` · ${teleopPhase ?? t("console2.teleopActive")}`}</span>}
        </b>
        <span title={teleopDetails}>{teleopLine}</span>
        {xboxFresh && snapshot.xboxPreview?.action_pending === true && snapshot.xboxPreview.action_status?.startsWith("Y") === true
          && <small className="c-accent">{t("console2.teleYPending")}</small>}
        {xboxFresh && snapshot.xboxPreview?.action_status != undefined && snapshot.xboxPreview.action_pending !== true &&
          <small className="c-hint c-trunc" title={snapshot.xboxPreview.action_status}>
            {t("console2.telePadSays", { status: snapshot.xboxPreview.action_status })}</small>}
      </div>
      {teleopActive ? (
        <button type="button" className="c-btn c-sm" disabled={gates.teleopStandBlocked || busy.stand === true}
          onClick={() => void invoke("stand")}>{t("console2.teleopStop")}</button>
      ) : (
        <span className={`c-ybtn ${teleopBlocked == undefined ? "" : "c-ybtn-off"}`} aria-hidden="true">{"Y"}</span>
      )}
    </div>
  );
  // Teleop panel: appears only after Y (Runner in TELEOP). Phase steps, one instruction for the current phase,
  // live LT / sticks / pressed keys, and a key map that flags A / X (they leave Teleop) (operator 2026-11-17).
  const pad = xboxFresh ? snapshot.xboxPreview : undefined;
  const padButtons = Array.isArray(pad?.buttons) ? pad.buttons.filter((b): b is string => typeof b === "string") : [];
  const padPressed = new Set<string>([...padButtons, ...(pad?.lb === true ? ["LB"] : []), ...(pad?.rb === true ? ["RB"] : []),
    ...(pad?.lt === true ? ["LT"] : [])]);
  const telePhase = teleop?.[3] ?? 0;
  const teleMoving = teleop?.[9] === 1 || pad?.lt === true;
  const teleRiskyKey = ["A", "X"].find((key) => padPressed.has(key));
  const teleGuide = teleRiskyKey != undefined
    ? t("console2.teleRisky", { key: teleRiskyKey, effect: teleRiskyKey === "A" ? t("console2.teleEffectA") : t("console2.teleEffectX") })
    : telePhase === 1 ? t("console2.teleGuideEntering")
      : telePhase === 3 ? t("console2.teleGuideStopping")
        : telePhase === 2 ? (teleMoving ? t("console2.teleGuideMoving") : t("console2.teleGuideHoldLt"))
          : t("console2.teleGuideWaiting");
  const teleAxes: { label: string; input: number; command?: number; unit: string }[] = [
    { label: t("console2.teleAxisVx"), input: pad?.axes[0] ?? 0, command: teleop?.[6], unit: "m/s" },
    { label: t("console2.teleAxisVy"), input: pad?.axes[1] ?? 0, command: teleop?.[7], unit: "m/s" },
    { label: t("console2.teleAxisWz"), input: pad?.axes[2] ?? 0, command: teleop?.[8], unit: "rad/s" },
  ];
  const teleKeys: { key: string; label: string; tone: Tone; pressedBy: string[] }[] = [
    { key: "LT", label: t("console2.teleKeyLt"), tone: "ok", pressedBy: ["LT"] },
    { key: "LS", label: t("console2.teleKeyLeft"), tone: "ok", pressedBy: [] },
    { key: "RS", label: t("console2.teleKeyRight"), tone: "ok", pressedBy: [] },
    { key: "A", label: t("console2.teleKeyA"), tone: "warn", pressedBy: ["A"] },
    { key: "X", label: t("console2.teleKeyX"), tone: "warn", pressedBy: ["X"] },
    { key: "B", label: t("console2.teleKeyB"), tone: "idle", pressedBy: ["B"] },
    { key: "Y", label: t("console2.teleKeyY"), tone: "idle", pressedBy: ["Y"] },
    { key: "LB+RB", label: t("console2.teleKeyEstop"), tone: "bad", pressedBy: ["LB", "RB"] },
  ];
  const teleInputAge = teleop == undefined || (teleop[4] ?? 1e9) >= 1e9 ? t("hopeA3Console.noInputReceivedByRunner")
    : `${Math.min(9999, (teleop[4] ?? 9999) * 1000).toFixed(0)} ms`;
  const telePanel = (
    <div className={`c-card c-tele-panel ${teleRiskyKey != undefined ? "c-tele-risky" : ""}`} data-card="teleop"
      title={teleopDetails}>
      <div className="c-tele-head">
        <span className={`c-dot c-${telePhase === 2 ? "ok" : "busy"}`} />
        <b>{t("console2.teleop")}</b>
        <span className="c-tele-dev">{controllerLive ? controllerName : "—"}</span>
        <span className="c-sp" />
        <button type="button" className="c-btn c-sm" disabled={gates.teleopStandBlocked || busy.stand === true}
          onClick={() => void invoke("stand")}>{t("console2.teleopStop")}</button>
      </div>
      <div className="c-tele-steps">
        {[1, 2, 3].map((phase) => (
          <span key={phase} className={`c-tstep ${phase === telePhase ? "c-tstep-on" : ""} ${phase < telePhase ? "c-tstep-done" : ""}`}>
            {[t("console2.teleStepEntering"), t("console2.teleStepActive"), t("console2.teleStepStopping")][phase - 1]}
          </span>
        ))}
      </div>
      <div className={`c-tele-guide ${teleRiskyKey != undefined ? "c-tele-guide-bad" : teleMoving && telePhase === 2 ? "c-tele-guide-ok" : ""}`}>
        {teleGuide}
      </div>
      <div className="c-tele-live">
        <div className={`c-ltbox ${pad?.lt === true ? "c-ltbox-on" : ""}`}>
          <b>{"LT"}</b>
          <span>{pad == undefined ? t("hopeA3Console.noFreshXboxInput") : pad.lt ? t("console2.teleLtHeld") : t("console2.teleLtReleased")}</span>
        </div>
        <div className="c-axes">
          {teleAxes.map((axis) => (
            <div key={axis.label} className="c-axis">
              <span className="c-axis-k">{axis.label}</span>
              <span className="c-axis-bar">
                <i style={{ left: `${50 + Math.min(0, Math.max(-1, axis.input)) * 50}%`,
                  width: `${Math.abs(Math.max(-1, Math.min(1, axis.input))) * 50}%` }} />
              </span>
              <span className="c-axis-v c-mono">{axis.command == undefined ? "—" : axis.command.toFixed(2)} <small>{axis.unit}</small></span>
            </div>
          ))}
        </div>
      </div>
      <div className="c-tele-keys">
        {teleKeys.map((key) => (
          <span key={key.key} className={`c-tkey c-tkey-${key.tone} ${key.pressedBy.length > 0 && key.pressedBy.every((b) => padPressed.has(b)) ? "c-tkey-pressed" : ""}`}>
            <kbd>{key.key}</kbd>{key.label}
          </span>
        ))}
      </div>
      <div className="c-tele-foot c-mono c-dim c-trunc">
        {t("console2.teleInputAge", { age: teleInputAge })}
        {pad?.action_status != undefined ? ` · ${t("console2.telePadSays", { status: pad.action_status })}` : ""}
      </div>
    </div>
  );
  const walkPhase = walkStandPhase(walkStandActive ? snapshot.runnerMode : undefined, teleop);
  const walkPhaseText = walkPhase == undefined ? undefined : {
    entering: t("console2.walkStandEntering"), active: t("console2.walkStandActive"),
    stopping: t("console2.walkStandStopping"), waiting: t("console2.walkStandWaiting"),
  }[walkPhase];
  const walkStatusText = dockActive ? `${t("console2.reposition")} ${dockPhaseName(dockState) ?? ""}`
    : walkStandActive ? (walkPhaseText ?? t("console2.walkStandActive"))
      : runnerFresh && snapshot.runnerMode === "PD_STAND" ? t("console2.subPdStandToWalk")
        : runnerFresh ? snapshot.runnerMode ?? "—" : "—";
  const dockTuningEcho = decodeDockTuning(dockState);
  // One dropdown like Hit tuning: Walk stand status + Reposition in the summary,
  // Reposition status, target and speed limits in the body.
  const walkStandBar = (
    <details className={`c-card c-tune ${walkStandActive || dockActive ? "c-tele-on" : ""}`}>
      <summary title={t("console2.walkStandHint")}>
        <span className={`c-dot c-${walkStandActive || dockActive ? "ok" : "idle"}`} />
        <b>{t("console2.walkStand")}</b>
        <span className="c-tune-vals">
          <span>{returnPending ? t("console2.returnToReceive") : walkStatusText}</span>
          {!kernelMode && <span>{dockTuningEcho == undefined ? "x — · y —"
            : `x ${dockTuningEcho.values.targetX.toFixed(2)} · y ${dockTuningEcho.values.targetY.toFixed(2)}`}</span>}
        </span>
        {!kernelMode && <button type="button" className="c-btn c-sm" data-runner-shortcut="restoreArms"
          title={t("console2.restoreArmsHelp")} aria-label={t("console2.restoreArmsHelp")}
          disabled={!runnerFresh || !isFresh(snapshot, TOPICS.runnerMode, now, 1000) ||
            !isFresh(snapshot, TOPICS.runnerFault, now, 1000) || snapshot.runnerFault !== false ||
            !["PD_STAND", "WALK_STAND", "TELEOP", "DOCK"].includes(snapshot.runnerMode ?? "") || lifecycleState !== "RUNNING" ||
            lifecycleBusy || estopAsserted || !!pendingTarget || busy.restoreArms === true}
          onClick={(event) => { event.preventDefault(); void invoke("restoreArms"); }}>{t("console2.restoreArms")} <kbd>{RUNNER_SHORTCUT_KEYS.restoreArms}</kbd></button>}
        {!kernelMode && (dockActive || returnPending ? (
          <button type="button" className="c-btn c-sm" disabled={busy.stand === true}
            onClick={(event) => { event.preventDefault(); void invoke("stand"); }}>{t("console2.repositionStop")}</button>
        ) : (
          <button type="button" className="c-btn c-sm" title={dockBlocked ?? undefined}
            disabled={dockBlocked != undefined || pendingTarget !== ""}
            onClick={(event) => { event.preventDefault(); void requestReposition(); }}>{t("console2.reposition")}</button>
        ))}
      </summary>
      <div className="c-tune-body">
        <div className="c-hint">{walkStandActive ? t("console2.walkStandOn")
          : runnerFresh && snapshot.runnerMode === "PD_STAND" ? t("console2.walkStandPdStand")
            : t("console2.walkStandOff", { mode: runnerFresh ? snapshot.runnerMode ?? "—" : "—" })}</div>
        {!kernelMode && <div className="c-hint" title={dockBlocked ?? undefined}>
          {`${t("console2.reposition")}: ${dockFresh || dockActive ? formatDockStatus(dockState, t)
            : dockBlocked ?? formatDockStatus(undefined, t)}`}</div>}
        {!kernelMode && <DockTuningCard key={dockState?.[1] ?? "offline"} embedded context={context} echo={dockTuningEcho}
          boot={dockState?.[1]} blocked={dockActive ? t("dockTuning.busy") : undefined} />}
      </div>
    </details>
  );
  const tuneCard = (
    <details className="c-card c-tune" hidden={kernelMode}>
      <summary>
        <b>{t("console2.hitTuning")}</b>
        <span className="c-tune-vals">
          <span>{t("console2.tuneScale", { scale: plannerFresh && snapshot.hitSpeedScale != undefined ? String(snapshot.hitSpeedScale.scale) : "—" })}</span>
          <span>{plannerFresh && snapshot.planeMode != undefined ? t.text(PLANE_MODE_LABELS[snapshot.planeMode.requested]) : "—"}</span>
          <span>{plannerFresh && snapshot.ballYFilter != undefined
            ? (snapshot.ballYFilter.enabled ? t("console2.tuneYOn") : t("console2.tuneYOff")) : "—"}</span>
        </span>
      </summary>
      <div className="c-tune-body">
        <HitSpeedScaleControls key={configHost}
          state={isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) ? snapshot.hitSpeedScale : undefined}
          available={!kernelMode && context.callService != undefined &&
            isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) && snapshot.hitSpeedScale?.configurable === true}
          kernelMode={kernelMode} onApply={applyHitSpeedScale}
        />
        <FlightTimeControls
          key={`flight-${configHost ?? ""}`}
          state={isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) ? snapshot.flightTime : undefined}
          available={!kernelMode && context.callService != undefined &&
            isFresh(snapshot, TOPICS.plannerDiagnostics, now, 2_500) && snapshot.flightTime?.configurable === true}
          kernelMode={kernelMode} onApply={applyFlightTime}
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
    </details>
  );

  // ---- diagnostics ----

  const procValues = [pmFresh ? snapshot.agibotPm : undefined, hduFresh ? snapshot.hduActive : undefined,
    mduFresh ? snapshot.mduActive : undefined];
  // The MDU Runner only has to run while the lifecycle is RUNNING; stopped is normal otherwise.
  const runnerExpected = lifecycleState === "RUNNING";
  const procDown = procValues[0] === false || procValues[1] === false || (runnerExpected && procValues[2] === false);
  const sysTone: Tone = procDown || recordingState === "ERROR" ? "bad"
    : estopAsserted ? "warn"
      : procValues[0] === true && procValues[1] === true && (!runnerExpected || procValues[2] === true) ? "ok" : "idle";
  const ntpText = ntpFresh && snapshot.ntpOffsetMs != undefined ? snapshot.ntpOffsetMs.toFixed(2) : "—";
  const latencyText = latencyFresh && snapshot.latencyMs != undefined ? snapshot.latencyMs.toFixed(1) : "—";
  const cpuText = cpuFresh && snapshot.cpuPercent != undefined ? snapshot.cpuPercent.toFixed(0) : "—";
  // Play-tab System health (former Health tab): clock, latency, CPU and processes stay visible;
  // E-stop backend, config runtime, recording, ball pairing and the activity log sit under "More".
  const diagCard = (
    <div className={`c-card c-diag ${sysTone === "bad" ? "c-diag-bad" : ""}`} data-card="diag">
      <div className="c-lbl">
        <span className={`c-dot c-${sysTone}`} />
        {t("console2.sysHealth")}
        <span className="c-r c-mono c-trunc" title={TOPICS.cpuTopProcess}>{snapshot.cpuTopProcess ?? t("hopeA3Console.topProcessWarmingUp")}</span>
      </div>
      <div className="c-dtiles">
        <div className="c-tile" title={`${TOPICS.ntpOffset}${snapshot.ntpDispersionMs != undefined ? t("hopeA3Console.disp", {dispersion: snapshot.ntpDispersionMs.toFixed(2)}) : ""}${snapshot.ntpSkewPpm != undefined ? t("hopeA3Console.skewPpmAuditOnly", {skew: snapshot.ntpSkewPpm.toFixed(2)}) : ""}${ntpFresh ? "" : t("hopeA3Console.stale")}`}>
          <span className="c-k">{t("console2.tileNtp")}</span>
          <span className={`c-v ${ntpFresh ? "" : "c-stale"}`}>{ntpText}<small>{t("hopeA3Console.ms")}</small></span>
        </div>
        <div className="c-tile" title={`${TOPICS.latency}${latencyFresh ? "" : t("hopeA3Console.stale")}`}>
          <span className="c-k">{t("console2.tileLatency")}</span>
          <span className={`c-v ${latencyFresh ? "" : "c-stale"}`}>{latencyText}<small>{t("hopeA3Console.ms")}</small></span>
        </div>
        <div className="c-tile c-tile-cpu" title={`${TOPICS.cpu} · ${t("console2.cpuChart")}`}>
          <span className="c-k">{t("console2.cpuChart")}</span>
          <span className={`c-v ${cpuFresh ? "" : "c-stale"}`}>{cpuText}<small>{"%"}</small></span>
          <svg className="c-spark" viewBox="0 0 600 160" preserveAspectRatio="none" aria-label={t("hopeA3Console.cpuLoadOver120Seconds")}>
            <line x1="0" y1="40" x2="600" y2="40" />
            <line x1="0" y1="80" x2="600" y2="80" />
            <line x1="0" y1="120" x2="600" y2="120" />
            {cpuPoints.length > 0 && <polyline className="c-spark-line" points={cpuPoints} />}
          </svg>
        </div>
      </div>
      <div className="c-proc c-gap-top" aria-label={t("console2.processes")}>
        <ProcessPill name="agibot_pm" topic={TOPICS.agibotPm} value={procValues[0]} />
        <ProcessPill name="HDU" topic={TOPICS.hduActive} value={procValues[1]} />
        <ProcessPill name={t("hopeA3Console.runnerProcess")} topic={TOPICS.mduActive} value={procValues[2]}
          optional={!runnerExpected} />
      </div>
      <details className="c-more c-diag-more" open={diagOpen}
        onToggle={(event) => { setDiagOpen(event.currentTarget.open); }}>
        <summary>
          {t("console2.diagMore")}
        </summary>
        <div className="c-diag-body">
          <div className="c-dsec">
            <div className="c-lbl">{t("console2.activity")}</div>
            <div className="c-log">
              {noticeLog.map((entry) => (
                <div key={entry.id}><time>{clock(entry.at)}</time><span>{t.text(entry.text)}</span></div>
              ))}
            </div>
          </div>
          <div className="c-mono c-dim">
            {t("hopeA3Console.version", { version: extensionVersion })} · {context.dataSourceProfile?.toUpperCase() ?? t("hopeA3Console.ros2")} {t("hopeA3Console.control8766")}
          </div>
        </div>
      </details>
    </div>
  );
  const confirmEnabled = confirmation != undefined && confirmationReason == undefined && !pendingTarget &&
    busy[confirmation.key] !== true;
  const confirmTone = confirmation?.key === "killAllAndCollect" ? "c-danger"
    : confirmation?.key === "startLifecycle" ? (kernelMode ? "c-gok" : "c-go")
      : confirmation?.key === "openGripper" ? "c-warnb" : "c-primary";

  return (
    <LanguageContext.Provider value={t}>
    <div className={`hope-console ${kernelMode ? "c-kernel" : ""}`} tabIndex={0} ref={rootRef}>
      <div className="c-scroll">
        <header className={`c-head ${kernelMode ? "c-head-kernel" : ""}`}>
          <div className="c-row1">
            <div className="c-statusblock">
              <div className="c-conn">
                <span className="c-ip" title={`${t("hopeA3Console.version", { version: extensionVersion })} · ${context.dataSourceProfile?.toUpperCase() ?? t("hopeA3Console.ros2")} ${t("hopeA3Console.control8766")}\n${connectionStatus}`}>
                  <span className={`c-dot c-${connectionTone}`} />
                  <span className="c-mono">{shownIp}</span>
                  <span className="c-dim c-mono">{":8766"}</span>
                </span>
                <button type="button" className="c-ghost" aria-expanded={switchOpen || switching}
                  onClick={() => { setSwitchOpen((open) => !open); }}>{t("console2.switchRobot")}</button>
                <div className="language-switch" role="group" aria-label={t("language.label")}>
                  {(["zh", "en"] as const).map((language) => <button key={language} type="button"
                    aria-pressed={locale === language} onClick={() => {
                      setLocale(language);
                      savedPanel.current = { ...savedPanel.current, language };
                      context.saveState(savedPanel.current);
                    }}>{t(language === "zh" ? "language.chinese" : "language.english")}</button>)}
                </div>
              </div>
              <fieldset className="robot-controls" disabled={switching}>
                <div className="c-pills">
                  <span className={`c-pill c-${lifecycleTone}`} title={`${t("console2.pillLifecycle")} · ${snapshot.lifecycleStep ?? ""}`}>
                    <small>{kernelMode ? t("console2.modeKernelShort") : t("console2.modePlayShort")}</small>{lifecycleLabel}
                  </span>
                  <span className={`c-pill c-${runnerTone}`} title={runnerSummaryText}>
                    <small>{t("hopeA3Console.runner")}</small>{runnerLabel}
                  </span>
                  <span className="c-pill" title={`${t("console2.pillPolicyRole")}\n${selectedPolicyLabel}`}>
                    <small>{policyName}</small>{roleLabel}
                  </span>
                  <button type="button" role="switch" aria-checked={recordingActive}
                    className={`c-pill c-rec ${recordingActive ? "c-rec-on" : ""} ${recordingState === "ERROR" ? "c-bad" : ""}`}
                    disabled={!fieldFresh || recordBusy || recordingState === "STOPPING"}
                    title={recordTitle}
                    onClick={() => void fieldCommand({ op: recordingActive ? "record_off" : "record_on" })}>
                    <span className="c-dot" />
                    {recordBusy ? t("console2.recordUpdating") : recordingActive
                      ? (recordingState === "ON" ? t("console2.recordOn") : recordingState) : t("console2.recordOff")}
                  </button>
                </div>
              </fieldset>
            </div>
            <fieldset className="robot-controls" disabled={switching}>
              {/* Reset software E-stop sits next to the E-stop (operator request 2026-11-17). */}
              <button
                className={`c-estop-reset ${estopAsserted ? "c-estop-reset-hot" : ""}`}
                type="button"
                aria-label={t("console2.resetSoftwareEstop")}
                disabled={busy.estop === true || busy.resetSoftwareEstop === true}
                onClick={() => void invoke("resetSoftwareEstop")}
                title={t("hopeA3Console.clearsTheSoftwareLatchOnlyDoesNotRelease")}
              >
                {busy.resetSoftwareEstop === true ? t("hopeA3Console.resetting") : t("console2.resetShort")}
                <small>{t("console2.resetShortSub")}</small>
              </button>
              <button
                className={`c-estop ${estopAsserted ? "c-estop-latched" : ""}`}
                type="button"
                disabled={busy.estop === true}
                title={estopTitle}
                onClick={() => void invoke("estop")}
              >
                {busy.estop === true ? t("console2.estopAsserting") : estopAsserted ? t("console2.estopLatched") : t("console2.estop")}
                <small>{estopSubtitle}</small>
              </button>
            </fieldset>
          </div>
          <div className="robot-connection c-connedit" hidden={!switchOpen && !switching}>
            <label>{t("hopeA3Console.connectionTargetIp")} <input className="c-inp" aria-label={t("hopeA3Console.robotConnectionIp")} value={robotTarget}
              onChange={(event) => { setRobotTarget(event.target.value); setTargetEdited(true); }} /></label>
            <button type="button" className="c-btn c-sm c-primary" disabled={!validRobotTarget || Object.values(busy).some(Boolean) || fieldBusy || recordBusy}
              onClick={connectRobot}>{t(configHostFresh && configHost && robotTarget.trim() !== configHost
                ? "connection.directOtherRobot" : "hopeA3Console.confirmConnection")}</button>
            <div role="status" className="c-hint">{connectionStatus}</div>
            {configHostFresh && configHost && robotTarget.trim() !== configHost &&
              <div className="c-hint">{t("connection.directNoScene")}</div>}
            <div className="c-hint">{t("connection.savedAddress", { ip: snapshot.lifecycleConfig.hdu_wifi_ip ?? "?" })}</div>
            {snapshot.configRuntime?.robotIdentity && <div className="c-hint">{t("connection.robotIdentity", {
              id: snapshot.configRuntime.robotIdentity.machine_id.slice(0, 12) })}</div>}
            {pendingTarget && <button type="button" className="c-btn c-sm" disabled={!configHostFresh || !configHost} onClick={() => { setPendingTarget(""); setRobotTarget(configHost ?? ""); setTargetEdited(false); savedPanel.current = { ...savedPanel.current, robotTarget: configHost }; context.saveState(savedPanel.current); }}>
              {t("hopeA3Console.cancelSwitchAndContinueWithTheCurrentRobot")} {configHost}
            </button>}
          </div>
          <div className="c-health">
            {headerChips.map((chip) => (
              <HealthChip key={chip.key} label={chip.label} value={chip.value} warn={chip.warn} detail={chip.detail}
                onOpen={chip.open} />
            ))}
          </div>
          {estopAsserted && (
            <fieldset className="robot-controls" disabled={switching}>
              <div className="c-banner c-bad">
                <span>{t("console2.estopBanner")}</span>
                <span className="c-sp" />
                <button
                  type="button"
                  disabled={busy.estop === true || busy.resetSoftwareEstop === true}
                  onClick={() => void invoke("resetSoftwareEstop")}
                  title={t("hopeA3Console.clearsTheSoftwareLatchOnlyDoesNotRelease")}
                >
                  {busy.resetSoftwareEstop === true ? t("hopeA3Console.resetting") : t("console2.resetSoftwareEstop")}
                </button>
              </div>
            </fieldset>
          )}
          <div className="c-tabs" role="tablist" aria-label={t("console2.tabs")}>
            {tabs.map((entry) => (
              <button key={entry.id} type="button" role="tab" aria-selected={activeTab === entry.id}
                className={`c-tab ${entry.id === "play" && kernelMode ? "c-tab-k" : ""}`}
                onClick={() => { setTab(entry.id); }}>
                {entry.label}
                {entry.badge != undefined && <span className={`c-badge c-${entry.badge}`} />}
              </button>
            ))}
          </div>
        </header>

        <fieldset className="robot-controls" disabled={switching}>
          <section className="c-body c-setup" role="tabpanel" aria-label={t("console2.tabSetup")} hidden={activeTab !== "setup"}>
            {pairingCard}
            {sessionCard}
            {hwCard}
            {/* Configuration status + Confirm stay pinned to the bottom of Setup while the settings scroll. */}
            <div className="c-cfgbar">
              {lifecycleView === "running" && <div className="c-hint">{t("console2.lockedWhileRunning")}</div>}
              {field?.error != undefined && field.error !== "" && <div className="c-why c-bad">{field.error}</div>}
              {configRow}
            </div>
          </section>

          <section className="c-body" role="tabpanel" aria-label={kernelMode ? t("console2.tabKernel") : t("console2.tabPlay")} hidden={activeTab !== "play"}>
            {startCard}
            {diagCard}
            <div className={`c-card ${kernelMode ? "c-kcard" : ""}`}>
              <div className="c-diagram" style={{ aspectRatio: `${GRAPH_W} / ${graphHeight({ kernelMode })}` }}>
                <GraphEdges kernelMode={kernelMode} edge={edgeState} autoLive={currentNode === "serve"}
                  receiveLabel={kernelMode ? t("console2.nodeReceiveTest") : t("console2.edgeReady")} />
                {graphNodes.map((node) => <GraphNode key={node.id} {...node} />)}
              </div>
              <div className="c-keys c-hint" title={t("hopeA3Console.runnerShortcuts")}>
                {kernelMode ? t("console2.shortcutLegendKernel") : t("console2.shortcutLegend")}
              </div>
              {!runnerFresh && (
                <div className="c-why c-warn">
                  <span>{t("console2.runnerNotRunning")}</span>
                  <span className="c-sp" />
                  <button type="button" onClick={() => { openTab("play", "life"); }}>{t("console2.goToStart")}</button>
                </div>
              )}
              <div className="c-status c-gap-top"><span>{mocapSafetyLine}</span></div>
              <div className="c-under">
                <span className="c-mono c-hint c-trunc" title={runnerSummaryText}>{runnerSummaryText}</span>
                <span className="c-sp" />
                <GripperAction
                  label={t("console2.openGripper")}
                  detail={gripperDetail}
                  disabled={gates.openGripperBlocked}
                  busy={busy.openGripper === true}
                  onClick={() => {
                    invokeConfirmed(
                      "openGripper",
                      msg("hopeA3Console.openGripperConfirm"),
                    );
                  }}
                />
              </div>
              {snapshot.lastAction != undefined && snapshot.lastAction !== "NONE" &&
                <div className="c-mono c-dim c-trunc" title={lastAction}>{lastAction}</div>}
              {localBallIntentFailed && <div className="c-why c-bad" role="alert">{t("hopeA3Console.ballIntentFailed")}</div>}
            </div>
            {kernelMode && (
              <div className="c-card c-kcard">
                <div className="c-lbl">
                  {t("console2.csvUnderTest")}
                  <span className="c-r"><button type="button" className="c-ghost" onClick={() => { openTab("setup", "csv"); }}>{t("console2.change")}</button></span>
                </div>
                <div className="c-filebox c-filebox-flat">
                  <span className="c-n" title={csvName}>{csvName}</span>
                  <span className="c-m c-mono">{csvMeta}</span>
                </div>
                <CsvTimeline csv={csvStatus} kernelMode={kernelMode} />
              </div>
            )}
            {tuneCard}
            {walkStandBar}
            {xboxFresh && snapshot.xboxPreview?.keyboard_walk && snapshot.xboxPreview.keyboard_walk !== "idle" ? (
              <div className="c-card c-tele c-tele-on" role="status">
                <b>{t("console2.keyboardWalk")}</b>
                <span>{snapshot.xboxPreview.keyboard_walk === "moving"
                  ? velocityText(teleop?.slice(6, 9)) : t("console2.keyboardHandoff")}</span>
              </div>
            ) : teleopActive ? telePanel : teleBar}
          </section>
        </fieldset>
      </div>

      <footer className={`operator-notice ${estopAsserted ? "operator-notice-danger" : ""}`} title={t.text(notice)}>
        <time>{noticeLog[0] != undefined ? clock(noticeLog[0].at) : ""}</time>
        <span>{t.text(notice)}</span>
      </footer>

      {confirmation != undefined && (
        <div className="c-scrim">
          <section className="action-confirmation c-dlg" role="dialog" aria-modal="true" aria-label={t("hopeA3Console.confirmationTitle")}
            onKeyDown={(event) => {
              if (event.key === "Escape") { cancelConfirmation(); return; }
              if (event.key === "Enter") {
                event.preventDefault();
                if (!event.repeat && confirmEnabled) { submitConfirmation(); }
              }
            }}>
            <h3>{t("hopeA3Console.confirmationTitle")}</h3>
            <p>{t.text(confirmation.prompt)}</p>
            {confirmationReason != undefined && <p role="status" className="c-bad-text">{t.text(confirmationReason)}</p>}
            <div className="c-dlg-acts">
              <button ref={confirmationCancel} type="button" className="c-btn c-key-esc" onClick={cancelConfirmation}>{t("hopeA3Console.cancelAction")}</button>
              <button type="button" className={`c-btn c-key-enter ${confirmTone}`} disabled={!confirmEnabled}
                onClick={submitConfirmation}>{t("hopeA3Console.confirmAction")}</button>
            </div>
          </section>
        </div>
      )}
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
