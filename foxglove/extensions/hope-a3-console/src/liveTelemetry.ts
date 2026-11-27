import { Translator, defaultTranslator } from "./i18n";

export type LiveVec = readonly number[];
export type LiveTelemetrySnapshot = {
  timestamp_s: number;
  session_id: string;
  generation: number;
  shot: number;
  runner_mode?: string | null;
  runner_age_ms?: number | null;
  display_pose_held?: boolean;
  display_mocap_body?: boolean;
  display_joints_reference?: boolean;
  robot_available: boolean;
  robot_age_ms: number | null;
  /** Mocap pelvis pose only; present without a Runner. */
  base_available?: boolean;
  base_age_ms?: number | null;
  play_state_age_ms: number | null;
  play_state: {
    state: string;
    reason: string;
    observation_missing: boolean;
    forwarding: boolean;
    serve_armed?: boolean;
    receive_armed?: boolean;
  } | null;
  robot_xyz: LiveVec | null;
  robot_rpy_deg: LiveVec | null;
  last_pose_age_s?: number;
  last_robot_xyz?: LiveVec;
  last_robot_rpy_deg?: LiveVec;
  racket_xyz: LiveVec | null;
  racket_speed: number | null;
  ball_xyz: LiveVec | null;
  target_xyz: LiveVec | null;
  target_swing_sign?: number;
  policy_swing_sign?: number;
  frozen: { generation: number; timestamp_s: number; ball_xyz: LiveVec } | null;
  policy_frozen: {
    generation: number;
    timestamp_s: number;
    racket_xyz: LiveVec;
    delta_xyz: LiveVec | null;
    sample_age_ms: number;
  } | null;
};
export function parseLiveTelemetry(raw: unknown): LiveTelemetrySnapshot | undefined {
  if (typeof raw !== "string") {
    return undefined;
  }
  try {
    const value = JSON.parse(raw) as LiveTelemetrySnapshot;
    if (
      !Number.isFinite(value.timestamp_s) ||
      typeof value.session_id !== "string" ||
      !Number.isSafeInteger(value.generation) ||
      value.generation < 0
    ) {
      return undefined;
    }
    return {
      ...value,
      frozen: value.frozen?.generation === value.generation ? value.frozen : null,
      policy_frozen:
        value.policy_frozen?.generation === value.generation ? value.policy_frozen : null,
    };
  } catch {
    return undefined;
  }
}
export function liveTelemetryFresh(
  value: LiveTelemetrySnapshot | undefined,
  receivedAt: number,
  now: number,
): boolean {
  return (
    value != undefined &&
    now >= receivedAt &&
    now - receivedAt < 1000 &&
    Math.abs(now - value.timestamp_s * 1000) < 1000
  );
}
export function liveSourceFresh(age: number | null | undefined): boolean {
  return typeof age === "number" && Number.isFinite(age) && age >= 0 && age < 1000;
}

export function liveRunnerLabel(value: LiveTelemetrySnapshot | undefined, t: Translator = defaultTranslator): string {
  if (!liveSourceFresh(value?.runner_age_ms) || value?.runner_mode == undefined) {
    return t("telemetry.unknown");
  }
  const names: Record<string, string> = {
    PASSIVE: t("telemetry.passive"),
    PD_STAND: t("telemetry.stand"),
    MOTION: t("telemetry.motion"),
    SERVE: t("telemetry.serve"),
    TELEOP: t("telemetry.teleop"),
    WALK_STAND: t("telemetry.walkStand"),
    DOCK: t("telemetry.dock"),
  };
  return names[value.runner_mode] ?? value.runner_mode;
}

export type LiveShotEvents = { target: boolean; plane: boolean; strike: boolean; deltaMm?: number };

/** Per-ball event lights for the current generation. Display only; uses the parsed snapshot. */
export function liveShotEvents(value: LiveTelemetrySnapshot | undefined): LiveShotEvents {
  const delta = value?.policy_frozen?.delta_xyz;
  const target = value?.target_xyz;
  const deltaMm = delta?.length === 3 && delta.every((v) => Number.isFinite(v))
    ? Math.hypot(delta[0] ?? 0, delta[1] ?? 0, delta[2] ?? 0) * 1000 : undefined;
  return {
    target: target?.length === 3,
    plane: value?.frozen != undefined,
    strike: value?.policy_frozen != undefined,
    deltaMm: value?.policy_frozen != undefined ? deltaMm : undefined,
  };
}
