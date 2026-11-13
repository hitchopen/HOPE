export type LiveVec = readonly number[];
export type LiveTelemetrySnapshot = {
  timestamp_s: number;
  session_id: string;
  generation: number;
  shot: number;
  runner_mode?: string | null;
  runner_age_ms?: number | null;
  robot_available: boolean;
  robot_age_ms: number | null;
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

export function liveRunnerLabel(value: LiveTelemetrySnapshot | undefined): string {
  if (!liveSourceFresh(value?.runner_age_ms) || value?.runner_mode == undefined) {
    return "状态未知";
  }
  const names: Record<string, string> = {
    PASSIVE: "被动 · PASSIVE",
    PD_STAND: "站立 · PD_STAND",
    MOTION: "动作 · MOTION",
    SERVE: "发球 · SERVE",
    TELEOP: "遥操作 · TELEOP",
  };
  return names[value.runner_mode] ?? value.runner_mode;
}
