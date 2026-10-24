export const PLANE_MODE_SERVICE = "/hope/planner/set_plane_mode";
export const PLANE_MODES = ["fixed", "per_ball", "stable_per_ball"] as const;
export type PlaneMode = typeof PLANE_MODES[number];
export const PLANE_MODE_LABELS: Record<PlaneMode, string> = {
  fixed: "固定平面（会话初始位置）",
  per_ball: "每球按当前位置定平面",
  stable_per_ball: "姿态稳定后更新（实验）",
};
export type PlaneModeState = {
  requested: PlaneMode; lastCommand?: PlaneMode; configurable: boolean;
  supported: boolean; stable: boolean; flight: number;
  referenceX?: number; planeX?: number; offset?: number; reason: string;
};
function isMode(value: unknown): value is PlaneMode {
  return PLANE_MODES.some((mode) => mode === value);
}
function finite(value: string | undefined): number | undefined {
  if (value == undefined || value.trim() === "") { return undefined; }
  const result = Number(value);
  return Number.isFinite(result) ? result : undefined;
}
export function decodePlaneMode(message: unknown): PlaneModeState | undefined {
  const statuses = (message as { status?: Array<{ name?: string; values?: Array<{ key?: unknown; value?: unknown } | null> } | null> } | undefined)?.status;
  if (!Array.isArray(statuses)) { return undefined; }
  for (const status of statuses) {
    if (status?.name !== "hope_planner_cpp/realtime" || !Array.isArray(status.values)) { continue; }
    const fields = new Map<string, string>();
    for (const field of status.values) {
      if (typeof field?.key === "string" && typeof field.value === "string") { fields.set(field.key, field.value); }
    }
    const requested = fields.get("plane_mode_requested");
    if (!isMode(requested)) { continue; }
    const last = fields.get("plane_mode_last_command");
    return { requested, lastCommand: isMode(last) ? last : undefined,
      configurable: fields.get("plane_mode_configurable") === "true",
      supported: fields.get("plane_runtime_supported") === "true",
      stable: fields.get("plane_posture_stable") === "true",
      flight: finite(fields.get("plane_last_flight")) ?? 0,
      referenceX: finite(fields.get("plane_reference_x")), planeX: finite(fields.get("plane_world_x")),
      offset: finite(fields.get("session_home_plane_x")), reason: fields.get("plane_selection_reason") ?? "waiting_for_ball" };
  }
  return undefined;
}
type ModeRequest = { parameters: Array<{ name: string; value: {
  type: number; string_value: string; bool_value: boolean; integer_value: number; double_value: number;
  byte_array_value: number[]; bool_array_value: boolean[]; integer_array_value: number[];
  double_array_value: number[]; string_array_value: string[];
} }> };
export function planeModeRequest(mode: PlaneMode): ModeRequest {
  if (!isMode(mode)) { throw new Error("未知平面模式"); }
  return { parameters: [{ name: "mode", value: {
    type: 4, string_value: mode, bool_value: false, integer_value: 0, double_value: 0,
    byte_array_value: [], bool_array_value: [], integer_array_value: [], double_array_value: [], string_array_value: [],
  } }] };
}
