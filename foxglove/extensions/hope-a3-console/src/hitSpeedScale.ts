export const HIT_SPEED_SCALE_SERVICE = "/hope/planner/set_hit_speed_scale";
export type HitSpeedScaleState = { scale: number; configurable: boolean; persistent: boolean; lastCommand?: number };
export function parseHitSpeedScale(value: string): number | undefined {
  if (value.trim() === "") { return undefined; }
  const scale = Number(value);
  return Number.isFinite(scale) && scale >= 0 ? scale : undefined;
}
export function decodeHitSpeedScale(message: unknown): HitSpeedScaleState | undefined {
  const statuses = (message as { status?: Array<{ name?: string; values?: Array<{ key?: unknown; value?: unknown } | null> } | null> } | undefined)?.status;
  if (!Array.isArray(statuses)) { return undefined; }
  for (const status of statuses) {
    if (status?.name !== "hope_planner_cpp/realtime" || !Array.isArray(status.values)) { continue; }
    const fields = new Map<string, string>();
    for (const field of status.values) {
      if (typeof field?.key === "string" && typeof field.value === "string") { fields.set(field.key, field.value); }
    }
    const scale = parseHitSpeedScale(fields.get("hit_speed_scale") ?? "");
    if (scale == undefined) { continue; }
    return { scale, configurable: fields.get("hit_speed_scale_configurable") === "true",
      persistent: fields.get("hit_speed_scale_persistent") === "true",
      lastCommand: parseHitSpeedScale(fields.get("hit_speed_scale_last_command") ?? "") };
  }
  return undefined;
}
type ScaleRequest = { parameters: Array<{ name: string; value: {
  type: number; double_value: number; string_value: string; bool_value: boolean; integer_value: number;
  byte_array_value: number[]; bool_array_value: boolean[]; integer_array_value: number[];
  double_array_value: number[]; string_array_value: string[];
} }> };
export function hitSpeedScaleRequest(scale: number): ScaleRequest {
  if (!Number.isFinite(scale) || scale < 0) { throw new Error("击球力度 scale 必须是大于或等于 0 的有限数值"); }
  return { parameters: [{ name: "scale", value: {
    type: 3, double_value: scale, string_value: "", bool_value: false, integer_value: 0,
    byte_array_value: [], bool_array_value: [], integer_array_value: [], double_array_value: [], string_array_value: [],
  } }] };
}
