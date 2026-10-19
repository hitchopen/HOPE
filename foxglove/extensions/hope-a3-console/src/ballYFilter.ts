export const BALL_Y_FILTER_SERVICE = "/hope/planner/set_ball_y_filter";
export const DEFAULT_BALL_Y_FILTER = {
  enabled: false, min: -1.525, max: 0,
};
export type BallYFilterState = typeof DEFAULT_BALL_Y_FILTER & {
  configurable: boolean; persistent: boolean;
};

export function parseBallYBounds(min: string, max: string): { min: number; max: number } | undefined {
  if (min.trim() === "" || max.trim() === "") { return undefined; }
  const lo = Number(min), hi = Number(max);
  return Number.isFinite(lo) && Number.isFinite(hi) && lo < hi ? { min: lo, max: hi } : undefined;
}

export function decodeBallYFilter(message: unknown): BallYFilterState | undefined {
  const statuses = (message as { status?: Array<{ values?: Array<{ key?: unknown; value?: unknown } | null> } | null> } | undefined)?.status;
  if (!Array.isArray(statuses)) { return undefined; }
  for (const status of statuses) {
    if (!Array.isArray(status?.values)) { continue; }
    const values = new Map<string, string>();
    for (const value of status.values) {
      if (typeof value?.key === "string" && typeof value.value === "string") {
        values.set(value.key, value.value);
      }
    }
    const enabled = values.get("ball_y_filter_enabled");
    const bounds = parseBallYBounds(values.get("ball_y_filter_min_m") ?? "", values.get("ball_y_filter_max_m") ?? "");
    if ((enabled === "true" || enabled === "false") && bounds != undefined) {
      return { enabled: enabled === "true", ...bounds,
        configurable: values.get("ball_y_filter_runtime_config") === "true",
        persistent: values.get("ball_y_filter_persistent") === "true" };
    }
  }
  return undefined;
}

type FilterRequest = { parameters: Array<{ name: string; value: {
  type: number; bool_value: boolean; double_value: number; integer_value: number;
  string_value: string; byte_array_value: number[]; bool_array_value: boolean[];
  integer_array_value: number[]; double_array_value: number[]; string_array_value: string[];
} }> };
export function ballYFilterRequest(config: typeof DEFAULT_BALL_Y_FILTER): FilterRequest {
  return { parameters: [
    { name: "enabled", value: config.enabled }, { name: "min_m", value: config.min },
    { name: "max_m", value: config.max },
  ].map(({ name, value }) => ({ name, value: {
    type: typeof value === "boolean" ? 1 : 3,
    bool_value: typeof value === "boolean" ? value : false,
    double_value: typeof value === "number" ? value : 0,
    integer_value: 0, string_value: "", byte_array_value: [], bool_array_value: [],
    integer_array_value: [], double_array_value: [], string_array_value: [],
  } })) };
}
