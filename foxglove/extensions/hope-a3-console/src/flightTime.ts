import { message as msg, LocalizedError } from "./i18n";

export const FLIGHT_TIME_SERVICE = "/hope/planner/set_flight_time";
export type FlightTimeState = { seconds: number; forehand: number; backhand: number; maxSeconds: number; lastRequested?: number; configurable: boolean; persistent: boolean; lastCommand?: number };
export function parseFlightTime(value: string): number | undefined {
  if (value.trim() === "") { return undefined; }
  const scale = Number(value);
  return Number.isFinite(scale) && scale > 0 ? scale : undefined;
}
export function decodeFlightTime(message: unknown): FlightTimeState | undefined {
  const statuses = (message as { status?: Array<{ name?: string; values?: Array<{ key?: unknown; value?: unknown } | null> } | null> } | undefined)?.status;
  if (!Array.isArray(statuses)) { return undefined; }
  for (const status of statuses) {
    if (status?.name !== "hope_planner_cpp/realtime" || !Array.isArray(status.values)) { continue; }
    const fields = new Map<string, string>();
    for (const field of status.values) {
      if (typeof field?.key === "string" && typeof field.value === "string") { fields.set(field.key, field.value); }
    }
    const scale = parseFlightTime(fields.get("flight_time_s") ?? "");
    if (scale == undefined) { continue; }
    const forehand = parseFlightTime(fields.get("flight_time_fh_s") ?? "");
    const backhand = parseFlightTime(fields.get("flight_time_bh_s") ?? "");
    const maxSeconds = parseFlightTime(fields.get("flight_time_max_s") ?? "");
    if (forehand == undefined || backhand == undefined || maxSeconds == undefined) { continue; }
    return { seconds: scale, forehand, backhand, maxSeconds,
      lastRequested: parseFlightTime(fields.get("flight_time_last_requested_s") ?? ""), configurable: fields.get("flight_time_configurable") === "true",
      persistent: fields.get("flight_time_persistent") === "true",
      lastCommand: parseFlightTime(fields.get("flight_time_last_command_s") ?? "") };
  }
  return undefined;
}
type FlightTimeRequest = { parameters: Array<{ name: string; value: {
  type: number; double_value: number; string_value: string; bool_value: boolean; integer_value: number;
  byte_array_value: number[]; bool_array_value: boolean[]; integer_array_value: number[];
  double_array_value: number[]; string_array_value: string[];
} }> };
export function flightTimeRequest(scale: number): FlightTimeRequest {
  if (!Number.isFinite(scale) || scale <= 0) { throw new LocalizedError(msg("flightTime.invalid")); }
  return { parameters: [{ name: "flight_time_s", value: {
    type: 3, double_value: scale, string_value: "", bool_value: false, integer_value: 0,
    byte_array_value: [], bool_array_value: [], integer_array_value: [], double_array_value: [], string_array_value: [],
  } }] };
}
