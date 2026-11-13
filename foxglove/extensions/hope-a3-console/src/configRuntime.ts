// One atomic saved-config/runtime report; legacy scalar topics cannot overwrite it.
export const CONFIG_KEYS = ["laptop_wifi_ip", "hdu_wifi_ip", "mdu_internal_ip", "motive_ip",
  "vrpn_ip", "mocap_source", "vrpn_port", "vrpn_units", "table_side", "operator_profile",
  "gripper_profile", "pants_profile", "vrpn_pelvis_asset_confirmed", "vrpn_world_frame_confirmed"] as const;
export const MOCAP_BINDING_KEYS = ["hdu_wifi_ip", "mocap_source", "motive_ip", "vrpn_ip", "vrpn_port", "vrpn_units", "pants_profile", "table_side"] as const;
export function sameMocapBinding(draft: Partial<RuntimeConfigValues>, saved: Partial<RuntimeConfigValues>): boolean {
  return MOCAP_BINDING_KEYS.every((key) => draft[key] === saved[key]);
}
export function configForSave(draft: RuntimeConfigValues, saved: Partial<RuntimeConfigValues>): RuntimeConfigValues {
  return draft.mocap_source === "vrpn" && sameMocapBinding(draft, saved) ? { ...draft } :
    { ...draft, vrpn_pelvis_asset_confirmed: "false", vrpn_world_frame_confirmed: "false" };
}
export type RuntimeConfigValues = Record<(typeof CONFIG_KEYS)[number], string>;
export type ConfigRuntime = {
  revision: number;
  source: string;
  config_sha256: string;
  values: RuntimeConfigValues;
  applied: boolean;
  state: string;
  reason: string;
  mocapProcesses: Array<{ kind: string; program: string; expected: string; startup: string; files: string; matches: boolean }>;
  services: Array<{ service: string; matches: boolean; expected_peers: string; actual_peers: string | null; reason: string }>;
};
export function decodeConfigRuntime(raw: unknown): ConfigRuntime | undefined {
  try {
    const s = typeof raw === "string" ? JSON.parse(raw) as Record<string, unknown> : undefined;
    if (!s || s.schema_version !== 1 || !Number.isSafeInteger(s.revision) || (s.revision as number) < 0 ||
      s.source !== "/var/lib/hope-lifecycle/config.json" || typeof s.config_sha256 !== "string" ||
      !/^[0-9a-f]{64}$/.test(s.config_sha256) || typeof s.applied !== "boolean" ||
      typeof s.state !== "string" || typeof s.reason !== "string" || !Array.isArray(s.services) ||
      s.values == null || typeof s.values !== "object") { return undefined; }
    const values = s.values as Record<string, unknown>;
    if (!CONFIG_KEYS.every((k) => typeof values[k] === "string") ||
      !["vrpn_pelvis_asset_confirmed", "vrpn_world_frame_confirmed"].every((k) => values[k] === "true" || values[k] === "false")) { return undefined; }
    const services: ConfigRuntime["services"] = [];
    for (const rawService of s.services as unknown[]) {
      if (!rawService || typeof rawService !== "object") { return undefined; }
      const row = rawService as Record<string, unknown>;
      if (typeof row.service !== "string" || typeof row.matches !== "boolean" ||
          typeof row.expected_peers !== "string" || !(row.actual_peers === null || typeof row.actual_peers === "string")) { return undefined; }
      services.push({ service: row.service, matches: row.matches, expected_peers: row.expected_peers,
        actual_peers: row.actual_peers, reason: typeof row.reason === "string" ? row.reason : "" });
    }
    const mocapProcesses: ConfigRuntime["mocapProcesses"] = [];
    const laptop = s.laptop as { processes?: unknown } | undefined;
    if (Array.isArray(laptop?.processes)) {
      for (const value of laptop.processes as unknown[]) {
        if (!value || typeof value !== "object") { continue; }
        const p = value as Record<string, unknown>;
        const endpoint = p.endpoint as { expected?: unknown; startup_parameters?: unknown } | undefined;
        if (!endpoint || typeof p.kind !== "string") { continue; }
        const files = Array.isArray(p.parameter_files) ? p.parameter_files as Array<{ path?: unknown }> : [];
        mocapProcesses.push({ kind: p.kind, program: typeof p.program === "string" ? p.program : "",
          expected: JSON.stringify(endpoint.expected) ?? "", startup: JSON.stringify(endpoint.startup_parameters) ?? "",
          files: files.map((f) => typeof f.path === "string" ? f.path : "").join(" · "), matches: p.matches === true });
      }
    }
    const applied = s.applied && s.state === "APPLIED" && (s.revision as number) > 0 &&
      services.length > 0 && services.every((row) => row.matches);
    return { revision: s.revision as number, source: s.source, config_sha256: s.config_sha256,
      values: values as RuntimeConfigValues, applied, state: s.state, reason: s.reason, services, mocapProcesses };
  } catch { return undefined; }
}
export function configRuntimeReady(status: ConfigRuntime | undefined, receivedAt: number | undefined, now: number): boolean {
  return status?.applied === true && receivedAt !== undefined && now >= receivedAt && now - receivedAt < 1500;
}
export function savedConfigObserved(status: ConfigRuntime | undefined, request: { revision: number; values: RuntimeConfigValues }): boolean {
  return status != undefined && status.revision > request.revision &&
    CONFIG_KEYS.every((key) => status.values[key] === request.values[key]);
}
