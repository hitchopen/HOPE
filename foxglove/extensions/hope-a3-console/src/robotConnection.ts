import { message as msg, LocalizedError } from "./i18n";

export function robotConnectionUrl(ip: string): string {
  const parts = ip.split(".");
  if (parts.length !== 4 || parts.some((part) => !/^(0|[1-9][0-9]{0,2})$/.test(part) || Number(part) > 255) ||
      !(parts[0] === "10" || (parts[0] === "192" && parts[1] === "168") ||
        (parts[0] === "172" && Number(parts[1]) >= 16 && Number(parts[1]) <= 31))) {
    throw new LocalizedError(msg("robotConnection.enterAValidLocalIpv4AddressForThe"));
  }
  return `foxglove://open?ds=foxglove-websocket&ds.url=${encodeURIComponent(`ws://${ip}:8766`)}`;
}

// Switching only opens a data source. It must never invoke the current robot's
// apply_config, stop, start or command services.
export function switchRobot(ip: string, saveTarget: (ip: string) => void, open: (url: string) => void): void {
  const url = robotConnectionUrl(ip);
  saveTarget(ip);
  open(url);
}

// Confirming the current robot must preserve the Host data source and layout.
// A different robot remains an explicit direct-connect fallback, never a config write.
export function confirmRobotConnection(ip: string, liveHost: string | undefined, fresh: boolean,
  confirmed: (ip: string) => void, switching: (ip: string) => void, open: (url: string) => void): void {
  robotConnectionUrl(ip);
  if (!fresh || !liveHost) throw new LocalizedError(msg("connection.waitForIdentity"));
  if (ip === liveHost) { confirmed(ip); return; }
  switchRobot(ip, switching, open);
}

// A saved target is only a draft. Only a fresh robot receipt can confirm it.
export function reconcileRobotTarget(target: string, edited: boolean, pending: string, liveHost: string) {
  if (!liveHost) { return { target, edited, pending }; }
  if (pending === liveHost) { return { target: liveHost, edited: false, pending: "" }; }
  return { target: !edited && !pending ? liveHost : target, edited, pending };
}

export type RobotIdentity = { machine_id: string; boot_id: string;
  connection?: { hdu_ip: string; laptop_ip: string } };
export function decodeRobotIdentity(raw: unknown): RobotIdentity | undefined {
  if (raw == undefined || typeof raw !== "object") { return undefined; }
  const r = raw as Record<string, unknown>;
  if (typeof r.machine_id !== "string" || !/^[0-9a-f]{32}$/.test(r.machine_id) ||
      typeof r.boot_id !== "string" || !/^[0-9a-f-]{36}$/.test(r.boot_id)) { return undefined; }
  const result: RobotIdentity = { machine_id: r.machine_id, boot_id: r.boot_id };
  if (r.connection != undefined && typeof r.connection === "object") {
    const c = r.connection as Record<string, unknown>;
    if (typeof c.hdu_ip === "string" && typeof c.laptop_ip === "string") {
      try { robotConnectionUrl(c.hdu_ip); robotConnectionUrl(c.laptop_ip);
        result.connection = { hdu_ip: c.hdu_ip, laptop_ip: c.laptop_ip };
      } catch { /* Ambiguous or unsupported endpoint is never a confirmed target. */ }
    }
  }
  return result;
}
