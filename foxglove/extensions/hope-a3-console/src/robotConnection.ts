export function robotConnectionUrl(ip: string): string {
  const parts = ip.split(".");
  if (parts.length !== 4 || parts.some((part) => !/^(0|[1-9][0-9]{0,2})$/.test(part) || Number(part) > 255) ||
      !(parts[0] === "10" || (parts[0] === "192" && parts[1] === "168") ||
        (parts[0] === "172" && Number(parts[1]) >= 16 && Number(parts[1]) <= 31))) {
    throw new Error("请输入机器人的有效局域网 IPv4 地址");
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
