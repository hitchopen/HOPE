import { message as msg, LocalText } from "./i18n";

export const MOCAP_LABELS: Record<string, LocalText> = {
  none: msg("mocapSetup.noMocapRunnerOnly"),
  motive: "Motive · NatNet",
  vrpn: msg("mocapSetup.chingmuCmtrackerVrpn"),
};

// This is a copyable command for the Laptop HOST, from the repository root.
// Values come from the supervisor's confirmed configuration, never the draft.
export function mocapSetupCommand(config: Record<string, string>, session?: string): string {
  if (!["motive", "vrpn"].includes(config.mocap_source ?? "")) {
    return "";
  }
  const quote = (value: string) => `'${value.replace(/'/g, "'\\''")}'`;
  const args = [config.laptop_wifi_ip, config.hdu_wifi_ip, config.mdu_internal_ip,
    config.motive_ip === "" ? "NONE" : config.motive_ip, config.table_side, "OPTITRACK", "DEFAULT",
    config.gripper_profile, config.mocap_source, config.vrpn_ip === "" ? "NONE" : config.vrpn_ip,
    config.vrpn_port, config.vrpn_units, config.pants_profile];
  const sessionValue = session && /^model21800_\d{8}T\d{6}Z$/.test(session)
    ? quote(session) : '"model21800_$(date -u +%Y%m%dT%H%M%SZ)"';
  const command = `bash foxglove/helpers/hope-lifecycle start-laptop ${sessionValue} ${args.map((value) => quote(value ?? "")).join(" ")}`;
  return config.operator_profile === "mac"
    ? `# Run inside the configured Mac operator container, from the HOPE repository root:\n${command}`
    : command;
}
