import { ExtensionContext } from "@foxglove/extension";

import { initHopeA3Console } from "./HopeA3Console";
import { initHopeLiveStatus } from "./HopeLiveStatus";
import { initLiveTelemetry, initLiveViewPreview } from "./LiveTelemetry";
import { initReplayTelemetry, initReplayViewControls } from "./ReplayTelemetry";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "HOPE Live Telemetry", initPanel: initLiveTelemetry });
  extensionContext.registerPanel({
    name: "HOPE Live View Preview",
    initPanel: initLiveViewPreview,
  });
  extensionContext.registerPanel({ name: "HOPE Replay Telemetry", initPanel: initReplayTelemetry });
  extensionContext.registerPanel({
    name: "HOPE Replay View Controls",
    initPanel: initReplayViewControls,
  });
  extensionContext.registerPanel({ name: "HOPE Live Status", initPanel: initHopeLiveStatus });
  extensionContext.registerPanel({
    name: "HOPE A3 Console",
    initPanel: initHopeA3Console,
  });
}
