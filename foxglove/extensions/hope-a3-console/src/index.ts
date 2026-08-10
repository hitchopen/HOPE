import { ExtensionContext } from "@foxglove/extension";

import { initHopeA3Console } from "./HopeA3Console";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "HOPE A3 Console",
    initPanel: initHopeA3Console,
  });
}
