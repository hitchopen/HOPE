import { ExtensionContext } from "@foxglove/extension";

import { initHopeA3Console } from "./HopeA3Console";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "hope-a3-console",
    initPanel: initHopeA3Console,
  });
}
