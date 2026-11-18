export type RunnerShortcut = "stand" | "ready" | "serve" | "passive";

export const RUNNER_SHORTCUT_KEYS: Record<RunnerShortcut, string> = {
  stand: "I",
  ready: "R",
  serve: "S",
  passive: "ArrowLeft+ArrowRight",
};

type ConsoleOwners = { panels: Set<HTMLElement>; active?: HTMLElement };
const consoleOwners = new WeakMap<Document, ConsoleOwners>();

/** Listen at the window's document: disabled action buttons lose DOM focus. */
export function bindRunnerShortcuts(panel: HTMLElement): () => void {
  const doc = panel.ownerDocument;
  const view = doc.defaultView;
  const owners: ConsoleOwners = consoleOwners.get(doc) ?? { panels: new Set<HTMLElement>() };
  consoleOwners.set(doc, owners);
  owners.panels.add(panel);
  const held = new Set<string>();
  let passiveLatched = false;
  const clear = () => { held.clear(); passiveLatched = false; };
  const ownsKeyboard = () => {
    const visible = Array.from(owners.panels)
      .filter((candidate) => candidate.isConnected && candidate.getClientRects().length > 0);
    return visible.length === 1 ? visible[0] === panel : owners.active === panel && visible.includes(panel);
  };
  const onInteraction = (event: Event) => {
    clear();
    if (event.composedPath().includes(panel)) { owners.active = panel; }
  };
  const keyFor = (event: KeyboardEvent): string => {
    if (["KeyI", "KeyR", "KeyS"].includes(event.code)) { return event.code; }
    if (event.key === "ArrowLeft" || event.key === "ArrowRight") { return event.key; }
    if (!event.code && /^[irs]$/i.test(event.key)) { return `Key${event.key.toUpperCase()}`; }
    return "";
  };
  const onKeyDown = (event: KeyboardEvent) => {
    // Multiple Console panels must never send the same key to two robots.
    if (!ownsKeyboard()) { clear(); return; }
    // Include composedPath for inputs/contenteditable inside shadow roots.
    const editing = event.composedPath().some((target) => {
      const element = target as HTMLElement;
      return element.nodeType === 1 && (element.isContentEditable ||
        element.matches("input, textarea, select, [role='textbox'], [role='combobox'], [role='spinbutton'], [role='slider']"));
    });
    // Some IME boundary events report 229 before/after isComposing is set.
    // eslint-disable-next-line @typescript-eslint/no-deprecated
    const composing = event.isComposing || event.keyCode === 229;
    if (event.defaultPrevented || composing ||
        event.ctrlKey || event.metaKey || event.altKey || editing || doc.hidden ||
        Array.from(doc.querySelectorAll("[role='dialog'], [role='alertdialog'], [aria-modal='true']"))
          .some((dialog) => dialog.getClientRects().length > 0)) {
      clear();
      return;
    }
    const key = keyFor(event);
    if (!key) { return; }
    event.preventDefault();
    event.stopPropagation();
    if (event.repeat || held.has(key)) { return; }
    held.add(key);
    let action: RunnerShortcut;
    if (key === "KeyI") { action = "stand"; }
    else if (key === "KeyR") { action = "ready"; }
    else if (key === "KeyS") { action = "serve"; }
    else {
      if (passiveLatched || !held.has("ArrowLeft") || !held.has("ArrowRight")) { return; }
      passiveLatched = true;
      action = "passive";
    }
    // :disabled also covers the fieldset used while switching robots. Clicking
    // the real button preserves busy/freshness/role gates and local ball intent.
    const buttons = Array.from(panel.querySelectorAll<HTMLButtonElement>(`button[data-runner-shortcut="${action}"]`))
      .filter((button) => !button.matches(":disabled"));
    // The two Serve stages share S. Dispatch only the uniquely enabled stage.
    if (buttons.length === 1) { buttons[0]?.click(); }
  };
  const onKeyUp = (event: KeyboardEvent) => {
    held.delete(keyFor(event));
    if (!held.has("ArrowLeft") && !held.has("ArrowRight")) { passiveLatched = false; }
  };
  const capture = { capture: true };
  doc.addEventListener("keydown", onKeyDown, capture);
  doc.addEventListener("pointerdown", onInteraction, capture);
  doc.addEventListener("focusin", onInteraction, capture);
  doc.addEventListener("keyup", onKeyUp, capture);
  doc.addEventListener("visibilitychange", clear);
  view?.addEventListener("blur", clear);
  return () => {
    clear();
    owners.panels.delete(panel);
    if (owners.active === panel) { owners.active = undefined; }
    if (owners.panels.size === 0) { consoleOwners.delete(doc); }
    doc.removeEventListener("keydown", onKeyDown, capture);
    doc.removeEventListener("pointerdown", onInteraction, capture);
    doc.removeEventListener("focusin", onInteraction, capture);
    doc.removeEventListener("keyup", onKeyUp, capture);
    doc.removeEventListener("visibilitychange", clear);
    view?.removeEventListener("blur", clear);
  };
}
