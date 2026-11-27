import { PanelExtensionContext } from "@foxglove/extension";
import { FLOAT64_ARRAY_DATATYPES } from "./operatorHeartbeat";

export const KEYBOARD_WALK_TOPIC = "/hope/operator/keyboard_walk";
export const WALK_KEYS = ["Numpad8", "Numpad5", "Numpad4", "Numpad6", "Numpad7", "Numpad9"];

export type WalkTarget = { boot: number; fingerprint: number; standing: boolean };

export class KeyboardWalk {
  private keys = new Set<string>();
  private target?: WalkTarget;
  private gesture = 0;
  private sequence = 0;
  private sentAt = 0;
  private advertised = false;
  constructor(private context: PanelExtensionContext, private readTarget: () => WalkTarget | undefined) {}

  advertise(): void {
    if (this.advertised || !this.context.advertise || !this.context.publish) { return; }
    try {
      this.context.advertise(KEYBOARD_WALK_TOPIC, "std_msgs/Float64MultiArray", {datatypes: FLOAT64_ARRAY_DATATYPES});
      this.advertised = true;
    } catch { /* A disconnected data source can recover on the next key press. */ }
  }

  down(key: string): void {
    if (!this.target) {
      const target = this.readTarget();
      if (!target?.standing) { return; }
      this.advertise();
      if (!this.advertised) { return; }
      this.target = target;
      const random = crypto.getRandomValues(new Uint32Array(2));
      this.gesture = ((random[0]! & 0x7ffff) * 0x100000000 + random[1]!) || 1;
      this.sequence = 0;
    }
    this.keys.add(key);
    this.tick();
  }

  up(key: string): void {
    if (!this.keys.delete(key)) { return; }
    if (this.keys.size) { this.tick(); } else { this.stop(); }
  }

  get active(): boolean { return this.target != undefined; }

  tick(): void {
    if (!this.target) { return; }
    const next = this.readTarget();
    if (!next || next.boot !== this.target.boot || next.fingerprint !== this.target.fingerprint ||
        (this.sequence > 0 && performance.now() - this.sentAt > 170)) {
      this.stop();
      return;
    }
    this.send(true);
  }

  private send(held: boolean): void {
    if (!this.target) { return; }
    const axis = (plus: string, minus: string) => Number(this.keys.has(plus)) - Number(this.keys.has(minus));
    try {
      this.context.publish?.(KEYBOARD_WALK_TOPIC, {layout: {dim: [], data_offset: 0}, data: [
        1, this.target.boot, this.target.fingerprint, this.gesture, ++this.sequence, Date.now(), Number(held),
        axis("Numpad8", "Numpad5"), axis("Numpad4", "Numpad6"), axis("Numpad7", "Numpad9"),
      ]});
      this.sentAt = performance.now();
    } catch {
      this.target = undefined;
      this.keys.clear();
      this.advertised = false;
    }
  }

  stop(): void {
    this.keys.clear();
    this.send(false);
    this.target = undefined;
  }

  close(): void {
    this.stop();
    if (this.advertised) {
      try { this.context.unadvertise?.(KEYBOARD_WALK_TOPIC); } catch { /* disconnected */ }
    }
  }
}
