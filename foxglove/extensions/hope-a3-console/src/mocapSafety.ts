import { Translator, defaultTranslator } from "./i18n";

// Both the authoritative pelvis monitor and the base consumer must agree.
// Missing, false, stale, or future-dated receipts fail closed.
export function pelvisReady(input: {
  baseFresh?: boolean; tfReady?: boolean;
  baseReceived?: number; tfReceived?: number;
}, now: number): boolean {
  const fresh = (stamp: number | undefined) => stamp != undefined &&
    Number.isFinite(stamp) && now >= stamp && now - stamp < 500;
  return input.baseFresh === true && input.tfReady === true &&
    fresh(input.baseReceived) && fresh(input.tfReceived);
}
// Read-only Runner telemetry; the schema-2 action/state contract is unchanged.
export type MocapSafety = {
  boot: number;
  required: boolean;
  fresh: boolean;
  stops: number;
  interrupted: boolean;
  held?: boolean;
  ageMs?: number;
  operatorRequired?: boolean;
  uiFresh?: boolean;
  computerFresh?: boolean;
  operatorInterrupted?: number;
};

export function decodeMocapSafety(value: unknown): MocapSafety | undefined {
  const v: unknown[] | undefined = Array.isArray(value) ? value
    : ArrayBuffer.isView(value) && !(value instanceof DataView)
      ? Array.from(value as unknown as ArrayLike<unknown>) : undefined;
  if ((v?.length !== 8 && v?.length !== 15) || v[0] !== 1 ||
      !v.every((x): x is number => typeof x === "number" && Number.isFinite(x)) ||
      !v.every((x, i) => i === 9 ? x >= -1 : Number.isSafeInteger(x) && x >= 0) ||
      v[1]! < 1 || v[2]! < 1 || v[3]! < 1 ||
      ![v[4], v[5], v[7]].every((x) => x === 0 || x === 1)) {
    return undefined;
  }
  if (v.length === 15 && (![v[8], v[10], v[11], v[12]].every(x => x === 0 || x === 1) ||
      ![0, 36, 37].includes(v[13]!))) { return undefined; }
  return { boot: v[1]!, required: v[4] === 1, fresh: v[5] === 1,
    stops: v[6]!, interrupted: v[7] === 1, held: v[8] === 1, ageMs: v[9],
    operatorRequired: v[10] === 1, uiFresh: v[11] === 1, computerFresh: v[12] === 1,
    operatorInterrupted: v[13] };
}

export function mocapSafetyText(state: MocapSafety | undefined, t: Translator = defaultTranslator): string {
  if (state == undefined) { return t("mocapSafety.mocapSafetyStatusUnavailable"); }
  if (!state.required) { return t("mocapSafety.offlineModeMocapProtectionNotRequired"); }
  if (state.interrupted) {
    return state.fresh ? t("mocapSafety.stoppedMocapLostStreamRestoredExplicitlyReEnter")
      : t("mocapSafety.stoppedMocapLostControlledPdStandRecovery");
  }
  if (state.held === true) {
    return t("mocapSafety.holdingLastPelvisPoseRealSampleAgeMs", {age: Math.round(state.ageMs ?? 0)});
  }
  return state.fresh ? t("mocapSafety.mocapFresh") : t("mocapSafety.mocapUnavailablePlayEntryBlocked");
}
