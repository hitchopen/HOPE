export type BallPlayState = {
  schema: 2;
  producer_instance_id: string;
  intent_revision: string;
  local_stopped: boolean;
  session_id: string;
  state: "INVALID" | "OPPONENT_RALLY" | "RALLY" | "OPPONENT_SERVE" | "SERVE";
  reason: string;
  forecast_advisory?: string;
  generation: string;
  forwarding: boolean;
  history_known: boolean;
  observation_missing?: boolean;
  return_confirmed?: boolean;
  serve_armed: boolean;
  receive_armed?: boolean;
  striker_bounces: number;
  receiver_bounces: number;
};

export function decodeBallPlayState(value: unknown): BallPlayState | undefined {
  if (typeof value !== "string") { return undefined; }
  try {
    const s: unknown = JSON.parse(value);
    if (s == undefined || typeof s !== "object") { return undefined; }
    const p = s as Record<string, unknown>;
    if (p.schema !== 2 || typeof p.session_id !== "string" ||
        typeof p.producer_instance_id !== "string" || p.producer_instance_id === "" ||
        typeof p.intent_revision !== "string" || !/^\d+$/.test(p.intent_revision) ||
        typeof p.local_stopped !== "boolean" ||
        (p.local_stopped && (p.state !== "INVALID" || p.forwarding || p.serve_armed || p.receive_armed)) ||
        typeof p.state !== "string" || !["INVALID", "OPPONENT_RALLY", "RALLY", "OPPONENT_SERVE", "SERVE"].includes(p.state) ||
        (p.forecast_advisory != undefined && typeof p.forecast_advisory !== "string") ||
        (p.observation_missing != undefined && typeof p.observation_missing !== "boolean") ||
        typeof p.reason !== "string" || typeof p.generation !== "string" || !/^\d+$/.test(p.generation) ||
        typeof p.forwarding !== "boolean" || typeof p.history_known !== "boolean" || typeof p.serve_armed !== "boolean" ||
        (p.return_confirmed != undefined && typeof p.return_confirmed !== "boolean") ||
        (p.return_confirmed === true && p.state !== "OPPONENT_RALLY") ||
        (p.receive_armed != undefined && typeof p.receive_armed !== "boolean") ||
        (p.receive_armed === true && (p.state !== "OPPONENT_SERVE" || p.serve_armed)) ||
        ![p.striker_bounces, p.receiver_bounces].every((n) => typeof n === "number" && Number.isSafeInteger(n) && n >= 0) ||
        (p.forwarding && (p.observation_missing === true || p.serve_armed || !["OPPONENT_RALLY", "OPPONENT_SERVE"].includes(p.state)))) {
      return undefined;
    }
    return p as BallPlayState;
  } catch { return undefined; }
}

export function currentBallPlayState(state: BallPlayState | undefined,
  receivedAt: number | undefined, now: number): BallPlayState | undefined {
  return receivedAt != undefined && now >= receivedAt && now - receivedAt < 1000 ? state : undefined;
}

// This feed and its button intents are local to the operator computer. Robot
// state, service results and lifecycle/session identity never control it.
export const LOCAL_BALL_STATUS_URL = "http://127.0.0.1:8768/ball-state";
export async function readLocalBallPlayState(fetcher: typeof fetch, signal: AbortSignal, localKey = ""): Promise<BallPlayState | undefined> {
  const response = await fetcher(LOCAL_BALL_STATUS_URL, {
    method: "GET", cache: "no-store", credentials: "omit", signal,
    headers: localKey ? { "X-Hope-Local-Key": localKey } : {},
  });
  if (!response.ok) return undefined;
  const text = await response.text();
  return text.length <= 4096 ? decodeBallPlayState(text) : undefined;
}

export type LocalBallIntent = "READY" | "SERVE" | "STOP";
export function localIntentForAction(key: string): LocalBallIntent | undefined {
  if (key === "ready") return "READY";
  if (key === "prepareServe" || key === "readyToServe") return "SERVE";
  if (["stand", "killAllAndCollect", "passive", "estop"].includes(key)) return "STOP";
  return undefined;
}

export async function sendLocalBallIntent(fetcher: typeof fetch, action: LocalBallIntent,
  signal: AbortSignal, localKey = ""): Promise<void> {
  const state = await readLocalBallPlayState(fetcher, signal, localKey);
  if (state == undefined) throw new Error("LOCAL_BALL_UNAVAILABLE");
  const response = await fetcher(`${LOCAL_BALL_STATUS_URL}/intent/${action}`, {
    method: "POST", cache: "no-store", credentials: "omit", signal,
    headers: { "X-Hope-Producer": state.producer_instance_id,
      "X-Hope-Intent-Revision": state.intent_revision,
      ...(localKey ? { "X-Hope-Local-Key": localKey } : {}) },
  });
  // Do not retry a conflict with a newer revision: a delayed Ready must never
  // reopen the gate after a newer Stop from another Console.
  if (response.status !== 202) throw new Error("LOCAL_BALL_INTENT_REJECTED");
  const reply = await response.json() as { intent_revision?: unknown };
  if (typeof reply.intent_revision !== "string" || !/^\d+$/.test(reply.intent_revision))
    throw new Error("LOCAL_BALL_REPLY_INVALID");
  // Complete this operation only after the ROS timer applied it. This also
  // serializes consecutive local buttons without waiting for any robot ACK.
  while (!signal.aborted) {
    const current = await readLocalBallPlayState(fetcher, signal, localKey);
    if (current?.producer_instance_id !== state.producer_instance_id)
      throw new Error("LOCAL_BALL_RESTARTED");
    if (current.intent_revision === reply.intent_revision) return;
    if (BigInt(current.intent_revision) > BigInt(reply.intent_revision))
      throw new Error("LOCAL_BALL_INTENT_SUPERSEDED");
    await new Promise((resolve) => setTimeout(resolve, 10));
  }
  throw new Error("LOCAL_BALL_INTENT_TIMEOUT");
}

// Keep the private pairing key in this Foxglove installation only, never in
// context.saveState (layouts may be exported/shared) or a URL/ROS message.
const LOCAL_KEY_STORAGE = "hope.localBallControlKey";
export function readLocalControlKey(): string {
  try {
    const value = window.localStorage.getItem(LOCAL_KEY_STORAGE) ?? "";
    return /^[a-f0-9]{64}$/.test(value) ? value : "";
  } catch { return ""; }
}
export function saveLocalControlKey(value: string): string {
  const key = value.trim();
  if (!/^[a-f0-9]{64}$/.test(key)) throw new Error("LOCAL_BALL_KEY_INVALID");
  window.localStorage.setItem(LOCAL_KEY_STORAGE, key);
  return key;
}
