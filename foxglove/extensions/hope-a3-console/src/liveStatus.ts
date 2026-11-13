import { defaultTranslator, Translator } from "./i18n";
import type { BallPlayState } from "./ballPlayState";

export type StatusTone = "good" | "waiting" | "neutral" | "unavailable";
export type StatusDisplay = { title: string; detail: string; tone: StatusTone };

export function ballStatusDisplay(state: BallPlayState | undefined, t: Translator = defaultTranslator): StatusDisplay {
  if (!state) return { title: t("liveStatus.unavailable"), detail: t("liveStatus.waitService"), tone: "unavailable" };
  if (state.local_stopped) return { title: t("liveStatus.stopped"), detail: t("liveStatus.stopApplied"), tone: "neutral" };
  if (state.serve_armed) return { title: t("liveStatus.waitServe"), detail: t("liveStatus.serveHeld"), tone: "waiting" };
  if (state.receive_armed) return { title: t("liveStatus.waitBall"), detail: t("liveStatus.receiveHeld"), tone: "waiting" };
  if (state.state === "INVALID") return {
    title: t("liveStatus.noFlight"),
    detail: state.reason === "startup" ? t("liveStatus.waitFlight") : `${t("liveStatus.notForwarding")} · ${state.reason}`, tone: "neutral",
  };
  const titles = { SERVE: t("liveStatus.ourServe"), RALLY: t("liveStatus.ourRally"), OPPONENT_SERVE: t("liveStatus.theirServe"), OPPONENT_RALLY: t("liveStatus.theirRally") };
  if (state.observation_missing || ["ball_pose_missing", "ball_tracking_gap", "ball_observation_missing", "position_discontinuity", "source_clock_rebased"].includes(state.reason))
    return { title: titles[state.state], detail: t("liveStatus.paused"), tone: "waiting" };
  return { title: titles[state.state], detail: state.forwarding ? t("liveStatus.forwarding") : t("liveStatus.observing"),
    tone: state.forwarding ? "good" : "neutral" };
}

export type TimedValue = { value: unknown; receivedAt: number };
export function freshValue(value: TimedValue | undefined, now: number, maxAge = 1000): unknown {
  return value && now >= value.receivedAt && now - value.receivedAt < maxAge ? value.value : undefined;
}

export function poseDetails(value: unknown, now: number, t: Translator = defaultTranslator): string | undefined {
  const m = value as {header?: {stamp?: {sec?: number; nsec?: number; nanosec?: number}};
    pose?: {position?: {x?: number;y?: number;z?: number};orientation?: {x?:number;y?:number;z?:number;w?:number}}} | undefined;
  const stamp = m?.header?.stamp, p = m?.pose?.position, q = m?.pose?.orientation;
  if (!stamp || !p || !q) return undefined;
  const nsec = stamp.nsec ?? stamp.nanosec;
  if (!Number.isInteger(stamp.sec) || !Number.isInteger(nsec) || nsec! < 0 || nsec! >= 1e9) return undefined;
  const age = now - (stamp.sec! * 1000 + nsec! / 1e6);
  if (age < -100 || age > 1000 || ![p.x,p.y,p.z,q.x,q.y,q.z,q.w].every(v => typeof v === "number" && Number.isFinite(v))) return undefined;
  const norm = Math.hypot(q.x!,q.y!,q.z!,q.w!);
  if (norm < .5 || norm > 1.5) return undefined;
  const [x,y,z,w] = [q.x!/norm,q.y!/norm,q.z!/norm,q.w!/norm];
  const r = Math.atan2(2*(w!*x!+y!*z!),1-2*(x!*x!+y!*y!))*180/Math.PI;
  const pitch = Math.asin(Math.max(-1,Math.min(1,2*(w!*y!-z!*x!))))*180/Math.PI;
  const yaw = Math.atan2(2*(w!*z!+x!*y!),1-2*(y!*y!+z!*z!))*180/Math.PI;
  return t("liveStatus.pose", {x:p.x!.toFixed(3),y:p.y!.toFixed(3),z:p.z!.toFixed(3),roll:r.toFixed(1),pitch:pitch.toFixed(1),yaw:yaw.toFixed(1)});
}
