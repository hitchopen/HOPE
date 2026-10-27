// Both mocap relays publish metre player poses BEFORE the P2 view rotation.
export type PlayerPosition = { x: number; receivedAt: number; sec: number; nanosec: number };
const MAX_AGE_MS = 250;

export function decodePlayerPosition(message: unknown, now: number,
  previous?: PlayerPosition): PlayerPosition | undefined {
  const msg = message as { header?: { frame_id?: string; stamp?: { sec?: number; nanosec?: number } };
    pose?: { position?: { x?: number; y?: number; z?: number };
      orientation?: { x?: number; y?: number; z?: number; w?: number } } } | null;
  const stamp = msg?.header?.stamp, p = msg?.pose?.position, q = msg?.pose?.orientation;
  if (msg?.header?.frame_id !== 'hope_table_global' || !stamp || !p || !q) {
    return undefined;
  }
  const values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w];
  if (!values.every(v => typeof v === 'number' && Number.isFinite(v)) ||
      !Number.isInteger(stamp.sec) || !Number.isInteger(stamp.nanosec) ||
      (stamp.sec ?? 0) <= 0 || (stamp.nanosec ?? -1) < 0 || (stamp.nanosec ?? 1e9) >= 1e9) {
    return undefined;
  }
  const norm = Math.hypot(q.x!, q.y!, q.z!, q.w!);
  if (norm < .5 || norm > 1.5) {
    return undefined;
  }
  if (previous && (stamp.sec! < previous.sec ||
      (stamp.sec === previous.sec && stamp.nanosec! <= previous.nanosec))) {
    return previous;
  }
  return { x: p.x!, receivedAt: now, sec: stamp.sec!, nanosec: stamp.nanosec! };
}

export function playerSideAvailable(side: 'P1' | 'P2', players: (PlayerPosition | undefined)[], now: number): boolean {
  return players.some(p => p != undefined && now >= p.receivedAt && now-p.receivedAt <= MAX_AGE_MS &&
    (side === 'P1' ? p.x < 0 : p.x > 2.74));
}
