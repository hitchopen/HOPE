// Both mocap relays publish metre player poses BEFORE the P2 view rotation.
export type PlayerPosition = { x: number; receivedAt: number; sec: number; nanosec: number };
const MAX_AGE_MS = 250;

export function decodePlayerPosition(message: unknown, now: number,
  previous?: PlayerPosition): PlayerPosition | undefined {
  const msg = message as { header?: { frame_id?: string; stamp?: { sec?: number; nsec?: number; nanosec?: number } };
    pose?: { position?: { x?: number; y?: number; z?: number };
      orientation?: { x?: number; y?: number; z?: number; w?: number } } } | null;
  const stamp = msg?.header?.stamp, p = msg?.pose?.position, q = msg?.pose?.orientation;
  if (msg?.header?.frame_id !== 'hope_table_global' || !stamp || !p || !q) {
    return undefined;
  }
  // Foxglove normalizes ROS 2 Time to sec/nsec before passing it to panels.
  // Keep nanosec support for raw ROS 2 messages, using one value for validation
  // and replay checks regardless of the input representation.
  const nanosec = stamp.nsec !== undefined ? stamp.nsec : stamp.nanosec;
  const values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w];
  if (!values.every(v => typeof v === 'number' && Number.isFinite(v)) ||
      !Number.isInteger(stamp.sec) || !Number.isInteger(nanosec) ||
      (stamp.sec ?? 0) <= 0 || (nanosec ?? -1) < 0 || (nanosec ?? 1e9) >= 1e9) {
    return undefined;
  }
  const norm = Math.hypot(q.x!, q.y!, q.z!, q.w!);
  if (norm < .5 || norm > 1.5) {
    return undefined;
  }
  if (previous && (stamp.sec! < previous.sec ||
      (stamp.sec === previous.sec && nanosec! <= previous.nanosec))) {
    return previous;
  }
  return { x: p.x!, receivedAt: now, sec: stamp.sec!, nanosec: nanosec! };
}

export function playerSideAvailable(side: 'P1' | 'P2', players: (PlayerPosition | undefined)[], now: number): boolean {
  return players.some(p => p != undefined && now >= p.receivedAt && now-p.receivedAt <= MAX_AGE_MS &&
    (side === 'P1' ? p.x < 0 : p.x > 2.74));
}
