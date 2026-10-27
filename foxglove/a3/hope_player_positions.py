"""START-only table-end evidence in the unrotated, metre table frame.

Keep boundaries, freshness and replay behavior aligned with the Console's
playerPositions.ts and update both sets of contract tests together.
"""
import math

TABLE_LENGTH_M = 2.74
PLAYER_POSITION_MAX_AGE_S = 0.25
PLAYER_POSITION_FRAME = "hope_table_global"


class PlayerPositions:
    def __init__(self):
        self._samples = {}

    def update(self, player, position, quaternion, stamp_ns, frame, now):
        if player not in {"UCB_P1", "UCB_P2"}:
            return
        if (frame != PLAYER_POSITION_FRAME or type(stamp_ns) is not int or stamp_ns <= 0
                or not all(math.isfinite(v) for v in (*position, *quaternion, now))
                or len(position) != 3 or len(quaternion) != 4
                or not 0.5 <= math.sqrt(sum(v*v for v in quaternion)) <= 1.5):
            self._samples.pop(player, None)
            return
        previous = self._samples.get(player)
        # Duplicates/replays cannot keep a lost player alive. Coordinates may
        # remain identical while a stationary robot is genuinely being tracked.
        if previous is not None and stamp_ns <= previous[2]:
            return
        self._samples[player] = (position[0], now, stamp_ns)

    def status(self, now):
        live = {name: sample[0] for name, sample in self._samples.items()
                if 0 <= now - sample[1] <= PLAYER_POSITION_MAX_AGE_S}
        return {"frame": PLAYER_POSITION_FRAME, "table_length_m": TABLE_LENGTH_M,
                "max_age_s": PLAYER_POSITION_MAX_AGE_S, "players_x_m": live,
                "P1": any(x < 0.0 for x in live.values()),
                "P2": any(x > TABLE_LENGTH_M for x in live.values())}

    def require(self, side, now):
        if not self.status(now).get(side, False):
            boundary = "x < 0 m" if side == "P1" else "x > 2.74 m"
            raise ValueError(f"PLAYER_POSITION_UNVERIFIED: {side} requires a fresh tracked player at {boundary}")
