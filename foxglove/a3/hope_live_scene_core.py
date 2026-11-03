"""Read-only visualization lifecycle. A tracking gap is never a new shot.

All coordinates are table coordinates; only wire target/base z subtract 0.76.
No methods in this module publish controls or alter the play-state machine.
"""
from collections import deque
import math


def finite_vec(value, n=3):
    return isinstance(value, (list, tuple)) and len(value) == n and all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in value)


class LiveSceneState:
    def __init__(self):
        self.session_id = ""
        self.producer = ""
        self.generation = 0
        self.started_s = 0.
        self.target = None
        self.frozen = None
        self.policy_frozen = None
        self.policy_sample = None
        self.previous = None
        self.trail = deque(maxlen=2500)
        self.plane_x = None
        self.plane_stamp = 0.
        self.play_state = None
        self.play_state_received = 0.
        self.last_command_seq = 0
        self.last_command_stamp = 0.
        self.incoming_since = None
        self.events = deque(maxlen=100)

    def begin(self, timestamp, reason):
        self.generation += 1
        self.started_s = timestamp
        self.target = self.frozen = self.policy_frozen = self.policy_sample = None
        self.events.append(dict(generation=self.generation, timestamp_s=timestamp, reason=reason))

    def status(self, value, now):
        identity = (str(value.get('session_id', '')), str(value.get('producer_instance_id', '')))
        if identity != (self.session_id, self.producer):
            self.session_id, self.producer = identity
            self.generation = 0
            self.target = self.frozen = self.policy_frozen = self.policy_sample = None
            self.previous = None
            self.trail.clear()
            self.plane_x = None
            self.plane_stamp = 0.
            self.last_command_seq = 0
            self.last_command_stamp = 0.
            self.incoming_since = None
        self.play_state = dict(value)
        self.play_state_received = now

    def set_plane(self, values, now):
        if len(values) >= 3 and values[0] == 1 and all(math.isfinite(x) for x in values[:3]):
            if -.1 <= now-values[1] <= 2 and values[1] > self.plane_stamp:
                self.plane_stamp, self.plane_x = values[1], values[2]

    def command(self, values, now):
        if len(values) < 19 or values[0] not in (2, 3, 4, 5) or values[1] != 1:
            return False
        if not all(math.isfinite(x) for x in values):
            return False
        stamp = values[12] + values[13]*1e-9
        if not (-.1 <= now-stamp <= 1 and now-.1 <= values[10] <= now+3):
            return False
        if stamp < self.last_command_stamp or values[14] <= self.last_command_seq:
            return False
        self.last_command_seq, self.last_command_stamp = values[14], stamp
        flight = int(values[15])
        if self.target is None:
            # A ball can be observed (even cross the plane) before its command
            # arrives. Join that open shot instead of erasing its measured point.
            if self.generation and values[10] < self.started_s-.05:
                return False
            if self.generation == 0 or not (-.3 <= stamp-self.started_s <= 2):
                self.begin(stamp, 'new_target')
        elif self.target['flight_id'] != flight:
            # An occlusion can cause another solver flight ID. A new target
            # before the old strike deadline belongs to the same visible shot.
            if stamp > self.target['deadline_s'] + .05 or self.frozen is not None:
                self.begin(stamp, 'next_target')
        self.target = dict(generation=self.generation, flight_id=flight,
                           command_seq=int(values[14]), timestamp_s=stamp,
                           deadline_s=values[10], xyz=[values[3], values[4], values[5]-.76],
                           velocity=list(values[6:9]), swing_sign=values[2])
        return True

    def ball(self, stamp, xyz, now):
        if not finite_vec(xyz) or not math.isfinite(stamp) or not (-.1 <= now-stamp <= .3):
            return False
        previous = self.previous
        if previous is not None and stamp <= previous[0]:
            return False
        self.previous = (stamp, list(xyz))
        self.trail.append((stamp, list(xyz)))
        while self.trail and self.trail[0][0] < stamp-1.8:
            self.trail.popleft()
        if previous is None:
            return True
        dt = stamp-previous[0]
        if not (.001 <= dt <= .020001) or math.dist(xyz, previous[1])/dt > 35:
            self.incoming_since = None
            return True  # never interpolate a ghost through occlusion/jump
        vx = (xyz[0]-previous[1][0])/dt
        plane = self.target['xyz'][0] if self.target else self.plane_x
        if plane is None:
            return True
        # A single noisy reverse sample from the old ball on the floor/table
        # is not evidence of a new flight. Require a continuous visible approach
        # in the playing area. Late first visibility is still caught by the
        # measured plane crossing below, or by a new Planner command.
        approaching = (vx < -.5 and plane+.10 < xyz[0] <= 2.74
                       and -1.575 <= xyz[1] <= .05 and .02 <= xyz[2] <= 2.5)
        if approaching:
            if self.incoming_since is None:
                self.incoming_since = previous[0]
        else:
            self.incoming_since = None
        completed = self.frozen is not None or (self.target and stamp > self.target['deadline_s']+.25)
        if (approaching and stamp-self.incoming_since >= .05-1e-6
                and (self.generation == 0 or completed)):
            self.begin(self.incoming_since, 'observed_next_incoming')
            plane = self.plane_x if self.plane_x is not None else plane
        if previous[1][0] > plane >= xyz[0] and vx < 0:
            fraction = (plane-previous[1][0])/(xyz[0]-previous[1][0])
            crossing = previous[0] + fraction*dt
            pos = [previous[1][i]+fraction*(xyz[i]-previous[1][i]) for i in range(3)]
            # Do not replace a useful hit-plane point with the old ball rolling
            # under the table or being carried far outside the playing area.
            if not (-1.675 <= pos[1] <= .15 and .02 <= pos[2] <= 2.5):
                return True
            if self.frozen is not None and crossing-self.frozen['timestamp_s'] < .10:
                return True
            if self.generation == 0 or self.frozen is not None:
                self.begin(previous[0], 'observed_plane_crossing')
            self.frozen = dict(generation=self.generation, timestamp_s=crossing,
                               ball_xyz=pos, bracket_s=[previous[0], stamp], plane_x=plane)
        return True

    def freeze_policy(self, sample, racket_xyz, target_time):
        if self.target is None or self.policy_frozen is not None or sample is None:
            return False
        age = target_time-sample['timestamp_s']
        if not (0 <= age <= .030001):
            return False
        self.policy_sample = sample
        self.policy_frozen = dict(generation=self.generation, timestamp_s=target_time,
            racket_xyz=list(racket_xyz), delta_xyz=[racket_xyz[i]-self.target['xyz'][i] for i in range(3)],
            sample_age_ms=age*1000)
        return True

    def snapshot(self, now):
        state_age = None if not self.play_state_received else (now-self.play_state_received)*1000
        return dict(timestamp_s=now, session_id=self.session_id, generation=self.generation,
            shot=self.generation, play_state=self.play_state if state_age is not None and state_age<1000 else None,
            play_state_age_ms=state_age, robot_available=False, robot_age_ms=None,
            robot_xyz=None, robot_rpy_deg=None, racket_xyz=None, racket_speed=None,
            ball_xyz=self.previous[1] if self.previous and 0<=now-self.previous[0]<.1 else None,
            target_xyz=self.target['xyz'] if self.target else None,
            target_swing_sign=self.target['swing_sign'] if self.target else None,
            frozen=self.frozen, policy_frozen=self.policy_frozen)
