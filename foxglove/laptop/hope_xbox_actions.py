"""ROS-free Xbox button edges and receipt-driven fixed Runner action sequences."""
from collections import deque


# Verified on the USB xpad Xbox Series controller and Linux input-event-codes.h.
# BTN_X=307 (alias BTN_NORTH), BTN_Y=308 (alias BTN_WEST): cardinal names
# must NOT be interpreted as the physical location of Xbox letter buttons.
# https://github.com/torvalds/linux/blob/master/drivers/input/joystick/xpad.c
FACE_BUTTON_CODES = {'A': 304, 'B': 305, 'X': 307, 'Y': 308}


def face_button_names(keys):
    return {name for name, code in FACE_BUTTON_CODES.items() if code in keys}


class ButtonEdges:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = set()
        self.armed = False

    def sample(self, connected, buttons, modifier=True):
        buttons = set(buttons)
        if not connected:
            self.reset()
            return None
        rising = buttons - self.previous
        self.previous = buttons
        if not buttons:
            self.armed = True
        # A held button on reconnect/restart, or an ambiguous chord, is not
        # an operator edge. Pressing LT after an already held face key is not one.
        if not self.armed or len(buttons) != 1 or len(rising) != 1 or not modifier:
            return None
        return next(iter(rising))


class ActionSequence:
    """Only proposes existing Trigger services; never produces joint commands.

    One sequence at a time. Every intermediate step needs BOTH its service
    success and an authoritative Runner state before the next request can run.
    No retry or delayed replay after disconnect, timeout, fault or a new boot.
    """
    SERVICES = {
        'stand': 'enter_pd_stand', 'server': 'set_server', 'receiver': 'set_receiver',
        'prepare': 'prepare_serve', 'play': 'ready_to_serve',
        'ready': 'enter_motion', 'teleop': 'enter_teleop',
    }

    def __init__(self):
        self.steps = deque()
        self.inflight = None
        self.accepted = False
        self.boot = None
        self.button = None
        self.deadline = 0.
        self.status = 'A prepare · B serve · X receive Ready · Y Stand then Teleop'

    @property
    def busy(self):
        return bool(self.steps)

    def cancel(self, reason):
        self.steps.clear()
        self.inflight = None
        self.accepted = False
        self.status = reason

    def request(self, button, state, now):
        if self.busy:
            self.status = f'{self.button} in progress; release and press {button} again when done'
            return False
        if state is None or state.command_fault_latched:
            self.status = 'No fresh healthy Runner; button ignored'
            return False
        mode, role = state.run_mode, state.local_role
        steps = []
        if button == 'A':
            if state.serve_capability != 'AVAILABLE':
                self.status = 'Serve controller unavailable'
                return False
            if mode == 'SERVE':
                self.status = 'Serve already selected; use B when the ball is loaded'
                return False
            if not (role == 'SERVER' and mode in ('PD_STAND', 'MOTION')):
                if mode != 'PD_STAND':
                    steps.append('stand')
                if role != 'SERVER':
                    steps.append('server')
            steps.append('prepare')
        elif button == 'B':
            if mode != 'SERVE' or role != 'SERVER' or state.serve_state != 'WAIT_READY_TO_SERVE':
                self.status = 'B ignored: wait for the loading pose, load the ball, then press B'
                return False
            steps = ['play']
        elif button == 'X':
            if mode == 'MOTION' and role == 'RECEIVER':
                self.status = 'RECEIVER is already Ready'
                return False
            if mode != 'PD_STAND':
                steps.append('stand')
            if role != 'RECEIVER':
                steps.append('receiver')
            steps.append('ready')
        elif button == 'Y':
            if mode == 'TELEOP':
                self.status = 'Teleop already selected; release LT, center sticks, then hold LT'
                return False
            if mode != 'PD_STAND':
                steps.append('stand')
            steps.append('teleop')
        else:
            return False
        self.steps = deque(steps)
        self.boot = state.boot_id
        self.start_action_id = state.last_action_id
        self.button = button
        self.status = f'{button}: {steps[0]}'
        self.deadline = now + 20.
        return True

    def acknowledge(self, success, message=''):
        if self.inflight is None:
            return
        if not success:
            self.cancel(f'{self.button} rejected: {message}')
        else:
            self.accepted = True

    @staticmethod
    def reached(step, state):
        if step == 'stand':
            return state.run_mode == 'PD_STAND'
        if step in ('server', 'receiver'):
            return state.run_mode == 'PD_STAND' and state.local_role == step.upper()
        if step == 'prepare':
            return state.run_mode == 'SERVE' and state.serve_state in ('PREPARING_STAND', 'TRANSITION_TO_LOAD', 'WAIT_READY_TO_SERVE')
        if step == 'play':
            return (state.run_mode == 'SERVE' and state.serve_state in ('PLAYING_PRE_RELEASE', 'RELEASE_PENDING', 'STRIKE', 'FOLLOW_THROUGH', 'RECOVERY')) or (state.run_mode in ('MOTION', 'TELEOP', 'PD_STAND') and state.serve_state == 'COMPLETE')
        return state.run_mode == {'ready':'MOTION', 'teleop':'TELEOP'}[step]

    def advance(self, state, now, connected=True):
        if not self.busy:
            return None
        if not connected or state is None or state.boot_id != self.boot or state.command_fault_latched:
            self.cancel('Button sequence cancelled: controller or Runner state lost; press again')
            return None
        if now > self.deadline:
            self.cancel('Button sequence timed out; inspect Runner, then press again')
            return None
        if state.last_action == 'EMERGENCY_PASSIVE' and state.last_action_id != self.start_action_id:
            self.cancel('Button sequence cancelled by Passive')
            return None
        if self.inflight is not None:
            if not self.accepted or not self.reached(self.steps[0], state):
                self.status = f'{self.button}: waiting for {self.steps[0]} · {state.run_mode}'
                return None
            self.steps.popleft()
            self.inflight = None
            self.accepted = False
            if not self.steps:
                self.status = {'A':'A accepted: preparing serve; load ball, clear hands, then press B',
                               'B':'B accepted: serving; Kernel automatically lowers arms to Stand, normal play to Ready',
                               'X':'X complete: RECEIVER Ready',
                               'Y':'Y complete: Teleop selected; wait ACTIVE, release then hold LT'}[self.button]
                return None
        self.inflight = self.steps[0]
        self.deadline = now + 20.
        return self.SERVICES[self.inflight]


class EmergencyChord:
    """LB+RB asserts the existing E-stop; release alone never clears its latch."""
    def __init__(self):
        self.held = False
        self.latched = False
        self.remote_asserted = False
        self.pending = False
        self.status = ''

    def sample(self, lb, rb):
        held = lb and rb
        if held and not self.held:
            self.latched = True
            self.pending = True
            self.status = 'E-STOP requested by LB + RB'
        self.held = held

    def observe_latch(self, asserted):
        if asserted:
            if not self.remote_asserted:
                self.status = 'E-STOP LATCHED · click Reset Software E-stop in Foxglove'
            self.remote_asserted = True
            self.latched = True
        elif self.remote_asserted and not self.held:
            # Only an authoritative clear AFTER an observed assertion can clear
            # our local inhibit. Old false packets during the request cannot.
            self.remote_asserted = False
            self.latched = False
            self.pending = False
            self.status = 'Software E-stop reset; release controls before re-entering'
