"""State and bounded motion for an explicitly supported, policy-free stance trial."""
import math
import threading
import time

from policy_coordinates import HOME, IDS


PAIRS = {'hip': (22, 12), 'knee': (23, 13), 'ankle': (24, 14)}
STEP_DEG = .5
MAX_TRIM_DEG = 8.
STEP_MM = .5
MAX_SHIFT_MM = 5.
HEARTBEAT_SECONDS = .8


def angle_vector(values):
    if isinstance(values, dict):
        converted = {int(k): v for k, v in values.items()}
        if set(converted) != set(IDS):
            raise ValueError('A complete 15-joint pose is required')
        values = [converted[sid] for sid in IDS]
    if not isinstance(values, (list, tuple)) or len(values) != len(IDS):
        raise ValueError('A complete 15-joint pose is required')
    if any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > math.pi for v in values):
        raise ValueError('Joint angles must be finite actuator-range radians')
    return [float(v) for v in values]


def finite_number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + ' must be a finite number')
    return float(value)


class StanceState:
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.last_seen = None
        self.stopped = None
        self.ready = False
        self.moving = False
        self.pending = None
        self.q0 = None
        self.target = None
        self.home = tuple(HOME)
        self.group_offsets_deg = {name: 0. for name in PAIRS}
        self.body_shift_mm = 0.
        self.last_command_error = None
        self.revision = 0

    def initialize(self, measured, reset_target=None):
        values = angle_vector(measured)
        baseline = angle_vector(reset_target) if reset_target is not None else values
        with self.lock:
            if self.q0 is not None:
                raise RuntimeError('Startup pose is immutable for this session')
            self.q0 = tuple(baseline)
            self.target = baseline[:]
            self.home = tuple(baseline) if reset_target is not None else tuple(HOME)
            self.ready = False

    def abort_reason(self):
        with self.lock:
            if self.stopped:
                return self.stopped
            if self.last_seen is not None and self.clock()-self.last_seen > HEARTBEAT_SECONDS:
                self.stopped = 'Operator connection lost during supported stance control'
                self.ready = False
                self.pending = None
                return self.stopped
            return None

    def receive(self, value):
        if not isinstance(value, dict):
            raise ValueError('Command must be an object')
        action = value.get('action')
        with self.lock:
            if action == 'off':
                self.stopped = 'Operator requested torque-off'
                self.ready = False
                self.pending = None
                return
            if self.abort_reason():
                raise ValueError('Session is stopped; it cannot be rearmed')
            if action == 'heartbeat':
                self.last_seen = self.clock()
                return
            if action not in ('adjust_joint', 'body_shift', 'home', 'capture', 'reset'):
                raise ValueError('Unknown stance command; walking is unavailable')
            if not self.ready or self.moving or self.pending is not None or self.q0 is None:
                raise ValueError('Wait for the supported pose to settle before adjusting')
            command = {'action': action}
            if action == 'adjust_joint':
                group = value.get('group')
                if group not in PAIRS:
                    raise ValueError('Joint group must be hip, knee or ankle')
                degrees = finite_number(value.get('degrees'), 'Joint trim')
                if abs(degrees) > MAX_TRIM_DEG or abs(degrees-self.group_offsets_deg[group]) > STEP_DEG+1e-7:
                    raise ValueError('Joint trim must be within +/-8 degrees, at most 0.5 degree per step')
                command.update(group=group, degrees=degrees)
            elif action == 'body_shift':
                mm = finite_number(value.get('millimeters'), 'Body shift')
                if abs(mm) > MAX_SHIFT_MM or abs(mm-self.body_shift_mm) > STEP_MM+1e-7:
                    raise ValueError('Body shift must be within +/-5 mm, at most 0.5 mm per step')
                command['millimeters'] = mm
            self.revision += 1
            command['revision'] = self.revision
            self.pending = command
            self.ready = False
            self.moving = True
            self.last_command_error = None
            self.last_seen = self.clock()

    def take_command(self):
        with self.lock:
            command = self.pending
            self.pending = None
            return command

    def plan(self, command, measured, gravity, *, shift_function=None):
        actual = angle_vector(measured)
        with self.lock:
            if self.target is None or self.q0 is None:
                raise RuntimeError('No startup pose')
            target = self.target[:]
            offsets = self.group_offsets_deg.copy()
            shift = self.body_shift_mm
            q0 = self.q0
        # A body-shift solve may run on a slower board for many loop ticks.
        # Never hold the command/heartbeat/stop lock during pure computation.
        action = command['action']
        details = None
        if action == 'adjust_joint':
            group = command['group']
            delta = math.radians(command['degrees']-offsets[group])
            left, right = PAIRS[group]
            target[IDS.index(left)] += delta
            target[IDS.index(right)] -= delta
            offsets[group] = command['degrees']
        elif action == 'body_shift':
            if shift_function is None:
                raise ValueError('Body-shift solver is unavailable')
            distance = command['millimeters']-shift
            details = shift_function(dict(zip(IDS, actual)), list(gravity), distance)
            solved = angle_vector(details['angles'])
            # The IK baseline is the fresh measured leg pose. Apply only its
            # increment to the held goal, preserving existing load deflection.
            # Other joints continue holding their existing goals as well.
            for sid in (22, 23, 24, 12, 13, 14):
                index = IDS.index(sid)
                target[index] += solved[index]-actual[index]
            details = {**details, 'baseline_actual_rad': dict(zip(IDS, actual)),
                       'applied_delta_rad': {sid: solved[IDS.index(sid)]-actual[IDS.index(sid)]
                                             for sid in (22, 23, 24, 12, 13, 14)},
                       'final_target_rad': dict(zip(IDS, target))}
            achieved = finite_number(details['actual_shift_mm'], 'Achieved body shift')
            if abs(achieved) > abs(distance)+1e-6 or achieved*distance < -1e-8 or abs(shift+achieved) > MAX_SHIFT_MM+1e-6:
                raise ValueError('Body-shift solver returned an invalid displacement')
            shift += achieved
        elif action in ('home', 'reset', 'capture'):
            target = list(self.home) if action == 'home' else list(q0) if action == 'reset' else actual
            offsets = {name: 0. for name in PAIRS}
            shift = 0.
        else:
            raise ValueError('Unknown stance motion')
        return {'command': command.copy(), 'target': angle_vector(target),
                'group_offsets_deg': offsets, 'body_shift_mm': shift, 'kinematics': details}

    def commit(self, plan):
        with self.lock:
            if self.abort_reason() or plan['command']['revision'] != self.revision:
                raise RuntimeError('Stance command was cancelled before motion')
            self.target = angle_vector(plan['target'])
            self.group_offsets_deg = plan['group_offsets_deg'].copy()
            self.body_shift_mm = plan['body_shift_mm']

    def reject(self, reason):
        with self.lock:
            self.last_command_error = str(reason)
            self.ready = False
            self.moving = False

    def update_gate(self, *, moving, settled):
        with self.lock:
            if self.pending is not None:
                self.moving = True
                self.ready = False
            else:
                self.moving = bool(moving)
                self.ready = bool(settled and not moving and not self.abort_reason())

    def public(self):
        with self.lock:
            allowed = bool(self.ready and not self.moving and self.pending is None and not self.stopped)
            return {'ready': allowed, 'can_adjust_pose': allowed, 'moving': self.moving,
                    'drive_unlocked': False, 'can_confirm_balance': False,
                    'group_offsets_deg': self.group_offsets_deg.copy(),
                    'body_shift_mm': self.body_shift_mm, 'stop_reason': self.stopped,
                    'last_command_error': self.last_command_error,
                    'operator_age_s': None if self.last_seen is None else self.clock()-self.last_seen}


class PoseRamp:
    def __init__(self, initial, *, speed_deg_s=8.):
        self.commanded = angle_vector(initial)
        self.target = self.commanded[:]
        self.speed = math.radians(finite_number(speed_deg_s, 'Ramp speed'))
        if not 0 < self.speed <= math.radians(8.):
            raise ValueError('Ramp speed must be within 0..8 degrees per second')

    def set_target(self, target):
        self.target = angle_vector(target)

    def advance(self, dt):
        # A delayed tick cannot cause a large catch-up jump.
        step = self.speed*min(.05, max(0., finite_number(dt, 'Loop interval')))
        delta = [goal-current for goal,current in zip(self.target,self.commanded)]
        distance = max(map(abs,delta))
        if distance:
            fraction = min(1.,step/distance)
            self.commanded = (self.target[:] if fraction == 1. else
                              [current+fraction*change for current,change in zip(self.commanded,delta)])
        return self.commanded[:]

    @property
    def moving(self):
        return any(abs(a-b) > 1e-10 for a, b in zip(self.commanded, self.target))


def posture_metrics(gravity):
    if not isinstance(gravity, (list, tuple)) or len(gravity) != 3:
        raise ValueError('A fresh three-axis gravity vector is required')
    values = [finite_number(v, 'Gravity') for v in gravity]
    norm = math.sqrt(sum(v*v for v in values))
    if not .5-1e-9 <= norm <= 1.5+1e-9:
        raise ValueError('Invalid gravity norm')
    x, y, z = [v/norm for v in values]
    return {'tilt_deg': math.degrees(math.acos(max(-1., min(1., -z)))),
            'pitch_deg': math.degrees(math.atan2(x, math.hypot(y, z))),
            'roll_deg': math.degrees(math.atan2(-y, -z))}


class TrackingGuard:
    def __init__(self):
        self.excess_since = None

    def check(self, commanded, measured, now):
        errors = [abs(math.degrees(a-b)) for a, b in zip(angle_vector(commanded), angle_vector(measured))]
        largest = max(errors)
        if largest > 12.+1e-9:
            if self.excess_since is None:
                self.excess_since = now
            if now-self.excess_since >= 2.:
                sid = IDS[errors.index(largest)]
                raise RuntimeError(f'ID {sid}: tracking error {largest:.1f} degrees exceeded 12 degrees for 2 seconds')
        else:
            self.excess_since = None
        return largest
