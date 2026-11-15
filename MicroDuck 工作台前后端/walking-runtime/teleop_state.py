"""Manual walking commands: stale motion becomes zero, emergency stop is latched."""
import math
import threading
import time


class TeleopState:
    def __init__(self, *, clock=time.monotonic, step_speed=.05):
        if type(step_speed) not in (int,float) or step_speed not in (.05,.1,.15,.2,.3):
            raise ValueError('Unsupported configured step speed')
        self.clock = clock
        self.step_speed=float(step_speed)
        self.lock = threading.RLock()
        self.last_seen = self.last_drive = None
        self.sequence = -1
        self.twist = (0., 0., 0.)
        self.ready = False
        self.unlocked = False
        self.stopped = None
        self.stand_pitch_deg = 0.
        self.stand_height_mm = 0.
        self.pulse_until = None
        self.recovering = False
        self.recovery_confirmation_required = False

    def receive(self, value):
        if not isinstance(value, dict):
            raise ValueError('Command must be an object')
        action=value.get('action')
        with self.lock:
            if action=='off':
                self.stopped='Operator requested torque-off'
                self.twist=(0.,0.,0.)
                return
            if self.stopped:
                raise ValueError('Session is stopped; it cannot be rearmed')
            if action=='heartbeat':
                self.last_seen=self.clock()
                return
            if action=='confirm_balance':
                if not self.ready or self.recovering:raise ValueError('Policy and loop health are not ready')
                if self.stand_pitch_deg:
                    raise ValueError('Return standing pitch to zero before confirming independent standing')
                if self.stand_height_mm:
                    raise ValueError('Return standing height to zero before confirming independent standing')
                self.unlocked=True
                self.recovery_confirmation_required=False
                self.last_seen=self.clock()
                return
            if action=='stand_pitch':
                pitch=value.get('degrees')
                if type(pitch) not in (int,float) or not math.isfinite(pitch) or not -6<=pitch<=16:
                    raise ValueError('Standing pitch must be -6..16 degrees')
                if self.unlocked or not self.ready:
                    raise ValueError('Pitch adjustment requires a ready, supported standing session')
                if abs(pitch-self.stand_pitch_deg)>2.000001:
                    raise ValueError('Change standing pitch by at most 2 degrees per step')
                self.stand_pitch_deg=float(pitch)
                self.ready=False
                self.last_seen=self.clock()
                return
            if action=='stand_height':
                height=value.get('millimeters')
                if type(height) not in (int,float) or not math.isfinite(height) or not -10<=height<=0:
                    raise ValueError('Standing height must be -10..0 millimeters')
                if self.unlocked or not self.ready:
                    raise ValueError('Height adjustment requires a ready, supported standing session')
                if abs(height-self.stand_height_mm)>1.000001:
                    raise ValueError('Change standing height by at most 1 millimeter per step')
                self.stand_height_mm=float(height)
                self.ready=False
                self.last_seen=self.clock()
                return
            if action=='step':
                vx=value.get('vx');seconds=value.get('seconds')
                if not self.unlocked or not self.ready or self.recovery_confirmation_required:
                    raise ValueError('Confirm stable dynamic balance before a step')
                if type(vx) not in (int,float) or vx not in (-self.step_speed,self.step_speed):
                    raise ValueError('A single step must match the configured forward/backward speed')
                if type(seconds) not in (int,float) or not math.isfinite(seconds) or not .1<=seconds<=.5:
                    raise ValueError('A single step lasts at most 0.5 seconds')
                if self.pulse_until is not None and self.clock()<self.pulse_until:
                    raise ValueError('Wait until the current step finishes')
                self.twist=(vx,0.,0.);self.pulse_until=self.clock()+seconds
                self.last_drive=self.last_seen=self.clock()
                return
            if action!='drive':raise ValueError('Unknown command')
            sequence=value.get('sequence')
            if type(sequence) is not int or sequence<=self.sequence:
                raise ValueError('Out-of-order drive command')
            numbers=[value.get(k) for k in ('vx','vy','vyaw')]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in numbers):
                raise ValueError('Velocity must be finite')
            vx,vy,vyaw=numbers
            if abs(vx)>1. or vy!=0 or abs(vyaw)>.6:
                raise ValueError('Velocity outside manual-control range')
            if any(numbers) and (not self.unlocked or self.recovery_confirmation_required):
                raise ValueError('Confirm independent standing before walking')
            self.sequence=sequence
            self.pulse_until=None
            self.twist=tuple(numbers)
            self.last_drive=self.last_seen=self.clock()

    def command(self):
        with self.lock:
            if self.recovery_confirmation_required:
                return (0.,0.,0.)
            if self.pulse_until is not None:
                if not self.stopped and self.unlocked and self.clock()<self.pulse_until:
                    return self.twist
                self.pulse_until=None;self.twist=(0.,0.,0.)
            if self.stopped or not self.unlocked or self.last_drive is None or self.clock()-self.last_drive>.3:
                return (0.,0.,0.)
            return self.twist

    def observe_recovery(self, active):
        with self.lock:
            self.recovering=bool(active)
            if self.recovering:
                self.recovery_confirmation_required=True
                self.ready=False
                self.twist=(0.,0.,0.)
                self.pulse_until=None
                self.last_drive=None

    def abort_reason(self):
        with self.lock:
            if self.stopped:return self.stopped
            # During the supported bring-up, losing the operator must clean up.
            # After independent standing is confirmed, stale commands only stop
            # walking; the on-board policy remains alive to balance the robot.
            if not self.unlocked and self.last_seen is not None and self.clock()-self.last_seen>.8:
                return 'Operator connection lost during supported bring-up'
            return None

    def public(self):
        with self.lock:
            return {'drive_unlocked':self.unlocked and not self.recovery_confirmation_required,'can_confirm_balance':self.ready and not self.recovering and self.stand_pitch_deg==0 and self.stand_height_mm==0 and not self.stopped,
                    'native_recovery_active':self.recovering,
                    'recovery_confirmation_required':self.recovery_confirmation_required,
                    'can_adjust_pitch':self.ready and not self.unlocked and not self.stopped,
                    'can_adjust_height':self.ready and not self.unlocked and not self.stopped,
                    'stand_pitch_deg':self.stand_pitch_deg,
                    'stand_height_mm':self.stand_height_mm,
                    'pulse_active':self.pulse_until is not None and self.clock()<self.pulse_until,
                    'command':list(self.command()),'stop_reason':self.stopped,
                    'operator_age_s':None if self.last_seen is None else self.clock()-self.last_seen}

    def pose(self):
        with self.lock:
            # Native robotd zeroes velocity while pose mode is active. Leave it
            # only after zero pitch has settled and the operator unlocks walking.
            return {'z':self.stand_height_mm/1000.,'roll':0.,'pitch':math.radians(self.stand_pitch_deg),'active':not self.unlocked}

    def set_ready_for_pose(self, applied_pose, ready):
        with self.lock:
            # A newly accepted command must not inherit the previous pose's
            # settled window while the control loop is finishing its old tick.
            self.ready=bool(ready and not self.stopped and not self.recovering and applied_pose==self.pose())
