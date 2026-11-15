"""Software joint coordinates; no serial access and no register writes.

The encoder phase at the manually arranged pose defines logical zero. EEPROM
Homing Offset is removed on read, independently of its value. Angle tracking
unwraps successive samples; command conversion uses the currently observed
servo branch and refuses goals outside the selected operating mode's limits.
"""
from __future__ import annotations

import math

TICKS = 4096
RADIANS_PER_TICK = math.tau / TICKS


def wrap_ticks(value):
    return (value + TICKS / 2) % TICKS - TICKS / 2


class JointCalibration:
    def __init__(self, profile):
        if profile.get('schema_version') != 1:
            raise ValueError('Unsupported calibration profile')
        self.joints = {int(k): v for k, v in profile['joints'].items()}
        for joint in self.joints.values():
            if joint['direction'] not in (-1, 1):
                raise ValueError('Joint direction must be +1 or -1')
            if not 0 <= joint['zero_encoder_phase'] < TICKS:
                raise ValueError('Reference encoder phase must be within one turn')
            if not math.isfinite(joint['angle_at_reference_rad']):
                raise ValueError('Reference angle must be finite')
        self.phase = {}
        self.offset_ticks = {}
        self.angles = {}

    def update(self, positions, homing_offsets):
        offsets = {int(k): int(v) for k, v in homing_offsets.items()}
        updated = {}
        for key, present in positions.items():
            sid = int(key)
            if sid not in self.joints or sid not in offsets:
                continue
            joint = self.joints[sid]
            phase = (int(present) - offsets[sid]) % TICKS
            if sid in self.phase:
                self.offset_ticks[sid] += wrap_ticks(phase - self.phase[sid])
            else:
                self.offset_ticks[sid] = wrap_ticks(phase - joint['zero_encoder_phase'])
            self.phase[sid] = phase
            angle = joint['angle_at_reference_rad'] + joint['direction'] * self.offset_ticks[sid] * RADIANS_PER_TICK
            self.angles[sid] = angle
            updated[sid] = angle
        return updated

    def goal_for_angle(self, sid, target_rad, *, present_position, current_angle_rad,
                       operating_mode, min_position, max_position, joint_limits_rad):
        """Compute only. Caller must validate current samples and send separately.

        Never wrap or clamp a control goal into range: that can select the wrong
        turn. Bind to the current present-position branch after any mode change,
        reboot or torque transition, rather than reusing a pre-transition goal.
        """
        if not all(math.isfinite(v) for v in (target_rad, current_angle_rad)):
            raise ValueError('Angles must be finite')
        if joint_limits_rad is None:
            raise ValueError('Physical joint limits must be verified before control')
        lo, hi = joint_limits_rad
        if not lo <= current_angle_rad <= hi or not lo <= target_rad <= hi:
            raise ValueError('Current angle or target is outside verified joint limits')
        joint = self.joints[int(sid)]
        goal = int(round(present_position + joint['direction'] *
                         (target_rad - current_angle_rad) / RADIANS_PER_TICK))
        if operating_mode == 3:
            lower, upper = max(0, min_position), min(4095, max_position)
        elif operating_mode in (4, 5):
            lower, upper = -1048575, 1048575
        else:
            raise ValueError('This operating mode does not accept position targets')
        if not lower <= goal <= upper:
            raise ValueError(f'Goal {goal} is outside mode {operating_mode} range [{lower}, {upper}]')
        return goal
