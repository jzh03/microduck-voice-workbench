"""Reversible policy coordinate reference; original calibration and IMU stay intact."""
import hashlib
import json
import math
import time
from pathlib import Path

from policy_coordinates import HOME, IDS


def vector(values):
    if (not isinstance(values, (list, tuple)) or len(values) != len(IDS) or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError('Reference needs 15 finite joint angles')
    return list(values)


class StanceReference:
    def __init__(self, value, profile_sha256, contract_sha256):
        if (value.get('profile_sha256') != profile_sha256 or
                value.get('contract_sha256') != contract_sha256 or
                value.get('kind') not in ('operator_standing_reference_v1','model_home_reference_v1')):
            raise ValueError('Standing reference does not match calibration and model contract')
        self.pose = vector(value['joint_radians'])
        self.bias = [q - home for q, home in zip(self.pose, HOME)]
        if value['kind']=='model_home_reference_v1' and any(abs(v)>1e-12 for v in self.bias):
            raise ValueError('Model HOME reference cannot contain standing-pose offsets')
        if max(map(abs, self.bias)) > math.radians(8) + 1e-9:
            raise ValueError('Standing reference is more than 8 degrees from model HOME')
        self.value = value

    @classmethod
    def load(cls, path, profile_sha256, contract_sha256):
        return cls(json.loads(Path(path).read_text()), profile_sha256, contract_sha256)

    def to_policy(self, physical):
        return [q - offset for q, offset in zip(vector(physical), self.bias)]

    def to_physical(self, policy):
        return [q + offset for q, offset in zip(vector(policy), self.bias)]


def from_saved(saved, profile, contract, source):
    state = saved['state']
    targets = state.get('commanded_joint_degrees', {})
    if set(targets) != set(map(str, IDS)) or state.get('healthy') is not True:
        raise ValueError('Saved stance needs healthy, complete commanded joints')
    pose = []
    for sid in IDS:
        q = targets[str(sid)]
        if type(q) not in (int, float) or not math.isfinite(q):
            raise ValueError('Saved stance contains invalid joint angles')
        pose.append(math.radians(q))
    value = {'kind': 'operator_standing_reference_v1', 'joint_radians': pose,
             'profile_sha256': hashlib.sha256(profile).hexdigest(),
             'contract_sha256': hashlib.sha256(contract).hexdigest(),
             'source_saved_stance': str(source), 'saved_unix': saved['saved_unix'],
             'mechanical_zero_verified': False, 'dynamic_walking_verified': False,
             'imu_rotation_changed': False, 'automatic_start': False}
    StanceReference(value, value['profile_sha256'], value['contract_sha256'])
    return value


class PolicyReferenceHardware:
    """Only RemoteIo joint positions/goals change coordinates; guards use real angles."""
    def __init__(self, hardware, reference, *, record_targets_only=False):
        if type(record_targets_only) is not bool:
            raise ValueError('Target recording mode must be explicit')
        self.hardware, self.reference = hardware, reference
        self.record_targets_only = record_targets_only
        self.handoff_diagnostic = None
        self.handoff_until = 0.
        self.handoff_limit = math.radians(12.)
        self.handoff_rejection = None

    def begin_handoff(self, *, clock=time.monotonic):
        self.handoff_clock=clock
        self.handoff_until=clock()+1.

    def __getattr__(self, name):
        return getattr(self.hardware, name)

    def fallback_targets(self):
        return self.reference.pose[:]

    def dispatch(self, request):
        if request.get('op') == 'read':
            frame = self.hardware.dispatch(request)
            if hasattr(self.hardware,'record'):
                self.hardware.record('policy_sensor_input',physical=frame,
                                     policy_positions=self.reference.to_policy(frame['positions']))
            return {**frame, 'positions': self.reference.to_policy(frame['positions'])}
        if request.get('op') == 'write':
            native_targets=vector(request.get('targets'))
            if any(abs(v)>math.pi+1e-8 for v in native_targets):
                raise ValueError('Native targets exceed actuator range before reference mapping')
            mapped=self.reference.to_physical(native_targets)
            # Native actuator bounds live in policy coordinates. Preserve the
            # bias after that validation, including at +/-pi; do not clip away
            # the PD effort requested by the official policy.
            targets=mapped
            if hasattr(self.hardware,'record'):
                self.hardware.record('native_target',native_targets=native_targets,
                                     physical_targets=mapped,actuator_targets=targets[:],
                                     physical_actuator_clamped_ids=[],
                                     target_gap_diagnostic_only=self.record_targets_only)
            if self.handoff_rejection is not None:
                return self.hardware.dispatch({**request,'targets':self.fallback_targets()})
            if self.handoff_until and self.handoff_clock()<self.handoff_until:
                measured=vector(self.hardware.frame['positions'])
                errors=[abs(goal-actual) for goal,actual in zip(targets,measured)]
                index=max(range(len(IDS)),key=errors.__getitem__)
                if self.handoff_diagnostic is None or errors[index]>math.radians(self.handoff_diagnostic['max_error_degrees']):
                    self.handoff_diagnostic={'joint_id':IDS[index],
                        'max_error_degrees':math.degrees(errors[index]),
                        'physical_targets':targets[:],
                        'target_gap_blocks_policy':not self.record_targets_only}
                if not self.record_targets_only and errors[index]>self.handoff_limit+1e-9:
                    self.handoff_rejection={'joint_id':IDS[index],
                        'error_degrees':math.degrees(errors[index]),'rejected_physical_targets':targets,
                        'fallback':'hold_verified_saved_stance'}
                    return self.hardware.dispatch({**request,'targets':self.fallback_targets()})
            return self.hardware.dispatch({**request, 'targets': targets})
        return self.hardware.dispatch(request)


def ready_for_handoff(measured, expected, gravity, hz, native_policy, *, floor_recovery=False):
    actual,target=vector(measured),vector(expected)
    if (not isinstance(gravity,(list,tuple)) or len(gravity)!=3 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in gravity)):
        return False
    norm=math.sqrt(sum(v*v for v in gravity))
    if not .5<=norm<=1.5:return False
    tilt=math.degrees(math.acos(max(-1.,min(1.,-gravity[2]/norm))))
    return (native_policy=='held' and hz>=45 and
            (floor_recovery or (tilt<=12.+1e-9 and
             max(abs(a-b) for a,b in zip(actual,target))<=math.radians(6.)+1e-9)))
