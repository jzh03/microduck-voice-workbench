"""Opt-in, supported preparation pose; original calibration and policy stay fixed.

The saved stance remains the recovery pose. A separate, profile-bound candidate
is reached while the native policy is disabled. The original 12-degree takeover
guard remains enabled. No torque operation is introduced by this adapter.
"""
import hashlib
import json
import math
from pathlib import Path
import time

from policy_coordinates import HOME
from stance_reference import PolicyReferenceHardware, vector
from stance_state import PoseRamp,TrackingGuard


def gravity_angles(g):
    if (not isinstance(g,(list,tuple)) or len(g)!=3 or
            any(type(x) not in (float,int) or not math.isfinite(x) for x in g)):
        raise ValueError('Expected finite gravity')
    norm=math.sqrt(sum(x*x for x in g))
    if not .5<=norm<=1.5:raise ValueError('Invalid gravity magnitude')
    x,y,z=(v/norm for v in g)
    return math.degrees(math.atan2(x,math.hypot(y,z))),math.degrees(math.atan2(-y,-z))


class HandoffPose:
    def __init__(self,value,reference,reference_sha256):
        if (value.get('kind')!='supported_handoff_pose_v1' or
                value.get('profile_sha256')!=reference.value['profile_sha256'] or
                value.get('contract_sha256')!=reference.value['contract_sha256'] or
                value.get('reference_sha256')!=reference_sha256 or
                value.get('requires_human_support') is not True):
            raise ValueError('Handoff pose does not match this saved stance and model')
        self.pose=vector(value.get('joint_radians'))
        if max(abs(a-b) for a,b in zip(self.pose,reference.pose))>math.radians(8)+1e-9:
            raise ValueError('Preparation pose exceeds eight degrees from saved stance')
        self.expected_gravity=vector_gravity=value.get('expected_gravity')
        self.pitch,self.roll=gravity_angles(vector_gravity)
        if abs(self.pitch)>6 or abs(self.roll)>6:
            raise ValueError('Preparation pose must predict a near-upright trunk')
        self.value=value

    @classmethod
    def load(cls,path,reference,reference_path):
        return cls(json.loads(Path(path).read_text()),reference,
                   hashlib.sha256(Path(reference_path).read_bytes()).hexdigest())

    def matches_gravity(self,actual):
        pitch,roll=gravity_angles(actual)
        return abs(pitch-self.pitch)<=3 and abs(roll-self.roll)<=3


class PreparedPoseHardware(PolicyReferenceHardware):
    def __init__(self,hardware,reference,plan,*,clock=time.monotonic):
        super().__init__(hardware,reference)
        self.plan=plan;self.clock=clock
        self.bridge_ramp=None;self.bridge_time=None
        self.await_policy=False;self.return_ramp=None;self.return_time=None
        self.first_policy_target_checked=False
        self.preparation_tracking=TrackingGuard()

    def prepare_bridge(self):
        if self.handoff_until or self.handoff_rejection is not None:
            raise ValueError('This supported session already attempted takeover')
        start=getattr(self.hardware,'last_commanded_targets',None) or self.reference.pose
        self.bridge_ramp=PoseRamp(start)
        self.bridge_ramp.set_target(self.plan.pose)
        self.bridge_time=self.clock()
        self.preparation_tracking=TrackingGuard()

    def reject_preparation(self,reason):
        errors=[abs(a-b) for a,b in zip(self.plan.pose,self.hardware.frame['positions'])]
        index=max(range(len(errors)),key=errors.__getitem__)
        from policy_coordinates import IDS
        self.handoff_rejection={'reason':reason,'joint_id':IDS[index],
            'error_degrees':math.degrees(errors[index]),'fallback':'ramped_saved_stance_without_torque_operation'}

    @property
    def bridge_complete(self):
        return self.bridge_ramp is not None and not self.bridge_ramp.moving

    @property
    def can_retry(self):
        try:
            pitch,roll=gravity_angles(self.hardware.frame['imu']['gravity'])
            tilt=math.degrees(math.acos(max(-1.,min(1.,math.cos(math.radians(pitch))*math.cos(math.radians(roll))))))
        except (KeyError,ValueError):return False
        return (self.handoff_rejection is not None and self.return_ramp is not None and
                not self.return_ramp.moving and tilt<=12 and
                max(abs(a-b) for a,b in zip(self.hardware.frame['positions'],self.reference.pose))<=math.radians(6))

    def retry_bridge(self):
        if not self.can_retry:raise ValueError('Return to the saved stance before retrying')
        self.handoff_rejection=None;self.handoff_until=0.;self.first_policy_target_checked=False
        self.return_ramp=None;self.return_time=None;self.await_policy=False
        self.prepare_bridge()

    def begin_handoff(self,*,clock=None):
        if not self.bridge_complete:raise ValueError('Preparation trajectory is not complete')
        self.await_policy=True
        super().begin_handoff(clock=self.clock)
        # Until an actual actor output arrives, a delayed native hold tick must
        # not consume a one-second timeout and bypass the first-target check.
        self.handoff_until=math.inf

    def fallback_targets(self):
        now=self.clock()
        if self.handoff_rejection is not None:
            self.handoff_rejection['fallback']='ramped_saved_stance_without_torque_operation'
        if self.return_ramp is None:
            start=getattr(self.hardware,'last_commanded_targets',None) or self.hardware.frame['positions']
            self.return_ramp=PoseRamp(start)
            self.return_ramp.set_target(self.reference.pose)
            self.return_time=now
        result=self.return_ramp.advance(now-self.return_time)
        self.return_time=now
        return result

    def dispatch(self,request):
        if request.get('op')=='write' and self.handoff_rejection is None and self.bridge_ramp is not None:
            native_target=vector(request.get('targets'))
            if self.await_policy:
                # A final native HOME hold tick can race robot.enable. Retain
                # the measured preparation pose until the first policy output.
                if max(abs(a-b) for a,b in zip(native_target,HOME))<1e-9:
                    return self.hardware.dispatch({**request,'targets':self.plan.pose[:]})
                self.await_policy=False;self.bridge_ramp=None
            else:
                now=self.clock();target=self.bridge_ramp.advance(now-self.bridge_time);self.bridge_time=now
                try:self.preparation_tracking.check(target,self.hardware.frame['positions'],now)
                except RuntimeError as exc:
                    self.reject_preparation(str(exc));target=self.fallback_targets()
                return self.hardware.dispatch({**request,'targets':target})
        result=super().dispatch(request)
        if (request.get('op')=='write' and self.handoff_until and not self.await_policy and
                self.handoff_rejection is None):
            # The 12-degree test protects the transition's first actor target.
            # Applying it to every later PD target confuses load-bearing servo
            # error with another takeover. Native safety and hardware limits
            # remain active on every tick.
            self.first_policy_target_checked=True
            self.handoff_until=0.
        return result
