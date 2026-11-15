"""Source-bound nominal joint ranges, shared by the UI and serial worker.

These are the official alpha model's anatomical ranges intersected with the
runtime actuator clamp. They are not a claim that this robot's zero pose has
already been calibrated for a walking policy.
"""
import hashlib
import json
import math
from pathlib import Path


class JointLimits:
    def __init__(self,path,*,profile_sha256=None,ids=None):
        raw=Path(path).read_bytes();self.sha=hashlib.sha256(raw).hexdigest()
        self.data=json.loads(raw)
        if self.data.get('schema_version')!=1:raise ValueError('关节限位文件版本不匹配')
        if profile_sha256 and self.data.get('software_reference_sha256')!=profile_sha256:
            raise ValueError('关节限位与当前软件零位不匹配')
        self.joints={int(k):v for k,v in self.data['joints'].items()}
        if ids is not None and set(self.joints)!=set(ids):raise ValueError('关节限位 ID 不完整')
        for entry in self.joints.values():
            lo,hi=entry['degrees']
            if not all(math.isfinite(v) for v in (lo,hi)) or not -180<=lo<hi<=180:
                raise ValueError('关节限位必须在运行时电机行程内')

    def bounds(self,sid):return tuple(self.joints[int(sid)]['degrees'])

    def clamp(self,sid,value):
        if not math.isfinite(value):raise ValueError('目标角度必须为有限数值')
        lo,hi=self.bounds(sid)
        return max(lo,min(hi,float(value)))

    def public(self):
        return {'sha256':self.sha,'source':'official-alpha-model',
                'runtime_commit':self.data['runtime_commit'],
                'joints':{str(k):v for k,v in self.joints.items()}}
