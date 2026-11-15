"""Pure shared coordinates for policy observation and motor goals; no motor I/O."""
import json
import math
from pathlib import Path
import statistics
import struct
import sys

HERE = Path(__file__).resolve().parent
parent = HERE.parent
sys.path.insert(0, str(parent / 'local-joint-viewer' if (parent / 'local-joint-viewer').exists() else parent))
from joint_calibration import JointCalibration

CONTRACT = json.loads((HERE / 'runtime-contract.json').read_text())
IDS = tuple(CONTRACT['joint_ids'])
HOME = tuple(CONTRACT['home_rad'])
POLICY_INDICES = tuple(i for i in range(15) if i != CONTRACT['mouth_index'])


def mul(a, b):
    w, x, y, z = a
    q, r, s, t = b
    return [w*q-x*r-y*s-z*t, w*r+x*q+y*t-z*s,
            w*s-x*t+y*q+z*r, w*t+x*s-y*r+z*q]


def conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]


def rotate(q, v):
    return mul(mul(q, [0., *v]), conjugate(q))[1:]


class TrunkImu:
    """Official zero3 SFLP transform and median-of-three spike rejection.

    Without an explicit mount, use the official zero3 preset. Hardware may supply
    a separately recorded robot mount. A gravity check alone does not establish
    all dynamic axis signs required for walking.
    """
    def __init__(self, mount=None):
        self.mount = mount or CONTRACT['zero3_imu_mount_wxyz']
        self.quat = [1., 0., 0., 0.]
        self.gyros = [[0., 0., 0.], [0., 0., 0.]]
        self.gravities = [[0., 0., -1.], [0., 0., -1.]]
        self.samples = 0

    def decode(self, block):
        if len(block) != 12:
            raise ValueError('IMU frame must have 12 bytes')
        raw = struct.unpack('<3h', block[:6])
        gyro = rotate(self.mount, [v * CONTRACT['gyro_radians_per_count'] for v in raw])
        xyz = struct.unpack('<3e', block[6:12])
        if any(block[6:12]) and all(math.isfinite(v) for v in xyz):
            n = sum(v*v for v in xyz)
            if n <= 1.02:
                q = mul([math.sqrt(max(0., 1.-n)), *xyz], conjugate(self.mount))
                norm = math.sqrt(sum(v*v for v in q))
                if norm > .5:
                    self.quat = [v / norm for v in q]
                    self.samples += 1
        gravity = rotate(conjugate(self.quat), [0., 0., -1.])
        norm = math.sqrt(sum(v*v for v in gravity))
        gravity = [v / norm for v in gravity]
        result = {'gyro': [statistics.median(v) for v in zip(*self.gyros, gyro)],
                  'gravity': [statistics.median(v) for v in zip(*self.gravities, gravity)],
                  'quat': self.quat[:], 'ready': self.samples >= 25}
        self.gyros = [self.gyros[-1], gyro]
        self.gravities = [self.gravities[-1], gravity]
        return result


class Coordinates:
    def __init__(self, profile):
        self.calibration = JointCalibration(profile)
        if set(self.calibration.joints) != set(IDS):
            raise ValueError('Calibration must cover all 15 runtime joints')

    def sensors(self, positions, velocity_counts, homing_offsets):
        positions = {int(k): v for k, v in positions.items()}
        velocity_counts = {int(k): v for k, v in velocity_counts.items()}
        homing_offsets = {int(k): v for k, v in homing_offsets.items()}
        for values in (positions, velocity_counts, homing_offsets):
            if set(values) != set(IDS):
                raise ValueError('Incomplete sensor frame')
        angle = self.calibration.update(positions, homing_offsets)
        velocity = {sid: self.calibration.joints[sid]['direction'] * velocity_counts[sid] *
                    CONTRACT['velocity_radians_per_count'] for sid in IDS}
        return [angle[sid] for sid in IDS], [velocity[sid] for sid in IDS]

    @staticmethod
    def observation(angles, velocities, imu, previous, twist=(0., 0., 0.),
                    head=(0., 0., 0., 0.), body=(0., 0., 0.)):
        if tuple(map(len, (angles, velocities, previous, twist, head, body))) != (15, 15, 14, 3, 4, 3):
            raise ValueError('Wrong observation block widths')
        obs = [*imu['gyro'], *imu['gravity'],
               *(angles[i]-HOME[i] for i in POLICY_INDICES),
               *(velocities[i] for i in POLICY_INDICES), *previous,
               *twist, *head, 0., 0., body[0], body[1], body[2], 0.]
        if len(obs) != 61 or not all(math.isfinite(v) for v in obs):
            raise ValueError('Observation must be 61 finite values')
        return obs

    def raw_target(self, sid, target, present, angle, limits):
        """Compute a mode-4 target on the observed encoder branch; never send it."""
        return self.calibration.goal_for_angle(sid, target, present_position=present,
            current_angle_rad=angle, operating_mode=4, min_position=0,
            max_position=4095, joint_limits_rad=limits)
