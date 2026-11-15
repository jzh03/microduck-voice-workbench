"""Evaluate operator-labelled, torque-off trunk motions in the policy frame."""
import math
import statistics

STAGES = ('upright', 'forward', 'right', 'left_turn')
REFERENCE_MAX_TILT_DEG = 25.


def reference_pose(gravity):
    """A direction-test reference may be held slightly tilted; never zero it."""
    return gravity[2] < -math.cos(math.radians(REFERENCE_MAX_TILT_DEG))


def median_vector(frames, name):
    return [statistics.median(f['imu'][name][axis] for f in frames) for axis in range(3)]


def evaluate(stage, baseline, frames):
    if stage not in STAGES or len(frames) < 35 or len(baseline) < 5:
        raise ValueError('Not enough fresh samples')
    gaps = [b['t']-a['t'] for a, b in zip(frames, frames[1:])]
    if any(not 0 < dt < .18 for dt in gaps):
        raise ValueError('IMU stream was interrupted')
    before = median_vector(baseline, 'gravity')
    after = median_vector(frames[-15:], 'gravity')
    bias = median_vector(baseline, 'gyro')
    integral = [0., 0., 0.]
    for a, b in zip(frames, frames[1:]):
        dt = b['t']-a['t']
        for i in range(3):
            integral[i] += ((a['imu']['gyro'][i]+b['imu']['gyro'][i])/2-bias[i])*dt
    if stage == 'upright':
        pose_ok = reference_pose(after)
        settled = max(abs(v) for v in median_vector(frames[-15:], 'gyro')) < .08
        passed = pose_ok and settled
        if not pose_ok:
            tilt = math.degrees(math.acos(max(-1., min(1., -after[2]))))
            reason = f'机身倾斜约 {tilt:.0f}°，请大致扶正到 25° 内；不要求绝对静止'
        elif not settled:
            reason = '机身还在持续转动，请短暂停住；正常手抖可以'
        else:
            reason = '起始姿态已记录，正常手抖不影响；倾斜读数原样保留'
    else:
        if not reference_pose(before):
            raise ValueError('请先把机身大致扶正到 25° 内，再点本步骤；不要求完全竖直')
        if max(abs(v) for v in bias) > .08:
            raise ValueError('请先暂停大幅转动一秒，再点本步骤；正常手抖可以')
        axis = {'forward': 1, 'right': 0, 'left_turn': 2}[stage]
        expected = integral[axis]
        cross = max(abs(v) for i, v in enumerate(integral) if i != axis)
        motion_ok = .07 < expected < .9 and cross < max(.10, expected*.65)
        if stage == 'forward':
            pose_ok = after[0]-before[0] > .10 and abs(after[1]-before[1]) < .12 and after[2] < -.80
        elif stage == 'right':
            pose_ok = after[1]-before[1] < -.10 and abs(after[0]-before[0]) < .12 and after[2] < -.80
        else:
            pose_ok = reference_pose(after)
        passed = motion_ok and pose_ok
        reason = '方向与策略坐标一致' if passed else '动作方向或幅度不符；回正后按本步骤说明重试'
    return {'stage': stage, 'passed': passed, 'reason': reason,
            'gravity_before': before, 'gravity_after': after,
            'reference_max_tilt_deg': REFERENCE_MAX_TILT_DEG,
            'gravity_zeroed': False,
            'gyro_integral_rad': integral, 'samples': len(frames),
            'maximum_gap_s': max(gaps)}
