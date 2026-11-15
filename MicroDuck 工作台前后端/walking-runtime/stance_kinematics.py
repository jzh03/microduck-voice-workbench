"""Pure fixed-height stance geometry; no hardware, network, or file I/O.

Coordinates and chain constants match the bound robot_walk.xml. Positive X is
forward. A positive shift moves the pelvis forward relative to the feet while
preserving body orientation and height. Only the six sagittal leg joints change.
The caller owns freshness, exclusive motor control, ramping, and physical safety.
"""
from __future__ import annotations

from collections.abc import Mapping
import math

MODEL_XML_SHA256 = '80fc4424ed71e430ec4919110dfdb5c714dc1c4e1138e1832a16865cf31eb8c8'
IDS = (20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14)
# Source-bound nominal limits; the hardware layer independently enforces its
# profile-bound ranges as well. Do not silently clamp a measured baseline.
LIMITS_DEG = {
    10: (-30., 25.), 11: (-22., 22.), 12: (-90., 90.),
    13: (-90., 90.), 14: (-90., 90.),
    20: (-25., 30.), 21: (-22., 22.), 22: (-90., 90.),
    23: (-90., 90.), 24: (-90., 90.),
    30: (-90., 60.), 31: (-90., 90.), 32: (-170., 170.),
    33: (-25., 25.), 34: (-5., 30.),
}
MODEL_SIDES = {
    'left': (
        (20, (.006, .0175, -.005), (0, -.707107, -.707107, 0)),
        (21, (0, .0165, .0125), (.707107, -.707107, 0, 0)),
        (22, (.025, 0, -.0185), (.5, -.5, .5, -.5)),
        (23, (.022, .0357771, -.004), (0, .707107, .707107, 0)),
        (24, (0, .042, -.026), (0, -1, 0, 0)),
    ),
    'right': (
        (10, (.006, -.0175, -.005), (0, -.707107, -.707107, 0)),
        (11, (0, .0165, .0125), (0, 0, .707107, .707107)),
        (12, (.025, 0, -.0185), (.5, -.5, .5, -.5)),
        (13, (-.022, .0357771, -.004), (0, 1, 0, 0)),
        (14, (-.042, 0, -.026), (0, -.707107, -.707107, 0)),
    ),
}
SITES = {
    'left': ((0, -.0238146, -.0140852), (0, 0, -.707107, -.707107)),
    'right': ((0, -.0238146, -.0140852), (.707107, -.707107, 0, 0)),
}
SAGITTAL_IDS = {'left': (22, 23, 24), 'right': (12, 13, 14)}
EYE = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
MAX_STEP_DEG = 1.
MAX_LATERAL_DRIFT_MM = .05
MAX_REQUEST_MM = .5
MAX_TILT_DEG = 20.
# At a 0.5 mm request this leaves <0.00013 mm resolution while keeping the
# command-time solve bounded. The motor loop must not wait on this solver.
BISECTION_STEPS = 12


def matmul(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mv(a, v):
    return tuple(sum(a[i][k]*v[k] for k in range(3)) for i in range(3))


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def quat_matrix(q):
    n = math.sqrt(sum(x*x for x in q))
    w, x, y, z = (v/n for v in q)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.), (s, c, 0.), (0., 0., 1.))


def root_rotation(pitch_rad, roll_rad):
    """Ry(pitch) Rx(roll), matching the existing gravity-to-pitch convention."""
    if not all(type(v) in (float, int) and math.isfinite(v) for v in (pitch_rad, roll_rad)):
        raise ValueError('Pitch/roll must be finite radians')
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    return ((cp, sp*sr, sp*cr), (0., cr, -sr), (-sp, cp*sr, cp*cr))


def foot_pose(q, side, base_rotation=EYE):
    """Foot site relative to the pelvis origin, expressed in world coordinates."""
    rotation = base_rotation
    position = (0., 0., 0.)
    for sid, origin, quat in MODEL_SIDES[side]:
        position = add(position, mv(rotation, origin))
        rotation = matmul(rotation, matmul(quat_matrix(quat), rz(q[sid])))
    origin, quat = SITES[side]
    return add(position, mv(rotation, origin)), matmul(rotation, quat_matrix(quat))


def _solve_side(q0, side, base_rotation, forward_mm):
    """Exact world X/Z constraints; report free lateral residual explicitly.

    Parallel pitch axes allow Δankle = -Δhip + Δknee to preserve the entire
    foot orientation. Two remaining variables solve foot X and Z. A displaced
    pelvis is represented by the opposite displacement of the feet.
    """
    hip, knee, ankle = SAGITTAL_IDS[side]
    p0, r0 = foot_pose(q0, side, base_rotation)
    wanted = (p0[0]-forward_mm/1000., p0[2])

    def candidate(h, k):
        q = dict(q0)
        q[hip] += h
        q[knee] += k
        q[ankle] += -h+k
        return q

    def error(h, k):
        p, _ = foot_pose(candidate(h, k), side, base_rotation)
        return (p[0]-wanted[0], p[2]-wanted[1])

    h = k = 0.
    for _ in range(25):
        ex, ez = error(h, k)
        norm = math.hypot(ex, ez)
        if norm < 1e-11:
            break
        eps = 1e-6
        hp, hm = error(h+eps, k), error(h-eps, k)
        kp, km = error(h, k+eps), error(h, k-eps)
        a, c = ((hp[i]-hm[i])/(2*eps) for i in (0, 1))
        b, d = ((kp[i]-km[i])/(2*eps) for i in (0, 1))
        det = a*d-b*c
        if abs(det) < 1e-8:
            raise ValueError('Sagittal IK is near singular at this measured pose')
        dh, dk = (-d*ex+b*ez)/det, (c*ex-a*ez)/det
        fraction = min(1., .1/max(abs(dh), abs(dk), 1e-30))
        for _ in range(15):
            nh, nk = h+fraction*dh, k+fraction*dk
            if math.hypot(*error(nh, nk)) < norm:
                h, k = nh, nk
                break
            fraction *= .5
        else:
            raise ValueError('Sagittal IK failed its monotonic convergence check')
    else:
        raise ValueError('Sagittal IK did not converge')

    q = candidate(h, k)
    p, r = foot_pose(q, side, base_rotation)
    orientation_error = max(abs(r[i][j]-r0[i][j]) for i in range(3) for j in range(3))
    if abs(p[0]-wanted[0]) > 1e-9 or abs(p[2]-wanted[1]) > 1e-9 or orientation_error > 1e-10:
        raise ValueError('Sagittal IK failed final pose verification')
    return {
        'targets_rad': {sid: q[sid] for sid in (hip, knee, ankle)},
        'delta_deg': {sid: math.degrees(q[sid]-q0[sid]) for sid in (hip, knee, ankle)},
        'foot_translation_mm': [(p[i]-p0[i])*1000. for i in range(3)],
        'foot_rotation_matrix_max_error': orientation_error,
    }


def _checked_angles(angles):
    if not isinstance(angles, Mapping) or set(angles) != set(IDS):
        raise ValueError('A fresh complete 15-joint measured pose is required')
    q0 = {}
    for sid, value in angles.items():
        if type(sid) is not int or type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError('Measured pose must map integer IDs to finite radians')
        q0[sid] = float(value)
        lo, hi = (math.radians(x) for x in LIMITS_DEG[sid])
        if not lo <= q0[sid] <= hi:
            raise ValueError(f'Measured ID {sid} is outside nominal joint limits')
    return q0


def _gravity_angles(gravity):
    try:
        values = tuple(gravity)
    except TypeError as exc:
        raise ValueError('Gravity must contain three finite numbers') from exc
    if len(values) != 3 or any(type(v) not in (float, int) or not math.isfinite(v) for v in values):
        raise ValueError('Gravity must contain three finite numbers')
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Gravity vector must have a finite nonzero norm')
    gx, gy, gz = (v/norm for v in values)
    tilt = math.degrees(math.acos(max(-1., min(1., -gz))))
    if tilt > MAX_TILT_DEG+1e-10:
        raise ValueError('Standing geometry requires tilt at most 20 degrees')
    return (gx, gy, gz), math.atan2(gx, math.hypot(gy, gz)), math.atan2(-gy, -gz)


def shift_pose(angles, gravity3, distance_mm=.5):
    """Return a bounded pelvis shift relative to this measured baseline.

    ``angles`` is a complete ``int servo ID -> radians`` mapping. ``gravity3``
    uses the existing trunk convention, upright [0, 0, -1]. ``distance_mm`` is
    this one step, positive forward and negative backward, within +/-0.5 mm.

    The returned ``angles`` has integer keys and all 15 joints;
    ``actual_shift_mm`` is the signed displacement for this step, not a total.
    Exact model X/Z and foot orientation are retained. A small lateral residual
    from the tilted axes is reported and bounded, never disguised as exact XYZ.
    """
    if (type(distance_mm) not in (float, int) or not math.isfinite(distance_mm)
            or abs(distance_mm) > MAX_REQUEST_MM):
        raise ValueError('A stance shift must be within +/-0.5 mm per step')
    q0 = _checked_angles(angles)
    gravity, pitch_rad, roll_rad = _gravity_angles(gravity3)
    rotation = root_rotation(pitch_rad, roll_rad)

    def solve(mm):
        sides = {side: _solve_side(q0, side, rotation, mm) for side in SAGITTAL_IDS}
        allowed = all(abs(delta) <= MAX_STEP_DEG for s in sides.values() for delta in s['delta_deg'].values())
        allowed = allowed and all(-math.pi/2 <= q <= math.pi/2 for s in sides.values() for q in s['targets_rad'].values())
        allowed = allowed and all(abs(s['foot_translation_mm'][1]) <= MAX_LATERAL_DRIFT_MM for s in sides.values())
        return sides, allowed

    sides, allowed = solve(float(distance_mm))
    applied = float(distance_mm)
    if not allowed:
        lo, hi = 0., 1.
        sides, _ = solve(0.)
        for _ in range(BISECTION_STEPS):
            mid = (lo+hi)/2
            mid_sides, mid_allowed = solve(float(distance_mm)*mid)
            if mid_allowed:
                lo, sides = mid, mid_sides
            else:
                hi = mid
        applied = float(distance_mm)*lo
    targets = dict(q0)
    for side in sides.values():
        targets.update(side['targets_rad'])
    return {
        'requested_shift_mm': float(distance_mm),
        'actual_shift_mm': applied,
        'limited': abs(applied) < abs(distance_mm),
        'pelvis_height_delta_mm': 0.,
        'pitch_delta_rad': 0.,
        'roll_delta_rad': 0.,
        'joint_delta_limit_deg': MAX_STEP_DEG,
        'lateral_model_drift_limit_mm': MAX_LATERAL_DRIFT_MM,
        'angles': targets,
        'joint_delta_deg': {sid: math.degrees(targets[sid]-q0[sid]) for sid in IDS},
        'normalized_gravity': list(gravity),
        'pitch_rad': pitch_rad,
        'roll_rad': roll_rad,
        'sides': sides,
        'model_sha256': MODEL_XML_SHA256,
    }
