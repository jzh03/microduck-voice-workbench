#!/usr/bin/env python3
"""Radxa: read a stable torque-off reference; emit JSON, never write registers."""
import datetime
import fcntl
import json
import statistics
import sys
import time

sys.path.insert(0, 'tools')
import dxl

IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)


def main():
    bus = dxl.Bus('/dev/ttyS2', baud=1000000, timeout=.04)
    try:
        deadline = time.monotonic() + 5
        while True:
            try:
                fcntl.flock(bus.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Bus is still owned by another reader')
                time.sleep(.1)

        def read(sid, address, count, signed=False):
            err, data, _ = bus.read(sid, address, count)
            if err is None or err & 0x7f or len(data) != count:
                raise RuntimeError(f'Incomplete read: ID {sid}, address {address}, status {err}')
            return int.from_bytes(data, 'little', signed=signed)

        registers = {}
        for sid in IDS:
            row = {
                'model': read(sid, 0, 2), 'firmware': read(sid, 6, 1),
                'drive_mode': read(sid, 10, 1), 'operating_mode': read(sid, 11, 1),
                'homing_offset': read(sid, 20, 4, True),
                'min_position': read(sid, 52, 4), 'max_position': read(sid, 48, 4),
                'torque': read(sid, 64, 1), 'hardware_error': read(sid, 70, 1),
            }
            if row['model'] != 1200 or row['drive_mode'] != 0 or row['torque'] != 0:
                raise RuntimeError(f'Unsupported model/drive mode or torque enabled on ID {sid}')
            registers[sid] = row

        samples = []
        for _ in range(40):
            started = time.monotonic()
            samples.append({sid: read(sid, 132, 4, True) for sid in IDS})
            time.sleep(max(0, .05 - (time.monotonic() - started)))

        joints = {}
        for sid in IDS:
            row = registers[sid]
            if read(sid, 64, 1) != 0 or read(sid, 20, 4, True) != row['homing_offset']:
                raise RuntimeError(f'Torque or homing configuration changed during capture on ID {sid}')
            unwrapped = [samples[0][sid]]
            for before, after in zip(samples, samples[1:]):
                unwrapped.append(unwrapped[-1] + (after[sid] - before[sid] + 2048) % 4096 - 2048)
            span = max(unwrapped) - min(unwrapped)
            if span > 12:
                raise RuntimeError(f'Pose moved during capture: ID {sid}, span {span} ticks')
            median = int(round(statistics.median(unwrapped)))
            joints[sid] = {
                'zero_encoder_phase': (median - row['homing_offset']) % 4096,
                'angle_at_reference_rad': 0.0, 'direction': 1,
                'present_position_at_capture': median,
                'homing_offset_at_capture': row['homing_offset'],
                'span_ticks': span, 'physical_limits_rad': None,
            }
        profile = {
            'schema_version': 1,
            'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'reference_pose': 'User-arranged model-zero candidate, lying flat with supports',
            'basis': 'User pose confirmation and 40 stable, complete, torque-off samples',
            'samples': len(samples), 'joints': joints,
            'direction_basis': 'Existing +1 viewer convention; dedicated sign verification pending',
            'pose_precision_verified': False, 'powered_motion_verified': False,
            'registers_written': False, 'motion_commanded': False,
        }
        print(json.dumps({'profile': profile, 'registers': registers}))
    finally:
        bus.close()


if __name__ == '__main__':
    main()
