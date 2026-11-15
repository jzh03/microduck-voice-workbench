#!/usr/bin/env python3
"""Explicit torque-off recovery. This program has no torque-on or goal-write path."""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import sys
import time


ROOT = Path('/root/microduck-imu-update/walking-runtime')


def session_matches(arguments, identity):
    directory = identity.get('directory', '')
    runner = identity.get('runner', '')
    if not re.fullmatch(r'/run/microduck-manual-\d{8}-\d{6}-[a-f0-9]{8}', directory):
        return False
    if runner not in ('run_teleop.py', 'run_manual_stance.py'):
        return False
    def option(name, value):
        return any(a == name and b == value for a, b in zip(arguments, arguments[1:]))
    return (str(ROOT / runner) in arguments and
            option('--output-dir', directory) and option('--token-file', directory + '/token'))


def validate_packet(packet, original, ids, fields, unstuff):
    instruction = packet[7]
    if instruction in (2, 0x82, 0x8a):
        return
    payload = unstuff(packet[8:-2])
    address = int.from_bytes(payload[:2], 'little')
    if instruction == 0x83:
        if (address != 64 or payload[2:4] != b'\x01\x00' or
                len(payload[4:]) != len(ids) * 2 or
                set(payload[4::2]) != set(ids) or any(payload[5::2])):
            raise ValueError('Recovery permits only complete torque-off sync writes')
        return
    if instruction != 3 or address not in fields or packet[4] not in original:
        raise ValueError('Recovery refuses goal, torque-on and unjournalled writes')
    width, signed = fields[address]
    if (len(payload[2:]) != width or
            int.from_bytes(payload[2:], 'little', signed=signed) != original[packet[4]][address]):
        raise ValueError('Recovery write differs from the saved configuration')


def bus_owners():
    owners = []
    for directory in Path('/proc').glob('[0-9]*/fd'):
        try:
            for descriptor in directory.iterdir():
                try:
                    if os.readlink(descriptor) == '/dev/ttyS2':
                        owners.append(int(directory.parent.name))
                except (FileNotFoundError, PermissionError):
                    pass
        except (FileNotFoundError, PermissionError):
            pass
    return sorted(set(owners))


def stop_session(identity):
    pid = identity.get('pid')
    if type(pid) is not int or pid <= 1:
        return
    path = Path('/proc') / str(pid) / 'cmdline'
    try:
        arguments = path.read_bytes().decode().rstrip('\0').split('\0')
    except FileNotFoundError:
        return
    # A PID may have been reused after a reboot. Never signal a different process.
    if not session_matches(arguments, identity):
        return
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 7
    while pid in bus_owners() and time.monotonic() < deadline:
        time.sleep(.1)


def recover(identity):
    sys.path.insert(0, str(ROOT))
    from hardware_bridge import Hardware, WireBus, LOCAL, IDS, FIELDS, dxl, save_journal
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    stop_session(identity)
    if bus_owners():
        raise RuntimeError('Another serial controller is still active; it was not stopped')
    journal_path = ROOT / 'bridge-recovery.json'
    profile = (LOCAL / 'software-zero-reference.json').read_bytes()
    original, homing = {}, None
    if journal_path.exists():
        journal = json.loads(journal_path.read_text())
        if hashlib.sha256(profile).hexdigest() != journal['profile_sha256']:
            raise RuntimeError('Saved recovery configuration has a different calibration')
        original = {int(s): {int(a): v for a, v in values.items()}
                    for s, values in journal['original'].items()}
        homing = {int(s): v for s, v in journal['homing'].items()}
        if (set(original) != set(IDS) or set(homing) != set(IDS) or
                any(set(v) != set(FIELDS) or v[11] != 3 or v[98] != 0 for v in original.values())):
            raise RuntimeError('Saved recovery configuration is incomplete or unexpected')
    class RecoveryBus(WireBus):
        def send(self, packet, *args, **kwargs):
            validate_packet(packet, original, IDS, FIELDS, dxl.unstuff)
            return super().send(packet, *args, **kwargs)
    bus = RecoveryBus('/dev/ttyS2', baud=1000000, timeout=.025, live=True)
    try:
        hardware = Hardware(bus, profile, LOCAL / 'runtime-joint-limits.json',
                            mode='live', journal=journal_path if original else None)
        for sid in IDS:
            if hardware.read_reg(sid, 0, 2) != 1200 or hardware.read_reg(sid, 6, 1) != 53:
                raise RuntimeError('Unexpected servo identity or firmware')
        # This is an explicit operator reset, including when torque survived a
        # host interruption. No motor is armed during recovery.
        hardware.sync(64, {sid: 0 for sid in IDS})
        torque = hardware.torque()
        if set(torque) != set(IDS) or any(torque.values()):
            raise RuntimeError('All 15 motors must report torque off')
        measured_homing = {sid: hardware.read_reg(sid, 20, 4, True) for sid in IDS}
        if homing is not None and measured_homing != homing:
            raise RuntimeError('Homing offsets changed; torque is off but recovery is blocked')
        if original:
            hardware.original, hardware.homing = original, homing
            hardware.stop()
            if not hardware.cleanup or not hardware.cleanup['restored']:
                raise RuntimeError('Configuration restoration could not be verified')
            registers = {sid: {a: hardware.read_reg(sid, a, *spec)
                               for a, spec in FIELDS.items()} for sid in IDS}
            if registers != original:
                raise RuntimeError('Restored configuration readback differs')
            final = json.loads(journal_path.read_text())
            final.update(state='restored', restoration_verified_unix=time.time())
            save_journal(journal_path, final)
        torque = hardware.torque()
        if any(torque.values()) or Path('/proc/sys/kernel/random/boot_id').read_text().strip() != boot:
            raise RuntimeError('Motor state or boot identity changed during recovery')
        return {'verified': True, 'boot_id': boot, 'torque_after': torque,
                'cleanup': {'torque_off': True, 'restored': True, 'errors': []},
                'homing_unchanged': True, 'motion_commanded': False,
                'register_write_packets': bus.write_packets}
    finally:
        bus.close()


if __name__ == '__main__':
    try:
        result = recover(json.load(sys.stdin))
    except Exception as error:
        result = {'verified': False, 'error': str(error), 'motion_commanded': False}
    print(json.dumps(result))
