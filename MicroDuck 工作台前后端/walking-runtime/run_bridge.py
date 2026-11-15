#!/usr/bin/env python3
"""Supervise official robotd and its calibrated hardware endpoint.

Default shadow mode reads REAL sensors and discards all actuator output. Live
requires both --mode live and --supported. This command never starts at boot.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time

from hardware_bridge import Hardware, WireBus, Endpoint, LOCAL, ROOT, IDS
from policy_coordinates import CONTRACT
from control_lease import ControlLease, watch_stdin

ORT = '/opt/robot/onnxruntime/onnxruntime-linux-aarch64-1.28.0/lib/libonnxruntime.so'


def rpc(path, method, params=None):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(3)
        client.connect(str(path))
        client.sendall((json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': method,
                                  'params': params or {}})+'\n').encode())
        answer = json.loads(client.makefile('r').readline())
        if 'error' in answer:
            raise RuntimeError(answer)
        return answer['result']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('observe', 'shadow', 'live'), default='shadow')
    parser.add_argument('--supported', action='store_true', help='Operator is physically supporting robot')
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--output', type=Path, default=ROOT/'bridge-check.json')
    parser.add_argument('--control-token', help='Require live USB heartbeat and stop commands on stdin')
    args = parser.parse_args()
    if args.control_token and args.mode != 'live':
        parser.error('The USB control lease applies only to a supported live trial')
    if not .5 <= args.seconds <= 120 or (args.mode == 'live' and not args.supported):
        parser.error('Use 0.5..120 seconds; live additionally requires --supported')
    deployment = json.loads((ROOT/'bridge-deployment.json').read_text())
    for name, want in deployment['sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != want:
            raise RuntimeError('Incomplete or modified bridge deployment: '+name)
    if hashlib.sha256((LOCAL/'software-zero-reference.json').read_bytes()).hexdigest() != deployment['profile_sha256']:
        raise RuntimeError('Calibration changed since bridge deployment; redeploy the reviewed profile binding')
    model = Path('/opt/robot/policies/current/velstand.onnx')
    if hashlib.sha256(model.read_bytes()).hexdigest() != CONTRACT['policy_sha256']:
        raise RuntimeError('Installed walking model differs from the reviewed contract')
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    lease = ControlLease(args.control_token) if args.control_token else None
    guard_finished = threading.Event()
    bus = WireBus('/dev/ttyS2', baud=1000000, timeout=.025, live=args.mode == 'live')
    hardware = Hardware(bus, (LOCAL/'software-zero-reference.json').read_bytes(),
                        LOCAL/'runtime-joint-limits.json', mode=args.mode,
                        journal=ROOT/'bridge-recovery.json',
                        cancelled=lease.abort_reason if lease else lambda: None)
    endpoint = daemon = None
    report = {'mode': args.mode, 'profile_sha256': hardware.profile_sha,
              'imu_mount_wxyz': hardware.imu.mount,
              'imu_mount_evidence': CONTRACT.get('robot_imu_mount_evidence'),
              'backend': 'official RemoteIo v1 over loopback with calibrated physical bus',
              'physical_balance_test': False, 'success': False}
    try:
        if lease:
            watch_stdin(lease, guard_finished, lambda: os.kill(os.getpid(), signal.SIGTERM))
            while lease.last is None:
                time.sleep(.01)
            print(json.dumps({'type': 'trial', 'phase': 'checking', 'active': False}), flush=True)
        hardware.start()
        report['torque_before'] = hardware.torque()
        report['homing_before'] = hardware.homing.copy()
        if args.mode == 'live':
            if lease:
                print(json.dumps({'type': 'trial', 'phase': 'preparing', 'active': False}), flush=True)
            hardware.prepare()
        endpoint = Endpoint(hardware)
        endpoint.start()
        with tempfile.TemporaryDirectory(prefix='duck-bridge-', dir='/tmp') as directory:
            socket_path = Path(directory)/'robotd.sock'
            env = {**os.environ, 'ORT_DYLIB_PATH': ORT, 'RUST_LOG': 'info'}
            argv = ['/opt/robot/daemon/current/bin/robotd', '--sim', f'127.0.0.1:{endpoint.port}',
                    '--params', str(ROOT/'bridge-robotd.toml'), '--socket', str(socket_path)]
            report['argv'] = argv
            with args.output.with_suffix('.log').open('w') as log:
                daemon = subprocess.Popen(argv, env=env, stdin=subprocess.DEVNULL,
                                          stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic()+10
                while not socket_path.exists():
                    if daemon.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Official daemon did not start')
                    time.sleep(.05)
                time.sleep(.3)
                report['daemon_serial_fds'] = []
                for fd in Path(f'/proc/{daemon.pid}/fd').iterdir():
                    try:
                        target = os.readlink(fd)
                    except FileNotFoundError:
                        continue
                    if '/dev/tty' in target:
                        report['daemon_serial_fds'].append(target)
                if report['daemon_serial_fds']:
                    raise RuntimeError('The official daemon unexpectedly opened a serial device')
                if args.mode == 'shadow':
                    report['enable'] = rpc(socket_path, 'robot.enable', {'on': True})
                elif args.mode == 'live':
                    # First live stage is the official slow home ramp and hold;
                    # walking policy activation is a separate operator action.
                    report['init'] = rpc(socket_path, 'robot.init')
                started = time.monotonic()
                last_progress = 0.
                while time.monotonic()-started < args.seconds:
                    if daemon.poll() is not None or hardware.fault:
                        raise RuntimeError(hardware.fault or 'Official daemon exited')
                    if lease and time.monotonic() - last_progress >= .1:
                        last_progress = time.monotonic()
                        print(json.dumps({'type': 'trial', 'phase': 'holding',
                            'active': hardware.active, 'elapsed_s': last_progress-started,
                            'seconds': args.seconds, 'frame': hardware.frame,
                            'volts_min': hardware.minimum_volts,
                            'temperature_max': max(hardware.slow_frame['temps_c'])}), flush=True)
                    time.sleep(.05)
                report['health'] = rpc(socket_path, 'robot.health')
                rpc(socket_path, 'robot.relax')
                # RPC acknowledges intent; wait for the control loop to apply it.
                deadline = time.monotonic() + 1.
                while hardware.active and time.monotonic() < deadline:
                    time.sleep(.02)
                if hardware.active or hardware.fault:
                    raise RuntimeError(hardware.fault or 'Official relax was not applied')
                report['success'] = True
    except BaseException as exc:
        report['error'] = (lease.abort_reason() if lease else None) or str(exc) or type(exc).__name__
    finally:
        guard_finished.set()
        if daemon and daemon.poll() is None:
            daemon.send_signal(signal.SIGTERM)
            try:
                daemon.wait(timeout=5)
            except subprocess.TimeoutExpired:
                daemon.kill()
                daemon.wait()
        if endpoint:
            endpoint.close()
        else:
            hardware.stop()
        try:
            report['torque_after'] = hardware.torque()
            report['homing_after'] = {sid: hardware.read_reg(sid, 20, 4, True) for sid in IDS}
            if any(report['torque_after'].values()):
                report['success'] = False
                report['error'] = 'Torque-off not confirmed'
        except Exception as exc:
            report['success'] = False
            report['error'] = str(exc)
        report.update(register_write_packets=bus.write_packets,
                      read_retries=bus.read_retries,
                      read_retries_by_operation=bus.read_retries_by_operation,
                      motor_motion_commanded=args.mode == 'live' and bus.write_packets > 0,
                      sensor_frames=hardware.reads, target_frames=hardware.target_frames,
                      discarded_idle_targets=hardware.idle_target_frames,
                      nominal_limit_clamps=hardware.clamps,
                      last_encoder_targets=hardware.last_goals,
                      minimum_volts=getattr(hardware, 'minimum_volts', None),
                      last_sensor_frame=hardware.frame, cleanup=hardware.cleanup,
                      fault=hardware.fault, failure_cause=hardware.failure_cause)
        if hardware.durations:
            durations = sorted(hardware.durations)
            report['sensor_ms_p95'] = durations[int(.95*(len(durations)-1))]*1000
        if args.mode == 'shadow' and (hardware.target_frames < 50 or bus.write_packets):
            report['success'] = False
            report.setdefault('error', 'Shadow check must receive targets and send zero register writes')
        if hardware.cleanup and not hardware.cleanup['restored']:
            report['success'] = False
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        bus.close()
    print(json.dumps({k: v for k, v in report.items() if k not in ('last_sensor_frame', 'argv')}, indent=2))
    if not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
