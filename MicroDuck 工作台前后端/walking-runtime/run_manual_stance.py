#!/usr/bin/env python3
"""Explicitly supported manual stance control; optional slow startup HOME, no policy."""
import argparse
import hashlib
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import threading
import time

from policy_coordinates import HOME, IDS
from stance_kinematics import LIMITS_DEG, shift_pose
from stance_state import StanceState, PoseRamp, TrackingGuard, angle_vector, posture_metrics
from stance_reference import StanceReference
from flight_recorder import FlightRecorder,emit


class FakeHardware:
    """In-memory integration fixture, never a hardware or standing verification."""
    def __init__(self, cancelled=lambda: None):
        self.cancelled = cancelled
        self.active = False
        self.fault = self.failure_cause = self.first_motor_frame_failure = None
        self.homing = {sid: 0 for sid in IDS}
        self.cleanup = None
        self.minimum_volts = self.maximum_volts = 7.5
        self.slow_frame = {'temps_c': [25.]*15, 'volts': 7.5}
        self.frame = {'positions': list(HOME), 'velocities': [0.]*15,
                      'currents_ma': [0.]*15, 'imu': {'gravity': [0., 0., -1.]}}
        self.last_requested_targets = self.last_commanded_targets = None
        self.reads = self.target_frames = self.clamps = 0
        self.prepare_pose_change_deg = 0.
        self.sample_time = self.target_time = time.monotonic()
        self.original = {}
        self.limits = self

    def bounds(self, sid): return LIMITS_DEG[sid]
    def start(self): self.read()
    def prepare(self): self.original = {'fake': True}
    def enable(self):
        if self.cancelled(): raise RuntimeError(self.cancelled())
        self.active = True
        self.target_time = time.monotonic()
    def read(self):
        if self.fault: raise RuntimeError(self.fault)
        self.sample_time = time.monotonic()
        self.reads += 1
        return self.frame
    def targets(self, values):
        if self.cancelled(): raise RuntimeError(self.cancelled())
        if not self.active: raise RuntimeError('Fake target while torque is off')
        values = angle_vector(values)
        self.last_requested_targets = values[:]
        self.last_commanded_targets = values[:]
        self.frame['positions'] = values[:]
        self.target_frames += 1
        self.target_time = time.monotonic()
    def watchdog(self):
        if self.active and time.monotonic()-self.target_time > .25:
            self.fault = self.failure_cause = 'Target stream stopped for 250 ms'
            self.stop()
    def stop(self):
        self.active = False
        self.original = {}
        self.cleanup = {'torque_off': True, 'restored': True, 'errors': []}
    def torque(self): return {sid: int(self.active) for sid in IDS}
    def read_reg(self, sid, address, width, signed=False):
        if address != 20: raise ValueError('Unsupported fake register')
        return self.homing[sid]


def check_pose_limits(hardware, values):
    for sid, value in zip(IDS, angle_vector(values)):
        lo, hi = hardware.limits.bounds(sid)
        if not lo-1e-8 <= math.degrees(value) <= hi+1e-8:
            raise ValueError(f'ID {sid}: requested pose outside nominal joint range')


class Session:
    def __init__(self, args, *, hardware_factory=None):
        self.args = args
        self.hardware_factory = hardware_factory
        self.commands = StanceState()
        self.lock = threading.RLock()
        self.finished = threading.Event()
        self.close_requested = threading.Event()
        self.monitor_finished = threading.Event()
        self.monitor_error = None
        self.trace = None
        self.state = {'phase': 'waiting', 'message': '等待控制页连接；请扶稳机身和双腿',
                      'running': True, 'mode': args.mode, 'policy': 'manual_stance',
                      'control_kind': 'manual_stance', 'drive_unlocked': False}
        self.samples = deque(maxlen=1800)
        self.report = {'mode': args.mode, 'control_kind': 'manual_stance', 'success': False,
                       'policy_loaded': False, 'walking_observed': False,
                       'independent_standing_confirmed_by_operator': False,
                       'automatic_home': bool(getattr(args, 'start_home', False)), 'commands': []}

    def update(self, **values):
        with self.lock: self.state.update(values)

    def public(self):
        with self.lock: return {**self.state, **self.commands.public()}

    def monitor(self, hardware):
        while not self.monitor_finished.wait(.02):
            try:
                if self.commands.abort_reason() and hardware.active:
                    hardware.stop()
                hardware.watchdog()
            except Exception as exc:
                self.monitor_error = str(exc)
                return

    def run(self):
        args = self.args
        hardware = bus = monitor = None
        started = None
        try:
            trace_path=Path(__file__).resolve().parent/'diagnostics'/args.output_dir.name/'flight.jsonl'
            self.trace=FlightRecorder(trace_path)
            self.report['diagnostic_log']=str(trace_path)
            boot=Path('/proc/sys/kernel/random/boot_id')
            emit(self.trace,'session_started',boot_id=boot.read_text().strip() if boot.exists() else 'local-fake',
                 mode=args.mode,control_kind='manual_stance',joint_ids=list(IDS),session=str(args.output_dir))
            reset_pose = list(HOME) if getattr(args, 'start_home', False) else None
            if args.mode == 'live':
                if not args.supported or not args.certificate:
                    raise RuntimeError('Live stance control requires physical support and IMU evidence')
                from run_teleop import verify_files
                self.report['profile_sha256'] = verify_files(args.certificate, physical=True)
                if getattr(args, 'reference', None):
                    from hardware_bridge import ROOT
                    reference = StanceReference.load(args.reference, self.report['profile_sha256'],
                        hashlib.sha256((ROOT/'runtime-contract.json').read_bytes()).hexdigest())
                    reset_pose = reference.pose
                    self.report['standing_reference'] = reference.value
            deadline = time.monotonic()+12.
            while self.commands.last_seen is None:
                reason = self.commands.abort_reason()
                if reason or time.monotonic() > deadline:
                    raise RuntimeError(reason or 'Control page did not connect')
                time.sleep(.02)
            if self.commands.abort_reason(): raise RuntimeError(self.commands.abort_reason())
            self.update(phase='preparing', message='准备关节配置；请托住机身，保持双脚当前姿态')
            if self.hardware_factory:
                hardware = self.hardware_factory(self.commands.abort_reason)
            elif args.mode == 'fake':
                hardware = FakeHardware(self.commands.abort_reason)
            else:
                from hardware_bridge import Hardware, WireBus, LOCAL, ROOT
                bus = WireBus('/dev/ttyS2', baud=1000000, timeout=.025, live=True)
                hardware = Hardware(bus, (LOCAL/'software-zero-reference.json').read_bytes(),
                                    LOCAL/'runtime-joint-limits.json', mode='live',
                                    journal=ROOT/'bridge-recovery.json', cancelled=self.commands.abort_reason,recorder=self.trace,
                                    hold_mouth_closed=True)
            def hardware_cancelled():
                reason = self.commands.abort_reason()
                if reason: return reason
                if hardware.frame:
                    try:
                        if posture_metrics(hardware.frame['imu']['gravity'])['tilt_deg'] > 20.+1e-9:
                            return 'Supported trunk tilt exceeded 20 degrees'
                    except ValueError as exc:
                        return str(exc)
                return None
            # Hardware.enable checks cancellation again after its own fresh
            # sensor read and goal seeding, immediately before torque-on.
            hardware.cancelled = hardware_cancelled
            hardware.start()
            self.report['homing_before'] = hardware.homing.copy()
            hardware.prepare()
            hardware.read()
            if posture_metrics(hardware.frame['imu']['gravity'])['tilt_deg'] > 20.+1e-9:
                raise RuntimeError('Hold the supported trunk within 20 degrees before torque-on')
            if reset_pose is not None:
                check_pose_limits(hardware, reset_pose)
            monitor = threading.Thread(target=self.monitor, args=(hardware,), daemon=True)
            monitor.start()
            hardware.enable()
            # enable() refreshes the frame immediately before seeding the exact
            # encoder positions. Compare against that frame, not the earlier
            # unpowered pose during the potentially long preparation.
            arm_seed = angle_vector(hardware.frame['positions'])
            fresh = hardware.read()
            q0 = angle_vector(fresh['positions'])
            arm_change = max(abs(math.degrees(a-b)) for a, b in zip(arm_seed, q0))
            self.report['arm_pose_change_deg'] = arm_change
            if arm_change > .5+1e-9:
                raise RuntimeError('Measured pose changed by more than 0.5 degree on torque-on')
            self.commands.initialize(q0, reset_target=reset_pose)
            ramp = PoseRamp(q0)
            ramp.set_target(self.commands.target)
            tracking = TrackingGuard()
            self.report['arm_pose_rad'] = dict(zip(map(str, IDS), q0))
            self.report['q0_rad'] = dict(zip(map(str, IDS), self.commands.q0))
            self.report['reset_pose_rad'] = self.report['q0_rad'].copy()
            started = previous = next_tick = time.monotonic()
            ticks = deque(maxlen=51)
            healthy_since = unhealthy_since = None
            last_sample = -1e10
            planning = None
            while not self.commands.abort_reason():
                tick = time.monotonic()
                if self.monitor_error or hardware.fault:
                    raise RuntimeError(self.monitor_error or hardware.fault)
                frame = hardware.read()
                measured = angle_vector(frame['positions'])
                metrics = posture_metrics(frame['imu']['gravity'])
                if metrics['tilt_deg'] > 20.+1e-9:
                    raise RuntimeError('Supported trunk tilt exceeded 20 degrees')
                now = time.monotonic()
                largest_error = tracking.check(ramp.commanded, measured, now)
                command = self.commands.take_command()
                if command:
                    healthy_since = None
                    if command['action'] == 'body_shift':
                        # Solving a constrained shift can exceed one 20 ms tick
                        # on the board. Keep polling sensors/holding the old goal
                        # while a pure worker computes; it never touches motors.
                        planning = {'command': command, 'measured': measured[:],
                                    'gravity': list(frame['imu']['gravity']),
                                    'started': now, 'done': threading.Event()}
                        def solve(candidate):
                            try:
                                candidate['plan'] = self.commands.plan(candidate['command'], candidate['measured'],
                                    candidate['gravity'], shift_function=shift_pose)
                            except Exception as exc:
                                candidate['error'] = str(exc)
                            finally:
                                candidate['done'].set()
                        threading.Thread(target=solve, args=(planning,), daemon=True).start()
                        command = None
                ready_plan = None
                if planning and planning['done'].is_set():
                    # Revalidate against a frame acquired after the result was
                    # observed, not the earlier frame from this control tick.
                    frame = hardware.read()
                    measured = angle_vector(frame['positions'])
                    metrics = posture_metrics(frame['imu']['gravity'])
                    if metrics['tilt_deg'] > 20.+1e-9:
                        raise RuntimeError('Supported trunk tilt exceeded 20 degrees')
                    now = time.monotonic()
                    largest_error = tracking.check(ramp.commanded, measured, now)
                    command = planning['command']
                    try:
                        if planning.get('error'): raise ValueError(planning['error'])
                        drift = max(abs(math.degrees(a-b)) for a, b in zip(planning['measured'], measured))
                        old_metrics = posture_metrics(planning['gravity'])
                        if drift > .5+1e-9 or max(abs(old_metrics[k]-metrics[k]) for k in ('pitch_deg', 'roll_deg')) > 1.:
                            raise ValueError('Pose changed during body-shift calculation; hold still and retry')
                        ready_plan = planning['plan']
                        ready_plan.update(calculation_s=now-planning['started'], baseline_drift_deg=drift)
                    except ValueError as exc:
                        self.commands.reject(str(exc))
                        self.report['commands'].append({'command': command, 'rejected': str(exc), 'elapsed_s': now-started})
                        command = None
                    planning = None
                if command:
                    try:
                        plan = ready_plan or self.commands.plan(command, measured, frame['imu']['gravity'], shift_function=shift_pose)
                        check_pose_limits(hardware, plan['target'])
                        self.commands.commit(plan)
                        ramp.set_target(plan['target'])
                        self.report['commands'].append({**plan, 'elapsed_s': now-started})
                    except ValueError as exc:
                        self.commands.reject(str(exc))
                        self.report['commands'].append({'command': command, 'rejected': str(exc), 'elapsed_s': now-started})
                hardware.targets(ramp.advance(now-previous))
                previous = now
                ticks.append(now)
                hz = (len(ticks)-1)/(ticks[-1]-ticks[0]) if len(ticks) > 1 else 0.
                healthy = hz >= 45.
                moving = ramp.moving or planning is not None
                settled = not moving and largest_error <= 6.+1e-9 and healthy
                healthy_since = (now if healthy_since is None else healthy_since) if settled else None
                self.commands.update_gate(moving=moving, settled=healthy_since is not None and now-healthy_since >= .5)
                if now-started > 2. and not healthy:
                    unhealthy_since = now if unhealthy_since is None else unhealthy_since
                    if now-unhealthy_since > 2.:
                        raise RuntimeError('Manual stance loop remained below 45 Hz for 2 seconds')
                else:
                    unhealthy_since = None
                message = ('可以继续小步调整；请持续扶稳机身' if self.commands.public()['ready'] else
                           '正在缓慢调整或等待关节稳定；请持续扶稳机身')
                if self.commands.last_command_error:
                    message += '；本次调整未执行：'+self.commands.last_command_error
                values = {**metrics, 'phase': 'adjusting' if moving else 'holding',
                          'message': message, 'control_hz': hz, 'healthy': healthy,
                          'elapsed_s': now-started, 'tracking_error_deg': largest_error,
                          'volts_min': hardware.minimum_volts, 'temperature_max': max(hardware.slow_frame['temps_c']),
                          'joint_degrees': dict(zip(map(str, IDS), map(math.degrees, measured))),
                          'commanded_joint_degrees': dict(zip(map(str, IDS), map(math.degrees, ramp.commanded))),
                          'requested_joint_degrees': dict(zip(map(str, IDS), map(math.degrees, ramp.target)))}
                self.update(**values)
                self.update(diagnostic_log=str(self.trace.path),diagnostic_log_error=self.trace.error)
                emit(self.trace,'manual_control_state',**values,gravity=list(frame['imu']['gravity']))
                if now-last_sample >= .1:
                    sample = {**values, **self.commands.public(), 'gravity': list(frame['imu']['gravity']),
                              'joint_currents_ma': dict(zip(map(str, IDS), frame['currents_ma'])),
                              'max_current_ma': max(frame['currents_ma']),
                              'read_retries': bus.read_retries if bus else 0}
                    self.samples.append(sample)
                    self.update(diagnostics=sample)
                    last_sample = now
                if getattr(args, 'test_seconds', None) and now-started >= args.test_seconds:
                    self.commands.receive({'action': 'off'})
                next_tick += .02
                if next_tick < time.monotonic(): next_tick = time.monotonic()
                self.monitor_finished.wait(max(0., next_tick-time.monotonic()))
            reason = self.commands.abort_reason()
            if reason != 'Operator requested torque-off': raise RuntimeError(reason)
            self.report['success'] = True
        except BaseException as exc:
            self.report['error'] = str(exc) or type(exc).__name__
            emit(self.trace,'session_exception',reason=self.report['error'],exception_type=type(exc).__name__)
        finally:
            emit(self.trace,'session_cleanup_started',reason=self.report.get('error') or self.commands.abort_reason())
            self.monitor_finished.set()
            with self.commands.lock:
                self.commands.ready = False
                self.commands.moving = False
            self.update(phase='stopping', message='正在卸力并恢复配置；请托住机身')
            if hardware:
                try:
                    hardware.stop()
                    self.report.update(torque_after=hardware.torque(), cleanup=hardware.cleanup,
                        homing_after={sid: hardware.read_reg(sid, 20, 4, True) for sid in IDS},
                        hardware_failure_cause=hardware.failure_cause,
                        motor_frame_failure=hardware.first_motor_frame_failure,
                        sensor_frames=hardware.reads, target_frames=hardware.target_frames,
                        nominal_limit_clamps=hardware.clamps,
                        prepare_pose_change_deg=hardware.prepare_pose_change_deg,
                        last_sensor_frame=hardware.frame,
                        last_requested_targets=hardware.last_requested_targets,
                        last_commanded_targets=hardware.last_commanded_targets)
                    unchanged = bus is not None and bus.write_packets == 0
                    restored = unchanged or bool(hardware.cleanup and hardware.cleanup.get('restored'))
                    if len(self.report['torque_after']) != 15 or any(self.report['torque_after'].values()) or not restored:
                        raise RuntimeError('Torque-off or restoration not confirmed')
                    if self.report.get('homing_before') != self.report['homing_after']:
                        raise RuntimeError('Homing offset changed during manual stance session')
                except Exception as exc:
                    self.report['success'] = False
                    self.report['cleanup_error'] = str(exc)
                    self.report.setdefault('error', str(exc))
            if monitor: monitor.join(timeout=1.)
            if bus:
                self.report.update(register_write_packets=bus.write_packets, read_retries=bus.read_retries,
                                   read_retries_by_operation=bus.read_retries_by_operation)
                bus.close()
            self.report.update(samples=list(self.samples), elapsed_s=time.monotonic()-started if started else 0.,
                               finished_unix=time.time())
            confirmed = (len(self.report.get('torque_after', {})) == 15 and
                         not any(self.report['torque_after'].values()) and not self.report.get('cleanup_error'))
            self.report['safe_off'] = confirmed
            args.output_dir.mkdir(parents=True, exist_ok=True)
            (args.output_dir/'result.json').write_text(json.dumps(self.report, indent=2)+'\n')
            emit(self.trace,'session_finished',report=self.report)
            if self.trace:self.trace.close()
            message = '已卸力，配置已恢复' if confirmed else '停止后状态未确认，请继续托稳'
            if self.report.get('error'): message += '；'+self.report['error']
            self.update(phase='complete' if self.report['success'] else 'error', message=message,
                        running=False, safe_off=confirmed, report=self.report)
            self.finished.set()


def make_server(session, token, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_POST(self):
            if self.headers.get('X-Control-Token') != token:
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 <= size <= 1000: raise ValueError('Command too large')
                value = json.loads(self.rfile.read(size) or b'{}')
                if self.path == '/command':
                    if isinstance(value,dict) and value.get('action')!='heartbeat':emit(session.trace,'operator_command',command=value)
                    session.commands.receive(value)
                elif self.path == '/close':
                    if session.public()['running']: raise ValueError('Session still running')
                    session.close_requested.set()
                elif self.path != '/status': raise ValueError('Unknown endpoint')
                response, status = session.public(), 200
            except (ValueError, TypeError) as exc:
                response, status = {'error': str(exc)}, 400
            body = json.dumps(response, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('live', 'fake'), required=True)
    parser.add_argument('--supported', action='store_true')
    parser.add_argument('--start-home', action='store_true', help='After measured-position arming, slowly move to default HOME before manual adjustments')
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--certificate', type=Path)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=18769)
    parser.add_argument('--test-seconds', type=float, help='Only for fake integration verification')
    args = parser.parse_args()
    if args.mode == 'live' and (not args.supported or not args.certificate or args.test_seconds):
        parser.error('Live requires physical support and IMU evidence; timed torque-off is fake-only')
    token = args.token_file.read_text().strip()
    if len(token) < 32: parser.error('Invalid control token')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    session = Session(args)
    server = make_server(session, token, args.port)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *_: session.commands.receive({'action': 'off'}))
    signal.signal(signal.SIGINT, lambda *_: session.commands.receive({'action': 'off'}))
    session.run()
    session.close_requested.wait(60)
    server.shutdown()
    server.server_close()


if __name__ == '__main__': main()
