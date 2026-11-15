"""Run the workbench's floor recovery and an optional forward move."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
from threading import Event
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from voice_control import LEASE, ZERO, VoiceControl


ACTION_DELAY = 1.
FORWARD_SPEED = .5
FORWARD_SECONDS = 5.
FORWARD_DECELERATION = 1.
FORWARD_REFRESH = .08


class StandupControl(VoiceControl):
    def __init__(self, runtime, certificate, reference, *, simulation=False, walk=False):
        super().__init__(None, self.log, directions={'standup'})
        self.runtime, self.certificate, self.reference = runtime, certificate, reference
        self.simulation = simulation
        self.walk = walk
        self.stopping = self.busy = False
        self.cancelled = None
        self.standing_ready = Event()
        self.forward_requested = Event()
        self.forward_outcome = None
        self.start_action_at = None

    @staticmethod
    def log(event, **fields):
        print(json.dumps(dict(event=event, **fields), ensure_ascii=False), flush=True)

    def _ready(self, healthy):
        if self.stopping or not healthy:
            return False
        return not self.busy or (self.directions == {'forward'} and not self.cancelled.is_set())

    def _log_rejection(self, request, healthy, now):
        self.log('standup_rejected', request=request, busy=self.busy,
                 lease_current=now - self.last_seen < LEASE)

    def _send(self, **message):
        if message.get('event') == 'confirm':
            self.start_action_at = self.clock() + ACTION_DELAY
            label = ('地面起身' if message['direction'] == 'standup'
                     else f'前进（最高 {FORWARD_SPEED:.2f} m/s，共 {FORWARD_SECONDS:g} 秒）')
            print(f'收到{label}指令，{ACTION_DELAY:g} 秒后执行。', flush=True)
            return
        return super()._send(**message)

    def _handle(self, message, healthy):
        if isinstance(message, dict) and message.get('action') == 'confirm':
            return
        super()._handle(message, healthy)

    def start_scheduled_action(self, now):
        if self.pending is None or self.start_action_at is None or now < self.start_action_at:
            return
        request, _, token, _ = self.pending
        self.start_action_at = None
        super()._handle(dict(action='confirm', request=request, token=token), True)

    def request_stop(self, *_):
        self.stopping = True

    def _disconnect_motion(self):
        if not self.busy:
            super()._disconnect_motion()
        else:
            self._cancel_pending('connection_lost')
            self.cancelled.set()
            self.last_seen = float('-inf')

    def preflight(self):
        from hardware_bridge import IDS, WireBus
        from run_teleop import verify_files
        from stance_reference import StanceReference

        profile = verify_files(self.certificate, physical=True)
        contract = hashlib.sha256((self.runtime / 'runtime-contract.json').read_bytes()).hexdigest()
        StanceReference.load(self.reference, profile, contract)
        if subprocess.run(['fuser', '/dev/ttyS2'], capture_output=True).returncode != 1:
            return 'serial_unavailable'
        with_bus = WireBus('/dev/ttyS2', baud=1000000, timeout=.025, live=False)
        try:
            readings = {}
            for sid in IDS:
                values = []
                for register, width in ((64, 1), (70, 1), (144, 2), (32, 2), (34, 2)):
                    status, data, _ = with_bus.read(sid, register, width)
                    if status is None or status & 0x7f or len(data) != width:
                        return 'servo_readback_unavailable'
                    values.append(int.from_bytes(data, 'little'))
                readings[sid] = values
            self.log('servo_preflight', columns=['torque', 'fault', 'voltage_dv', 'max_dv', 'min_dv'],
                     readings=readings)
            if any(torque for torque, *_ in readings.values()):
                return 'servo_torque_enabled'
            # Match the workbench's control bridge. It checks actual voltage and
            # temperature before energizing and treats bit 0 as advisory.
            if any(fault & ~1 for _, fault, *_ in readings.values()):
                return 'servo_electrical_fault'
            return None
        finally:
            with_bus.close()

    def run_confirmed(self, cancelled):
        reason = None if self.simulation else self.preflight()
        if cancelled.is_set():
            return 'connection_lost'
        if reason:
            return reason
        if self.simulation:
            self.log('simulated_floor_recovery')
            if self.walk:
                self.standing_ready.set()
                while not self.forward_requested.is_set():
                    if cancelled.wait(.05):
                        return 'connection_lost'
                if cancelled.is_set():
                    return 'connection_lost'
                self.log('simulated_forward_step')
                return 'forward_step_simulated'
            return 'standup_simulated'
        return self.run_trial(cancelled)

    def run_trial(self, cancelled):
        os.umask(0o077)
        trial = Path(tempfile.mkdtemp(prefix='voice-rise-', dir='/run'))
        token_path = trial / 'token'
        token = os.urandom(24).hex()
        token_path.write_text(token, encoding='ascii')
        with socket.socket() as address:
            address.bind(('127.0.0.1', 0))
            port = address.getsockname()[1]

        def post(endpoint, *, timeout=.5, **body):
            request = Request(f'http://127.0.0.1:{port}/{endpoint}', data=json.dumps(body).encode(),
                              headers={'X-Control-Token': token})
            with urlopen(request, timeout=timeout) as response:
                return json.load(response)

        if cancelled.is_set():
            token_path.unlink(missing_ok=True)
            return 'connection_lost'
        # Let run_teleop run its normal SIGTERM cleanup if this supervisor dies.
        parent_guard = '''
import ctypes, os, signal, sys
if ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) or os.getppid() != int(sys.argv[1]):
    raise SystemExit('Lost the voice control supervisor')
os.execv(sys.executable, [sys.executable, '-B', *sys.argv[2:]])
'''
        with (trial / 'driver.log').open('wb') as log:
            controller = subprocess.Popen([sys.executable, '-B', '-c', parent_guard, str(os.getpid()),
                str(self.runtime / 'run_teleop.py'),
                '--mode', 'live', '--supported', '--floor-recovery', '--supported-direct-handoff',
                '--certificate', str(self.certificate),
                '--reference', str(self.reference), '--token-file', str(token_path), '--output-dir', str(trial),
                '--port', str(port)], stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        self.log('floor_recovery_started', directory=str(trial))
        started = self.clock()
        outcome = 'floor_recovery_failed'
        try:
            while controller.poll() is None:
                if cancelled.is_set():
                    outcome = 'connection_lost'
                    break
                try:
                    state = post('command', action='heartbeat')
                except (OSError, http.client.HTTPException):
                    if self.clock() - started >= 2:
                        outcome = 'control_unavailable'
                        break
                else:
                    if not state.get('running') or state.get('phase') == 'stopping':
                        outcome = 'floor_recovery_failed'
                        break
                    if (state.get('handoff_gates') or {}).get('blocked'):
                        self.log('floor_recovery_handoff_blocked',
                                 diagnostic=state.get('handoff_target_diagnostic'))
                        outcome = 'handoff_blocked'
                        break
                    if self.walk_ready(state):
                        self.standing_ready.set()
                        print('已进入站立平衡。', flush=True)
                        if self.walk:
                            print('确认已站稳且前方空旷后，再说“往前走”。', flush=True)
                        outcome = self.hold_balance(post, cancelled)
                        break
                cancelled.wait(.1)
        finally:
            try:
                stopped = self.finish_trial(controller, post)
            finally:
                token_path.unlink(missing_ok=True)
        if not stopped:
            self.log('controller_cleanup_pending', directory=str(trial), pid=controller.pid)
        result_path = trial / 'result.json'
        if not result_path.is_file():
            return 'result_unknown'
        report = json.loads(result_path.read_bytes())
        torque = report.get('torque_after', {})
        self.log('floor_recovery_result', outcome=outcome, error=report.get('error'), torque=torque,
                 handoff_rejection=report.get('handoff_rejection'),
                 nonzero_updates=report.get('nonzero_command_updates'), directory=str(trial))
        if report.get('cleanup_error') or len(torque) != 15 or any(torque.values()):
            return 'torque_state_unknown'
        if not stopped:
            return 'controller_cleanup_pending'
        if cancelled.is_set():
            return 'connection_lost'
        return outcome if report.get('success') or outcome == 'connection_lost' else 'floor_recovery_failed'

    @staticmethod
    def balance_healthy(state):
        return (state.get('running') and state.get('healthy') and state.get('policy') in ('stand', 'walk')
                and not state.get('native_fallen') and not state.get('native_recovery_active'))

    def walk_ready(self, state):
        return (self.balance_healthy(state) and state.get('can_confirm_balance') and
                tuple(state.get('command', ())) == ZERO)

    def hold_balance(self, post, cancelled):
        while not cancelled.is_set():
            state = post('command', action='heartbeat')
            if not state.get('running') or state.get('stop_reason'):
                return 'floor_recovery_failed'
            if self.forward_requested.is_set():
                self.forward_requested.clear()
                self.forward_outcome = (self.run_confirmed_step(post, cancelled)
                                        if self.walk_ready(state) else 'balance_not_ready')
            cancelled.wait(.1)
        return 'connection_lost'

    def run_confirmed_step(self, post, cancelled):
        if cancelled.is_set():
            return 'connection_lost'
        try:
            post('command', action='confirm_balance')
        except HTTPError as exc:
            if exc.code != 400:
                raise
            exc.close()
            return 'balance_not_ready'
        # Unlocking changes the pose sent to robotd, so give the controller
        # time to apply it before accepting the next readiness sample.
        cancelled.wait(.25)
        deadline = self.clock() + 6
        while self.clock() < deadline and not cancelled.is_set():
            state = post('command', action='heartbeat')
            if not state.get('drive_unlocked') or not state.get('healthy') or state.get('native_fallen'):
                return 'balance_lost'
            if self.walk_ready(state):
                break
            cancelled.wait(.1)
        else:
            return 'connection_lost' if cancelled.is_set() else 'balance_not_ready'
        deadline = self.clock() + FORWARD_SECONDS
        sequence = 0
        accepted = False
        try:
            while not cancelled.is_set() and self.clock() < deadline:
                remaining = deadline - self.clock()
                if remaining <= 0.:
                    break
                fraction = min(1., remaining / FORWARD_DECELERATION)
                speed = FORWARD_SPEED * fraction * fraction * (3. - 2. * fraction)
                sequence += 1
                state = post('command', timeout=.2, action='drive', sequence=sequence,
                             vx=speed, vy=0., vyaw=0.)
                if not self.balance_healthy(state) or not state.get('drive_unlocked'):
                    return 'balance_lost'
                accepted |= tuple(state.get('command', ())) == (speed, 0., 0.)
                cancelled.wait(max(0., min(FORWARD_REFRESH, deadline - self.clock())))
        finally:
            stopped = post('command', timeout=.2, action='drive', sequence=sequence + 1,
                           vx=0., vy=0., vyaw=0.)
        if cancelled.is_set():
            return 'connection_lost'
        if not accepted or tuple(stopped.get('command', ())) != ZERO:
            return 'forward_step_unverified'
        self.log('forward_command_zeroed', updates=sequence,
                 speed=FORWARD_SPEED, requested_seconds=FORWARD_SECONDS)
        print('前进速度已归零，继续保持平衡。', flush=True)
        return 'forward_step_completed'

    def finish_trial(self, controller, post):
        if controller.poll() is not None:
            return True
        try:
            post('command', timeout=1, action='off')
        except (OSError, http.client.HTTPException):
            controller.terminate()
        # run_teleop publishes its final state after writing the result file.
        # /close can reject during that interval even though torque-off succeeded.
        deadline = self.clock() + 9
        while controller.poll() is None and self.clock() < deadline:
            try:
                post('close', timeout=1)
                break
            except HTTPError as exc:
                exc.close()
                if exc.code != 400:
                    break
            except (OSError, http.client.HTTPException):
                pass
            time.sleep(.1)
        try:
            controller.wait(timeout=3)
        except subprocess.TimeoutExpired:
            return False
        return True

    def serve(self, path):
        worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='floor-recovery')
        trial = None
        try:
            self.open(path)
            while not self.stopping:
                self._poll(True)
                now = self.clock()
                if now - self.last_seen >= LEASE:
                    self._disconnect_motion()
                if self.pending is not None and now >= self.pending[3]:
                    self._cancel_pending('expired_or_unavailable')
                self.start_scheduled_action(now)
                if trial is not None:
                    if self.peer is None:
                        self.cancelled.set()
                    if trial.done():
                        try:
                            status = trial.result()
                        except (OSError, RuntimeError, ValueError, http.client.HTTPException) as exc:
                            self.log('standup_error', reason=str(exc))
                            status = 'control_unavailable'
                        if self.active is not None:
                            self._send(event='result', request=self.active[0], status=status)
                            self.active = None
                        elif self.walk:
                            self._cancel_pending(status)
                            self.log('standup_walk_result', status=status)
                        trial = None
                        self.busy = False
                        self.cancelled = None
                        self.directions = {'standup'}
                    elif (self.standing_ready.is_set() and self.directions == {'standup'} and
                          self.active is not None and not self.cancelled.is_set()):
                        status = ('standup_simulated' if self.simulation else
                                  'ready_to_walk' if self.walk else 'standing_balanced')
                        self._send(event='result', request=self.active[0], status=status)
                        self.active = None
                        self.directions = {'forward'} if self.walk else set()
                    elif self.active is not None and self.active[1] == 'forward':
                        if self.forward_outcome is not None:
                            self._send(event='result', request=self.active[0], status=self.forward_outcome)
                            self.active = None
                        elif self.directions:
                            self.directions = set()
                            self.forward_requested.set()
                elif self.active is not None:
                    self.busy = True
                    self.cancelled = Event()
                    self.standing_ready.clear()
                    self.forward_requested.clear()
                    self.forward_outcome = None
                    trial = worker.submit(self.run_confirmed, self.cancelled)
                time.sleep(.02)
        finally:
            if trial is not None:
                self.cancelled.set()
            worker.shutdown(wait=True)
            self._disconnect()
            if self.listener is not None:
                self.listener.close()
            if self.path is not None:
                self.path.unlink(missing_ok=True)
            if self.lock_fd is not None:
                os.close(self.lock_fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--certificate', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--simulation', action='store_true')
    parser.add_argument('--walk', action='store_true')
    args = parser.parse_args()
    sys.path.insert(0, str(args.runtime.resolve(strict=True)))
    control = StandupControl(args.runtime, args.certificate, args.reference,
                             simulation=args.simulation, walk=args.walk)
    signal.signal(signal.SIGTERM, control.request_stop)
    signal.signal(signal.SIGINT, control.request_stop)
    control.serve(args.socket)


if __name__ == '__main__':
    main()
