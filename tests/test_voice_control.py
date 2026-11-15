"""Deterministic control-loop tests. No hardware files or ports are opened."""

from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from voice_control import COOLDOWN, CONFIRM_WINDOW, DURATION, LEASE, PRESETS, VoiceControl, ZERO


class FakeCommands:
    def __init__(self):
        self.lock = threading.RLock()
        self.ready = self.unlocked = True
        self.recovering = self.recovery_confirmation_required = self.stopped = False
        self.twist = ZERO

    def command(self):
        return self.twist

    def receive(self, value):
        if value['action'] == 'drive':
            self.twist = tuple(value[key] for key in ('vx', 'vy', 'vyaw'))


class VoiceControlTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.commands = FakeCommands()
        self.events = []
        self.bridge = VoiceControl(self.commands, lambda *a, **kw: None, clock=lambda: self.now)
        self.bridge._send = lambda **message: self.events.append(message)

    def send(self, action, **extra):
        self.bridge._handle(dict(action=action, **extra), True)

    def drive(self, healthy=True, accepted=True):
        def rpc(method, parameters):
            self.assertEqual(method, 'robot.move')
            self.events.append(tuple(parameters[key] for key in ('vx', 'vy', 'vyaw')))
            return {'accepted': accepted}

        return self.bridge.send_command(rpc, healthy)

    def request(self, number, direction='forward'):
        self.send('heartbeat')
        self.send('request', request=number, direction=direction)
        return self.events[-1]

    def confirm(self, request):
        self.send('confirm', request=request['request'], token=request['token'])

    def test_confirmation_is_required_and_bound_to_request(self):
        prompt = self.request(1)
        self.assertEqual(self.drive(), ZERO)
        self.send('confirm', request=2, token=prompt['token'])
        self.assertEqual(self.drive(), ZERO)
        self.send('confirm', request=1, token='wrong')
        self.assertEqual(self.events[-1]['status'], 'rejected')
        self.confirm(prompt)
        self.assertEqual(self.drive(), ZERO)

    def test_one_pulse_then_zero_and_cooldown(self):
        prompt = self.request(1, 'left')
        self.confirm(prompt)
        self.confirm(prompt)
        self.assertEqual(self.drive(), PRESETS['left'])
        self.now += DURATION / 2
        self.send('heartbeat')
        self.now += DURATION / 2 + .01
        self.assertEqual(self.drive(), ZERO)
        self.assertEqual(self.events[-1]['status'], 'completed')
        self.assertEqual(self.request(2, 'right')['status'], 'rejected')
        self.now += COOLDOWN
        self.assertEqual(self.request(3, 'right')['direction'], 'right')

    def test_rejects_expired_cancelled_invalid_and_unready(self):
        prompt = self.request(1)
        self.now += CONFIRM_WINDOW
        self.send('heartbeat')
        self.confirm(prompt)
        self.assertEqual(self.drive(), ZERO)
        prompt = self.request(2)
        self.send('cancel', request=2)
        self.confirm(prompt)
        self.assertEqual(self.drive(), ZERO)
        self.assertEqual(self.request(3, 'reverse')['status'], 'rejected')
        self.commands.ready = False
        self.assertEqual(self.request(4)['status'], 'rejected')

    def test_stale_lease_and_closed_peer_revoke(self):
        self.confirm(self.request(1))
        self.assertEqual(self.drive(), PRESETS['forward'])
        self.now += LEASE + .01
        self.assertEqual(self.drive(), ZERO)
        self.assertEqual(self.events[-2]['status'], 'connection_lost')
        self.now += COOLDOWN
        self.confirm(self.request(2))
        self.assertEqual(self.drive(), PRESETS['forward'])
        self.bridge._disconnect()
        self.assertEqual(self.drive(), ZERO)

    def test_nonzero_manual_command_takes_over_but_neutral_does_not(self):
        self.confirm(self.request(1))
        self.bridge.operator(dict(action='drive', vx=0., vy=0., vyaw=0.))
        self.assertEqual(self.drive(), PRESETS['forward'])
        self.bridge.operator(dict(action='drive', vx=-.05, vy=0., vyaw=0.))
        self.assertEqual(self.events[-1]['status'], 'manual_takeover')
        self.assertEqual(self.drive(), (-.05, 0., 0.))

    def test_health_loss_and_missing_rpc_ack(self):
        self.confirm(self.request(1))
        self.assertEqual(self.drive(healthy=False), ZERO)
        self.assertEqual(self.events[-1]['status'], 'control_unavailable')
        self.now += COOLDOWN
        self.confirm(self.request(2))
        with self.assertRaisesRegex(RuntimeError, 'rejected velocity'):
            self.drive(accepted=False)
        self.assertEqual(self.events[-1]['status'], 'unknown')


if __name__ == '__main__':
    unittest.main()
