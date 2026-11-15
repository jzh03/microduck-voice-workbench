"""Regression checks for the supervised workbench handoff and shutdown."""

import io
import json
from pathlib import Path
import runpy
import tempfile
from threading import Event
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'voice-standup.py'
StandupControl = runpy.run_path(str(SCRIPT))['StandupControl']


class StandupTrialTest(unittest.TestCase):
    def test_forward_tapers_to_zero_inside_five_seconds(self):
        bridge = StandupControl(None, None, None, walk=True)
        ready = {'running': True, 'healthy': True, 'drive_unlocked': True, 'can_confirm_balance': True,
                 'native_fallen': False, 'policy': 'stand', 'command': [0, 0, 0]}
        now = 0.
        bridge.clock = lambda: now
        cancelled = Mock()
        cancelled.is_set.return_value = False

        def wait(seconds):
            nonlocal now
            now += seconds

        cancelled.wait.side_effect = wait
        speeds = []

        def respond(endpoint, **request):
            if request['action'] == 'drive':
                speeds.append((now, request['vx']))
            return {**ready, 'command': [request.get('vx', 0), 0, 0]}

        post = Mock(side_effect=respond)
        self.assertEqual(bridge.run_confirmed_step(post, cancelled), 'forward_step_completed')
        confirm, heartbeat, *drive, halt = [entry.kwargs for entry in post.call_args_list]
        self.assertEqual(confirm, {'action': 'confirm_balance'})
        self.assertEqual(heartbeat, {'action': 'heartbeat'})
        self.assertEqual(speeds[0][1], .5)
        self.assertTrue(all(0 < packet['vx'] <= .5 for packet in drive))
        self.assertTrue(all(new <= old for (_, old), (_, new) in zip(speeds, speeds[1:])))
        self.assertLess(speeds[-2][1], .01)
        self.assertAlmostEqual(speeds[-1][0] - speeds[0][0], 5.)
        self.assertEqual([packet['sequence'] for packet in drive + [halt]], list(range(1, len(drive) + 2)))
        self.assertEqual((halt['vx'], halt['vy'], halt['vyaw']), (0, 0, 0))
        self.assertAlmostEqual(now, 5.25)

    def test_lost_balance_prevents_step(self):
        bridge = StandupControl(None, None, None, walk=True)
        post = Mock(side_effect=[{}, {'drive_unlocked': False, 'healthy': True}])
        self.assertEqual(bridge.run_confirmed_step(post, Event()), 'balance_lost')
        self.assertFalse(any(call.kwargs['action'] == 'drive' for call in post.call_args_list))

    def test_disconnect_during_forward_clears_drive(self):
        bridge = StandupControl(None, None, None, walk=True)
        cancelled = Event()
        ready = {'running': True, 'healthy': True, 'drive_unlocked': True, 'can_confirm_balance': True,
                 'native_fallen': False, 'policy': 'stand', 'command': [0, 0, 0]}

        def respond(endpoint, **body):
            if body.get('vx'):
                cancelled.set()
            return ready

        post = Mock(side_effect=respond)
        self.assertEqual(bridge.run_confirmed_step(post, cancelled), 'connection_lost')
        self.assertEqual([(call.kwargs['sequence'], call.kwargs['vx'])
                          for call in post.call_args_list if call.kwargs['action'] == 'drive'], [(1, .5), (2, 0)])

    def test_automatic_start_waits_one_second_and_ignores_terminal_confirmation(self):
        bridge = StandupControl(None, None, None, walk=True)
        now = 0.
        bridge.clock = lambda: now
        bridge._handle({'action': 'heartbeat'}, True)
        request = {'action': 'request', 'direction': 'standup', 'request': 1}
        bridge._handle(request, True)
        bridge._handle({'action': 'confirm', 'request': 1, 'token': bridge.pending[2]}, True)
        for now in (.25, .5, .75, .99):
            bridge._handle({'action': 'heartbeat'}, True)
            bridge.start_scheduled_action(now)
            self.assertIsNone(bridge.active)
        now = 1.
        bridge.start_scheduled_action(now)
        self.assertEqual(bridge.active[:2], (1, 'standup'))
        self.assertIsNone(bridge.pending)
        bridge._handle(request, True)
        self.assertIsNone(bridge.pending)

    def test_disconnect_cancels_delayed_action(self):
        bridge = StandupControl(None, None, None, walk=True)
        now = 0.
        bridge.clock = lambda: now
        bridge._handle({'action': 'heartbeat'}, True)
        bridge._handle({'action': 'request', 'direction': 'standup', 'request': 1}, True)
        now = 1.
        bridge._handle({'action': 'heartbeat'}, True)
        bridge.start_scheduled_action(now)
        self.assertIsNone(bridge.active)
        self.assertIsNone(bridge.pending)

    def test_hold_keeps_heartbeats_before_and_after_forward(self):
        bridge = StandupControl(None, None, None, walk=True)
        now = 0.
        bridge.clock = lambda: now
        cancelled = Mock()
        cancelled.is_set.side_effect = lambda: now >= 100.

        def wait(seconds):
            nonlocal now
            now += 20.
            if now == 40.:
                bridge.forward_requested.set()

        cancelled.wait.side_effect = wait
        post = Mock(return_value={'running': True, 'healthy': True, 'policy': 'stand',
                                  'can_confirm_balance': True, 'command': [0, 0, 0]})
        with patch.object(bridge, 'run_confirmed_step', return_value='forward_step_completed') as forward:
            self.assertEqual(bridge.hold_balance(post, cancelled), 'connection_lost')
        forward.assert_called_once_with(post, cancelled)
        self.assertEqual(bridge.forward_outcome, 'forward_step_completed')
        self.assertEqual([call.kwargs['action'] for call in post.call_args_list], ['heartbeat'] * 5)

    def test_standup_keeps_waiting_after_ten_seconds_until_cancelled(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = StandupControl(root, root / 'certificate', root / 'reference', walk=True)
            now = 0.
            bridge.clock = lambda: now
            child = Mock()
            child.poll.return_value = None
            cancelled = Mock()
            cancelled.is_set.side_effect = lambda: now >= 15

            def wait(_):
                nonlocal now
                now += 3

            cancelled.wait.side_effect = wait
            post = Mock(side_effect=lambda *args, **kwargs: io.BytesIO(b'{"running":true}'))
            (root / 'result.json').write_text(json.dumps({
                'success': True, 'torque_after': {str(i): 0 for i in range(15)},
            }), encoding='ascii')
            with patch.dict(StandupControl.run_trial.__globals__, {'urlopen': post}), \
                    patch('tempfile.mkdtemp', return_value=directory), \
                    patch('subprocess.Popen', return_value=child), patch('os.umask'), \
                    patch.object(bridge, 'finish_trial', return_value=True) as finish:
                self.assertEqual(bridge.run_trial(cancelled), 'connection_lost')
            self.assertEqual(post.call_count, 5)
            finish.assert_called_once()

    def test_matches_workbench_flags_and_waits_for_close(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = StandupControl(root, root / 'certificate', root / 'reference')
            child = Mock()
            child.poll.return_value = None
            routes = []

            def post(request, *, timeout):
                route = request.full_url.rsplit('/', 1)[-1]
                body = json.loads(request.data)
                routes.append((route, body))
                if route == 'close' and routes.count(('close', {})) == 1:
                    raise HTTPError(request.full_url, 400, 'Session still running', None, io.BytesIO())
                if body.get('action') == 'off':
                    (root / 'result.json').write_text(json.dumps({
                        'success': True, 'torque_after': {str(i): 0 for i in range(15)},
                        'handoff_rejection': {'joint_id': 12},
                    }), encoding='ascii')
                return io.BytesIO(b'{"running":true,"handoff_gates":{"blocked":true}}')

            with patch.dict(StandupControl.run_trial.__globals__, {'urlopen': post}), \
                    patch('tempfile.mkdtemp', return_value=directory), \
                    patch('subprocess.Popen', return_value=child) as spawn, \
                    patch('os.umask'):
                self.assertEqual(bridge.run_trial(Event()), 'handoff_blocked')

            args = spawn.call_args.args[0]
            for option in ('--supported', '--floor-recovery', '--supported-direct-handoff'):
                self.assertIn(option, args)
            self.assertEqual(routes, [('command', {'action': 'heartbeat'}),
                                      ('command', {'action': 'off'}), ('close', {}), ('close', {})])
            child.terminate.assert_not_called()
            child.wait.assert_called_once_with(timeout=3)
            self.assertFalse((root / 'token').exists())


if __name__ == '__main__':
    unittest.main()
