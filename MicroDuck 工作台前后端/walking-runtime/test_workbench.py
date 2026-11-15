import copy
import tempfile
import time
import unittest
from unittest.mock import patch, Mock

from workbench_server import Workbench, normalize_state


def viewer(now=100.):
    return {'unix_time': now, 'source': 'software-zero-live', 'software_zero_data_ready': True,
            'torque': {str(i): 0 for i in range(15)}, 'logical_degrees': {'34': -5.1}, 'health': {}}


class FakeBackend:
    def __init__(self):
        self.state = {'phase': 'idle', 'running': False, 'can_start': True,
                      'selected_mode': 'walk', 'can_confirm_balance': False, 'drive_unlocked': False}
        self.commands = []

    def request(self, route, body=None):
        if route != '/status':
            self.commands.append((route, body))
        if route == '/speed':
            self.state['step_speed'] = body['speed']
        return copy.deepcopy(self.state)


class WorkbenchTests(unittest.TestCase):
    def test_log_storage_failure_does_not_reject_an_accepted_stop(self):
        bench, backend = self.make()
        with patch('pathlib.Path.open', side_effect=OSError(28, 'No space left')):
            state = bench.action('off', {})
        self.assertEqual(backend.commands, [('/off', None)])
        self.assertIn('No space left', state['log_error'])

    def test_telemetry_failure_preserves_resource_cleanup(self):
        from manual_walk_panel import Panel
        panel = Panel.__new__(Panel)
        panel.audit_error = None
        trace = Mock()
        trace.write.side_effect = OSError(28, 'No space left')
        trace.close.side_effect = OSError(28, 'No space left')
        self.assertIsNone(panel.write_trace(trace, {'elapsed_s': 1.}))
        panel.tunnel = Mock()
        panel.tunnel.poll.return_value = None
        guard = panel.control_guard = Mock()
        panel.close_local_resources(trace)
        panel.tunnel.terminate.assert_called_once()
        guard.close.assert_called_once()
        self.assertIsNone(panel.control_guard)

    def test_old_torque_readings_never_prove_current_relaxation(self):
        state = normalize_state({'phase': 'complete', 'running': False, 'can_start': True}, viewer(), now=103.)
        self.assertFalse(state['torque_off_verified'])
        self.assertFalse(state['can_start'])
        self.assertTrue(all(j['actual'] is None for j in state['joints']))

    def test_reconnecting_does_not_reuse_previous_balancing_metrics(self):
        state = normalize_state({'phase': 'reconnecting', 'running': True, 'healthy': True,
                                 'tilt_deg': 1., 'volts_min': 6.5, 'policy': 'walk'}, viewer(), now=100.)
        self.assertEqual(state['status'], 'unknown')
        self.assertIsNone(state['tilt_deg'])
        self.assertIsNone(state['volts_min'])
        self.assertFalse(state['torque_off_verified'])

    def test_handoff_wait_is_distinct_from_active_policy(self):
        state = normalize_state({'phase': 'homing', 'running': True, 'healthy': True, 'policy': 'held',
                                 'handoff_gates': {'policy_enable_requested': False, 'tracking_max_deg': 8.,
                                                   'tracking_limit_deg': 6.}}, viewer(), now=100.)
        self.assertTrue(state['settling'])
        self.assertNotEqual(state['status'], 'balancing')
        state = normalize_state({'phase': 'balancing', 'running': True, 'healthy': True, 'policy': 'walk',
                                 'handoff_gates': {'policy_enable_requested': True, 'tracking_max_deg': 60.}}, viewer(), now=100.)
        self.assertFalse(state['settling'])
        self.assertIsNone(state['tracking_max_deg'])

    def make(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        backend = FakeBackend()
        bench = Workbench(backend, directory.name)
        bench.test_mode = True
        return bench, backend

    def test_opening_and_polling_the_workbench_never_starts_a_controller(self):
        bench, backend = self.make()
        for _ in range(5):
            bench.action('status', {})
        self.assertFalse(backend.commands)

    def test_start_requires_explicit_physical_preparation(self):
        bench, backend = self.make()
        for body in ({}, {'supported': False}, {'supported': 'true'}):
            with self.assertRaises(ValueError):
                bench.action('dynamic', body)
        self.assertFalse(backend.commands)
        bench.action('dynamic', {'supported': True})
        self.assertEqual(backend.commands, [('/start', {'supported': True})])

    def test_static_start_uses_the_explicit_mode_transition(self):
        bench, backend = self.make()
        bench.action('static', {'supported': True})
        self.assertEqual(backend.commands, [('/switch', {'mode': 'manual', 'supported': True})])

    def test_recover_never_requests_an_automatic_mode_restart(self):
        bench, backend = self.make()
        bench.action('recover', {'supported': True})
        self.assertEqual(backend.commands, [('/recover', {'supported': True})])

    def test_emergency_stop_is_available_without_preparation_or_known_state(self):
        bench, backend = self.make()
        backend.state.update(phase='reconnecting', running=True, can_start=False)
        bench.action('off', {})
        self.assertEqual(backend.commands, [('/off', None)])

    def test_motion_requires_standing_confirmation_and_stays_bounded(self):
        bench, backend = self.make()
        with self.assertRaises(ValueError):
            bench.action('drive', {'vx': .05})
        backend.state.update(running=True, healthy=True, policy='walk', can_confirm_balance=True, drive_unlocked=True)
        for velocity in (.1, True, float('nan')):
            with self.assertRaises(ValueError):
                bench.action('drive', {'vx': velocity})
        bench.action('drive', {'vx': -.05})
        self.assertEqual(backend.commands[-1], ('/drive', {'vx': -.05, 'vy': 0, 'vyaw': 0}))

    def test_slider_sets_both_directions_and_rejects_out_of_range_values(self):
        bench, backend = self.make()
        backend.state.update(running=True, healthy=True, policy='walk', drive_unlocked=True)
        for value in (-.01, 1.01, .105, True, float('nan')):
            with self.assertRaises(ValueError):bench.action('speed', {'speed': value})
        bench.action('speed', {'speed': .1})
        self.assertEqual(backend.commands[-1], ('/speed', {'speed': .1}))
        for value in (.1, -.1, 0):bench.action('drive', {'vx': value})
        bench.action('speed', {'speed': 1.})
        for value in (1., -1., 0):bench.action('drive', {'vx': value})
        with self.assertRaises(ValueError):bench.action('drive', {'vx': .05})

    def test_unlocked_motion_survives_a_changed_confirmation_ready_flag(self):
        bench, backend = self.make()
        backend.state.update(phase='balancing', running=True, healthy=True, policy='walk',
                             drive_unlocked=True, can_confirm_balance=False)
        bench.action('drive', {'vx': .05})
        self.assertEqual(backend.commands[-1], ('/drive', {'vx': .05, 'vy': 0, 'vyaw': 0}))
        backend.state['native_recovery_active'] = True
        with self.assertRaises(ValueError):
            bench.action('drive', {'vx': .05})


if __name__ == '__main__':
    unittest.main()
