"""In-memory controller for exercising the HTTP workbench without hardware."""
import math
import threading
import time

from policy_coordinates import HOME, IDS
from stance_kinematics import shift_pose
from stance_state import StanceState

class SimulationBackend:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = 'walk'
        self.session = None
        self.recovery = False
        self.unlocked = False
        self.speed = .05
        self.velocity = 0.
        self.drive_at = 0.
        self.pose = self._new_pose()

    @staticmethod
    def _new_pose():
        pose = StanceState()
        pose.initialize(list(HOME))
        return pose

    def _status(self):
        running = self.session is not None
        elapsed = time.monotonic() - self.session if running else 0.
        moving = self.velocity if time.monotonic() - self.drive_at < .35 else 0.
        manual = self.mode == 'manual'
        settled = running and elapsed >= 1.
        phase = 'idle' if not running else ('homing' if not settled else 'holding' if manual else 'balancing')
        q = self.pose.public()
        angles = {str(s): math.degrees(a) for s, a in zip(IDS, self.pose.target)}
        tilt = (70 * max(0., 1. - elapsed) if self.recovery else 0.) + .5 + abs(moving)
        return {
            'phase': phase, 'running': running, 'healthy': settled, 'can_start': not running,
            'selected_mode': self.mode, 'policy': ('held' if manual or not settled else 'walk') if running else None,
            'can_adjust_pose': manual and settled, 'step_speed': self.speed,
            'drive_unlocked': self.unlocked, 'can_confirm_balance': settled and not manual,
            'tilt_deg': tilt if running else None, 'pitch_deg': tilt if running else None,
            'roll_deg': -.1 if running else None, 'volts_min': 6.6 if running else None,
            'temperature_max': 35 if running else None, 'control_hz': 50 if running else None,
            'body_shift_mm': q['body_shift_mm'], 'group_offsets_deg': q['group_offsets_deg'],
            'joint_degrees': angles, 'commanded_joint_degrees': angles,
            'diagnostics': {'joint_currents_ma': {s: 30 for s in angles}} if running else {},
            'message': ('模拟前进' if moving > 0 else '模拟后退') if moving else '离线模拟数据',
        }

    def request(self, route, body=None):
        body = body or {}
        with self.lock:
            if route in ('/start', '/standup', '/switch'):
                self.mode = body.get('mode', 'walk') if route != '/start' else self.mode
                self.recovery = route == '/standup'
                self.session = time.monotonic()
                self.pose = self._new_pose()
                self.unlocked = False
                self.velocity = 0.
            elif route in ('/off', '/recover'):
                self.session = None
                self.unlocked = False
                self.velocity = 0.
            elif route == '/confirm':
                if not self._status()['can_confirm_balance']:
                    raise ValueError('请等待模拟平衡就绪')
                self.unlocked = True
            elif route in ('/speed', '/drive', '/step'):
                if route == '/speed':
                    self.speed = body['speed']
                    self.velocity = 0.
                else:
                    self.velocity = body['vx']
                    self.drive_at = time.monotonic()
            elif route == '/adjust':
                self.pose.receive({'action': 'heartbeat'})
                self.pose.update_gate(moving=False, settled=True)
                self.pose.receive(body)
                command = self.pose.take_command()
                try:
                    plan = self.pose.plan(command, self.pose.target, [0., 0., -1.], shift_function=shift_pose)
                    self.pose.commit(plan)
                except ValueError as error:
                    self.pose.reject(error)
                    raise
                finally:
                    self.pose.update_gate(moving=False, settled=True)
            elif route == '/status' and self.session is not None:
                self.pose.receive({'action': 'heartbeat'})
            elif route not in ('/save', '/status'):
                raise ValueError('未知模拟控制接口')
            state = self._status()
            if route == '/save':
                state['saved_path'] = 'simulation'
            return state
