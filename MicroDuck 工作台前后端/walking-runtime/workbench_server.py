import argparse
from datetime import datetime
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import secrets
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parent.parent
ASSET = Path(__file__).with_name('workbench.html')
OUT = ROOT / 'outputs/workbench'
JOINTS = {
    '20': '左髋旋转', '21': '左髋侧摆', '22': '左髋俯仰', '23': '左膝', '24': '左踝',
    '30': '颈部俯仰', '31': '头部俯仰', '32': '头部旋转', '33': '头部侧摆', '34': '鸭嘴',
    '10': '右髋旋转', '11': '右髋侧摆', '12': '右髋俯仰', '13': '右膝', '14': '右踝',
}


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def normalize_state(state, viewer, *, now=None):
    now = time.time() if now is None else now
    running = state.get('running') is True
    phase = state.get('phase', 'unknown')
    unknown = phase in ('reconnecting', 'recovery_error', 'needs_recovery', 'unknown')
    report = state.get('report') or {}
    diagnostics = state.get('diagnostics') or {}
    fresh = bool(viewer and now - viewer.get('unix_time', 0) < 2 and
                 viewer.get('software_zero_data_ready') and viewer.get('source') == 'software-zero-live')
    torque = viewer.get('torque', {}) if fresh else {}
    off = len(torque) == 15 and not any(torque.values()) and not running
    if unknown:
        status, title, tone = 'unknown', '状态未知 · 请托稳', 'danger'
    elif state.get('recovering') or phase == 'recovering':
        status, title, tone = 'recovering', '正在恢复连接与配置', 'warning'
    elif running and state.get('healthy') and state.get('policy') in ('walk', 'stand'):
        status, title, tone = 'balancing', '动态平衡运行中', 'good'
    elif running and phase == 'holding':
        status, title, tone = 'holding', '固定站姿保持中', 'good'
    elif running:
        status, title, tone = 'preparing', '正在准备姿态', 'warning'
    elif off:
        status, title, tone = 'off', '已确认全部卸力', 'neutral'
    else:
        status, title, tone = 'unknown', '等待新鲜的关节状态', 'warning'
    actual = diagnostics.get('joint_degrees') or state.get('joint_degrees') or {}
    targets = diagnostics.get('commanded_joint_degrees') or state.get('commanded_joint_degrees') or {}
    if not running:
        actual = viewer.get('logical_degrees', {}) if fresh else {}
        targets = {}
    if unknown:
        actual, targets = {}, {}
    health = viewer.get('health', {}) if fresh and not running else {}
    voltages = [v['voltage'] for v in health.values() if isinstance(v.get('voltage'), (int, float))]
    temps = [v['temperature_c'] for v in health.values() if isinstance(v.get('temperature_c'), (int, float))]
    gates = state.get('handoff_gates') or {}
    settling = running and not gates.get('policy_enable_requested') and state.get('policy') in ('held', 'homing')
    result = {
        'status': status, 'title': title, 'tone': tone, 'phase': phase, 'running': running,
        'unknown': unknown, 'healthy': state.get('healthy') if not unknown else None,
        'message': state.get('message') or '', 'gate': state.get('gate'),
        'recovering': bool(state.get('recovering')), 'can_start': bool(state.get('can_start')) and off and not unknown,
        'can_confirm_balance': bool(state.get('can_confirm_balance')) and not unknown,
        'drive_unlocked': bool(state.get('drive_unlocked')) and not unknown,
        'can_drive': (running and state.get('healthy') is True and not unknown and
                      state.get('selected_mode', 'walk') == 'walk' and
                      state.get('policy') in ('walk', 'stand') and
                      state.get('drive_unlocked') is True and
                      not state.get('native_recovery_active') and
                      not state.get('recovery_confirmation_required') and
                      phase not in ('stopping', 'error', 'recovering')),
        'can_adjust_pose': bool(state.get('can_adjust_pose')) and not unknown,
        'selected_mode': state.get('selected_mode', 'walk'), 'policy': state.get('policy') if running else None,
        'step_speed': state.get('step_speed', .05),
        'torque_off_verified': off and not unknown, 'viewer_fresh': fresh,
        'tilt_deg': state.get('tilt_deg') if running and not unknown else None,
        'pitch_deg': state.get('pitch_deg') if running and not unknown else None,
        'roll_deg': state.get('roll_deg') if running and not unknown else None,
        'volts_min': (state.get('volts_min') if running else min(voltages, default=None)) if not unknown else None,
        'temperature_max': (state.get('temperature_max') if running else max(temps, default=None)) if not unknown else None,
        'control_hz': state.get('control_hz') if running and not unknown else None,
        'elapsed_s': state.get('elapsed_s') if running else None,
        'settling': settling and not unknown,
        'tracking_max_deg': gates.get('tracking_max_deg') if settling and not unknown else None,
        'tracking_limit_deg': gates.get('tracking_limit_deg') if settling and not unknown else None,
        'tilt_limit_deg': gates.get('tilt_limit_deg') if settling and not unknown else None,
        'joints': [{'id': sid, 'name': name, 'actual': actual.get(sid), 'target': targets.get(sid),
                    'current_ma': diagnostics.get('joint_currents_ma', {}).get(sid) if not unknown else None}
                   for sid, name in JOINTS.items()],
        'last_error': report.get('error') or report.get('hardware_failure_cause'),
        'log_available': bool(state.get('diagnostic_log') or report.get('diagnostic_log')),
        'body_shift_mm': state.get('body_shift_mm', 0),
        'group_offsets_deg': state.get('group_offsets_deg') or {'hip': 0, 'knee': 0, 'ankle': 0},
        'saved_path': state.get('saved_path'), 'server_time': now,
    }
    return result


class Backend:
    def __init__(self, port):
        self.url = f'http://127.0.0.1:{port}'
        self.token = None
        self.lock = threading.Lock()

    def request(self, route, body=None):
        with self.lock:
            if not self.token:
                with urllib.request.urlopen(self.url + '/', timeout=2) as response:
                    page = response.read().decode()
                self.token = re.search(r"const token='([^']+)'", page).group(1)
            request = urllib.request.Request(self.url + route, data=json.dumps(body or {}).encode(),
                                            headers={'Content-Type': 'application/json', 'X-Control-Token': self.token},
                                            method='POST')
            try:
                with urllib.request.urlopen(request, timeout=3) as response:
                    result = json.load(response)
            except urllib.error.HTTPError as error:
                if error.code == 403:
                    self.token = None
                    raise ValueError('控制服务已更新，请刷新工作台')
                try:
                    result = json.loads(error.read())
                except ValueError:
                    raise ValueError('控制服务没有返回有效状态')
                raise ValueError(result.get('error') or result.get('message') or '操作未接受')
            if result.get('error'):
                raise ValueError(result['error'])
            return result


class Workbench:
    def __init__(self, backend, output=OUT, *, host=None):
        self.backend = backend
        self.host = host
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.token = secrets.token_urlsafe(32)
        self.action_lock = threading.Lock()
        self.last_state = {}
        self.last_ui_seen = 0.
        self.imu = None
        self.imu_busy = False
        self.imu_error = None
        self.test_mode = False
        self.log_error = None

    def record(self, action, details):
        try:
            with (self.output / 'workbench-events.jsonl').open('a') as stream:
                stream.write(json.dumps({'unix_time': time.time(), 'action': action, **details}, ensure_ascii=False) + '\n')
        except OSError as error:
            self.log_error = '本机操作日志暂停：' + str(error)

    def snapshot(self):
        state = self.backend.request('/status')
        self.last_state = state
        viewer = {} if self.test_mode else read_json(ROOT / 'outputs/live-joint-viewer-20261017/state.json', {})
        result = normalize_state(state, viewer)
        if self.test_mode:
            result['can_start'] = bool(state.get('can_start'))
            if not state.get('running'):
                result.update(status='off', title='模拟 · 已卸力', tone='neutral', torque_off_verified=True)
        result.update(imu_sample=self.imu, imu_busy=self.imu_busy, imu_error=self.imu_error,
                      log_error=self.log_error or state.get('local_diagnostic_error'),
                      simulation=self.test_mode, config={'gain': 'P200 / I0 / D0', 'mouth': '默认闭合',
                                                        'imu': '支架竖直标定', 'speed': state.get('step_speed', .05)})
        return result

    def action(self, action, body):
        if action == 'status':
            self.last_ui_seen = time.monotonic()
            return self.snapshot()
        if action == 'off':
            self.backend.request('/off')
            self.record(action, {})
            return self.snapshot()
        if action == 'halt':
            self.backend.request('/drive', {'vx': 0, 'vy': 0, 'vyaw': 0})
            return self.snapshot()
        if action == 'drive':
            vx = body.get('vx')
            current = self.snapshot()
            speed = current['step_speed']
            if type(vx) not in (int, float) or vx not in (-speed, 0, speed) or not abs(vx)<=1.:
                raise ValueError('移动速度必须匹配滑块设置，最大 1.00 m/s')
            if vx:
                if not current['can_drive']:
                    raise ValueError('请先确认独立站稳并解锁移动')
            self.backend.request('/drive', {'vx': vx, 'vy': 0, 'vyaw': 0})
            return self.snapshot()
        if action == 'speed':
            speed = body.get('speed')
            if (type(speed) not in (int, float) or not math.isfinite(speed) or not 0 <= speed <= 1. or
                    abs(speed*100-round(speed*100)) > 1e-8):
                raise ValueError('速度范围为 0～1.00 m/s，每档 0.01')
            with self.action_lock:
                self.backend.request('/speed', {'speed': speed})
                self.record(action, {'speed': speed})
                return self.snapshot()
        with self.action_lock:
            state = self.snapshot()
            if self.imu_busy:
                raise ValueError('正在读取 IMU，完成后才能开始其他操作')
            if action in ('dynamic', 'static', 'standup', 'recover'):
                if body.get('supported') is not True:
                    raise ValueError('请先确认已摆好姿态并托稳躯干')
                if action != 'recover' and not state['can_start']:
                    raise ValueError(state.get('gate') or '需要先结束旧会话并确认全部卸力')
                if action == 'recover':
                    self.backend.request('/recover', {'supported': True})
                elif action == 'standup':
                    self.backend.request('/standup')
                else:
                    desired = 'manual' if action == 'static' else 'walk'
                    if state['selected_mode'] == desired:
                        self.backend.request('/start', {'supported': True})
                    else:
                        self.backend.request('/switch', {'mode': desired, 'supported': True})
            elif action == 'confirm':
                if body.get('independent') is not True:
                    raise ValueError('请确认已经能独立站稳')
                self.backend.request('/confirm')
            elif action == 'step':
                if body.get('direction') not in ('forward', 'backward'):
                    raise ValueError('无效的移动方向')
                self.backend.request('/step', {'vx': .05 if body['direction'] == 'forward' else -.05})
            elif action == 'adjust':
                if not state['can_adjust_pose']:
                    raise ValueError('请先启动固定站姿并等待稳定')
                kind = body.get('kind')
                delta = body.get('delta')
                if type(delta) not in (int, float) or delta not in (-.5, .5):
                    raise ValueError('每次只允许调整 0.5')
                if kind == 'shift':
                    command = {'action': 'body_shift', 'millimeters': state['body_shift_mm'] + delta}
                elif kind in ('hip', 'knee', 'ankle'):
                    command = {'action': 'adjust_joint', 'group': kind,
                               'degrees': state['group_offsets_deg'][kind] + delta}
                else:
                    raise ValueError('未知的站姿调整')
                self.backend.request('/adjust', command)
            elif action in ('reset', 'capture', 'save'):
                if not state['can_adjust_pose']:
                    raise ValueError('请先启动固定站姿并等待稳定')
                self.backend.request('/save' if action == 'save' else '/adjust',
                                     {} if action == 'save' else {'action': action})
            elif action == 'imu':
                if not state['torque_off_verified']:
                    raise ValueError('只读姿态采样需要先确认全部卸力')
                if self.test_mode:
                    self.imu = {'unix_time': time.time(), 'pitch_deg': .07, 'roll_deg': -.05, 'tilt_deg': .09,
                                'samples': 30, 'register_write_packets': 0}
                else:
                    self.imu_busy, self.imu_error = True, None
                    threading.Thread(target=self.read_imu, daemon=True).start()
            else:
                raise ValueError('未知操作')
            self.record(action, {k: v for k, v in body.items() if k in ('supported', 'kind', 'delta', 'direction', 'independent')})
        return self.snapshot()

    def read_imu(self):
        try:
            from workbench_imu import capture
            self.imu = capture(self.output, self.host)
        except Exception as error:
            self.imu_error = str(error)
        finally:
            self.imu_busy = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8770)
    parser.add_argument('--backend-port', type=int, default=8769)
    parser.add_argument('--output-dir', type=Path, default=OUT)
    parser.add_argument('--simulation', action='store_true')
    parser.add_argument('--host', help='Live board IPv4 address for IMU sampling')
    args = parser.parse_args()
    if args.simulation:
        from workbench_simulation import SimulationBackend
        backend = SimulationBackend()
    else:
        if not args.host:
            parser.error('--host is required for live mode')
        backend = Backend(args.backend_port)
    workbench = Workbench(backend, args.output_dir, host=args.host)
    workbench.test_mode = args.simulation

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, body, status=200, content_type='application/json'):
            data = body.encode() if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                page = ASSET.read_text().replace('__TOKEN__', workbench.token)
                self.reply(page, content_type='text/html')
            elif self.path == '/healthz':
                self.reply({'app': 'microduck-workbench', 'version': 1, 'simulation': workbench.test_mode,
                            'pid': __import__('os').getpid(), 'ui_age_s': time.monotonic() - workbench.last_ui_seen})
            elif self.path == '/download/session.json':
                report = {'exported_at': datetime.now().isoformat(), 'state': workbench.last_state,
                          'imu_sample': workbench.imu, 'automatic_start': False,
                          'simulation': workbench.test_mode}
                self.reply(report)
            else:
                self.reply({'error': 'Not found'}, 404)

        def do_POST(self):
            if self.headers.get('X-Workbench-Token') != workbench.token:
                return self.reply({'error': '请刷新工作台后重试'}, 403)
            origin = self.headers.get('Origin')
            if origin and origin not in (f'http://127.0.0.1:{args.port}', f'http://localhost:{args.port}'):
                return self.reply({'error': '不接受其他页面的控制请求'}, 403)
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 <= size <= 1000:
                return self.reply({'error': '请求过大'}, 400)
            try:
                body = json.loads(self.rfile.read(size) or b'{}')
                if not isinstance(body, dict) or not self.path.startswith('/api/'):
                    raise ValueError('无效请求')
                result = workbench.action(self.path[5:], body)
                self.reply(result)
            except ValueError as error:
                self.reply({'error': str(error)}, 400)
            except Exception as error:
                self.reply({'error': '控制连接不可用，请托稳并检查主板连接。', 'detail': type(error).__name__}, 503)

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    print(f'MicroDuck 工作台 http://127.0.0.1:{args.port}', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
