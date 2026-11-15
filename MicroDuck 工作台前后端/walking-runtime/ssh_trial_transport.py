"""Pinned-key SSH pipe for the supervised standing panel; never arms by itself."""
import contextlib
import ipaddress
import json
import os
from pathlib import Path
import re
import select
import shlex
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parent.parent
VIEWER = ROOT / 'outputs/live-joint-viewer-20261017'
ACCESS = ROOT / 'device-access'


def ssh_arguments(host, *, tty=False):
    ipaddress.IPv4Address(host)
    args = ['ssh', '-t' if tty else '-T']
    if os.name != 'nt' or ACCESS.is_dir():
        args += ['-F', os.devnull, '-i', str(ACCESS / 'id_ed25519_board'),
                 '-o', 'IdentitiesOnly=yes', '-o', 'UserKnownHostsFile=' + str(ACCESS / 'known_hosts')]
    return args + ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                   '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2',
                   '-o', 'ServerAliveCountMax=2', 'root@' + host]


def process_running(pid):
    if os.name == 'nt':
        import psutil
        return psutil.pid_exists(pid)
    result = subprocess.run(['ps', '-p', str(pid), '-o', 'stat='],
                            capture_output=True, text=True)
    state = result.stdout.strip()
    return bool(state and not state.startswith('Z'))


def start_viewer(host):
    VIEWER.mkdir(parents=True, exist_ok=True)
    script = ROOT / 'local-joint-viewer/viewer.py'
    if os.name == 'nt':
        stop_file = VIEWER / (uuid.uuid4().hex + '.stop')
        args = [sys.executable, '-X', 'utf8', str(script), '--stop-file', str(stop_file)]
        options = {'creationflags': subprocess.CREATE_NO_WINDOW}
    else:
        args = [str(ROOT / 'local-joint-viewer/.venv/bin/python'), str(script)]
        options = {'start_new_session': True}
    with (VIEWER / 'viewer.log').open('ab') as log:
        process = subprocess.Popen(args + ['--host', host, '--soft-support'],
                                   cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, **options)
    (VIEWER / 'viewer.pid').write_text(str(process.pid) + '\n')
    return process.pid


def stop_viewer(host):
    try:
        pid = int((VIEWER / 'viewer.pid').read_text())
    except (FileNotFoundError, ValueError):
        return False
    if os.name == 'nt':
        import psutil
        try:
            process = psutil.Process(pid)
            command = process.cmdline()
        except psutil.NoSuchProcess:
            return False
    else:
        result = subprocess.run(['ps', '-p', str(pid), '-o', 'command='], capture_output=True, text=True)
        if not result.stdout.strip():
            return False
        command = shlex.split(result.stdout)
    if (str(ROOT / 'local-joint-viewer/viewer.py') not in command or '--usb-serial' in command or
            '--host' not in command or command[command.index('--host') + 1] != host):
        raise RuntimeError('关节采集进程身份不匹配，未停止它')
    if os.name == 'nt':
        if '--stop-file' not in command:
            raise RuntimeError('关节采集未通过工作台启动，无法确认正常交接')
        stop_file = Path(command[command.index('--stop-file') + 1])
        if stop_file.parent != VIEWER or not re.fullmatch(r'[a-f0-9]{32}\.stop', stop_file.name):
            raise RuntimeError('关节采集的停止文件不匹配')
        stop_file.touch()
        try:
            process.wait(timeout=10)
        except psutil.TimeoutExpired as error:
            raise RuntimeError('关节采集未能释放串口，请稍后再试') from error
        finally:
            stop_file.unlink(missing_ok=True)
    else:
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 20
        while process_running(pid):
            if time.monotonic() >= deadline:
                raise RuntimeError('关节采集未能释放串口，请稍后再试')
            time.sleep(.1)
    return True


@contextlib.contextmanager
def pause_ssh_viewer(host):
    ssh_arguments(host)
    if not viewer_torque_off():
        raise RuntimeError('A fresh, fully torque-off viewer is required')
    if not stop_viewer(host):
        raise RuntimeError('关节采集进程已退出，请先恢复实时读数')
    resumption = {'allowed': True}
    try:
        yield resumption
    finally:
        if resumption['allowed']:
            start_viewer(host)


def viewer_torque_off(*, newer_than=0):
    try:
        state = json.loads((VIEWER / 'state.json').read_text(encoding='utf-8'))
        now = time.time()
        return (state['unix_time'] >= newer_than and 0 <= now - state['unix_time'] < 2 and
                state['software_zero_data_ready'] and len(state['torque']) == 15 and
                not any(state['torque'].values()) and not state['console'].get('soft_active') and
                not state['console']['active_id'])
    except (OSError, ValueError, KeyError, TypeError):
        return False


class SshConsole:
    uses_tty = False

    def __init__(self, host):
        self.argv = ssh_arguments(host)
        self.process = None

    def login(self):
        # Authentication happens when the single reviewed remote command starts.
        pass

    @property
    def fd(self):
        return self.process.stdout.fileno()

    def send(self, data):
        if self.process is None:
            self.process = subprocess.Popen(
                self.argv + [data.rstrip('\r\n')], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
            return
        payload = memoryview(data.encode() if isinstance(data, str) else data)
        deadline = time.monotonic() + 1
        while payload:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.process.stdin], [], remaining)[1]:
                raise TimeoutError('SSH control input stalled')
            count = os.write(self.process.stdin.fileno(), payload)
            if count <= 0:
                raise OSError('SSH control input closed')
            payload = payload[count:]

    def receive(self):
        data = os.read(self.fd, 65536)
        if not data:
            raise OSError('SSH process closed before the completion marker')
        return data

    def interrupt(self):
        # EOF lets the remote lease execute its normal torque-off cleanup.
        # A control-C byte in a non-TTY SSH pipe would not deliver SIGINT.
        if self.process and not self.process.stdin.closed:
            self.process.stdin.close()

    def until(self, pattern, timeout):
        text = ''
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], .1)[0]:
                text += self.receive().decode(errors='replace')
                if re.search(pattern, text):
                    return text
        raise TimeoutError('SSH cleanup completion was not confirmed')

    def close(self):
        if not self.process:
            return
        self.interrupt()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            # The panel keeps its recovery block when a verified report is absent.
            self.process.terminate()
        self.process.stdout.close()
