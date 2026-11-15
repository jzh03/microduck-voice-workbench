"""Hand the motor bus from the workbench viewer to the board's voice session."""
import json
from pathlib import Path
import subprocess
import sys
import time

from launch_workbench import CONFIG
from panel_recovery import control_lock
from ssh_trial_transport import pause_ssh_viewer, ssh_arguments, viewer_torque_off


def wait_for_off(*, newer_than=0):
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        if viewer_torque_off(newer_than=newer_than):
            return
        time.sleep(.2)
    raise RuntimeError('未读回 15 路卸力状态，请在工作台确认关节采集')


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('请在可用 Ctrl+C 退出的终端启动')
    host = json.loads(CONFIG.read_text(encoding='utf-8'))['host']
    ssh = ssh_arguments(host, tty=True) + ['/root/microduck-voice/voice.sh', 'standup-walk']
    launcher = Path(__file__).with_name('launch_workbench.py')
    subprocess.run([sys.executable, '-B', '-X', 'utf8', str(launcher), '--no-open'], check=True)
    with control_lock(host):
        wait_for_off()
        returned = time.time()
        try:
            with pause_ssh_viewer(host):
                print('已将串口交给语音控制。按 Ctrl+C 结束并恢复工作台采集。', flush=True)
                try:
                    result = subprocess.run(ssh, check=False).returncode
                finally:
                    returned = time.time()
        except KeyboardInterrupt:
            result = 130
        wait_for_off(newer_than=returned)
    return 0 if result in (0, 130) else result


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'语音工作台未完成：{error}', file=sys.stderr)
        raise SystemExit(1)
