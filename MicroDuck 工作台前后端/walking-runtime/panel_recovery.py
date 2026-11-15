"""Bounded local recovery for both panels; never starts a powered session."""
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

from ssh_trial_transport import ROOT, VIEWER, ssh_arguments, start_viewer, stop_viewer


def control_lock(host):
    ssh_arguments(host)
    directory = ROOT / 'outputs' / 'control-locks'
    directory.mkdir(exist_ok=True)
    stream = (directory / (host + '.lock')).open('a+b')
    try:
        if os.name == 'nt':
            import msvcrt
            stream.seek(0)
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        stream.close()
        raise ValueError('无法取得控制锁，请检查是否已有其他控制会话') from error
    return stream


def wait_for_board(host, update, *, timeout=25.):
    """Retry read-only boot probes; motor recovery itself is never replayed."""
    deadline=time.monotonic()+timeout
    attempt=0
    while time.monotonic()<deadline:
        attempt+=1
        update(f'等待主板开机并连接 Wi-Fi…第 {attempt} 次检查，请保持托稳')
        try:
            result=subprocess.run(ssh_arguments(host)+['cat /proc/sys/kernel/random/boot_id'],
                                  capture_output=True,text=True,timeout=6)
            boot=result.stdout.strip()
            if result.returncode==0 and re.fullmatch(r'[a-f0-9]{8}(?:-[a-f0-9]{4}){3}-[a-f0-9]{12}',boot):
                return boot
            if any(word in result.stderr for word in ('Permission denied','Host key verification failed','REMOTE HOST IDENTIFICATION')):
                raise RuntimeError('SSH 身份验证未通过；请检查主板身份或访问密钥')
        except subprocess.TimeoutExpired:
            pass
        remaining=deadline-time.monotonic()
        if remaining>0:time.sleep(min(.3,remaining))
    raise RuntimeError('等待主板联网超时；请确认电源和 Wi-Fi，恢复后再点复位')


def recover_panel(host, identity, output, update):
    boot=wait_for_board(host,update)
    update('主板已连通，正在释放采集并确认卸力…')
    stop_viewer(host)
    source = (ROOT / 'walking-runtime/recover_motor_state.py').read_text()
    try:
        result = subprocess.run(ssh_arguments(host) + ['python3 -c ' + shlex.quote(source)],
                                input=json.dumps(identity), capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError('复位超时，状态尚未确认；连接恢复后可再点复位') from exc
    if result.returncode:
        raise RuntimeError('复位连接再次中断；请等主板联网后再点复位')
    try:
        report = json.loads(result.stdout)
    except ValueError as exc:
        raise RuntimeError('未收到完整复位报告，请稍后再点复位') from exc
    path = output / ('recovery-' + str(time.time_ns()) + '.json')
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    ids = set(map(str, json.loads((ROOT / 'walking-runtime/runtime-contract.json').read_text())['joint_ids']))
    if (report.get('verified') is not True or set(report.get('torque_after', {})) != ids or
            any(report['torque_after'].values()) or report.get('motion_commanded') is not False or
            report.get('cleanup', {}).get('restored') is not True):
        raise RuntimeError('复位未通过：' + report.get('error', '卸力或配置读回不完整'))
    if report.get('boot_id')!=boot:
        raise RuntimeError('复位期间主板又重启了；请确认供电稳定后再点复位')
    update('已确认全部卸力，正在恢复关节采集…')
    start_viewer(host)
    deadline = time.monotonic() + 9
    while time.monotonic() < deadline:
        try:
            state = json.loads((VIEWER / 'state.json').read_text())
            if (time.time() - state['unix_time'] < 2 and state.get('software_zero_data_ready') and
                    set(state.get('torque', {})) == ids and not any(state['torque'].values())):
                return report, str(path)
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(.15)
    raise RuntimeError('舵机已卸力，但关节采集尚未恢复；等连接恢复后可再点复位')
