import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import urllib.request
import webbrowser

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'outputs/workbench'
CONFIG = Path(__file__).with_name('workbench-config.json')


def get_json(url):
    with urllib.request.urlopen(url, timeout=.6) as response:
        return json.load(response)


def status():
    with urllib.request.urlopen('http://127.0.0.1:8769/', timeout=1) as response:
        token = re.search(r"const token='([^']+)'", response.read().decode()).group(1)
    request = urllib.request.Request('http://127.0.0.1:8769/status', data=b'{}',
                                    headers={'Content-Type': 'application/json', 'X-Control-Token': token}, method='POST')
    with urllib.request.urlopen(request, timeout=1) as response:
        return json.load(response)


def spawn(name, argv):
    options = ({'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt'
               else {'start_new_session': True})
    with (OUT / (name + '.log')).open('ab') as log:
        process = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, **options)
    return process.pid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-open', action='store_true')
    args = parser.parse_args()
    if os.name == 'nt':
        os.environ['PYTHONUTF8'] = '1'
    OUT.mkdir(parents=True, exist_ok=True)
    url = 'http://127.0.0.1:8770'
    frontend_pid = None
    try:
        health = get_json(url + '/healthz')
        if health.get('app') != 'microduck-workbench' or health.get('simulation'):
            raise RuntimeError('8770 端口被其他服务占用')
        frontend_pid = health['pid']
    except (OSError, ValueError):
        pass
    if not CONFIG.is_file():
        raise RuntimeError('请先运行配置实机.py；离线体验请使用 --simulation')
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    for name in ('certificate', 'reference', 'manual_reference', 'deployment_report'):
        config[name] = str((ROOT / config[name]).resolve())
    argv = [sys.executable, '-u', str(ROOT / 'walking-runtime/manual_stance_panel.py'),
            '--host', config['host'], '--mode', 'live', '--initial-mode', 'walk',
            '--output-dir', str(OUT / 'controller'), '--certificate', config['certificate'],
            '--reference', config['reference'], '--manual-reference', config['manual_reference'],
            '--deployment-report', config['deployment_report'], '--supported-direct-handoff']
    manual = Path(config['manual_reference'])
    if not manual.exists():
        manual.write_bytes(Path(config['reference']).read_bytes())
    backend_pid = None
    current = {}
    try:
        current = status()
        if os.name == 'nt':
            import psutil
            controllers = [process for process in psutil.process_iter(['cmdline', 'ppid'])
                           if argv[2] in (process.info['cmdline'] or [])]
            # The Windows venv redirector and its child share a command line.
            parents = {process.info['ppid'] for process in controllers}
            matches = [(process.pid, process.info['cmdline']) for process in controllers
                       if process.pid not in parents]
        else:
            processes = subprocess.check_output(['ps', '-axo', 'pid=,command='], text=True)
            matches = []
            for line in processes.splitlines():
                parts = line.strip().split(None, 1)
                if len(parts) == 2 and str(ROOT / 'walking-runtime/manual_stance_panel.py') in parts[1]:
                    matches.append((int(parts[0]), parts[1]))
        if len(matches) != 1:
            raise RuntimeError('无法唯一识别已有控制服务；请先停止旧控制页')
        backend_pid, command = matches[0]
        if config['manual_reference'] not in command:
            if current.get('running'):
                raise RuntimeError('旧控制器正在运行；请先托稳并停止，再打开工作台')
            if os.name == 'nt':
                raise RuntimeError('已有控制服务使用另一套站姿配置，请关闭旧服务后再启动工作台')
            os.kill(backend_pid, signal.SIGTERM)
            for _ in range(50):
                result = subprocess.run(['ps', '-p', str(backend_pid), '-o', 'stat='], capture_output=True, text=True)
                if not result.stdout.strip() or result.stdout.strip().startswith('Z'):
                    break
                time.sleep(.1)
            backend_pid = None
    except (OSError, ValueError):
        backend_pid = None
    from ssh_trial_transport import process_running, start_viewer
    from panel_recovery import control_lock
    viewer_file = ROOT / 'outputs/live-joint-viewer-20261017/viewer.pid'
    viewer_pid = None
    try:
        viewer_pid = int(viewer_file.read_text())
        viewer_alive = process_running(viewer_pid)
    except (OSError, ValueError):
        viewer_alive = False
    if not viewer_alive and not (current.get('running') or current.get('recovering')):
        try:
            guard = control_lock(config['host'])
        except ValueError:
            # A voice session may have temporarily paused the viewer.
            viewer_pid = None
        else:
            with guard:
                viewer_pid = start_viewer(config['host'])
    if not backend_pid:
        backend_pid = spawn('controller', argv)
    for _ in range(50):
        try:
            status()
            break
        except (OSError, ValueError):
            time.sleep(.1)
    else:
        raise RuntimeError('控制服务未启动，请查看 outputs/workbench/controller.log')
    if frontend_pid is None:
        frontend_pid = spawn('workbench', [sys.executable, '-u', str(ROOT / 'walking-runtime/workbench_server.py'), '--host', config['host']])
    for _ in range(50):
        try:
            if get_json(url + '/healthz').get('app') == 'microduck-workbench':
                break
        except (OSError, ValueError):
            time.sleep(.1)
    else:
        raise RuntimeError('工作台未启动，请查看 outputs/workbench/workbench.log')
    (OUT / 'processes.json').write_text(json.dumps({'panel': {'pid': backend_pid, 'argv': argv},
                                                  'viewer': {'pid': viewer_pid},
                                                  'frontend': {'pid': frontend_pid}}, indent=2))
    if not args.no_open:
        if os.name == 'nt':
            webbrowser.open_new_tab(url)
        else:
            subprocess.run(['open', url], check=False)
    print('工作台已就绪：' + url + '（不会自动加力）')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('工作台启动失败：' + str(error), file=sys.stderr)
        raise SystemExit(1)
