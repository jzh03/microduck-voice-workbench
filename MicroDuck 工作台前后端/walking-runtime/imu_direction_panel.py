#!/usr/bin/env python3
"""Local guided IMU direction check. The remote collector cannot write registers."""
import argparse
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import select
import shlex
import subprocess
import threading
import time
import uuid

from imu_direction_validation import STAGES, evaluate
from ssh_trial_transport import ROOT, ssh_arguments, pause_ssh_viewer

COLLECTOR = r'''
import sys,json,time,select,threading,struct,hashlib
sys.path.insert(0,'/root/microduck-imu-update/walking-runtime')
from hardware_bridge import WireBus,LOCAL,IDS
from policy_coordinates import TrunkImu,CONTRACT
bus=WireBus('/dev/ttyS2',baud=1000000,timeout=.025,live=False)
stop=threading.Event()
def listen():
 while not stop.is_set():
  if select.select([sys.stdin],[],[],.1)[0] and not sys.stdin.buffer.read1(1024):stop.set()
threading.Thread(target=listen,daemon=True).start()
try:
 decoder=TrunkImu(mount=CONTRACT['robot_imu_mount_wxyz'])
 profile=hashlib.sha256((LOCAL/'software-zero-reference.json').read_bytes()).hexdigest()
 started=time.monotonic();last_torque=0.;counter=None;counter_time=started;read_retries=0
 while not stop.is_set() and time.monotonic()-started<600:
  tick=time.monotonic()
  if tick-last_torque>.25:
   motors,_=bus.sync_read(list(IDS),64,1)
   if set(motors)!=set(IDS) or any(e&0x7f or b!=b'\x00' for e,b in motors.values()):raise RuntimeError('15 个舵机全部卸力的回读未通过')
   last_torque=tick
  for attempt in range(2):
   error,block,_=bus.read(200,124,20)
   if error is not None and not error&0x7f and len(block)==20:break
   if attempt:raise RuntimeError('IMU read incomplete after one read-only retry')
   read_retries+=1
  current,status=struct.unpack_from('<HB',block,12)
  if status&3!=3 or status&4:raise RuntimeError('IMU is not ready')
  if current!=counter:counter=current;counter_time=tick
  elif tick-counter_time>.1:raise RuntimeError('IMU sample counter froze')
  frame=decoder.decode(block[:12])
  if frame['ready']:print(json.dumps({'t':tick,'imu':frame,'profile_sha256':profile,'writes':bus.write_packets,'read_retries':read_retries}),flush=True)
  time.sleep(max(0,.05-(time.monotonic()-tick)))
except BaseException as exc:print(json.dumps({'error':str(exc)}),flush=True)
finally:
 bus.close();print(json.dumps({'done':True,'writes':bus.write_packets}),flush=True)
'''

PAGE = '''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MicroDuck · 行走前方向核对</title><style>
body{font:17px/1.6 system-ui;background:#edf3f2;color:#183c42;margin:0;padding:28px}main{max-width:740px;margin:auto;background:white;padding:28px;border-radius:20px}h1{font-size:26px}button{display:block;width:100%;padding:15px;margin:12px 0;border:0;border-radius:9px;background:#087d77;color:white;font:600 17px system-ui}button:disabled{opacity:.4}#state{padding:16px;background:#e5f3ee;border-radius:10px}.muted{color:#526b70}li{margin:12px 0}#stop{background:#64777b}</style>
<main><div class="muted">MicroDuck · 行走准备</div><h1>核对机身运动方向</h1>
<p><strong>全部舵机保持卸力，这个页面不会驱动关节。</strong>用手托住躯干和腿部，小幅移动整个机身。前后左右以小鸭子自身为准。</p>
<div id="state">正在连接只读传感器…</div><p id="live"></p><p id="measure"></p>
<ol><li>大致扶正到倾斜 25° 内，不用完全竖直，也不用绝对静止。正常手抖可以。<button data-stage="upright">1 · 记录起始姿态（3 秒）</button></li>
<li>回到大致扶正的姿态，暂停转动一秒。点击后，缓慢向前倾 10–20°，停住直到计时结束。<button data-stage="forward">2 · 记录前倾（6 秒）</button></li>
<li>回到大致扶正的姿态，暂停转动一秒。点击后，缓慢向小鸭子右侧倾 10–20°（右侧降低），停住。<button data-stage="right">3 · 记录向右倾（6 秒）</button></li>
<li>回到大致扶正的姿态，暂停转动一秒。点击后，整体向左转约 20°，停住。<button data-stage="left_turn">4 · 记录向左转（6 秒）</button></li></ol>
<p id="results"></p><button id="restart" disabled>重新连接只读采集</button><button id="stop">结束只读核对并恢复关节页面</button>
<p class="muted">每一步通过后再做下一步；这不会修改保存的关节零位。</p></main><script>
const token='__TOKEN__';async function req(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify(body)});return r.json()}
document.querySelectorAll('[data-stage]').forEach(b=>b.onclick=async()=>{b.disabled=true;await req('/capture',{stage:b.dataset.stage})});document.querySelector('#stop').onclick=()=>req('/stop');document.querySelector('#restart').onclick=()=>req('/restart');
setInterval(async()=>{try{let s=await req('/status');document.querySelector('#state').textContent=s.message;document.querySelector('#measure').textContent=s.remaining_s!==null?'本步骤剩余 '+s.remaining_s.toFixed(1)+' 秒':'';
document.querySelector('#live').textContent=typeof s.tilt_deg==='number'?'实时倾斜 '+s.tilt_deg.toFixed(1)+'° · 起始姿态允许 25° 内，正常手抖可以':'';
document.querySelectorAll('[data-stage]').forEach(b=>b.disabled=!s.can_capture||b.dataset.stage!==s.next_stage);document.querySelector('#restart').disabled=!s.can_restart;document.querySelector('#results').textContent=s.completed.map(x=>({upright:'起始姿态',forward:'前倾',right:'右倾',left_turn:'左转'})[x]+' ✓').join('　');}catch(e){document.querySelector('#state').textContent='连接中断；本页仅采集，不会加力';}},200);
</script></html>'''


class Check:
    def __init__(self, host, output):
        self.host, self.output = host, output
        self.token = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.recent = deque(maxlen=30)
        self.reports = {}
        self.capture = None
        self.running = False
        self.starting = False
        self.message = '正在连接只读传感器…'
        self.profile_sha = None
        self.last_received = 0.
        # Re-evaluate real samples already captured by the operator. This is
        # reference-pose reuse, never a substitute for the three labelled motions.
        profile_path=ROOT/'local-joint-viewer/software-zero-reference.json'
        contract_path=ROOT/'walking-runtime/runtime-contract.json'
        expected_profile=hashlib.sha256(profile_path.read_bytes()).hexdigest()
        for recording in sorted(output.glob('upright-*.json'), reverse=True):
            try:
                if contract_path.stat().st_mtime>recording.stat().st_mtime:continue
                capture=json.loads(recording.read_text())['capture']
                if any(f.get('profile_sha256')!=expected_profile or f.get('writes')!=0
                       for f in capture['baseline']+capture['frames']):continue
                result=evaluate('upright',capture['baseline'],capture['frames'])
                if result['passed']:
                    self.reports['upright']={**result,'recording':recording.name,
                        'reused_original_recording':True,'reevaluated_unix':time.time(),
                        'profile_sha256':expected_profile,
                        'contract_sha256':hashlib.sha256(contract_path.read_bytes()).hexdigest()}
                    (output/'reference-reused.json').write_text(json.dumps(self.reports['upright'],indent=2)+'\n')
                    break
            except (OSError,ValueError,KeyError):continue

    def status(self):
        with self.lock:
            todo = [s for s in STAGES if s not in self.reports]
            fresh=self.running and self.recent and time.monotonic()-self.last_received<.3
            tilt=math.degrees(math.acos(max(-1.,min(1.,-self.recent[-1]['imu']['gravity'][2])))) if fresh else None
            return {'message': self.message, 'completed': list(self.reports),
                    'tilt_deg':tilt,
                    'next_stage': todo[0] if todo else None,
                    'can_restart': not self.running and not self.starting and bool(todo),
                    'can_capture': self.running and len(self.recent)>=10 and self.capture is None and time.monotonic()-self.last_received < .3,
                    'remaining_s': max(0,self.capture['end']-time.monotonic()) if self.capture else None}

    def start(self):
        with self.lock:
            if self.running or self.starting:return
            self.starting=True;self.stop.clear();self.recent.clear();self.capture=None
            self.message='正在连接只读传感器…'
        threading.Thread(target=self.run,daemon=True).start()

    def begin(self, stage):
        with self.lock:
            state = self.status()
            if not state['can_capture'] or stage != state['next_stage']:
                return
            self.capture = {'stage': stage, 'baseline': list(self.recent)[-10:],
                            'frames': [], 'end': time.monotonic()+(3 if stage=='upright' else 6)}
            self.message = '正在记录；请按本步骤动作，结束前保持末端姿态'

    def record(self, frame):
        with self.lock:
            self.profile_sha = frame['profile_sha256']
            self.recent.append(frame)
            self.last_received = time.monotonic()
            if not self.capture:
                return
            self.capture['frames'].append(frame)
            if time.monotonic() < self.capture['end']:
                return
            capture, self.capture = self.capture, None
            try:
                result = evaluate(capture['stage'], capture['baseline'], capture['frames'])
            except ValueError as exc:
                result = {'stage': capture['stage'], 'passed': False, 'reason': str(exc)}
            filename = f"{capture['stage']}-{time.time_ns()}.json"
            (self.output/filename).write_text(json.dumps({'result':result, 'capture':capture},indent=2)+'\n')
            self.message = result['reason']
            if result['passed']:
                self.reports[capture['stage']] = {**result, 'recording': filename}
            if len(self.reports) == len(STAGES):
                contract = ROOT/'walking-runtime/runtime-contract.json'
                evidence = {'verified':True, 'recorded_unix':time.time(),
                            'profile_sha256':self.profile_sha,
                            'contract_sha256':hashlib.sha256(contract.read_bytes()).hexdigest(),
                            'stages':self.reports, 'register_write_packets':0,
                            'scope':'Operator-labelled trunk axis signs; not a balance or walking test'}
                temporary=self.output/'imu-direction-verified.json.tmp'
                temporary.write_text(json.dumps(evidence,indent=2)+'\n')
                temporary.replace(self.output/'imu-direction-verified.json')
                self.message = '四个方向已通过，正在恢复关节页面'
                self.stop.set()

    def run(self):
        try:
            with pause_ssh_viewer(self.host) as resume:
                resume['allowed'] = False
                process = subprocess.Popen(ssh_arguments(self.host)+['python3 -u -c '+shlex.quote(COLLECTOR)],
                                           stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,bufsize=0)
                incoming, done = b'', False
                self.running = True
                self.message = ('你刚才的起始姿态记录已通过，无需重做第 1 步；可以开始前倾'
                                if 'upright' in self.reports else '只读采集已连接；正常手抖可以，请记录起始姿态')
                try:
                    deadline = time.monotonic()+605
                    collector_error=None
                    while time.monotonic()<deadline:
                        if self.stop.is_set() and not process.stdin.closed:
                            process.stdin.close()
                        if not select.select([process.stdout],[],[],.1)[0]:
                            continue
                        chunk = process.stdout.read(65536)
                        if not chunk:
                            break
                        incoming += chunk
                        while b'\n' in incoming:
                            line,incoming=incoming.split(b'\n',1)
                            try: frame=json.loads(line)
                            except ValueError: continue
                            if frame.get('done'):
                                done = frame['writes']==0
                            elif 'error' in frame:
                                collector_error=frame['error'];self.stop.set()
                            elif 'imu' in frame:
                                assert frame['writes']==0
                                self.record(frame)
                        if done:break
                    if not process.stdin.closed:process.stdin.close()
                    process.wait(timeout=5)
                    resume['allowed'] = done
                    if not done:raise RuntimeError('只读采集退出未确认，未重启关节页面')
                    if collector_error:raise RuntimeError(collector_error)
                finally:
                    self.running = False
                    if process.poll() is None:process.terminate()
                    process.stdout.close()
            self.message = '方向核对全部通过，关节页面已恢复' if len(self.reports)==4 else '只读核对已结束，关节页面已恢复'
        except Exception as exc:
            self.message = '核对停止：'+str(exc)
            self.running = False
        finally:
            self.starting=False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True)
    parser.add_argument('--port',type=int,default=8768)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    check=Check(args.host,args.output_dir)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def respond(self,data,kind='application/json',status=200):
            body=data.encode() if isinstance(data,str) else json.dumps(data,ensure_ascii=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind+'; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def do_GET(self):
            self.respond(PAGE.replace('__TOKEN__',check.token),'text/html') if self.path=='/' else self.respond({},status=404)
        def do_POST(self):
            if self.headers.get('X-Control-Token')!=check.token:return self.respond({},status=403)
            size=int(self.headers.get('Content-Length','0'))
            if not 0<=size<=1000:return self.respond({},status=413)
            try:body=json.loads(self.rfile.read(size) or b'{}')
            except ValueError:return self.respond({},status=400)
            if self.path=='/capture':check.begin(body.get('stage'))
            elif self.path=='/restart':check.start()
            elif self.path=='/stop':check.stop.set()
            elif self.path!='/status':return self.respond({},status=404)
            self.respond(check.status())
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    check.start()
    print(f'只读方向核对：http://127.0.0.1:{args.port}/',flush=True)
    try:server.serve_forever()
    finally:check.stop.set()


if __name__=='__main__':main()
