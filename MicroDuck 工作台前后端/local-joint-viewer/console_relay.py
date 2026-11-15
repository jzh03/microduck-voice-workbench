"""SSH transport for console_worker; no queued commands survive reconnection."""
import json
from collections import deque
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
from frame_log import FrameLog

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'outputs/live-joint-viewer-20261017'
ACCESS=ROOT/'device-access'


class ControlRelay:
    def __init__(self,host,usb_serial=None):
        self.host=host;self.usb_serial=usb_serial;self.lock=threading.RLock();self.write_lock=threading.Lock()
        self.stopped=threading.Event();self.process=None;self.frame=None;self.received=0
        self.arrivals=deque(maxlen=80);self.message='正在连接 Radxa'
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def run(self):
        args=['ssh','-F','/dev/null','-i',str(ACCESS/'id_ed25519_board'),'-o','IdentitiesOnly=yes',
              '-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','UserKnownHostsFile='+str(ACCESS/'known_hosts'),
              '-o','ConnectTimeout=5','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2',
              'root@'+self.host,'cd /root/microduck-imu-update && exec python3 -u console_worker.py']
        if os.name == 'nt' and not self.usb_serial:
            sys.path.insert(0, str(ROOT / 'walking-runtime'))
            from ssh_trial_transport import ssh_arguments
            args = ssh_arguments(self.host) + [args[-1]]
        if self.usb_serial:
            args=[sys.executable,str(Path(__file__).with_name('usb_console_transport.py')),self.usb_serial]
        with (OUT/'console-ssh.log').open('a',buffering=1,encoding='utf-8') as errors,FrameLog(OUT/'encoder-frames.jsonl') as frames:
            while not self.stopped.is_set():
                proc=None
                try:
                    options = {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {}
                    proc=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=errors,
                                          encoding='utf-8',errors='replace',bufsize=1,**options)
                    with self.lock:
                        self.process=proc;self.frame=None;self.received=0;self.arrivals.clear()
                    for line in proc.stdout:
                        if self.stopped.is_set():break
                        try:frame=json.loads(line)
                        except json.JSONDecodeError:continue
                        if frame.get('type')!='frame':continue
                        with self.lock:
                            self.frame=frame;self.received=time.monotonic();self.arrivals.append(self.received);self.message='USB 数据线已连接' if self.usb_serial else '已连接'
                        frames.write(json.dumps({'received_unix':time.time(),**frame})+'\n')
                except Exception as exc:
                    errors.write(str(exc)+'\n')
                finally:
                    if proc:
                        try:proc.stdin.close()
                        except Exception:pass
                        try:proc.wait(timeout=12 if self.usb_serial else 2)
                        except subprocess.TimeoutExpired:proc.terminate()
                    with self.lock:self.process=None;self.message='连接中断，正在重连'
                self.stopped.wait(2)

    def send(self,action,**values):
        with self.lock:
            proc=self.process;frame=self.frame
            max_age=8. if action in ('heartbeat','off') else .5
            if not proc or proc.poll() is not None or not frame or time.monotonic()-self.received>max_age:
                raise RuntimeError('实物连接未就绪')
            command={'action':action,'request_id':uuid.uuid4().hex,
                     'generation':frame['control']['generation'],**values}
        with self.write_lock:
            proc.stdin.write(json.dumps(command)+'\n');proc.stdin.flush()
        return command['request_id']

    def snapshot(self):
        with self.lock:
            arrivals=list(self.arrivals)
            span=arrivals[-1]-arrivals[0] if len(arrivals)>1 else 0
            hz=(len(arrivals)-1)/span if span>0 else 0
            return self.frame,self.received,hz,self.message

    def stop(self):
        try:self.send('off')
        except Exception:pass
        self.stopped.set()
        with self.lock:proc=self.process
        if proc:
            try:proc.stdin.close()
            except Exception:pass
        self.thread.join(timeout=13 if self.usb_serial else 3)
        if proc and proc.poll() is None:proc.terminate()
