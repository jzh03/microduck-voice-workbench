import json
import math
from pathlib import Path
import shlex
import statistics
import subprocess
import time

from panel_recovery import control_lock
from ssh_trial_transport import pause_ssh_viewer, ssh_arguments

SOURCE = r'''
import sys,json,time,struct
from pathlib import Path
sys.path.insert(0,'/root/microduck-imu-update/walking-runtime')
from hardware_bridge import WireBus,IDS
from policy_coordinates import TrunkImu,CONTRACT
bus=WireBus('/dev/ttyS2',baud=1000000,timeout=.025,live=False)
def off():
 values,_=bus.sync_read(list(IDS),64,1)
 assert set(values)==set(IDS) and all(not e&0x7f and d==b'\x00' for e,d in values.values()),'未确认15个关节全部卸力'
try:
 off();imu=TrunkImu(mount=CONTRACT['robot_imu_mount_wxyz']);rows=[];counter=None;changed=time.monotonic()
 for i in range(70):
  tick=time.monotonic()
  if i%5==0:off()
  e,data,_=bus.read(200,124,20);assert e is not None and not e&0x7f and len(data)==20,'IMU读取不完整'
  current,status=struct.unpack_from('<HB',data,12);assert status&3==3 and not status&4,'IMU尚未就绪'
  if counter!=current:counter=current;changed=tick
  assert tick-changed<.1,'IMU数据停止更新'
  frame=imu.decode(data[:12])
  if i>=40:
   assert frame['ready'];rows.append(frame)
  time.sleep(max(0,.05-(time.monotonic()-tick)))
 off();assert bus.write_packets==0
 print(json.dumps({'samples':rows,'register_write_packets':0}))
finally:bus.close()
'''


def capture(output, host):
    lock = control_lock(host)
    try:
        with pause_ssh_viewer(host):
            result = subprocess.run(ssh_arguments(host) + ['python3 -c ' + shlex.quote(SOURCE)],
                                    capture_output=True, text=True, timeout=12)
            if result.returncode:
                raise RuntimeError('IMU 采样未完成；请检查连接并确认全部卸力')
            data = json.loads(result.stdout)
            frames = data['samples']
            gravity = [statistics.mean(row['gravity'][i] for row in frames) for i in range(3)]
            norm = math.sqrt(sum(v*v for v in gravity))
            x, y, z = (v / norm for v in gravity)
            summary = {
                'unix_time': time.time(), 'pitch_deg': math.degrees(math.atan2(x, math.hypot(y, z))),
                'roll_deg': math.degrees(math.atan2(-y, -z)),
                'tilt_deg': math.degrees(math.acos(max(-1, min(1, -z)))),
                'samples': len(frames), 'register_write_packets': 0,
            }
            path = Path(output) / ('imu-read-' + str(time.time_ns()) + '.json')
            path.write_text(json.dumps({'summary': summary, 'capture': data}, indent=2) + '\n')
            return summary
    finally:
        lock.close()
