#!/usr/bin/env python3
"""Radxa console: one serial owner, explicit single-joint sessions, JSON stdin/out.

Starting this process only reads the bus. All motion requires an arm command,
profile-bound official model limits, and an expiring controller heartbeat.
"""
from __future__ import annotations

import datetime
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import uuid

from joint_calibration import JointCalibration
from joint_limits import JointLimits
from support_power import support_health
from soft_follow import SoftFollower

IDS = (10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
FIELDS = {'mode':(11,1,False),'pwm_limit':(36,2,False),'velocity_i':(76,2,False),'velocity_p':(78,2,False),
          'd':(80,2,False),'i':(82,2,False),'p':(84,2,False),'ff2':(88,2,False),
          'ff1':(90,2,False),'goal_pwm':(100,2,True),'goal_current':(102,2,True),
          'accel':(108,4,False),'velocity':(112,4,False)}
WIDTHS = {64:1,98:1,116:4,**{v[0]:v[1] for v in FIELDS.values()}}
SOFT_IDS = tuple(i for i in IDS if i != 34)
SOFT_CURRENT_MA = 120
SOFT_PWM = 120
HEARTBEAT_TIMEOUT_S = .5
MANUAL_PROFILE_VELOCITY = 4
MANUAL_SPEED_DEG_S = MANUAL_PROFILE_VELOCITY * .229 * 6


def soft_current(value):
    if type(value) not in (int,float) or not math.isfinite(value) or int(value)!=value or not 80<=value<=200:
        raise ValueError('支撑力度只允许 80–200 mA')
    return int(value)


def window_for(anchor,interval):
    low,high=map(float,interval)
    if not all(math.isfinite(v) for v in (low,high,anchor)) or high-low<1:
        raise ValueError('活动范围无效，请重新记录')
    if not low<=anchor<=high:raise ValueError('当前位置超出模型关节范围，请先核对零位和姿态')
    return low,high


def cycle_goals(anchor,window,amplitude,cycles):
    lo,hi=window
    if not math.isfinite(amplitude) or not .5<=amplitude<=hi-lo or type(cycles) is not int or not 1<=cycles<=3:
        raise ValueError('往复幅度必须处于当前关节范围内，次数为 1–3 次')
    positive=min(amplitude,hi-anchor)
    negative=min(amplitude,anchor-lo)
    # Prefer a full excursion; near a boundary use the available direction.
    excursion=positive if positive>=negative else -negative
    if abs(excursion)<.5:raise ValueError('已记录范围不足以往复，请先把关节放回中间')
    return [v for _ in range(cycles) for v in (anchor+excursion,anchor)]


class Device:
    def __init__(self,bus,sid):
        if sid not in IDS:raise ValueError('未知关节 ID')
        self.bus,self.sid=bus,sid
        self.writes=[]

    def read(self,addr,n,signed=False,sid=None):
        sid=self.sid if sid is None else sid
        err,data,_=self.bus.read(sid,addr,n)
        if err is None or err&0x7f or len(data)!=n:
            raise RuntimeError(f'ID {sid} 读取失败（地址 {addr}）')
        return int.from_bytes(data,'little',signed=signed)

    def write(self,addr,n,value,signed=False,*,prearm=False):
        if WIDTHS.get(addr)!=n:raise ValueError('不允许写入此寄存器')
        if (addr in (11,36) or prearm) and self.read(64,1)!=0:
            raise RuntimeError('配置前必须关闭力矩')
        if prearm and addr!=116:raise ValueError('启动校验仅适用于位置目标')
        payload=int(value).to_bytes(n,'little',signed=signed)
        entry={'id':self.sid,'address':addr,'value':value};self.writes.append(entry)
        err,_,_=self.bus.write(self.sid,addr,payload)
        if err is None or err&0x7f:raise RuntimeError(f'ID {self.sid} 写入失败（地址 {addr}）')
        if addr==11:time.sleep(.03)
        actual=self.read(addr,n,signed);entry['readback']=actual
        if prearm:
            if abs(actual-value)>2 or abs(actual-self.read(132,4,True))>2:
                raise RuntimeError('启动目标与当前位置不一致')
        elif actual!=value:raise RuntimeError(f'ID {self.sid} 回读不一致（地址 {addr}）')

    def release(self,original):
        off=False;errors=[]
        for _ in range(3):
            try:self.write(64,1,0)
            except Exception:pass
            try:off=self.read(64,1)==0
            except Exception:off=False
            if off:break
        if not off:return {'torque_off':False,'restored':False,'errors':['关闭力矩未确认，保留低输出参数']}
        # Goal Position follows Present Position with torque disabled. Only the
        # saved operating configuration is restored, never a stale motion goal.
        for name,spec in FIELDS.items():
            try:
                if self.read(*spec)!=original[name]:self.write(spec[0],spec[1],original[name],spec[2])
            except Exception as exc:errors.append(str(exc))
        after={}
        for name,spec in FIELDS.items():
            try:
                after[name]=self.read(*spec)
                if after[name]!=original[name]:errors.append('恢复核验失败：'+name)
            except Exception as exc:errors.append(str(exc))
        return {'torque_off':off,'restored':not errors,'errors':errors,'configuration_after':after}


class ConsoleEngine:
    def __init__(self,bus,profile_bytes,root,clock=time.monotonic):
        self.bus,self.root,self.clock=bus,Path(root),clock
        self.sha=hashlib.sha256(profile_bytes).hexdigest()
        self.calibration=JointCalibration(json.loads(profile_bytes))
        self.limits=JointLimits(self.root/'runtime-joint-limits.json',profile_sha256=self.sha,ids=IDS)
        self.ranges_path=self.root/'console-ranges.json'
        self.ranges={};self.pending=None;self.recording=None
        self.positions={};self.angles={};self.homing={};self.torque={};self.drive={};self.health={}
        self.metadata_time=-1000.;self.sample_time=-1000.
        self.session=None;self.soft=None;self.cancel_arm=lambda:False
        self.fault=None;self.last_cleanup=None;self.last_message='只读采集 · 尚未开启实机'
        self.generation=uuid.uuid4().hex;self.history=[];self.write_count=0;self.last_result=None
        journal=self.root/'console-soft-original.json'
        if journal.exists() and json.loads(journal.read_text()).get('active'):
            self.fault='上次支撑配置尚未确认恢复，请检查设备'
        if self.ranges_path.exists():
            saved=json.loads(self.ranges_path.read_text())
            if saved.get('profile_sha256')==self.sha:self.ranges=saved.get('joints',{})
        seed=self.root/'ankle-observed-range.json'
        if '14' not in self.ranges and seed.exists():
            saved=json.loads(seed.read_text())
            if saved.get('profile_sha256')==self.sha and saved.get('direction_confirmed_by_user'):
                self.ranges['14']={'degrees':saved['degrees'],'confirmed':True,'source':'previous user-confirmed right ankle check'}

    def sample(self):
        now=self.clock();reader=Device(self.bus,14)
        if now-self.metadata_time>=1:
            self.homing={};self.torque={};self.drive={}
            for sid in IDS:
                self.torque[sid]=reader.read(64,1,sid=sid)
                self.homing[sid]=reader.read(20,4,True,sid=sid)
                self.drive[sid]=reader.read(10,1,sid=sid)
                self.health[sid]={'voltage':reader.read(144,2,sid=sid)/10,
                    'temperature_c':reader.read(146,1,sid=sid),'hardware_error':reader.read(70,1,sid=sid),
                    'shutdown':reader.read(63,1,sid=sid)}
            self.metadata_time=self.clock()
        positions={}
        for sid in IDS:positions[sid]=reader.read(132,4,True,sid=sid)
        self.positions=positions
        self.angles=self.calibration.update({sid:v for sid,v in positions.items() if self.drive.get(sid)==0},self.homing)
        self.sample_time=self.clock()
        if self.recording:
            if any(self.torque.values()):raise RuntimeError('记录期间检测到力矩开启')
            record=self.recording;q=math.degrees(self.angles[record['id']])
            record['degrees']=[min(record['degrees'][0],q),max(record['degrees'][1],q)]
            record['samples']+=1
            if self.clock()-record['started']>60:self.finish_record()
        if self.session:self.monitor()
        if self.soft:self.monitor_soft()

    def finish_record(self):
        if not self.recording:raise ValueError('尚未开始记录')
        self.pending=self.recording;self.recording=None
        self.pending['record_id']=uuid.uuid4().hex
        self.last_message='范围已采集；核对模型方向后，点击保存'

    def arm(self,command):
        if self.session or self.soft:raise ValueError('请先停止当前实机控制')
        if self.fault:raise ValueError('上次恢复异常，必须检查设备后重启服务')
        sid=int(command['id']);token=command['token']
        if sid not in IDS or not isinstance(token,str) or len(token)<16:raise ValueError('无效的控制会话')
        if command.get('profile_sha256')!=self.sha:raise ValueError('零位文件不匹配')
        if command.get('limits_sha256')!=self.limits.sha:raise ValueError('界面与控制器的关节限位不一致')
        if self.recording:raise ValueError('请先结束范围记录')
        device=Device(self.bus,sid)
        if any(device.read(64,1,sid=i)!=0 for i in IDS):raise ValueError('其他关节力矩必须关闭')
        if (device.read(0,2)!=1200 or device.read(6,1)!=53 or device.read(10,1)!=0 or device.read(63,1)!=52):
            raise ValueError('舵机型号、固件或保护配置与已验证配置不符')
        if device.read(70,1)&~1 or device.read(146,1)>=55:raise ValueError('舵机故障或温度异常')
        if not 6.6<=device.read(144,2)/10<=8.2:raise ValueError('电池电压不在项目控制范围内')
        original={name:device.read(*spec) for name,spec in FIELDS.items()}
        if original['mode']!=3:raise ValueError('初始控制模式不是预期的模式 3')
        homing=device.read(20,4,True);present=device.read(132,4,True)
        angle=math.degrees(self.calibration.update({sid:present},{sid:homing})[sid])
        window=window_for(angle,self.limits.bounds(sid))
        self.session={'id':sid,'token':token,'device':device,'original':original,'homing':homing,
            'anchor':angle,'window':window,'target':angle,'started':self.clock(),'heartbeat':self.clock(),
            'phase':'arming','cycle':None,'last_goal':None,'tracking_started':None,'trace':[],
            'progress_position':angle,'progress_time':self.clock(),'slow_check_time':self.clock(),
            'tracking_deadline':None}
        # Persist the original configuration before the first write.
        (self.root/'console-last-original.json').write_text(json.dumps({'id':sid,'original':original,
            'homing_offset':homing,'profile_sha256':self.sha,'time':time.time()},indent=2)+'\n')
        try:
            device.write(11,1,4)
            for addr,n,value in ((100,2,50),(80,2,0),(82,2,0),(84,2,400),
                                 (88,2,0),(90,2,0),(108,4,1),(112,4,MANUAL_PROFILE_VELOCITY)):
                device.write(addr,n,value)
            present=device.read(132,4,True)
            q=math.degrees(self.calibration.update({sid:present},{sid:homing})[sid])
            if abs(q-angle)>.5:raise RuntimeError('开启前姿态发生变化，请重新操作')
            device.write(116,4,present,True,prearm=True)
            device.write(64,1,1)
            if abs(device.read(132,4,True)-present)>8:raise RuntimeError('开启力矩后位置编码发生跳变')
            self.session['phase']='armed';self.session['heartbeat']=self.clock()
            self.torque[sid]=1
            self.last_message=f'ID {sid} 已开启；模型范围 {window[0]:g}° ～ {window[1]:g}°，低速单关节控制'
        except Exception:
            self.release('开启失败，已尝试关闭力矩');raise

    def set_target(self,angle):
        s=self.session
        if not s:raise ValueError('请先开启当前关节的实机控制')
        angle=float(angle)
        if not math.isfinite(angle) or not s['window'][0]<=angle<=s['window'][1]:
            raise ValueError('目标超出本次活动范围')
        sid=s['id'];d=s['device'];present=d.read(132,4,True)
        q=self.calibration.update({sid:present},{sid:s['homing']})[sid]
        goal=self.calibration.goal_for_angle(sid,math.radians(angle),present_position=present,
            current_angle_rad=q,operating_mode=4,min_position=0,max_position=4095,
            # Allow encoder quantisation or small overshoot at a model boundary
            # while returning inward. The requested target was checked exactly above.
            joint_limits_rad=tuple(map(math.radians,(s['window'][0]-.5,s['window'][1]+.5))))
        if goal!=s['last_goal']:
            if s['tracking_started'] is None or abs(math.degrees(q)-s['target'])<=1:
                s['progress_position']=math.degrees(q);s['progress_time']=self.clock()
            d.write(116,4,goal,True);s['last_goal']=goal;s['tracking_started']=self.clock()
            s['tracking_deadline']=self.clock()+abs(angle-math.degrees(q))/MANUAL_SPEED_DEG_S+3.
        s['target']=angle

    def monitor(self):
        s=self.session;now=self.clock();d=s['device'];sid=s['id']
        if now-s['heartbeat']>HEARTBEAT_TIMEOUT_S:self.release('控制连接超时，已关闭力矩');return
        q=math.degrees(self.angles[sid]);current=d.read(126,2,True)
        if (q<s['window'][0]-.5 or q>s['window'][1]+.5 or
                abs(current)>180 or d.read(70,1)&~1 or d.read(64,1)!=1):
            self.release('检测到运动、电流或设备状态异常，已停止');return
        if now-s['slow_check_time']>=1:
            s['slow_check_time']=now
            if d.read(146,1)>=55 or not 6.6<=d.read(144,2)/10<=8.2:
                self.release('温度或电压超出手动调试范围，已停止');return
        s['trace'].append({'time':time.time(),'actual_deg':q,'target_deg':s['target'],'current_ma':current})
        if len(s['trace'])>500:s['trace'].pop(0)
        error=abs(q-s['target'])
        if error<=1 or abs(q-s['progress_position'])>=.2:
            s['progress_position']=q;s['progress_time']=now
        if s['cycle']:
            cycle=s['cycle'];cycle['settled']=cycle['settled']+1 if error<.8 else 0
            if cycle['settled']>=4:
                if not cycle['remaining']:self.release('往复完成，已关闭力矩');return
                target=cycle['remaining'].pop(0);cycle['settled']=0
                self.set_target(target)
        if error>1 and (now-s['progress_time']>2.5 or
                (s['tracking_deadline'] is not None and now>s['tracking_deadline'])):
            self.release('未能跟上目标，已停止；请检查支撑和阻力')

    def release(self,reason):
        if self.soft:self.release_soft(reason)
        s=self.session
        if not s:return
        self.session=None
        cleanup=s['device'].release(s['original'])
        self.last_cleanup=cleanup
        self.write_count+=len(s['device'].writes)
        if not cleanup['restored']:self.fault='恢复配置未完成，请检查设备';reason=self.fault
        self.last_message=reason;self.metadata_time=-1000
        stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        report={'id':s['id'],'reason':reason,'profile_sha256':self.sha,'original':s['original'],
                'limits_sha256':self.limits.sha,'limit_source':self.limits.public(),
                'anchor_degrees':s['anchor'],'window_degrees':s['window'],'cleanup':cleanup,
                'trace':s['trace'],'writes':s['device'].writes}
        (self.root/('console-session-'+stamp+'.json')).write_text(json.dumps(report,indent=2)+'\n')

    def arm_soft(self,command):
        if self.session or self.soft or self.recording:raise ValueError('请先停止当前控制或记录')
        if self.fault:raise ValueError(self.fault)
        token=command.get('token')
        if not isinstance(token,str) or len(token)<16:raise ValueError('无效的控制会话')
        if command.get('profile_sha256')!=self.sha or command.get('limits_sha256')!=self.limits.sha:
            raise ValueError('零位或关节范围不匹配')
        current_ma=soft_current(command.get('current_ma',SOFT_CURRENT_MA))
        devices={sid:Device(self.bus,sid) for sid in SOFT_IDS};original={};homing={}
        reader=devices[14]
        if any(reader.read(64,1,sid=i) for i in IDS):raise ValueError('开启支撑前必须关闭全部力矩')
        # Check every joint before the first configuration write. Bit 0 is expected
        # only for the verified MicroDuck direct-2S/Shutdown=52 operating profile.
        health={sid:{'voltage':d.read(144,2)/10,'temperature_c':d.read(146,1),
                     'hardware_error':d.read(70,1),'shutdown':d.read(63,1)}
                for sid,d in devices.items()}
        power=support_health(health)
        if not power['ok']:raise ValueError('；'.join(power['issues'][:3]))
        for sid,d in devices.items():
            if (d.read(0,2)!=1200 or d.read(6,1)!=53 or d.read(10,1)!=0 or d.read(63,1)!=52):
                raise ValueError(f'ID {sid} 配置不匹配')
            original[sid]={name:d.read(*spec) for name,spec in FIELDS.items()}
            if original[sid]['mode']!=3 or d.read(98,1)!=0:
                raise ValueError(f'ID {sid} 控制模式或看门狗状态异常')
            if d.read(38,2)<200 or original[sid]['pwm_limit']<SOFT_PWM:
                raise ValueError(f'ID {sid} 输出限值不匹配')
            homing[sid]=d.read(20,4,True)
            q=math.degrees(self.calibration.update({sid:d.read(132,4,True)},{sid:homing[sid]})[sid])
            window_for(q,self.limits.bounds(sid))
        journal={'profile_sha256':self.sha,'original':original,'homing_offsets':homing,
                 'ids':list(SOFT_IDS),'time':time.time(),'active':True}
        with (self.root/'console-soft-original.json').open('w') as f:
            json.dump(journal,f,indent=2);f.flush();os.fsync(f.fileno())
        self.soft={'token':token,'devices':devices,'original':original,'homing':homing,
                   'started':self.clock(),'heartbeat':self.clock(),'slow_check':self.clock(),
                   'targets':{},'followers':{},'currents':{},'current_limit_ma':current_ma,'trace':[]}
        try:
            for sid,d in devices.items():
                if self.cancel_arm():raise RuntimeError('支撑开启已取消')
                d.write(11,1,5)
                for addr,n,value in ((100,2,SOFT_PWM),(102,2,current_ma),
                    (80,2,0),(82,2,0),(84,2,80),(88,2,0),(90,2,0),(108,4,0),(112,4,0)):
                    d.write(addr,n,value)
            # Capture each fresh encoder immediately before that motor is enabled.
            for sid,d in devices.items():
                if self.cancel_arm():raise RuntimeError('支撑开启已取消')
                present=d.read(132,4,True)
                q=math.degrees(self.calibration.update({sid:present},{sid:homing[sid]})[sid])
                window_for(q,self.limits.bounds(sid))
                d.write(116,4,present,True,prearm=True)
                d.write(98,1,25)  # 500 ms bus-stall stop; software heartbeat also releases torque.
                d.write(64,1,1)
                if abs(d.read(132,4,True)-present)>8:raise RuntimeError(f'ID {sid} 开启时位置跳变')
                self.soft['targets'][sid]=q
                self.soft['followers'][sid]=SoftFollower(q,self.clock())
                self.torque[sid]=1
            self.soft['heartbeat']=self.clock()
            self.last_message=f'跟随支撑已开启 · 14 个关节各限流 {current_ma} mA；扶着手动调整，停手后保持新位置'
        except Exception:
            self.release_soft('支撑开启失败，已尝试释放');raise

    def capture_soft(self):
        s=self.soft
        if not s:raise ValueError('柔和支撑尚未开启')
        values={}
        for sid,d in s['devices'].items():
            present=d.read(132,4,True)
            q=math.degrees(self.calibration.update({sid:present},{sid:s['homing'][sid]})[sid])
            window_for(q,self.limits.bounds(sid));values[sid]=(present,q)
        for sid,(present,q) in values.items():
            s['devices'][sid].write(116,4,present,True);s['targets'][sid]=q
            s['followers'][sid]=SoftFollower(q,self.clock())
        self.last_message='已记住现在姿态 · 继续保持柔和支撑；空格释放'

    def set_soft_strength(self,value):
        current_ma=soft_current(value);s=self.soft
        if not s:raise ValueError('柔和支撑尚未开启')
        for d in s['devices'].values():d.write(102,2,current_ma)
        s['current_limit_ma']=current_ma
        self.last_message=f'支撑力度已调为每关节 {current_ma} mA；空格释放'

    def accept_heartbeat(self,command,received_at):
        """Record receipt independently of a potentially slow UART sampling pass.

        Called by the stdin reader; it performs no bus operations. A reference to
        an old session can only update that old dict, never a replacement session.
        """
        current=self.session or self.soft
        if (not current or command.get('generation')!=self.generation or
                command.get('token')!=current['token']):
            return False
        gap=max(0.,received_at-current['heartbeat'])
        current['max_heartbeat_gap_s']=max(current.get('max_heartbeat_gap_s',0.),gap)
        current['heartbeat']=received_at
        return True

    def monitor_soft(self):
        s=self.soft;now=self.clock()
        if now-s['heartbeat']>HEARTBEAT_TIMEOUT_S:
            self.release_soft('支撑连接超时，已关闭力矩');return
        for sid,d in s['devices'].items():
            lo,hi=self.limits.bounds(sid);q=math.degrees(self.angles[sid])
            current=d.read(126,2,True);s['currents'][sid]=current
            if (not lo-.5<=q<=hi+.5 or abs(current)>s['current_limit_ma']+40 or
                d.read(64,1)!=1 or d.read(70,1)&~1 or d.read(98,1)!=25):
                self.release_soft(f'ID {sid} 支撑状态异常，已释放全部关节');return
        if now-s['slow_check']>=1:
            s['slow_check']=now
            for sid,d in s['devices'].items():
                if d.read(63,1)!=52 or d.read(146,1)>=50 or not 6.6<=d.read(144,2)/10<=8.45:
                    self.release_soft(f'ID {sid} 保护配置、温度或电压超出范围，已释放');return
        for sid,d in s['devices'].items():
            q=self.angles[sid];lo,hi=self.limits.bounds(sid)
            target=s['followers'][sid].update(math.degrees(q),self.clock())
            if target is None:continue
            target=max(lo,min(hi,target))
            # No forward motion goal: the request stays within 0.6° of measured
            # position, or adopts that exact position when the hand stops.
            goal=self.calibration.goal_for_angle(sid,math.radians(target),
                present_position=self.positions[sid],current_angle_rad=q,
                operating_mode=5,min_position=0,max_position=4095,
                joint_limits_rad=tuple(map(math.radians,(lo-.5,hi+.5))))
            if abs(target-s['targets'][sid])>=360/4096:
                d.write(116,4,goal,True)
                s['targets'][sid]=target
        s['trace'].append({'time':time.time(),'currents_ma':s['currents'].copy(),
                           'angles_degrees':{i:math.degrees(self.angles[i]) for i in SOFT_IDS},
                           'targets_degrees':s['targets'].copy(),
                           'following_ids':[i for i,f in s['followers'].items() if f.dragging]})
        if len(s['trace'])>1000:s['trace'].pop(0)

    def release_soft(self,reason):
        s=self.soft
        if not s:return
        self.soft=None;results={}
        # Drop all torques before spending time restoring individual configurations.
        for d in s['devices'].values():
            try:d.write(64,1,0)
            except Exception:pass
        for sid,d in s['devices'].items():
            try:
                if d.read(64,1)==0:d.write(98,1,0)
                result=d.release(s['original'][sid])
                if d.read(98,1)!=0 or d.read(20,4,True)!=s['homing'][sid]:
                    result['restored']=False;result['errors'].append('看门狗或零位回读异常')
                results[sid]=result
            except Exception as exc:results[sid]={'torque_off':False,'restored':False,'errors':[str(exc)]}
        self.last_cleanup={'torque_off':all(v['torque_off'] for v in results.values()),
            'restored':all(v['restored'] for v in results.values()),'joints':results}
        writes=[w for d in s['devices'].values() for w in d.writes];self.write_count+=len(writes)
        if not self.last_cleanup['restored']:self.fault='支撑恢复配置未完成，请检查设备';reason=self.fault
        self.last_message=reason;self.metadata_time=-1000
        report={'kind':'soft_hold','reason':reason,'profile_sha256':self.sha,'original':s['original'],
                'cleanup':self.last_cleanup,'writes':writes,'trace':s['trace'],'time':time.time()}
        (self.root/f'console-soft-session-{time.time_ns()}.json').write_text(json.dumps(report,indent=2)+'\n')
        path=self.root/'console-soft-original.json';journal=json.loads(path.read_text())
        journal['active']=not self.last_cleanup['restored'];journal['cleanup']=self.last_cleanup
        path.write_text(json.dumps(journal,indent=2)+'\n')

    def process(self,command):
        action=command.get('action')
        if action=='off':self.release('已停止并关闭力矩');return
        if command.get('generation')!=self.generation:raise ValueError('连接已更新，请重新操作')
        current=self.session or self.soft
        if action in ('heartbeat','target','cycle','soft_capture','soft_strength'):
            if not current or command.get('token')!=current['token']:return
        if action=='heartbeat':self.accept_heartbeat(command,self.clock())
        elif action=='arm':self.arm(command)
        elif action=='soft_arm':self.arm_soft(command)
        elif action=='soft_capture':self.capture_soft()
        elif action=='soft_strength':self.set_soft_strength(command['current_ma'])
        elif action=='target':
            if self.soft:raise ValueError('柔和支撑只保持姿态，不接受运动目标')
            self.session['cycle']=None;self.set_target(command['degrees'])
        elif action=='cycle':
            if self.soft:raise ValueError('请先退出柔和支撑')
            s=self.session;goals=cycle_goals(s['anchor'],s['window'],float(command['amplitude']),command['cycles'])
            s['cycle']={'remaining':goals[1:],'settled':0};self.set_target(goals[0])
        elif action=='record_start':
            if self.session or self.soft or any(self.torque.values()):raise ValueError('手动记录前必须关闭全部力矩')
            if self.clock()-self.sample_time>.3:raise ValueError('等待新鲜的实物读数')
            sid=int(command['id']);q=math.degrees(self.angles[sid])
            self.pending=None;self.recording={'id':sid,'degrees':[q,q],'samples':0,'started':self.clock()}
            self.last_message='请轻轻活动所选实物关节，再放回活动范围中间'
        elif action=='record_finish':self.finish_record()
        elif action=='confirm_range':
            p=self.pending
            if not p or command.get('record_id')!=p['record_id']:raise ValueError('请先记录当前关节活动范围')
            if p['samples']<10 or not 3<=p['degrees'][1]-p['degrees'][0]<=60:
                raise ValueError('活动幅度需在 3–60° 之间，请重新手动记录小范围')
            self.ranges[str(p['id'])]={'degrees':p['degrees'],'confirmed':True,'source':'user-confirmed console recording'}
            temporary=self.ranges_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'profile_sha256':self.sha,'joints':self.ranges},indent=2)+'\n')
            temporary.replace(self.ranges_path);self.pending=None;self.last_message='此关节范围和方向已保存'
        else:raise ValueError('未知操作')

    def frame(self):
        s=self.session;now=self.clock()
        soft=self.soft
        control={'generation':self.generation,'active_id':s['id'] if s else None,
            'token':(s or soft)['token'] if (s or soft) else None,'anchor_degrees':s['anchor'] if s else None,
            'soft_active':bool(soft),'soft_ids':list(SOFT_IDS) if soft else [],
            'soft_targets_degrees':soft['targets'] if soft else {},
            'soft_currents_ma':soft['currents'] if soft else {},
            'soft_current_limit_ma':soft['current_limit_ma'] if soft else SOFT_CURRENT_MA,
            'soft_following_ids':[i for i,f in soft['followers'].items() if f.dragging] if soft else [],
            'soft_behavior':'drag-follow-then-hold',
            'target_degrees':s['target'] if s else None,'window_degrees':s['window'] if s else None,
            'cycle_running':bool(s and s['cycle']),'session_seconds':now-s['started'] if s else 0,
            'session_timeout_seconds':None,'limits':self.limits.public(),
            'heartbeat_age_s':max(0,now-(s or soft)['heartbeat']) if (s or soft) else None,
            'max_heartbeat_gap_s':(s or soft).get('max_heartbeat_gap_s',0) if (s or soft) else None,
            'manual_control':{'profile_velocity':MANUAL_PROFILE_VELOCITY,
                              'speed_deg_s':MANUAL_SPEED_DEG_S,'pwm_cap':50},
            'message':self.last_message,'fault':self.fault,'last_cleanup':self.last_cleanup,
            'ranges':self.ranges,'recording':self.recording,'pending_range':self.pending,
            'last_result':self.last_result,
            'register_write_count':self.write_count+(len(s['device'].writes) if s else 0)+
                (sum(len(d.writes) for d in soft['devices'].values()) if soft else 0)}
        return {'type':'frame','positions':self.positions,'torque':self.torque,'homing_offsets':self.homing,'health':self.health,
            'joint_angles_rad':self.angles,'profile_sha256':self.sha,
            'metadata_age_s':max(0,now-self.metadata_time),'control':control,'unix_time':time.time()}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--transport-lease',type=float,default=0,
                        help='Exit when USB link pings stop; zero keeps SSH EOF behavior')
    args=parser.parse_args()
    if args.transport_lease and not 1<=args.transport_lease<=10:
        parser.error('transport lease must be 1–10 seconds')
    sys.path.insert(0,'tools');import dxl
    root=Path.cwd();bus=dxl.Bus('/dev/ttyS2',baud=1000000,timeout=.025)
    fcntl.flock(bus.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    engine=ConsoleEngine(bus,(root/'software-zero-reference.json').read_bytes(),root)
    commands=queue.Queue(maxsize=64);stop=threading.Event();cancel=threading.Event()
    last_link_ping=[time.monotonic()]
    engine.cancel_arm=lambda:stop.is_set() or cancel.is_set()
    def reader():
        try:
            for line in sys.stdin:
                if len(line)>8192:continue
                try:command=json.loads(line)
                except Exception:continue
                if not isinstance(command,dict):continue
                if command.get('action')=='link_ping':
                    if command.get('generation')==engine.generation:last_link_ping[0]=time.monotonic()
                    continue
                if command.get('action')=='heartbeat':
                    engine.accept_heartbeat(command,time.monotonic())
                    continue
                if command.get('action')=='off':cancel.set()
                try:commands.put_nowait((time.monotonic(),command))
                except queue.Full:stop.set();break
        finally:stop.set()
    def shutdown(*_):stop.set()
    signal.signal(signal.SIGTERM,shutdown);signal.signal(signal.SIGINT,shutdown)
    threading.Thread(target=reader,daemon=True).start()
    if args.transport_lease:
        def link_watchdog():
            while not stop.wait(.1):
                if time.monotonic()-last_link_ping[0]>args.transport_lease:
                    stop.set();cancel.set();return
        threading.Thread(target=link_watchdog,daemon=True).start()
    try:
        while not stop.is_set():
            tick=time.monotonic()
            for _ in range(16):
                try:arrived,command=commands.get_nowait()
                except queue.Empty:break
                try:
                    if time.monotonic()-arrived>.5 and command.get('action')!='off':continue
                    engine.process(command)
                    if command.get('action')=='off':cancel.clear()
                    if command.get('action')!='heartbeat':
                        engine.last_result={'request_id':command.get('request_id'),'ok':True}
                except Exception as exc:
                    engine.last_result={'request_id':command.get('request_id'),'ok':False,'error':str(exc)}
                    engine.last_message=str(exc)
                    if engine.session or engine.soft:engine.release('控制命令失败：'+str(exc))
            try:engine.sample()
            except Exception as exc:
                engine.release('采样中断，已尝试关闭力矩')
                engine.angles={};engine.positions={};engine.last_message=str(exc)
            print(json.dumps(engine.frame()),flush=True)
            time.sleep(max(0,.05-(time.monotonic()-tick)))
    except (BrokenPipeError,KeyboardInterrupt):pass
    finally:
        try:engine.release('页面或采集服务已断开')
        finally:bus.close()


if __name__=='__main__':main()
