#!/usr/bin/env python3
"""Local manual walking UI, through pinned SSH to an on-board balance controller."""
import argparse
import contextlib
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time
import urllib.request
import urllib.error
import uuid

from ssh_trial_transport import ROOT, pause_ssh_viewer, ssh_arguments
from panel_recovery import control_lock, recover_panel

PAGE='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MicroDuck · 手动行走</title>
<style>body{font:17px/1.6 system-ui;background:#edf3f2;color:#183c42;margin:0;padding:24px}main{max-width:740px;margin:auto;background:#fff;padding:28px;border-radius:20px}h1{font-size:28px}button{padding:15px;border:0;border-radius:10px;font:600 17px system-ui;cursor:pointer;background:#087d77;color:#fff;touch-action:none}button:disabled{opacity:.35;cursor:default}.wide{display:block;width:100%;margin:12px 0}#off{background:#c83944}#status{padding:18px;border-radius:12px;background:#e4f3ed}label{display:block;margin:18px 0}input[type=checkbox]{width:20px;height:20px;vertical-align:middle}.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:20px 0}.pad button{min-height:70px}.pressed{background:#17474e;box-shadow:inset 0 0 0 3px #8de4c4}.muted{color:#566c71}#metrics{font-variant-numeric:tabular-nums}select{font:inherit;padding:8px}</style>
<main><div class="muted">MicroDuck · __MODE_LABEL__</div><h1>先零速平衡，再短距离前进</h1>
<p>从手动站姿切换时，先托住躯干，在<a href="http://127.0.0.1:8769/">站姿页</a>急停卸力，再在本页开始。行走策略会重新调整关节；等它能独立站稳，再启用方向控制。</p>
<div id="status">正在读取状态…</div><p id="metrics"></p><p id="gate"></p>
<label><input id="supported" type="checkbox"> __SUPPORT_LABEL__</label>
<button id="start" class="wide" disabled>开始站立和平衡</button>
<button id="confirm" class="wide" disabled>已能独立站稳，启用方向控制</button>
<button id="recover" class="wide">托稳后点：重新连接并复位</button>
<p class="muted">断连、保护停止或主板重启后都可点。复位会卸力、恢复配置并重新采集，完成后由你点“开始”；不会自动加力。</p>
<p>后倒诊断：只在双脚承重、双手扶稳时使用，每次调整后等待 3 秒。当前机身俯仰目标 <b id="pitch-target">0°</b>。</p>
<div class="pad"><button id="pitch-less" disabled>躯干后倾 −2°</button><button id="pitch-more" disabled>躯干前倾 +2°</button></div>
<p class="muted">当前已核验方向：正数让躯干向前倾，负数向后倾；脖子可能反向补偿。调整范围 −6～16°，用于对照实物倾向与脚底受力；恢复 0° 并独立站稳后才能启用方向控制。每次重新启动从 0° 开始。</p>
<p>腿部站姿：站立高度相对默认 <b id="height-target">0 mm</b>。</p>
<div class="pad"><button id="height-less" disabled>降低 1 mm</button><button id="height-more" disabled>升高 1 mm</button></div>
<p class="muted">用于小幅屈膝对照，范围 −10～0 mm。每次只调一个量，等待 3 秒，再看双腿和脚底受力；俯仰与高度都归零后才能启用行走。</p>
<p>先把双脚朝前、平放在防滑地面上，让脚底承重，手扶躯干保持稳定。站稳后按住方向键移动，松键停步。</p>
<label>前后速度 <select id="speed"><option value="0.05">0.05 m/s · 初次试走</option><option value="0.1">0.10 m/s</option><option value="0.15">0.15 m/s</option><option value="0.2">0.20 m/s</option><option value="0.3">0.30 m/s</option><option value="0.4">0.40 m/s</option></select></label>
<p class="muted">初次先短按 W / ↑ 约半秒，松开后观察停步能否站稳。速度是给模型的目标值，实际步幅和位移需要实测。</p>
<div class="pad"><span></span><button data-key="KeyW" disabled>W / ↑<br>前进</button><span></span>
<button data-key="KeyA" disabled>A / ←<br>左转</button><button id="halt" disabled>停步<br>Space</button><button data-key="KeyD" disabled>D / →<br>右转</button><span></span><button data-key="KeyS" disabled>S / ↓<br>后退</button><span></span></div>
<button id="off" class="wide" disabled>急停并卸力 · Esc</button>
<p class="muted">停步会把速度归零并继续平衡。急停会卸力，请先托住机身。方向控制启用后，页面或网络断开会停止行走指令，板上的平衡程序继续运行。</p>
<p><a href="http://127.0.0.1:8768/">机身方向核对</a> · <a href="http://127.0.0.1:8766/">关节对照</a></p></main>
<script>
const token='__TOKEN__',keys=new Set(),aliases={ArrowUp:'KeyW',ArrowDown:'KeyS',ArrowLeft:'KeyA',ArrowRight:'KeyD'};let state={running:false,drive_unlocked:false},busy=false;
async function req(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify(body)});let s=await r.json();if(!r.ok)throw Error(s.error||'操作失败');return s}
function release(){keys.clear();drawKeys();if(state.running)req('/drive',{vx:0,vy:0,vyaw:0}).catch(()=>{})}
function drawKeys(){document.querySelectorAll('[data-key]').forEach(b=>b.classList.toggle('pressed',keys.has(b.dataset.key)))}
function render(s){let wasRunning=state.running;state=s;if(wasRunning&&!s.running)document.querySelector('#supported').checked=false;document.querySelector('#status').textContent=s.message;document.querySelector('#gate').textContent=s.gate||'';
document.querySelector('#metrics').textContent=[typeof s.control_hz==='number'?'控制 '+s.control_hz.toFixed(1)+' Hz':'',typeof s.volts_min==='number'?'电压 '+s.volts_min.toFixed(1)+' V':'',typeof s.tilt_deg==='number'?'倾斜 '+s.tilt_deg.toFixed(1)+'°':'',typeof s.pitch_deg==='number'?'俯仰 '+s.pitch_deg.toFixed(1)+'°':'',typeof s.temperature_max==='number'?'温度 '+s.temperature_max+'℃':''].filter(Boolean).join(' · ');
document.querySelector('#start').disabled=!s.can_start||!document.querySelector('#supported').checked;document.querySelector('#supported').disabled=s.running;document.querySelector('#recover').disabled=!!s.recovering;
document.querySelector('#confirm').disabled=!s.running||!s.can_confirm_balance||s.drive_unlocked;document.querySelector('#off').disabled=!s.running;
let p=s.stand_pitch_deg||0;document.querySelector('#pitch-target').textContent=p.toFixed(0)+'°';document.querySelector('#pitch-less').disabled=!s.running||!s.can_adjust_pitch||p<=-6;document.querySelector('#pitch-more').disabled=!s.running||!s.can_adjust_pitch||p>=16;
let h=s.stand_height_mm||0;document.querySelector('#height-target').textContent=h.toFixed(0)+' mm';document.querySelector('#height-less').disabled=!s.running||!s.can_adjust_height||h<=-10;document.querySelector('#height-more').disabled=!s.running||!s.can_adjust_height||h>=0;
document.querySelector('#halt').disabled=!s.running;let canDrive=s.drive_unlocked&&s.running&&!s.stop_reason&&s.phase!=='stopping';document.querySelectorAll('[data-key]').forEach(b=>b.disabled=!canDrive);if(!canDrive)keys.clear();}
document.querySelector('#supported').onchange=()=>render(state);
document.querySelector('#start').onclick=async()=>{release();try{render(await req('/start',{supported:document.querySelector('#supported').checked}));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#confirm').onclick=async()=>{try{render(await req('/confirm'));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#recover').onclick=async()=>{release();try{render(await req('/recover',{supported:true}));}catch(e){document.querySelector('#status').textContent=e.message;}};
for(let [id,step] of [['pitch-less',-2],['pitch-more',2]])document.getElementById(id).onclick=async()=>{try{render(await req('/pitch',{degrees:(state.stand_pitch_deg||0)+step}));}catch(e){document.querySelector('#status').textContent=e.message;}};
for(let [id,step] of [['height-less',-1],['height-more',1]])document.getElementById(id).onclick=async()=>{try{render(await req('/height',{millimeters:(state.stand_height_mm||0)+step}));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#off').onclick=()=>{release();req('/off').catch(()=>{});};document.querySelector('#halt').onclick=release;
document.querySelectorAll('[data-key]').forEach(b=>{b.onpointerdown=e=>{if(!state.drive_unlocked)return;e.preventDefault();b.setPointerCapture(e.pointerId);keys.add(b.dataset.key);drawKeys()};b.onpointerup=b.onpointercancel=()=>{keys.delete(b.dataset.key);drawKeys();if(!keys.size)release()};});
window.addEventListener('keydown',e=>{if(e.code==='Escape'){release();if(state.running)req('/off').catch(()=>{});e.preventDefault();return}if(e.code==='Space'&&state.running){release();e.preventDefault();return}if(e.target.closest('input,select,textarea,[contenteditable=true]'))return;let k=aliases[e.code]||e.code;if(['KeyW','KeyA','KeyS','KeyD'].includes(k)&&state.drive_unlocked&&state.running&&!state.stop_reason){keys.add(k);drawKeys();e.preventDefault()}});
window.addEventListener('keyup',e=>{keys.delete(aliases[e.code]||e.code);drawKeys();if(!keys.size)release()});window.addEventListener('blur',release);document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});
setInterval(async()=>{if(busy)return;busy=true;try{let s=await req('/status');render(s);if(s.running){let v=Number(document.querySelector('#speed').value);await req('/drive',{vx:(keys.has('KeyW')?v:0)-(keys.has('KeyS')?v:0),vy:0,vyaw:(keys.has('KeyA')?.3:0)-(keys.has('KeyD')?.3:0)})}}catch(e){release();render({...state,phase:'reconnecting',can_start:false,can_confirm_balance:false,drive_unlocked:false,control_hz:null,volts_min:null,tilt_deg:null,temperature_max:null,message:'控制连接中断；当前舵机状态未知，请在旁保护'});}finally{busy=false}},100);
</script></html>'''


class CommandRejected(RuntimeError):
    pass


def report_confirms_safe_off(report, *, manual=False):
    torque=report.get('torque_after',{})
    cleanup=report.get('cleanup')
    return (len(torque)==15 and not any(torque.values()) and
            not report.get('cleanup_error') and
            (not manual or report.get('safe_off') is True) and
            (cleanup is None or cleanup.get('restored') is True))


class Panel:
    def __init__(self,args):
        self.args=args;self.token=uuid.uuid4().hex;self.lock=threading.RLock()
        self.audit=None;self.audit_error=None
        output=getattr(args,'output_dir',None)
        if output:
            from flight_recorder import FlightRecorder
            try:self.audit=FlightRecorder(output/'panel-events.jsonl')
            except OSError as exc:self.audit_error=str(exc)
        self.state={'phase':'idle','message':'尚未启动，等待准备','running':False,'drive_unlocked':False}
        self.safe_off=True;self.ui_seen=0.;self.drive_seen=0.;self.twist=(0.,0.,0.)
        self.stop=threading.Event();self.confirm=threading.Event();self.remote_token=None
        self.pending_pitch=None
        self.pending_height=None
        self.pending_manual=None
        self.pending_step=None;self.pulse_until=0.
        self.pose_revision=0
        self.sequence=0;self.tunnel=None
        self.running_thread=None;self.control_guard=None
        self.recovering=threading.Event();self.recovery_requested=threading.Event()
        self.session_identity={}
        output=getattr(args,'output_dir',None)
        if args.mode=='live' and output and output.exists():
            sessions=sorted((p for p in output.glob('????????-??????-*') if p.is_dir()),reverse=True)
            if sessions and (sessions[0]/'launch.json').exists() and not (sessions[0]/'result.json').exists():
                try:
                    self.session_identity={**json.loads((sessions[0]/'launch.json').read_text()),
                                           'runner':getattr(args,'runner','run_teleop.py')}
                    self.safe_off=False
                    self.state.update(phase='needs_recovery',message='上次控制已中断；托稳后点“重新连接并复位”即可重试')
                except (OSError,ValueError):
                    self.safe_off=False
                # A confirmed recovery after that launch remains valid across
                # a local panel restart. Fresh viewer and on-board startup
                # checks still gate every subsequent torque-on.
                recoveries=sorted(output.glob('recovery-[0-9]*.json'),key=lambda p:p.stat().st_mtime,reverse=True)
                if recoveries and recoveries[0].stat().st_mtime>(sessions[0]/'launch.json').stat().st_mtime:
                    try:
                        report=json.loads(recoveries[0].read_text())
                        if (report.get('verified') is True and report.get('homing_unchanged') is True and
                                report.get('motion_commanded') is False and report_confirms_safe_off(report)):
                            self.safe_off=True
                            self.state.update(phase='idle',message='已确认上次复位完成，全部卸力；扶稳后可开始')
                    except (OSError,ValueError):pass
            context=output/'recovery-context.json'
            if context.exists():
                value=json.loads(context.read_text())
                if value.get('pending'):
                    self.session_identity=value['identity'];self.safe_off=False
                    self.state.update(phase='needs_recovery',message='旧会话已中断；托稳后点重新连接并复位')

    def gate(self):
        if self.args.mode=='fake':return None
        try:
            e=json.loads(self.args.certificate.read_text())
            if not e.get('verified') or e['profile_sha256']!=hashlib.sha256((ROOT/'local-joint-viewer/software-zero-reference.json').read_bytes()).hexdigest() or e['contract_sha256']!=hashlib.sha256((ROOT/'walking-runtime/runtime-contract.json').read_bytes()).hexdigest():
                return '机身方向核对记录不匹配当前配置'
        except (OSError,ValueError,KeyError):return '请先完成机身方向核对页的四步'
        if not self.state['running']:
            try:
                s=json.loads((ROOT/'outputs/live-joint-viewer-20261017/state.json').read_text())
                if time.time()-s['unix_time']>2:return '等待关节采集恢复'
                if len(s['torque'])!=15:return '等待15个关节力矩读数，当前状态未知'
                if any(s['torque'].values()) or s['console'].get('active_id') or s['console'].get('soft_active'):
                    return '请先结束其他关节控制并确认全部卸力'
            except (OSError,ValueError,KeyError):return '等待新鲜的关节状态'
        return None

    def reference_path(self):
        if getattr(self.args,'runner','run_teleop.py')=='run_manual_stance.py':
            manual=getattr(self.args,'manual_reference',None)
            if manual is not None:return Path(manual)
        reference=getattr(self.args,'reference',None)
        return Path(reference) if reference is not None else None

    def public(self):
        with self.lock:
            gate=self.gate()
            return {**self.state,'gate':gate,'recovering':self.recovering.is_set(),
                    'selected_mode':'manual' if getattr(self.args,'runner','run_teleop.py')=='run_manual_stance.py' else 'walk',
                    'reference_loaded':bool(getattr(self.args,'reference',None)),
                    'separate_manual_reference':bool(getattr(self.args,'manual_reference',None)),
                    'handoff_pose_loaded':bool(getattr(self.args,'handoff_pose',None)),
                    'supported_direct_handoff':bool(getattr(self.args,'supported_direct_handoff',False)),
                    'local_diagnostic_recording':self.audit is not None and self.audit.error is None,
                    'local_diagnostic_error':self.audit_error or (self.audit.error if self.audit else None),
                    'step_speed':getattr(self.args,'drive_speed',getattr(self.args,'step_speed',.05)),
                    'can_start':not self.recovering.is_set() and not self.state['running'] and self.safe_off and not gate}

    def connection_lost(self):
        with self.lock:
            if self.audit and self.state.get('phase')!='reconnecting':
                self.audit.record('control_connection_lost',previous_phase=self.state.get('phase'),identity=self.session_identity)
            self.twist=(0.,0.,0.);self.drive_seen=0.;self.confirm.clear()
            self.pending_pitch=None
            self.pending_height=None
            self.pending_manual=None
            self.pending_step=None;self.pulse_until=0.
            self.state.update(phase='reconnecting',drive_unlocked=False,can_confirm_balance=False,can_adjust_pitch=False,can_adjust_height=False,can_adjust_pose=False,can_retry_handoff=False,
                              message='控制链路重连中；当前舵机状态未知，请在旁保护')
            for key in ('control_hz','volts_min','tilt_deg','pitch_deg','roll_deg','temperature_max','diagnostics','healthy'):
                self.state.pop(key,None)

    def remote(self,path,body=None):
        request=urllib.request.Request(f'http://127.0.0.1:{self.args.remote_port}'+path,
            data=json.dumps(body or {}).encode(),headers={'Content-Type':'application/json','X-Control-Token':self.remote_token},method='POST')
        try:
            with urllib.request.urlopen(request,timeout=.5) as response:return json.load(response)
        except urllib.error.HTTPError as exc:
            if exc.code==400 and path=='/command':
                try:reason=json.loads(exc.read()).get('error','控制器尚未接受调整')
                except (ValueError,TypeError):reason='控制器尚未接受调整'
                raise CommandRejected(reason) from exc
            raise

    def accept_remote_state(self, state, pose_revision):
        with self.lock:
            # A status request may have begun before a new local adjustment was
            # queued. Do not reopen its controls using that older response.
            if pose_revision!=self.pose_revision or self.pending_pitch is not None or self.pending_height is not None or self.pending_manual is not None:
                state={**state,'can_adjust_pitch':False,'can_adjust_height':False,'can_adjust_pose':False,'can_confirm_balance':False}
            self.state=state

    def open_tunnel(self,out):
        argv=ssh_arguments(self.args.host)
        with (out/'tunnel.log').open('ab') as log:
            self.tunnel=subprocess.Popen(argv[:-1]+['-N','-o','ExitOnForwardFailure=yes','-L',
                f'127.0.0.1:{self.args.remote_port}:127.0.0.1:{self.args.remote_port}',argv[-1]],
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    def start(self,supported,*,floor_recovery=False):
        with self.lock:
            if not supported:raise ValueError('请先扶稳并勾选准备确认')
            if not self.public()['can_start']:raise ValueError(self.gate() or '当前会话尚未结束')
            if floor_recovery and (getattr(self.args,'runner','run_teleop.py')!='run_teleop.py' or getattr(self.args,'handoff_pose',None)):
                raise ValueError('地面起身必须使用官方行走模型，且不使用站立接管过渡')
            self.args.floor_recovery=bool(floor_recovery)
            self.control_guard=control_lock(self.args.host) if self.args.mode=='live' else None
            self.recovery_requested.clear()
            self.stop.clear();self.confirm.clear();self.twist=(0.,0.,0.);self.drive_seen=0.;self.sequence=0
            self.pending_pitch=None
            self.pending_height=None
            self.pending_manual=None
            self.pending_step=None;self.pulse_until=0.
            self.state={'phase':'connecting','message':'正在连接板上控制器，请保持扶稳','running':True,'drive_unlocked':False}
        self.running_thread=threading.Thread(target=self.run,daemon=True)
        self.running_thread.start()

    def request_recovery(self,supported,next_mode=None):
        with self.lock:
            if supported is not True:raise ValueError('请先托稳躯干；复位会卸力')
            if self.recovering.is_set():raise ValueError('复位正在进行，请稍等')
            self.recovering.set();self.stop.set();self.confirm.clear()
            self.twist=(0.,0.,0.);self.drive_seen=0.
            self.pending_pitch=self.pending_height=self.pending_manual=None
            self.pending_step=None;self.pulse_until=0.
            self.state.update(phase='recovering',drive_unlocked=False,can_confirm_balance=False,
                              can_adjust_pose=False,can_adjust_pitch=False,can_adjust_height=False,
                              message='正在结束旧会话并复位；请持续托稳躯干')
        threading.Thread(target=self._recover,args=(next_mode,),daemon=True).start()

    def _recover(self,next_mode=None):
        guard=None
        try:
            if self.running_thread and self.running_thread.is_alive():
                self.running_thread.join(timeout=2)
                if self.running_thread.is_alive():
                    self.recovery_requested.set()
                    self.running_thread.join(timeout=9)
                if self.running_thread.is_alive():
                    raise RuntimeError('旧连接仍在退出，请稍后再点复位')
            self.safe_off=False
            if self.args.mode=='live':
                guard=control_lock(self.args.host)
                def update(message):
                    with self.lock:self.state.update(phase='recovering',message=message,running=False)
                report,path=recover_panel(self.args.host,self.session_identity,self.args.output_dir,update)
            else:
                report,path={'verified':True,'motion_commanded':False},None
            with self.lock:
                self.safe_off=True
                self.state={'phase':'idle','running':False,'drive_unlocked':False,
                            'message':'复位完成，已全部卸力；扶稳并勾选后即可重新开始',
                            'recovery_report':report,'recovery_path':path}
            output=getattr(self.args,'output_dir',None)
            if output and (output/'recovery-context.json').exists():
                (output/'recovery-context.json').write_text(json.dumps({'pending':False,'recovery_path':path})+'\n')
            if next_mode is not None:
                self.args.runner='run_manual_stance.py' if next_mode=='manual' else 'run_teleop.py'
        except Exception as exc:
            with self.lock:
                self.safe_off=False
                self.state={'phase':'recovery_error','running':False,'drive_unlocked':False,
                            'message':str(exc)+'；托稳后可再次点击复位'}
        finally:
            if guard:guard.close()
            self.recovering.clear()
        if next_mode is not None and self.safe_off:
            try:self.start(True)
            except Exception as exc:
                with self.lock:self.state.update(message='已复位，尚未加力：'+str(exc))

    def write_trace(self, trace, sample):
        if trace is None:return None
        try:
            trace.write(json.dumps({'received_unix':time.time(),**sample})+'\n')
            return trace
        except OSError as exc:
            self.audit_error='本机遥测日志暂停：'+str(exc)
            try:trace.close()
            except OSError:pass
            return None

    def close_local_resources(self, trace):
        try:
            if trace:
                try:trace.close()
                except OSError as exc:self.audit_error='本机日志关闭失败：'+str(exc)
        finally:
            try:
                if self.tunnel and self.tunnel.poll() is None:self.tunnel.terminate()
            finally:
                if self.control_guard:
                    self.control_guard.close();self.control_guard=None

    def run(self):
        session_id=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
        out=self.args.output_dir/session_id;out.mkdir(parents=True)
        self.remote_token=uuid.uuid4().hex+uuid.uuid4().hex
        remote_dir='/run/microduck-manual-'+session_id
        reference_path=self.reference_path()
        payload={'directory':remote_dir,'token':self.remote_token,'mode':self.args.mode,'port':self.args.remote_port,
                 'runner':getattr(self.args,'runner','run_teleop.py'),
                 'start_home':bool(getattr(self.args,'start_home',False)),
                 'reference':json.loads(reference_path.read_text()) if reference_path is not None else None,
                 'handoff_pose':json.loads(self.args.handoff_pose.read_text()) if getattr(self.args,'handoff_pose',None) and getattr(self.args,'runner','run_teleop.py')=='run_teleop.py' else None,
                 'step_speed':getattr(self.args,'step_speed',.05),
                 'floor_recovery':bool(getattr(self.args,'floor_recovery',False)),
                 'supported_direct_handoff':bool(getattr(self.args,'supported_direct_handoff',False)) and getattr(self.args,'runner','run_teleop.py')=='run_teleop.py',
                 'certificate':json.loads(self.args.certificate.read_text()) if self.args.mode=='live' else None}
        launcher=r'''import json,sys,pathlib,os,subprocess
p=json.load(sys.stdin);d=pathlib.Path(p['directory']);d.mkdir(mode=0o700)
token=d/'token';token.write_text(p['token']);token.chmod(0o600)
assert p['runner'] in ('run_teleop.py','run_manual_stance.py')
argv=['python3','-u','/root/microduck-imu-update/walking-runtime/'+p['runner'],'--mode',p['mode'],'--token-file',str(token),'--output-dir',str(d),'--port',str(p['port'])]
if p['runner']=='run_manual_stance.py' and p.get('start_home'):argv+=['--start-home']
if p.get('reference') is not None:
 r=d/'standing-reference.json';r.write_text(json.dumps(p['reference']));argv+=['--reference',str(r)]
if p.get('handoff_pose') is not None:
 # Preserve exact reference bytes: the pose is bound to their SHA256.
 r.write_text(p['reference_bytes']);h=d/'handoff-pose.json';h.write_text(json.dumps(p['handoff_pose']));argv+=['--handoff-pose',str(h)]
if p['runner']=='run_teleop.py':argv+=['--step-speed',str(p.get('step_speed',.05))]
if p.get('floor_recovery'):
 assert p['runner']=='run_teleop.py' and p.get('reference') is not None and not p.get('handoff_pose')
 argv+=['--floor-recovery']
if p.get('supported_direct_handoff'):
 assert p['runner']=='run_teleop.py' and p['mode']=='live' and p.get('handoff_pose') is None
 argv+=['--supported-direct-handoff']
if p['mode']=='live':
 c=d/'imu-direction.json';c.write_text(json.dumps(p['certificate']));argv+=['--supported','--certificate',str(c)]
logs=pathlib.Path('/root/microduck-imu-update/walking-runtime/diagnostics')/d.name;logs.mkdir(parents=True,exist_ok=True,mode=0o700)
with (logs/'launcher.log').open('ab') as log:
 child=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
print(json.dumps({'pid':child.pid,'directory':str(d),'boot_id':pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()}))'''
        pause=pause_ssh_viewer(self.args.host) if self.args.mode=='live' else contextlib.nullcontext({'allowed':True})
        trace=None;last_trace_time=-1.
        if payload['handoff_pose'] is not None:payload['reference_bytes']=reference_path.read_text()
        try:
            with pause as resume:
                resume['allowed']=False;self.safe_off=False
                result=subprocess.run(ssh_arguments(self.args.host)+['python3 -c '+shlex.quote(launcher)],
                                      input=json.dumps(payload),text=True,capture_output=True,timeout=15,check=True)
                launch=json.loads(result.stdout);(out/'launch.json').write_text(json.dumps(launch,indent=2)+'\n')
                if self.audit:self.audit.record('remote_session_launched',identity=launch,runner=payload['runner'])
                self.session_identity={**launch,'runner':payload['runner']}
                recovery={'host':self.args.host,'token':self.remote_token,'directory':remote_dir,'port':self.args.remote_port}
                fd=os.open(out/'reconnect.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
                with os.fdopen(fd,'w') as stream:json.dump(recovery,stream)
                self.open_tunnel(out)
                try:trace=(out/'telemetry.jsonl').open('w',buffering=1)
                except OSError as exc:self.audit_error='本机遥测日志暂停：'+str(exc)
                connected=False;started=time.monotonic();last_reconnect=0.
                while True:
                    if self.recovery_requested.is_set():raise RuntimeError('旧连接已结束，正在独立确认复位状态')
                    now=time.monotonic()
                    with self.lock:pose_revision=self.pose_revision
                    try:
                        if self.stop.is_set():self.remote('/command',{'action':'off'})
                        elif now-self.ui_seen<.6:self.remote('/command',{'action':'heartbeat'})
                        if self.confirm.is_set() and not self.stop.is_set():
                            self.confirm.clear()
                            self.remote('/command',{'action':'confirm_balance'})
                        with self.lock:pitch=self.pending_pitch;self.pending_pitch=None
                        if pitch is not None and not self.stop.is_set():
                            self.remote('/command',{'action':'stand_pitch','degrees':pitch})
                        with self.lock:height=self.pending_height;self.pending_height=None
                        if height is not None and not self.stop.is_set():
                            self.remote('/command',{'action':'stand_height','millimeters':height})
                        with self.lock:manual=self.pending_manual;self.pending_manual=None
                        if manual is not None and not self.stop.is_set():
                            self.remote('/command',manual)
                        with self.lock:step=self.pending_step;self.pending_step=None
                        if step is not None and not self.stop.is_set():
                            self.remote('/command',step)
                            self.pulse_until=time.monotonic()+step['seconds']+.1
                        with self.lock:
                            twist=self.twist if now-self.drive_seen<.25 and now-self.ui_seen<.6 else (0.,0.,0.)
                            unlocked=self.state.get('drive_unlocked',False)
                        if not unlocked:twist=(0.,0.,0.)
                        if getattr(self.args,'runner','run_teleop.py')=='run_teleop.py' and not self.stop.is_set() and now>=self.pulse_until and (now-self.ui_seen<.6 or unlocked):
                            self.sequence+=1
                            self.remote('/command',{'action':'drive','sequence':self.sequence,**dict(zip(('vx','vy','vyaw'),twist))})
                        state=self.remote('/status');connected=True
                        sample=state.get('diagnostics')
                        if sample and sample['elapsed_s']>last_trace_time:
                            trace=self.write_trace(trace,sample)
                            last_trace_time=sample['elapsed_s']
                        self.accept_remote_state(state,pose_revision)
                        if not state['running']:
                            report=state.get('report',{})
                            (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                            torque=report.get('torque_after',{});cleanup=report.get('cleanup')
                            confirmed=report_confirms_safe_off(report,manual=getattr(self.args,'runner','run_teleop.py')=='run_manual_stance.py')
                            resume['allowed']=confirmed;self.safe_off=confirmed
                            self.remote('/close');break
                    except CommandRejected as exc:
                        with self.lock:self.state.update(message='调整未接受，请等待姿态就绪：'+str(exc),can_adjust_pitch=False,can_adjust_height=False,can_adjust_pose=False,can_confirm_balance=False)
                    except Exception as exc:
                        self.connection_lost()
                        if not connected and now-started>12:
                            raise RuntimeError('无法连接板上控制器：'+str(exc))
                        if self.tunnel and self.tunnel.poll() is not None and now-last_reconnect>1:
                            self.open_tunnel(out);last_reconnect=now
                        if connected:
                            # A finished controller may have exited while disconnected.
                            try:
                                r=subprocess.run(ssh_arguments(self.args.host)+['cat '+shlex.quote(remote_dir+'/result.json')],capture_output=True,text=True,timeout=5)
                                if r.returncode==0:
                                    report=json.loads(r.stdout);torque=report.get('torque_after',{});cleanup=report.get('cleanup')
                                    confirmed=report_confirms_safe_off(report,manual=getattr(self.args,'runner','run_teleop.py')=='run_manual_stance.py')
                                    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
                                    resume['allowed']=confirmed;self.safe_off=confirmed
                                    with self.lock:self.state.update(running=False,drive_unlocked=False,message='会话已结束；'+('力矩已关闭' if confirmed else '关闭状态未确认'),report=report)
                                    break
                            except Exception:pass
                        time.sleep(.2)
                    time.sleep(.08)
        except Exception as exc:
            with self.lock:self.state.update(phase='error',running=False,drive_unlocked=False,message=str(exc))
        finally:
            self.close_local_resources(trace)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--mode',choices=('live','fake'),default='live')
    p.add_argument('--port',type=int,default=8769);p.add_argument('--remote-port',type=int,default=18769)
    p.add_argument('--certificate',type=Path,default=ROOT/'outputs/manual-walking-20261025/imu-direction-check/imu-direction-verified.json')
    p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True);panel=Panel(args)
    page=PAGE.replace('__TOKEN__',panel.token).replace('__MODE_LABEL__','无硬件模拟' if args.mode=='fake' else '实机手动控制').replace('__SUPPORT_LABEL__','确认开始无硬件模拟' if args.mode=='fake' else '双脚已平放地面并承重，我在旁扶稳机身')
    if args.mode=='fake':page=page.replace('已能独立站稳，启用方向控制','启用模拟方向控制')
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def respond(self,value,kind='application/json',status=200):
            data=value.encode() if isinstance(value,str) else json.dumps(value,ensure_ascii=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind+'; charset=utf-8');self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):self.respond(page,'text/html') if self.path=='/' else self.respond({},status=404)
        def do_POST(self):
            if self.headers.get('X-Control-Token')!=panel.token:return self.respond({},status=403)
            size=int(self.headers.get('Content-Length','0'))
            if not 0<=size<=1000:return self.respond({},status=413)
            try:
                body=json.loads(self.rfile.read(size) or b'{}');panel.ui_seen=time.monotonic()
                if self.path=='/start':panel.start(body.get('supported') is True)
                elif self.path=='/recover':panel.request_recovery(body.get('supported') is True)
                elif self.path=='/off':panel.twist=(0.,0.,0.);panel.stop.set()
                elif self.path=='/confirm':
                    if not panel.state.get('can_confirm_balance'):raise ValueError('平衡状态尚未就绪')
                    panel.confirm.set()
                elif self.path=='/pitch':
                    pitch=body.get('degrees')
                    with panel.lock:
                        if not panel.state.get('can_adjust_pitch') or not panel.state.get('running'):
                            raise ValueError('请先扶稳并等待站立控制就绪')
                        if type(pitch) not in (int,float) or not math.isfinite(pitch) or not -6<=pitch<=16 or abs(pitch-panel.state.get('stand_pitch_deg',0))>2.000001:
                            raise ValueError('俯仰范围为 −6～16°，每次最多调整 2°')
                        panel.pending_pitch=pitch
                        panel.pose_revision+=1
                        panel.state.update(can_adjust_pitch=False,can_adjust_height=False,can_confirm_balance=False)
                elif self.path=='/height':
                    height=body.get('millimeters')
                    with panel.lock:
                        if not panel.state.get('can_adjust_height') or not panel.state.get('running'):
                            raise ValueError('请先扶稳并等待站立控制就绪')
                        if type(height) not in (int,float) or not math.isfinite(height) or not -10<=height<=0 or abs(height-panel.state.get('stand_height_mm',0))>1.000001:
                            raise ValueError('高度范围为 −10～0 mm，每次最多调整 1 mm')
                        panel.pending_height=height
                        panel.pose_revision+=1
                        panel.state.update(can_adjust_pitch=False,can_adjust_height=False,can_confirm_balance=False)
                elif self.path=='/drive':
                    values=tuple(body.get(k) for k in ('vx','vy','vyaw'))
                    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values) or abs(values[0])>.4 or values[1]!=0 or abs(values[2])>.6:raise ValueError('速度指令无效')
                    with panel.lock:panel.twist=values;panel.drive_seen=time.monotonic()
                elif self.path!='/status':return self.respond({},status=404)
                self.respond(panel.public())
            except (ValueError,TypeError) as exc:self.respond({'error':str(exc)},status=400)
    print(f'手动控制页 http://127.0.0.1:{args.port}/ · {args.mode} · 默认未开启',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()


if __name__=='__main__':main()
