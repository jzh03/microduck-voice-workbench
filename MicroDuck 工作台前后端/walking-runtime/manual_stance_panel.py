#!/usr/bin/env python3
"""Unified posture and walking UI with one on-board controller owner."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import hashlib
import math
import os
from pathlib import Path
import time

from manual_walk_panel import Panel
from ssh_trial_transport import ROOT
from stance_reference import from_saved


class StancePanel(Panel):
    saved_path=None

    def public(self):
        return {**super().public(),'saved_path':self.saved_path}

    def set_speed(self, body):
        speed=body.get('speed')
        if type(speed) not in (int,float) or not math.isfinite(speed) or not 0<=speed<=1. or abs(speed*100-round(speed*100))>1e-8:
            raise ValueError('速度范围为 0～1.00 m/s，每档 0.01')
        with self.lock:
            if self.recovering.is_set() or self.state.get('phase') in ('reconnecting','needs_recovery','recovery_error','error'):
                raise ValueError('连接恢复后再调节速度')
            if self.state.get('running'):
                if not self.state.get('healthy') or self.state.get('policy') not in ('walk','stand'):
                    raise ValueError('动态平衡稳定后再调节速度')
            elif not self.safe_off or self.gate():
                raise ValueError('先确认全部卸力或等待动态平衡稳定')
            self.twist=(0.,0.,0.);self.drive_seen=0.;self.pending_step=None;self.pulse_until=0.
            self.args.drive_speed=float(speed)

    def start_floor_recovery(self):
        with self.lock:
            if not self.public()['can_start']:
                raise ValueError(self.gate() or '请先结束当前会话并确认卸力，再启动起身')
            previous=self.args.runner
            self.args.runner='run_teleop.py'
            try:
                self.start(True,floor_recovery=True)
            except Exception:
                self.args.runner=previous
                raise

    def gate(self):
        if self.args.mode=='live':
            try:
                deployment=json.loads(self.args.deployment_report.read_text())
                for name in ('run_manual_stance.py','stance_state.py','stance_kinematics.py',
                             'run_teleop.py','stance_reference.py','teleop_state.py','handoff_pose.py','flight_recorder.py','hardware_bridge.py','ground-robotd.toml'):
                    if deployment['sha256'][name]!=hashlib.sha256((ROOT/'walking-runtime'/name).read_bytes()).hexdigest():
                        return '手动站姿程序正在更新，完成后才能开始'
            except (OSError,ValueError,KeyError):return '等待手动站姿程序装入主板'
        return super().gate()


PAGE = '''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MicroDuck · 站姿与行走</title>
<style>
.direction-pad{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:14px 0}.direction-pad button{min-height:112px;font-size:20px;touch-action:none;user-select:none}.direction-pad .arrow{display:block;font-size:38px;line-height:1.1}.direction-pad .pressed{background:#064b48;box-shadow:inset 0 0 0 3px #8fd6b9}.direction-pad small{display:block;font-size:13px;font-weight:400;margin-top:4px}
*{box-sizing:border-box}body{margin:0;padding:24px;background:#edf3f2;color:#173b40;font:16px/1.55 system-ui}main{max-width:800px;margin:auto}h1{font-size:30px;margin:6px 0 10px}h2{font-size:20px;margin:0 0 8px}p{margin:8px 0 14px}.card{background:white;padding:22px;border-radius:16px;margin:14px 0}.muted{color:#597175}.status{padding:16px;background:#dcefe9;border-radius:12px}.pair{display:grid;grid-template-columns:1fr 1fr;gap:10px}button{font:600 17px system-ui;min-height:56px;padding:12px;border:0;border-radius:10px;background:#087d77;color:white;cursor:pointer;touch-action:manipulation}button:disabled{opacity:.38;cursor:default}.secondary{background:#dce9e7;color:#264b4f}.wide{width:100%;margin-top:10px}.value{font-variant-numeric:tabular-nums;font-weight:650}.tag{font-size:13px;letter-spacing:.08em;color:#5e797b}input{width:20px;height:20px;vertical-align:middle}label{display:block;margin:16px 0}.stop{background:#c83944}.control-actions{margin-top:10px}#motion-controls button:disabled{opacity:1;background:#d8e5e3;color:#56716f}#motion-controls .stop:disabled{background:#f0dadd;color:#8a5a60}#motion-controls .status{margin-bottom:12px}#drive-note{margin-bottom:0}.metrics{font-variant-numeric:tabular-nums}table{width:100%;border-collapse:collapse;font-size:14px}td,th{text-align:right;padding:6px;border-bottom:1px solid #e5eded}td:first-child,th:first-child{text-align:left}.note{font-size:14px;color:#647b7d}#saved{overflow-wrap:anywhere} @media(max-width:500px){body{padding:10px}.card{padding:16px}button{font-size:15px}}
</style>
<main><div class="tag">MICRODUCK · 站姿与行走</div><h1>站姿与行走控制</h1>
<p>按住方向键前进、后退，松开停步并继续平衡。</p>
<div class="card" id="startup-controls"><h2>手动启动起身</h2>
<div id="status" class="status" aria-live="polite">正在读取状态…</div><p id="metrics" class="metrics muted"></p><p id="gate" class="note"></p><p id="log-note" class="note"></p>
<button id="floor-standup" class="wide" disabled>平躺起身：开始尝试</button>
<p class="note">把鸭子平躺放稳，留出头、腿活动空间，人在旁保护。点击后使用新标定与官方起身模型；请保持本页打开。</p>
<p id="floor-start-note" class="note" aria-live="polite">正在检查是否可以启动…</p>
<div class="pair"><button id="floor-stop" class="stop" disabled>停止本次尝试并卸力</button><button id="recover" class="secondary">重新连接并复位</button></div>
<p class="note">停止会卸力，请先托稳。正常停止后可再次点“开始尝试”；断连或保护停止时先点“重新连接并复位”。每次启动单独保存日志。</p></div>
<div class="card" id="motion-controls"><h2>前进 / 后退</h2>
<div class="direction-pad" aria-label="前后方向控制"><button id="forward-drive" data-direction="forward" aria-keyshortcuts="ArrowUp W" disabled><span class="arrow">↑</span>按住前进<small>键盘 ↑ / W</small></button><button id="backward-drive" data-direction="backward" aria-keyshortcuts="ArrowDown S" disabled><span class="arrow">↓</span>按住后退<small>键盘 ↓ / S</small></button></div>
<div class="pair"><button id="forward-step" class="secondary" disabled>前进一下 · 0.5 秒</button><button id="backward-step" class="secondary" disabled>后退一下 · 0.5 秒</button></div>
<div class="pair control-actions"><button id="halt" class="secondary" disabled>停步 · Space</button><button id="off" class="stop" disabled>急停并卸力 · Esc</button></div>
<p id="drive-note" class="note" aria-live="polite">等待控制器状态；前进后退暂未开放。</p>
<button id="confirm" class="wide secondary" disabled>动态平衡已能独立站稳，启用前后移动</button>
<button id="retry-handoff" class="wide secondary" disabled>扶稳后重试接管（保持加力）</button>
<p class="note">目标速度 __STEP_SPEED_LABEL__ m/s。按住上方箭头或键盘 ↑ / W、↓ / S 移动，松开停步；“一下”按钮由主板计时半秒。急停会卸力。</p></div>
<div class="card"><h2>保存站姿与动态平衡</h2>
<label><input id="supported" type="checkbox"> 我已扶稳躯干，双脚平放并承重</label>
<button id="start" class="wide" disabled>开始：恢复保存的稳定站姿</button>
<button id="home" data-pose class="wide secondary" disabled>回到保存的稳定站姿</button>
<p id="mode-note" class="note">先恢复你保存的稳定站姿；微调与行走使用同一份参考。模式切换会先卸力，请托稳躯干。</p></div>
<div class="card"><h2>站姿与动态平衡切换</h2><p id="reference-note">行走使用保存站姿作为关节参考。切换会先卸力，请托稳躯干。原标定保留，补偿效果仍需实测。</p>
<div class="pair"><button id="manual-mode" class="secondary">切换到站姿微调（先卸力）</button><button id="walk-mode">切换到动态平衡（先卸力）</button></div>
</div>
<div class="card"><h2>身体前后移动</h2><p>保持设定高度，让髋、膝、踝一起调整。后倒时先试前移一点。</p><p>累计平移设定：<span id="shift-value" class="value">0.00 mm</span></p>
<div class="pair"><button id="body-forward" data-pose disabled>身体前移 0.5 mm</button><button id="body-backward" data-pose class="secondary" disabled>身体后移 0.5 mm</button></div></div>
<div class="card"><h2>大腿与躯干之间的角度</h2><p>左右髋关节同步旋转，调整躯干前倾／后倾。</p><p>相对前倾：<span id="hip-value" class="value">0.0°</span></p>
<div class="pair"><button id="hip-forward" data-pose disabled>转髋：前倾 0.5°</button><button id="hip-backward" data-pose class="secondary" disabled>转髋：后倾 0.5°</button></div></div>
<div class="card"><h2>膝盖与脚踝细调</h2><p class="note">左右同步，每次 0.5°。先用上面的身体前移或转髋，再按实物姿态细调。</p>
<p>屈膝偏移：<span id="knee-value" class="value">0.0°</span></p><div class="pair"><button id="knee-more" data-pose disabled>膝盖多弯 0.5°</button><button id="knee-less" data-pose class="secondary" disabled>膝盖少弯 0.5°</button></div>
<p>脚踝前倾：<span id="ankle-value" class="value">0.0°</span></p><div class="pair"><button id="ankle-forward" data-pose disabled>脚踝前倾 0.5°</button><button id="ankle-backward" data-pose class="secondary" disabled>脚踝后倾 0.5°</button></div></div>
<div class="card"><h2>保留与回退</h2><div class="pair"><button id="capture" data-pose class="secondary" disabled>以现在姿态为新起点</button><button id="reset" data-pose class="secondary" disabled>撤回微调，回保存站姿</button></div>
<button id="save" data-pose class="wide secondary" disabled>保存这套姿态</button><p id="saved" class="note"></p>
<p class="note">保存实际角度和设定值，方便复现；不会改写舵机零位，也不会在开机时自动加力。</p>
<details><summary>查看实际腿角与已下发目标</summary><p class="note">这里比较实际角度与已下发目标。接管时被拦截的模型目标没有下发，不会显示成当前跟随误差。</p><table><thead><tr><th>关节</th><th>实际角度</th><th>已下发目标</th><th>当前跟随误差</th></tr></thead><tbody id="joint-table"></tbody></table></details></div>
<p class="note">页面或网络失联会停止本次手动保持。</p></main>
<script>
const token='__TOKEN__'; let state={running:false},busy=false;const keys=new Set(),pointers=new Map();
const groups={hip:0,knee:0,ankle:0};
const stepSpeed=__STEP_SPEED__;
function movementState(s){
  if(s.recovering)return {mode:'正在复位，尚未进入动态平衡。',reason:'正在复位，前进后退暂不可用。',drive:false};
  if(s.phase==='preparing_pose')return {mode:'正在准备行走接管姿态，当前保持加力。',reason:s.message,drive:false};
  if(s.phase==='reconnecting'||s.healthy===false)return {mode:'控制器连接或健康状态尚未确认。',reason:'控制状态未确认，前进后退暂不可用。',drive:false};
  if(!s.running||s.stop_reason||['stopping','error','needs_recovery','recovery_error'].includes(s.phase))return {mode:'当前未运行动态平衡。',reason:'前进后退未开放：请先扶稳并启动动态平衡。',drive:false};
  if(s.selected_mode!=='walk')return {mode:'当前正在保持保存站姿，可进行姿态微调。',reason:'前进后退未开放：当前为站姿微调，请先切换到动态平衡。',drive:false};
  if(s.native_recovery_active||s.recovery_confirmation_required)return {mode:'正在恢复平衡，方向控制暂停。',reason:s.message||'恢复后重新确认独立站稳，再启用前后移动。',drive:false};
  if(!['walk','stand'].includes(s.policy)||!s.healthy||(!s.drive_unlocked&&!s.can_confirm_balance))return {mode:s.policy==='held'?'当前仅保持保存站姿，行走策略尚未接管。':'动态平衡尚未稳定，前进后退暂不可用。',reason:'前进后退未开放：'+(s.message||'等待行走策略接管并稳定。'),drive:false};
  if(!s.drive_unlocked)return {mode:'动态平衡已接管，等待你确认独立站稳。',reason:'确认机身已能独立站稳后，点击下方“启用前后移动”。',drive:false};
  return {mode:'动态平衡已接管，前后移动已启用。',reason:s.pulse_active?'正在执行半秒试走，请等本次动作结束。':'按住 ↑ / W 前进，↓ / S 后退；松开停步并继续平衡。',drive:true};
}
async function req(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify(body)});let s=await r.json();if(!r.ok)throw Error(s.error||'操作失败');return s;}
function render(s){let was=state.running;state=s;if(was&&!s.running)document.querySelector('#supported').checked=false;document.querySelector('#status').textContent=s.last_command_error?'本次调整未执行：'+s.last_command_error:(s.message||'等待准备');document.querySelector('#gate').textContent=s.gate||'';document.querySelector('#supported').disabled=!!s.running;document.querySelector('#start').disabled=!s.can_start||!document.querySelector('#supported').checked;document.querySelector('#off').disabled=!s.running;document.querySelector('#recover').disabled=!!s.recovering;
document.querySelector('#log-note').textContent=(s.local_diagnostic_error||s.diagnostic_log_error)?'诊断记录异常：'+(s.local_diagnostic_error||s.diagnostic_log_error):s.local_diagnostic_recording?(s.diagnostic_log?'诊断记录已开启：传感器、模型目标与软件卸力原因。':'操作记录已开启；启动后同时记录主板诊断。'):'';
let walking=s.selected_mode==='walk',movement=movementState(s);document.querySelector('#floor-standup').disabled=!s.can_start;document.querySelector('#floor-stop').disabled=!s.running;document.querySelector('#floor-start-note').textContent=s.running?'控制器运行中；需要结束时先托稳，再点“停止本次尝试并卸力”。':s.can_start?'已就绪，可点击上方按钮开始一次起身尝试。':(s.gate||s.message||'等待连接与舵机状态确认');document.querySelector('#retry-handoff').disabled=!(s.running&&s.healthy&&s.can_retry_handoff);document.querySelector('#start').textContent=walking?(s.separate_manual_reference?'开始：使用新标定进入动态平衡':s.supported_direct_handoff?'开始：从保存站姿接管（托稳试验）':s.handoff_pose_loaded?'开始：恢复站姿并准备行走接管姿态（需托稳）':'开始：使用保存站姿进入动态平衡'):'开始：恢复保存的稳定站姿';document.querySelector('#mode-note').textContent=movement.mode;document.querySelector('#drive-note').textContent=movement.reason;document.querySelector('#confirm').disabled=!walking||!s.running||!['walk','stand'].includes(s.policy)||!s.healthy||!s.can_confirm_balance||s.drive_unlocked;let drive=movement.drive;for(let id of ['forward-step','backward-step'])document.getElementById(id).disabled=!drive||!s.can_confirm_balance||!!s.pulse_active;for(let id of ['forward-drive','backward-drive'])document.getElementById(id).disabled=!drive||!!s.pulse_active;document.querySelector('#halt').disabled=!s.running;for(let id of ['manual-mode','walk-mode'])document.getElementById(id).disabled=!!s.recovering;if(!drive)clearMotion();
let can=!!(s.running&&s.can_adjust_pose&&!s.stop_reason);document.querySelectorAll('[data-pose]').forEach(b=>b.disabled=!can);let g=s.group_offsets_deg||groups;document.querySelector('#shift-value').textContent=(s.body_shift_mm||0).toFixed(2)+' mm';document.querySelector('#hip-value').textContent=(-g.hip||0).toFixed(1)+'°';document.querySelector('#knee-value').textContent=(g.knee||0).toFixed(1)+'°';document.querySelector('#ankle-value').textContent=(-g.ankle||0).toFixed(1)+'°';
document.querySelector('#metrics').textContent=[typeof s.tilt_deg==='number'?'躯干倾斜 '+s.tilt_deg.toFixed(1)+'°':'',typeof s.volts_min==='number'?s.volts_min.toFixed(1)+' V':'',typeof s.temperature_max==='number'?s.temperature_max+'℃':'',typeof s.control_hz==='number'?'控制 '+s.control_hz.toFixed(1)+' Hz':''].filter(Boolean).join(' · ');
let actual=s.joint_degrees||(s.diagnostics||{}).joint_degrees||{},target=s.commanded_joint_degrees||(s.diagnostics||{}).commanded_joint_degrees||{};document.querySelector('#joint-table').replaceChildren();for(let [id,name] of [[22,'左大腿 / 髋'],[12,'右大腿 / 髋'],[23,'左膝'],[13,'右膝'],[24,'左踝'],[14,'右踝']]){let tr=document.createElement('tr');for(let v of [name,actual[id],target[id],typeof actual[id]==='number'&&typeof target[id]==='number'?actual[id]-target[id]:undefined]){let td=document.createElement('td');td.textContent=typeof v==='number'?v.toFixed(2)+'°':(v||'—');tr.append(td)}document.querySelector('#joint-table').append(tr)}
if(s.saved_path)document.querySelector('#saved').textContent='已保存：'+s.saved_path;}
async function act(body){if(!state.can_adjust_pose)return;document.querySelectorAll('[data-pose]').forEach(b=>b.disabled=true);try{render(await req('/adjust',body));}catch(e){document.querySelector('#status').textContent=e.message;}}
document.querySelector('#supported').onchange=()=>render(state);
document.querySelector('#start').onclick=async()=>{try{render(await req('/start',{supported:document.querySelector('#supported').checked}));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#floor-standup').onclick=async()=>{clearMotion();try{render(await req('/standup'));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#recover').onclick=async()=>{try{render(await req('/recover',{supported:true}));}catch(e){document.querySelector('#status').textContent=e.message;}};
for(let [id,mode] of [['manual-mode','manual'],['walk-mode','walk']])document.getElementById(id).onclick=async()=>{if(state.running&&state.selected_mode===mode){document.querySelector('#status').textContent='当前会话已在此模式，保持现状，无需重复切换。';return;}clearMotion();try{render(await req('/switch',{mode,supported:true}));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#confirm').onclick=async()=>{try{render(await req('/confirm'));}catch(e){document.querySelector('#status').textContent=e.message;}};
document.querySelector('#retry-handoff').onclick=async()=>{clearMotion();try{render(await req('/retry-handoff',{supported:true}));}catch(e){document.querySelector('#status').textContent=e.message;}};
for(let [id,vx] of [['forward-step',stepSpeed],['backward-step',-stepSpeed]])document.getElementById(id).onclick=async()=>{clearMotion();try{render(await req('/step',{vx}));}catch(e){document.querySelector('#status').textContent=e.message;}};
function motionAxis(){let forward=keys.has('KeyW')||keys.has('ArrowUp')||[...pointers.values()].includes('forward'),backward=keys.has('KeyS')||keys.has('ArrowDown')||[...pointers.values()].includes('backward');return Number(forward)-Number(backward);}
function drawMotion(){for(let [id,axis] of [['forward-drive',1],['backward-drive',-1]])document.getElementById(id).classList.toggle('pressed',motionAxis()===axis);}
function clearMotion(){keys.clear();pointers.clear();drawMotion();}
function sendMotion(){if(state.running)return req('/drive',{vx:movementState(state).drive?stepSpeed*motionAxis():0,vy:0,vyaw:0}).catch(()=>{clearMotion();});return Promise.resolve();}
function halt(){clearMotion();if(state.running)req('/drive',{vx:0,vy:0,vyaw:0}).catch(()=>{});}document.querySelector('#halt').onclick=halt;
for(let button of document.querySelectorAll('[data-direction]')){button.addEventListener('pointerdown',e=>{if(button.disabled||!movementState(state).drive)return;e.preventDefault();button.setPointerCapture(e.pointerId);pointers.set(e.pointerId,button.dataset.direction);drawMotion();sendMotion();});for(let event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,e=>{if(!pointers.has(e.pointerId))return;pointers.delete(e.pointerId);drawMotion();sendMotion();});button.addEventListener('contextmenu',e=>e.preventDefault());}
for(let [id,delta] of [['body-forward',.5],['body-backward',-.5]])document.getElementById(id).onclick=()=>act({action:'body_shift',millimeters:(state.body_shift_mm||0)+delta});
for(let [id,group,delta] of [['hip-forward','hip',-.5],['hip-backward','hip',.5],['knee-more','knee',.5],['knee-less','knee',-.5],['ankle-forward','ankle',-.5],['ankle-backward','ankle',.5]])document.getElementById(id).onclick=()=>act({action:'adjust_joint',group,degrees:((state.group_offsets_deg||groups)[group]||0)+delta});
for(let action of ['home','capture','reset'])document.getElementById(action).onclick=()=>act({action});
document.querySelector('#save').onclick=async()=>{try{render(await req('/save'));}catch(e){document.querySelector('#status').textContent=e.message;}};
for(let id of ['off','floor-stop'])document.getElementById(id).onclick=()=>{clearMotion();req('/off').catch(()=>{});};window.addEventListener('keydown',e=>{if(e.target.closest('input,select,textarea,[contenteditable=true]'))return;if(e.code==='Escape'&&state.running){clearMotion();req('/off').catch(()=>{});e.preventDefault();return;}if(e.code==='Space'){halt();e.preventDefault();return;}if(movementState(state).drive&&!state.pulse_active&&['KeyW','KeyS','ArrowUp','ArrowDown'].includes(e.code)){keys.add(e.code);drawMotion();if(!e.repeat)sendMotion();e.preventDefault();}});window.addEventListener('keyup',e=>{if(!['KeyW','KeyS','ArrowUp','ArrowDown'].includes(e.code))return;keys.delete(e.code);drawMotion();sendMotion();});window.addEventListener('blur',halt);document.addEventListener('visibilitychange',()=>{if(document.hidden)halt();});
setInterval(async()=>{if(busy)return;busy=true;try{render(await req('/status'));if((keys.size||pointers.size)&&movementState(state).drive)await sendMotion();}catch(e){clearMotion();render({...state,can_start:false,can_adjust_pose:false,drive_unlocked:false,message:'控制连接中断，请托稳机身；可点重新连接并复位'});}finally{busy=false}},100);
</script></html>'''


def queue_adjustment(panel, body):
    with panel.lock:
        if not panel.state.get('running') or not panel.state.get('can_adjust_pose') or panel.stop.is_set():
            raise ValueError('请先保持姿态，并等待动作结束')
        action=body.get('action')
        if action=='adjust_joint':
            group=body.get('group');value=body.get('degrees')
            if group not in ('hip','knee','ankle') or type(value) not in (int,float) or not math.isfinite(value) or abs(value)>8:
                raise ValueError('关节调整范围为 ±8°')
            current=panel.state.get('group_offsets_deg',{}).get(group,0)
            if abs(value-current)>.500001:raise ValueError('每次调整 0.5°')
            command={'action':action,'group':group,'degrees':value}
        elif action=='body_shift':
            value=body.get('millimeters')
            if type(value) not in (int,float) or not math.isfinite(value) or abs(value)>5 or abs(value-panel.state.get('body_shift_mm',0))>.500001:
                raise ValueError('身体前后移动范围 ±5 mm，每次最多 0.5 mm')
            command={'action':action,'millimeters':value}
        elif action in ('home','capture','reset'):
            command={'action':action}
        else:raise ValueError('未知站姿操作')
        panel.pending_manual=command;panel.pose_revision+=1
        panel.state.update(can_adjust_pose=False,can_confirm_balance=False)


def main(on_ready=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--mode',choices=('live','fake'),default='live')
    p.add_argument('--port',type=int,default=8769);p.add_argument('--remote-port',type=int,default=18769)
    p.add_argument('--certificate',type=Path,default=ROOT/'outputs/manual-walking-20261025/imu-direction-check/imu-direction-verified.json')
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--deployment-report',type=Path,default=ROOT/'outputs/balance-recovery-20261026/leg-posture-analysis/manual-stance-deployment.json')
    p.add_argument('--reference',type=Path,default=ROOT/'outputs/balance-recovery-20261026/stable-standing-reference.json')
    p.add_argument('--manual-reference',type=Path,help='Separate saved physical stance for manual mode; walking uses --reference')
    handoff=p.add_mutually_exclusive_group()
    handoff.add_argument('--handoff-pose',type=Path,help='Opt-in supported handoff preparation; not enabled by default')
    handoff.add_argument('--supported-direct-handoff',action='store_true',help='Explicit supported trial; target-angle gaps are diagnostic only')
    p.add_argument('--step-speed',type=float,choices=(.05,.1,.15,.2,.3),default=.05)
    p.add_argument('--initial-mode',choices=('manual','walk'),default='manual')
    args=p.parse_args();args.runner='run_manual_stance.py' if args.initial_mode=='manual' else 'run_teleop.py';args.start_home=True;args.output_dir.mkdir(parents=True,exist_ok=True)
    panel=StancePanel(args);page=PAGE.replace('__TOKEN__',panel.token).replace('__STEP_SPEED__',str(args.step_speed)).replace('__STEP_SPEED_LABEL__',f'{args.step_speed:.2f}')
    if args.manual_reference:
        page=page.replace('行走使用保存站姿作为关节参考。切换会先卸力，请托稳躯干。原标定保留，补偿效果仍需实测。',
                          '站姿微调保留保存的稳定姿态；动态平衡按新标定控制。模式切换会先卸力，请托稳躯干。新标定下的起身和行走仍需实测。')
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
                body=json.loads(self.rfile.read(size) or b'{}');panel.ui_seen=time.monotonic();panel.browser_seen=panel.ui_seen
                if not isinstance(body,dict):raise ValueError('操作格式错误')
                if panel.audit and self.path not in ('/status','/drive'):
                    panel.audit.record('ui_request',route=self.path,
                                       command={k:v for k,v in body.items() if k in ('action','mode','supported','vx','vy','vyaw','degrees','millimeters')})
                if self.path=='/start':panel.start(body.get('supported') is True)
                elif self.path=='/standup':panel.start_floor_recovery()
                elif self.path=='/recover':panel.request_recovery(body.get('supported') is True)
                elif self.path=='/switch':
                    if body.get('mode') not in ('manual','walk'):raise ValueError('未知控制模式')
                    panel.request_recovery(body.get('supported') is True,next_mode=body['mode'])
                elif self.path=='/confirm':
                    if not panel.state.get('can_confirm_balance'):raise ValueError('动态平衡尚未就绪')
                    panel.confirm.set()
                elif self.path=='/retry-handoff':
                    with panel.lock:
                        if body.get('supported') is not True or not panel.state.get('running') or not panel.state.get('healthy') or not panel.state.get('can_retry_handoff'):
                            raise ValueError('扶稳并等待回到保存站姿后才能重试')
                        panel.pending_manual={'action':'retry_handoff','supported':True}
                        panel.state.update(can_retry_handoff=False,drive_unlocked=False,can_confirm_balance=False)
                elif self.path=='/step':
                    vx=body.get('vx')
                    with panel.lock:
                        if type(vx) not in (int,float) or vx not in (-args.step_speed,args.step_speed):raise ValueError('速度必须匹配当前试走设置')
                        if not panel.state.get('running') or not panel.state.get('drive_unlocked') or not panel.state.get('can_confirm_balance') or not panel.state.get('healthy'):raise ValueError('请先确认动态平衡已能独立站稳')
                        if panel.pending_step or time.monotonic()<panel.pulse_until:raise ValueError('请等上一次动作结束')
                        panel.pending_step={'action':'step','vx':vx,'seconds':.5};panel.pulse_until=time.monotonic()+.6
                elif self.path=='/drive':
                    values=tuple(body.get(k) for k in ('vx','vy','vyaw'))
                    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values) or abs(values[0])>getattr(args,'drive_speed',args.step_speed) or values[1:]!=(0,0):raise ValueError('试走速度超过当前设置')
                    with panel.lock:panel.twist=values;panel.drive_seen=time.monotonic();panel.pending_step=None;panel.pulse_until=0.
                elif self.path=='/speed':
                    panel.set_speed(body)
                elif self.path=='/off':panel.stop.set();panel.confirm.clear();panel.pending_step=None;panel.pulse_until=0.
                elif self.path=='/adjust':queue_adjustment(panel,body)
                elif self.path=='/save':
                    with panel.lock:
                        if panel.stop.is_set() or not panel.state.get('running') or not panel.state.get('can_adjust_pose'):raise ValueError('请等待动作结束再保存')
                        sample=panel.state.get('diagnostics',{})
                        if len(sample.get('joint_degrees',{}))!=15 or len(sample.get('commanded_joint_degrees',{}))!=15:raise ValueError('实际关节或设定状态不完整')
                        content=json.dumps({'saved_unix':time.time(),'automatic_start':False,'source':'manual_supported_stance','state':panel.state},ensure_ascii=False,indent=2)+'\n'
                    saved=args.output_dir/('saved-stance-'+time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns())+'.json')
                    with saved.open('x') as stream:stream.write(content);stream.flush();os.fsync(stream.fileno())
                    if json.loads(saved.read_text())!=json.loads(content):raise ValueError('姿态文件读回校验失败')
                    reference=from_saved(json.loads(content),(ROOT/'local-joint-viewer/software-zero-reference.json').read_bytes(),
                                         (ROOT/'walking-runtime/runtime-contract.json').read_bytes(),saved)
                    reference_path=panel.reference_path()
                    if reference_path is None:raise ValueError('当前模式未设置姿态保存位置')
                    temporary=reference_path.with_suffix('.pending')
                    with temporary.open('w') as stream:json.dump(reference,stream,ensure_ascii=False,indent=2);stream.flush();os.fsync(stream.fileno())
                    temporary.replace(reference_path)
                    panel.saved_path=str(saved)
                elif self.path!='/status':return self.respond({},status=404)
                self.respond(panel.public())
            except (ValueError,TypeError) as exc:self.respond({'error':str(exc)},status=400)
    print(f'手动站姿页 http://127.0.0.1:{args.port}/ · {args.mode} · 默认未开启',flush=True)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    if on_ready is not None:on_ready(panel)
    server.serve_forever()


if __name__=='__main__':main()
