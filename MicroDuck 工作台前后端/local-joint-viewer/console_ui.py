"""Native Viser joint controls, preview animation and measured/target overlay."""
from collections import deque
import math
from pathlib import Path
import threading
import time
import uuid

import numpy as np

from kinematics import HOME, JOINTS, MODEL_DIR, axis_quat, quat_mul
from joint_limits import JointLimits
from support_power import support_health


def align_z(axis):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    if axis[2]<-.999999:return np.array([0.,1.,0.,0.])
    q=np.r_[1+axis[2],np.cross([0.,0.,1.],axis)]
    return q/np.linalg.norm(q)


class JointConsole:
    def __init__(self,server,rig,relay,selected,mesh_cache,profile_sha):
        self.server,self.rig,self.relay,self.selected=server,rig,relay,selected
        self.profile_sha=profile_sha;self.lock=threading.RLock();self.sid=14
        self.limits=JointLimits(Path(__file__).with_name('runtime-joint-limits.json'),
                                profile_sha256=profile_sha,ids=JOINTS)
        self.actual={sid:0. for sid in JOINTS};self.targets=self.actual.copy();self.centers=self.actual.copy()
        self.ready=False;self.initialized=False;self.control={};self.frame_received=0
        self.owner=None;self.token=None;self.arm_request=None;self.arm_started=0;self.was_active=False
        self.pending_target=None;self.last_send=0;self.last_heartbeat=0
        self.preview_cycle=None;self.dragging=set();self.history=deque(maxlen=200)
        self.last_plot=0;self.last_info=0;self.notice='模型预览已开启；拖动圆环或按 W / S 调整目标'
        self.preview_frames={};self.gizmos={};self.gizmo_bases={};self.meshes=[]

        with server.gui.add_folder('关节操控',order=4,expand_by_default=True):
            self.mode=server.gui.add_dropdown('控制对象',['模型预览','实机单关节','柔和支撑摆姿态'],initial_value='模型预览')
            self.message=server.gui.add_markdown('模型预览：仅青色目标模型运动，实机力矩关闭。')
            self.readout=server.gui.add_html('')
            self.limit_info=server.gui.add_markdown('')
            self.body_status=server.gui.add_markdown('',visible=False)
            self.target=server.gui.add_slider('目标角度 / °',*self.limits.bounds(self.sid),.1,0.)
            self.step=server.gui.add_dropdown('微调步长',['0.25°','0.5°','1°','3°'],initial_value='0.5°')
            self.nudge=server.gui.add_button_group('调整',['− / S','返回 / R','+ / W'])
            self.previous_next=server.gui.add_button_group('选择',['上一关节 / A','下一关节 / D'])
            self.overlay=server.gui.add_checkbox('显示青色目标模型',True)
            self.reset_preview=server.gui.add_button_group('参考姿态',['目标跟随实物','模型零位','官方站姿'])
            self.support=server.gui.add_checkbox('已扶稳机身，关节活动空间无遮挡',False,visible=False)
            self.arm_button=server.gui.add_button('开启当前关节实机控制',color='orange',visible=False,disabled=True)
            self.soft_strength=server.gui.add_slider('全身阻力上限 / mA',80,200,20,120,visible=False)
            self.soft_arm_button=server.gui.add_button('开启全身支撑 · 14 个关节',color='orange',visible=False,disabled=True)
            self.soft_capture_button=server.gui.add_button('全身保持现在姿态',color='teal',visible=False,disabled=True)
            self.stop_button=server.gui.add_button('停止动作 / 关闭力矩 · 空格',color='red')
            self.shortcut_help=server.gui.add_markdown('**W / S** 增减角度 · **A / D** 切换关节 · **R** 返回起点\n\n拖动选中关节的圆环，或 **Shift + 拖动外壳**。快捷键在输入框编辑时停用。')
        with server.gui.add_folder('往复与逐个预览',order=5,expand_by_default=True) as self.cycle_folder:
            self.amplitude=server.gui.add_slider('幅度 / °',.5,15.,.5,3.)
            self.cycles=server.gui.add_slider('往复次数',1,3,1,1)
            self.period=server.gui.add_slider('模型往复周期 / 秒',2.,8.,.5,3.)
            self.cycle_button=server.gui.add_button('当前关节往复',color='teal')
            self.sequence_button=server.gui.add_button('逐个预览 15 个关节')
            server.gui.add_markdown('逐个预览仅播放目标模型。实机往复在到位后返回起点并自动关闭力矩。')
        with server.gui.add_folder('目标 / 实测曲线',order=6,expand_by_default=True) as self.plot_folder:
            self.plot=server.gui.add_uplot((np.array([0.]),np.array([0.]),np.array([0.])),
                ({'label':'秒'},{'label':'目标 °','stroke':'#0891b2','width':2},
                 {'label':'实测 °','stroke':'#ea8a23','width':2}),
                scales={'x':{'time':False}},height=150,aspect=1.8)
        with server.gui.add_folder('手动记录关节活动范围',order=7,expand_by_default=False) as self.range_folder:
            self.range_info=server.gui.add_markdown('')
            server.gui.add_markdown('关闭力矩后，点击开始，轻轻活动当前关节并放回中间。确认模型与实物同向，再保存。这里保留人工核对记录；控制限位采用上方官方模型范围。')
            self.record_start=server.gui.add_button('开始记录手动范围')
            self.record_finish=server.gui.add_button('结束记录',disabled=True)
            self.confirm=server.gui.add_button('方向一致，保存此次范围',disabled=True,color='teal')

        self.wire(self.target,self.slider_changed,'on_update')
        self.wire(self.mode,self.mode_changed,'on_update')
        self.wire(self.nudge,lambda e:self.nudge_target(-1 if self.nudge.value.startswith('−') else 1) if '返回' not in self.nudge.value else self.return_to_center())
        self.wire(self.previous_next,lambda e:self.select_relative(-1 if self.previous_next.value.startswith('上一') else 1))
        self.wire(self.stop_button,lambda e:self.stop())
        self.wire(self.arm_button,self.arm)
        self.wire(self.soft_arm_button,self.arm_soft)
        self.wire(self.soft_strength,self.strength_changed,'on_update')
        self.wire(self.soft_capture_button,lambda e:self.send('soft_capture',token=self.token))
        self.wire(self.cycle_button,lambda e:self.start_cycle(False))
        self.wire(self.sequence_button,lambda e:self.start_cycle(True))
        self.wire(self.reset_preview,self.preview_reference)
        self.wire(self.record_start,lambda e:self.send('record_start',id=self.sid))
        self.wire(self.record_finish,lambda e:self.send('record_finish'))
        self.wire(self.confirm,lambda e:self.send('confirm_range',record_id=self.control['pending_range']['record_id']))
        for key,label,fn in [('W','增大当前关节角度',lambda:self.nudge_target(1)),
                             ('S','减小当前关节角度',lambda:self.nudge_target(-1)),
                             ('A','上一个关节',lambda:self.select_relative(-1)),
                             ('D','下一个关节',lambda:self.select_relative(1)),
                             ('R','返回本次起点',self.return_to_center),
                             ('space','停止动作并关闭力矩',self.stop),('escape','紧急停止',self.stop)]:
            command=server.gui.add_command(label,hotkey=key)
            self.wire(command,lambda e,fn=fn:fn(),'on_trigger')
        server.on_client_disconnect(self.disconnected)
        self.build_overlay(mesh_cache)

    def wire(self,handle,fn,event='on_click'):
        async def callback(e):
            if event=='on_update' and e.client is None:return
            with self.lock:
                try:fn(e)
                except Exception as exc:self.notice='操作未执行：'+str(exc)
        getattr(handle,event)(callback)

    def send(self,action,**values):
        return self.relay.send(action,**values)

    def stop(self):
        self.preview_cycle=None;self.pending_target=None;self.owner=None;self.token=None
        self.arm_request=None;self.was_active=False
        try:self.send('off')
        except Exception:pass
        self.notice='已停止；正在确认力矩关闭'

    def disconnected(self,client):
        with self.lock:
            if self.owner==client.client_id:self.stop()

    def choose(self,sid):
        if self.mode.value=='柔和支撑摆姿态':return
        if sid==self.sid:return
        if self.token or self.was_active:self.stop()
        self.preview_cycle=None;self.sid=sid;self.targets=self.actual.copy();self.centers=self.actual.copy()
        self.history.clear();self.support.value=False
        self.selected.value=f'{sid} · {JOINTS[sid][1]}'
        self.update_slider()

    def select_relative(self,delta):
        ids=list(JOINTS);self.choose(ids[(ids.index(self.sid)+delta)%len(ids)])

    def mode_changed(self,e):
        self.stop();self.targets=self.actual.copy();self.centers=self.actual.copy();self.support.value=False
        self.update_slider()
        self.notice='模型预览' if self.mode.value=='模型预览' else '支撑当前关节，再开启实机控制'

    def update_slider(self):
        self.target.min,self.target.max=self.limits.bounds(self.sid)
        self.target.value=round(self.limits.clamp(self.sid,self.targets[self.sid]),1)

    def set_target(self,degrees,*,animation=False):
        if not math.isfinite(degrees):return
        if self.mode.value=='柔和支撑摆姿态':return
        if not animation:self.preview_cycle=None
        if self.mode.value=='模型预览':
            self.targets[self.sid]=self.limits.clamp(self.sid,degrees)
        else:
            if self.control.get('active_id')!=self.sid or self.control.get('token')!=self.token:
                self.notice='操作未执行：请先勾选支撑确认，再点击「开启当前关节实机控制」'
                self.update_slider()
                return
            lo,hi=self.control['window_degrees'];degrees=max(lo,min(hi,float(degrees)))
            self.targets[self.sid]=degrees;self.pending_target=degrees
        self.update_slider()

    def slider_changed(self,e):self.set_target(float(self.target.value))

    def nudge_target(self,direction):
        self.set_target(self.targets[self.sid]+direction*float(self.step.value.rstrip('°')))

    def return_to_center(self):
        center=self.control.get('anchor_degrees') if self.mode.value=='实机单关节' else self.centers[self.sid]
        if center is not None:self.set_target(center)

    def preview_reference(self,e):
        if self.mode.value!='模型预览':raise ValueError('参考姿态只用于模型预览')
        self.preview_cycle=None
        option=self.reset_preview.value
        self.targets=(self.actual.copy() if option=='目标跟随实物' else
                      {sid:math.degrees(HOME[sid]) if option=='官方站姿' else 0. for sid in JOINTS})
        self.targets={sid:self.limits.clamp(sid,q) for sid,q in self.targets.items()}
        self.centers=self.targets.copy();self.update_slider()

    def arm(self,e):
        if not self.support.value:raise ValueError('请先确认已支撑机身、关节不会碰物体')
        if not self.ready:raise ValueError('等待完整的新鲜读数')
        if self.mode.value!='实机单关节':raise ValueError('请先选择实机单关节')
        self.targets=self.actual.copy();self.owner=e.client_id;self.token=uuid.uuid4().hex
        self.arm_started=time.monotonic();self.was_active=False
        self.arm_request=self.send('arm',id=self.sid,token=self.token,profile_sha256=self.profile_sha,
                                   limits_sha256=self.limits.sha)
        self.notice='正在检查配置并开启当前关节…'

    def start_cycle(self,all_joints):
        if self.mode.value=='柔和支撑摆姿态':raise ValueError('柔和支撑不执行往复动作')
        if self.mode.value=='实机单关节':
            if all_joints:raise ValueError('逐个预览只播放模型')
            if not self.token or self.control.get('active_id')!=self.sid:raise ValueError('先开启当前关节实机控制')
            self.pending_target=None
            self.send('cycle',token=self.token,amplitude=float(self.amplitude.value),cycles=int(self.cycles.value))
        else:
            ids=list(JOINTS) if all_joints else [self.sid]
            self.preview_cycle={'ids':ids,'index':0,'started':time.monotonic(),
                'centers':self.targets.copy(),'amplitude':float(self.amplitude.value),
                'period':float(self.period.value),'cycles':int(self.cycles.value)}
            self.sid=ids[0];self.selected.value=f'{self.sid} · {JOINTS[self.sid][1]}';self.history.clear()
            self.notice='正在逐个预览模型' if all_joints else '当前关节模型往复中'

    def arm_soft(self,e):
        if not self.support.value:raise ValueError('请先确认已扶稳机身')
        if not self.ready or self.mode.value!='柔和支撑摆姿态':raise ValueError('请等待新鲜读数并选择柔和支撑')
        self.targets=self.actual.copy();self.owner=e.client_id;self.token=uuid.uuid4().hex
        self.arm_started=time.monotonic();self.was_active=False
        self.arm_request=self.send('soft_arm',token=self.token,profile_sha256=self.profile_sha,limits_sha256=self.limits.sha,
                                   current_ma=int(self.soft_strength.value))
        self.notice='正在准备低电流支撑，请继续扶稳机身…'

    def strength_changed(self,e):
        if self.mode.value=='柔和支撑摆姿态' and self.token and self.control.get('soft_active') and self.control.get('token')==self.token:
            self.send('soft_strength',token=self.token,current_ma=int(self.soft_strength.value))

    def build_overlay(self,mesh_cache):
        self.overlay_root=self.server.scene.add_frame('/target',show_axes=False)
        paths={}
        for link in self.rig.links:
            parent=paths[link.parent] if link.parent else '/target';path=parent+'/'+link.name;paths[link.name]=path
            self.preview_frames[link.name]=self.server.scene.add_frame(path,show_axes=False,
                position=tuple(link.position),wxyz=tuple(link.quaternion))
            for i,geom in enumerate(link.geometries):
                mesh=mesh_cache[geom.mesh]
                h=self.server.scene.add_mesh_simple(path+f'/ghost-{i}',vertices=np.asarray(mesh.vertices,dtype=np.float32),
                    faces=np.asarray(mesh.faces,dtype=np.uint32),color=(5,153,181),opacity=.28,
                    position=tuple(geom.position),wxyz=tuple(geom.quaternion),side='double',cast_shadow=False)
                sid=geom.motor_id or link.joint_id
                if sid is not None:self.attach_mesh(h,sid)
            if link.joint_id is not None:
                sid=link.joint_id;base=quat_mul(link.quaternion,align_z(link.axis));self.gizmo_bases[sid]=base
                gizmo=self.server.scene.add_transform_controls(parent+f'/control-{sid}',scale=.055,
                    active_axes=(True,True,False),disable_axes=True,disable_sliders=True,
                    depth_test=False,line_width=4,opacity=.9,position=tuple(link.position),wxyz=tuple(base),visible=sid==self.sid)
                self.gizmos[sid]=gizmo
                async def move(e,sid=sid,base=base):
                    with self.lock:
                        if e.phase=='start':self.choose(sid);self.dragging.add(sid)
                        relative=quat_mul(base*np.array([1.,-1.,-1.,-1.]),e.target.wxyz)
                        angle=math.degrees(2*math.atan2(relative[3],relative[0]));angle=(angle+180)%360-180
                        self.set_target(angle)
                        if e.phase=='end':self.dragging.discard(sid)
                gizmo.on_update(move)

    def attach_mesh(self,handle,sid):
        starts={}
        async def click(e):
            with self.lock:self.choose(sid)
        handle.on_click(click)
        async def drag(e):
            with self.lock:
                key=e.client_id
                if e.phase=='start':
                    self.choose(sid);starts[key]=(e.end_screen_pos,self.targets[sid])
                if key in starts:
                    (x,y),angle=starts[key]
                    self.set_target(angle+180*((e.end_screen_pos[0]-x)-(e.end_screen_pos[1]-y)))
                if e.phase=='end':starts.pop(key,None)
        handle.on_drag(drag,modifier='shift')

    def tick(self,actual,frame,received,ready):
        with self.lock:
            now=time.monotonic();self.ready=ready
            if ready:self.actual={sid:math.degrees(q) for sid,q in actual.items()}
            if ready and not self.initialized:
                self.targets=self.actual.copy();self.centers=self.actual.copy();self.initialized=True
                self.update_slider()
            sid=int(self.selected.value.split(' · ')[0])
            if sid!=self.sid:self.choose(sid)
            self.control=(frame or {}).get('control',{})
            soft_mode=self.mode.value=='柔和支撑摆姿态'
            active=bool((self.control.get('soft_active') if soft_mode else self.control.get('active_id')==self.sid)
                        and self.token and self.control.get('token')==self.token)
            preparing=bool(soft_mode and self.token and not self.was_active and now-self.arm_started<8)
            if self.token:
                if (not ready and not preparing) or self.owner not in self.server.get_clients():self.stop()
                elif active:
                    self.was_active=True;self.arm_request=None
                elif self.was_active or now-self.arm_started>(8 if soft_mode else 2):
                    self.token=None;self.owner=None;self.arm_request=None;self.was_active=False
                result=self.control.get('last_result') or {}
                if result.get('request_id')==self.arm_request and result.get('ok') is False:
                    self.token=None;self.owner=None;self.arm_request=None
                if self.token and now-self.last_heartbeat>.1:
                    try:self.send('heartbeat',token=self.token)
                    except Exception:self.stop()
                    self.last_heartbeat=now
            if soft_mode and active:
                self.targets.update({int(i):q for i,q in self.control.get('soft_targets_degrees',{}).items()})
                self.update_slider()
            if active and self.control.get('cycle_running'):
                self.targets[self.sid]=self.control['target_degrees'];self.target.value=round(self.targets[self.sid],1)
            if self.pending_target is not None and active and now-self.last_send>.08:
                try:self.send('target',token=self.token,degrees=self.pending_target)
                except Exception:self.stop()
                self.pending_target=None;self.last_send=now
            if self.preview_cycle:
                cycle=self.preview_cycle;t=now-cycle['started'];duration=cycle['period']*cycle['cycles']
                sid=cycle['ids'][cycle['index']]
                if t>=duration:
                    self.targets[sid]=cycle['centers'][sid];cycle['index']+=1
                    if cycle['index']>=len(cycle['ids']):self.preview_cycle=None;self.notice='模型往复预览已完成'
                    else:
                        self.sid=cycle['ids'][cycle['index']];self.selected.value=f'{self.sid} · {JOINTS[self.sid][1]}'
                        cycle['started']=now;self.history.clear()
                else:
                    self.targets[sid]=self.limits.clamp(sid,cycle['centers'][sid]+cycle['amplitude']*math.sin(2*math.pi*t/cycle['period']))
                    self.target.value=round(self.targets[sid],1)
            real=self.mode.value!='模型预览';self.support.visible=real;self.arm_button.visible=real and not soft_mode
            self.soft_arm_button.visible=soft_mode;self.soft_capture_button.visible=soft_mode
            self.soft_strength.visible=soft_mode
            for handle in (self.selected,self.readout,self.target,self.step,self.nudge,self.previous_next,
                           self.overlay,self.shortcut_help,self.cycle_folder,self.plot_folder,self.range_folder):
                handle.visible=not soft_mode
            self.body_status.visible=soft_mode
            self.soft_capture_button.disabled=not active
            self.sequence_button.disabled=real;self.reset_preview.visible=not real
            self.cycle_button.disabled=bool(soft_mode or (real and not active))
            self.period.visible=not real
            lo,hi=self.limits.bounds(self.sid)
            center=self.control.get('anchor_degrees') if active and not soft_mode else self.targets[self.sid]
            max_amplitude=max(.5,math.floor(max(hi-center,center-lo)*2)/2)
            if self.amplitude.value>max_amplitude:self.amplitude.value=max_amplitude
            self.amplitude.max=max_amplitude
            self.target.disabled=bool(soft_mode or (real and not active))
            if active and not soft_mode:self.target.min,self.target.max=self.control['window_degrees']
            else:self.target.min,self.target.max=lo,hi
            entry=self.control.get('ranges',{}).get(str(self.sid),{})
            limits_match=self.control.get('limits',{}).get('sha256')==self.limits.sha
            can_arm=bool(real and ready and limits_match and self.support.value and not self.token and not self.control.get('active_id') and not self.control.get('soft_active') and not self.control.get('fault'))
            health=(frame or {}).get('health',{})
            power=support_health(health)
            self.arm_button.disabled=not can_arm;self.soft_arm_button.disabled=not (can_arm and power['ok'])
            recording=self.control.get('recording');pending=self.control.get('pending_range')
            self.record_start.disabled=not ready or bool(self.control.get('active_id')) or bool(self.control.get('soft_active')) or bool(recording)
            self.record_finish.disabled=not bool(recording)
            self.confirm.disabled=not (pending and pending['id']==self.sid)
            self.overlay_root.visible=self.overlay.value
            angles={sid:math.radians(q) for sid,q in self.targets.items()}
            with self.server.atomic():
                for link in self.rig.links:self.preview_frames[link.name].wxyz=self.rig.local_quat(link,angles)
                for sid,gizmo in self.gizmos.items():
                    gizmo.visible=not soft_mode and self.overlay.value and sid==self.sid
                    if sid not in self.dragging:gizmo.wxyz=quat_mul(self.gizmo_bases[sid],axis_quat([0,0,1],angles[sid]))
            if now-self.last_plot>.15:
                self.history.append((now,self.targets[self.sid],self.actual[self.sid] if ready else math.nan))
                rows=np.asarray(self.history)
                self.plot.data=(rows[:,0]-rows[0,0],rows[:,1],rows[:,2]);self.last_plot=now
            if now-self.last_info>.2:
                measured=f'{self.actual[self.sid]:.2f}°' if ready else '离线'
                error=f'{self.targets[self.sid]-self.actual[self.sid]:+.2f}°' if ready else '—'
                self.readout.content=f'<div style="display:flex;gap:20px;padding:8px 0"><div>目标<br><b style="color:#0891b2;font-size:22px">{self.targets[self.sid]:.2f}°</b></div><div>实测<br><b style="color:#d97706;font-size:22px">{measured}</b></div><div>误差<br><b style="font-size:22px">{error}</b></div></div>'
                source='**模型预览：实物不会跟随**' if not real else ('**柔和支撑中** · 空格释放' if soft_mode and active else '**实机控制中** · 空格停止 / 关闭力矩' if active else '**实机尚未开启**')
                detail=self.control.get('message','') if real else self.notice
                if real and not active and not self.control.get('fault'):
                    if not limits_match:detail+='\n\n正在等待控制器确认相同的关节限位。'
                    elif not soft_mode or power['ok']:detail+=('\n\n扶稳机身，勾选支撑确认，再点击「开启全身支撑」。' if soft_mode else '\n\n支撑好机身，让当前关节不碰物体；勾选支撑确认，再点击「开启当前关节实机控制」。')
                if real and self.notice.startswith('操作未执行：'):detail+='\n\n'+self.notice
                if soft_mode:
                    detail+='\n\n**供电：MicroDuck 两节电池直供**'
                    if power['voltage_min'] is not None:
                        detail+=f' · {power["voltage_min"]:.1f}–{power["voltage_max"]:.1f} V · 最高 {power["temperature_max"]:g} °C'
                    if power['expected_voltage_alert_ids']:
                        detail+='\n\n已识别直供方案的电压提示；温度、过载及其他故障保护继续生效。'
                    if not power['ok']:
                        detail+='\n\n**暂不能开启**：'+'；'.join(power['issues'][:3])
                self.message.content=f'{source}\n\n{detail}'
                self.limit_info.content=f'**官方模型范围：{lo:g}° ～ {hi:g}°**\n\n按保存的模型零位控制；实机为低速单关节对照。'
                if soft_mode:
                    strength=self.control.get('soft_current_limit_ma',120) if active else int(self.soft_strength.value)
                    state='正在支撑 · 每关节电流上限' if active else '尚未加力 · 支撑预设'
                    following=self.control.get('soft_following_ids',[])
                    follow_text=' · 正在跟随：'+', '.join(map(str,following)) if following else ''
                    self.limit_info.content=f'**{state} {strength} mA**{follow_text}\n\n手推时带阻力跟随，停手约 0.25 秒后保持新位置。全身 14 个关节共用滑条（80–200 mA）；50°C 自动释放。\n\n请持续扶住机身：此模式不能区分手推与重力下坠，也不保持平衡。空格释放全部关节。'
                    torques={int(i):t for i,t in (frame or {}).get('torque',{}).items()}
                    groups=[('右腿',[10,11,12,13,14]),('左腿',[20,21,22,23,24]),('头颈',[30,31,32,33])]
                    known=ready and all(i in torques for _,ids in groups for i in ids)
                    total=sum(bool(torques.get(i)) for _,ids in groups for i in ids)
                    heading=f'全身已加力 **{total}/14**' if known else '等待全身力矩状态'
                    table='\n\n| 部位 | 关节 ID | 已加力 |\n|---|---|---|\n'
                    for name,ids in groups:
                        count=f'{sum(bool(torques.get(i)) for i in ids)}/{len(ids)}' if known else '待确认'
                        table+=f'| {name} | '+', '.join(map(str,ids))+f' | {count} |\n'
                    self.body_status.content=heading+table+'\n嘴部 ID 34 不参与支撑。'
                text=(f'**人工记录**：{entry["degrees"][0]:.2f}° ～ {entry["degrees"][1]:.2f}°' if entry.get('confirmed') else '**尚无人工记录**；可用此处核对实物与模型方向。')
                current=recording or pending
                if current:text+=f'\n\n当前记录 ID {current["id"]}：{current["degrees"][0]:.2f}° ～ {current["degrees"][1]:.2f}°'
                self.range_info.content=text;self.last_info=now
            return {'mode':self.mode.value,'selected_id':self.sid,'target_degrees':self.targets.copy(),
                    'preview_running':bool(self.preview_cycle),'active_id':self.control.get('active_id'),
                    'soft_active':self.control.get('soft_active',False),
                    'limits':self.limits.public(),'session_timeout_seconds':None,
                    'ranges':self.control.get('ranges',{}),'message':self.control.get('message'),
                    'register_write_count':self.control.get('register_write_count',0)}
