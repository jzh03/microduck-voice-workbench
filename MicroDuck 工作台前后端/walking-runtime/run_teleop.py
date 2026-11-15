#!/usr/bin/env python3
"""On-board, explicitly started manual balance/walk session; never starts at boot."""
import argparse
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

from hardware_bridge import Hardware, WireBus, Endpoint, ROOT, LOCAL, IDS
from policy_coordinates import CONTRACT, HOME
from run_bridge import rpc, ORT
from teleop_state import TeleopState
from stance_reference import StanceReference, PolicyReferenceHardware, ready_for_handoff
from handoff_pose import HandoffPose, PreparedPoseHardware, gravity_angles as gravity_angles_for_preparation
from flight_recorder import FlightRecorder,emit


def verify_native_runtime(contract, binary):
    override=contract.get('native_home_override')
    if override is None:return
    expected=override.get('binary_sha256')
    if not isinstance(expected,str) or len(expected)!=64:
        raise RuntimeError('Native HOME binary binding is incomplete')
    path=Path(binary)
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise RuntimeError('Native HOME runtime differs from the configured binary')


def verify_files(certificate, *, physical):
    deployment=json.loads((ROOT/'bridge-deployment.json').read_text())
    for name,want in deployment['sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=want:
            raise RuntimeError('Deployment differs: '+name)
    profile=hashlib.sha256((LOCAL/'software-zero-reference.json').read_bytes()).hexdigest()
    if profile!=deployment['profile_sha256']:raise RuntimeError('Calibration changed')
    if hashlib.sha256(Path('/opt/robot/policies/current/velstand.onnx').read_bytes()).hexdigest()!=CONTRACT['policy_sha256']:
        raise RuntimeError('Policy differs from reviewed model')
    if physical:
        verify_native_runtime(CONTRACT,Path('/opt/robot/daemon/current/bin/robotd'))
        evidence=json.loads(certificate.read_text())
        expected=hashlib.sha256((ROOT/'runtime-contract.json').read_bytes()).hexdigest()
        if (evidence.get('verified') is not True or evidence.get('profile_sha256')!=profile or
                evidence.get('contract_sha256')!=expected or evidence.get('register_write_packets')!=0 or
                set(evidence.get('stages',{}))!={'upright','forward','right','left_turn'} or
                not all(s.get('passed') is True for s in evidence['stages'].values())):
            raise RuntimeError('IMU direction verification is missing or does not match this configuration')
    return profile


class Session:
    def __init__(self, args):
        self.args=args
        self.commands=TeleopState(step_speed=getattr(args,'step_speed',.05))
        self.lock=threading.RLock()
        self.finished=threading.Event()
        self.close_requested=threading.Event()
        self.retry_requested=threading.Event()
        self.state={'phase':'waiting','message':'等待控制页连接，尚未加力','running':True,
                    'mode':args.mode,'policy':None,'can_confirm_balance':False}
        self.native=None
        self.trace=None
        self.native_time=0.
        self.last_native_recovery=None
        self.samples=deque(maxlen=1200)
        self.report={'mode':args.mode,'success':False,'independent_standing_confirmed_by_operator':False,
                     'walking_observed':False,'nonzero_command_updates':0}

    def update(self, **values):
        with self.lock:self.state.update(values)

    def public(self):
        with self.lock:return {**self.state,**self.commands.public()}

    def observe_native_recovery(self, native):
        safety=(native or {}).get('safety',{})
        flags=(bool(safety.get('fallen')),bool(safety.get('limp')))
        if flags!=self.last_native_recovery:
            emit(self.trace,'controller_recovery_changed',fallen=flags[0],limp=flags[1],
                 gravity=safety.get('gravity'),response='zero_motion_keep_native_controller')
            self.last_native_recovery=flags
        # Fallen is a native report; limp-fall is a native recovery sequence.
        # Relaxing here aborts that sequence. Hardware faults and explicit
        # operator stops still terminate the session independently.
        self.commands.observe_recovery(any(flags))
        return any(flags)

    def request_retry(self,supported):
        with self.lock:
            if (supported is not True or not self.state.get('running') or
                    not self.state.get('healthy') or self.state.get('policy')!='held' or
                    not self.state.get('can_retry_handoff')):
                raise ValueError('扶稳并等待回到保存站姿后，才能保持加力重试')
            self.state['can_retry_handoff']=False
        with self.commands.lock:
            if self.commands.stopped:raise ValueError('Stopped sessions cannot retry')
            self.commands.ready=False;self.commands.unlocked=False
            self.commands.twist=(0.,0.,0.);self.commands.pulse_until=None
            self.commands.last_seen=self.commands.clock()
        self.retry_requested.set()

    def subscribe(self, path):
        def run():
            try:
                with socket.socket(socket.AF_UNIX) as client:
                    # A cold model load can take over a second before the first
                    # telemetry frame. Torque remains off until that frame arrives.
                    client.settimeout(10.)
                    client.connect(str(path))
                    client.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":10}}\n')
                    with client.makefile('r') as stream:
                        for line in stream:
                            if self.finished.is_set():break
                            event=json.loads(line)
                            if event.get('method')=='robot.state':
                                emit(self.trace,'native_state',state=event['params'])
                                with self.lock:
                                    self.native=event['params'];self.native_time=time.monotonic()
                                client.settimeout(1.)
            except Exception as exc:
                if not self.finished.is_set():self.update(telemetry_error=str(exc))
        thread=threading.Thread(target=run,daemon=True);thread.start()
        return thread

    def run(self):
        args=self.args;bus=hardware=endpoint=daemon=None;log=None
        socket_path=args.output_dir/'robotd.sock'
        started=None;reference_endpoint=None
        try:
            trace_path=ROOT/'diagnostics'/args.output_dir.name/'flight.jsonl'
            self.trace=FlightRecorder(trace_path)
            self.report['diagnostic_log']=str(trace_path)
            emit(self.trace,'session_started',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip() if args.mode=='live' else 'fake',
                 uptime=Path('/proc/uptime').read_text().strip() if args.mode=='live' else None,
                 session=str(args.output_dir),mode=args.mode,joint_ids=list(IDS),
                 policy_sha256=CONTRACT['policy_sha256'],profile_sha256=hashlib.sha256((LOCAL/'software-zero-reference.json').read_bytes()).hexdigest(),
                 supported_direct_handoff=bool(getattr(args,'supported_direct_handoff',False)))
            self.report['profile_sha256']=verify_files(args.certificate,physical=args.mode=='live')
            reference=None
            if getattr(args,'reference',None):
                reference=StanceReference.load(args.reference,self.report['profile_sha256'],
                    hashlib.sha256((ROOT/'runtime-contract.json').read_bytes()).hexdigest())
                self.report['standing_reference']=reference.value
            preparation=None
            if getattr(args,'handoff_pose',None):
                if reference is None:raise RuntimeError('Preparation pose requires the saved standing reference')
                preparation=HandoffPose.load(args.handoff_pose,reference,args.reference)
                self.report['supported_handoff_pose']=preparation.value
            direct_trial=bool(getattr(args,'supported_direct_handoff',False))
            if direct_trial and (reference is None or preparation is not None or not getattr(args,'supported',False)):
                raise RuntimeError('Direct model trial requires explicit physical support and the saved stance, without preparation mode')
            self.report['supported_direct_handoff']=direct_trial
            floor_recovery=bool(getattr(args,'floor_recovery',False))
            if floor_recovery and (reference is None or preparation is not None or not getattr(args,'supported',False)):
                raise RuntimeError('Floor recovery needs explicit authorization and the saved reference, without preparation mode')
            self.report['floor_recovery']=floor_recovery
            if floor_recovery:
                stand_sha=hashlib.sha256(Path('/opt/robot/policies/current/alpha_stand.onnx').read_bytes()).hexdigest()
                if stand_sha!='1569268713e40deea795dd2922dba50d3621e15a872855408b6b1b125b1c094b':
                    raise RuntimeError('Official v8 standing expert differs from the reviewed model')
                self.report['floor_policy']='alpha_stand v8 (official zero-speed standing slot)'
                self.report['floor_policy_sha256']=stand_sha
            emit(self.trace,'startup_orientation_mode',floor_recovery=floor_recovery,
                 policy='alpha_stand v8 zero-speed recovery' if floor_recovery else 'velstand v8',walking_command_allowed=False)
            deadline=time.monotonic()+12
            while self.commands.last_seen is None:
                if self.commands.abort_reason() or time.monotonic()>deadline:raise RuntimeError('Control page did not connect')
                time.sleep(.02)
            if args.mode=='live':
                self.update(phase='preparing',message='检查并准备关节；请扶稳机身和腿部')
                bus=WireBus('/dev/ttyS2',baud=1000000,timeout=.025,live=True)
                hardware=Hardware(bus,(LOCAL/'software-zero-reference.json').read_bytes(),
                                  LOCAL/'runtime-joint-limits.json',mode='live',
                                  journal=ROOT/'bridge-recovery.json',cancelled=self.commands.abort_reason,recorder=self.trace,
                                  allow_floor_start=floor_recovery,hold_mouth_closed=True,
                                  policy_target_bias=reference.bias if reference else [0.]*len(IDS))
                hardware.start();self.report['homing_before']=hardware.homing.copy()
                hardware.prepare()
                reference_endpoint=(PreparedPoseHardware(hardware,reference,preparation) if preparation else
                                    PolicyReferenceHardware(hardware,reference,record_targets_only=direct_trial) if reference else None)
                endpoint=Endpoint(reference_endpoint if reference_endpoint else hardware);endpoint.start()
            argv=['/opt/robot/daemon/current/bin/robotd']
            argv+=['--fake'] if args.mode=='fake' else ['--sim',f'127.0.0.1:{endpoint.port}']
            argv+=['--params',str(ROOT/('ground-robotd.toml' if floor_recovery else 'bridge-robotd.toml')),'--socket',str(socket_path)]
            # /run disappears on a host restart; preserve the controller's stderr.
            controller_log=trace_path.parent/'robotd.log'
            self.report['controller_log']=str(controller_log)
            log=controller_log.open('w',buffering=1)
            daemon=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                                    env={**os.environ,'ORT_DYLIB_PATH':ORT,'RUST_LOG':'warn'})
            emit(self.trace,'controller_spawned',pid=daemon.pid,argv=argv,log=str(controller_log))
            deadline=time.monotonic()+10
            while not socket_path.exists():
                if daemon.poll() is not None or time.monotonic()>deadline or self.commands.abort_reason():
                    raise RuntimeError(self.commands.abort_reason() or 'Official controller failed to start')
                time.sleep(.02)
            for fd in Path(f'/proc/{daemon.pid}/fd').iterdir():
                try:target=os.readlink(fd)
                except FileNotFoundError:continue
                if '/dev/tty' in target:raise RuntimeError('Official controller unexpectedly opened a serial port')
            self.subscribe(socket_path)
            deadline=time.monotonic()+10
            while self.native is None:
                reason=self.commands.abort_reason()
                if reason or daemon.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError(reason or 'Controller telemetry unavailable before torque-on')
                if hardware and hardware.fault:raise RuntimeError(hardware.fault)
                time.sleep(.02)
            disabled=rpc(socket_path,'robot.enable',{'on':False})
            if disabled.get('accepted') is not True:raise RuntimeError(str(disabled))
            initialized=rpc(socket_path,'robot.init')
            if initialized.get('accepted') is not True:raise RuntimeError(str(initialized))
            self.report['init']=initialized
            policy_enabled=False;home_settled_since=None;handoff_blocked=False;bridge_prepared=False
            bridge_started=None
            started=time.monotonic();last_health=0.;healthy_since=None;unhealthy_since=None
            health={};last_sample=0.;last_pitch=0.;last_pose=None
            while not self.commands.abort_reason():
                now=time.monotonic()
                if daemon.poll() is not None:
                    code=daemon.returncode
                    emit(self.trace,'controller_exited',pid=daemon.pid,returncode=code,
                         terminating_signal=-code if code<0 else None,requested_by_session=False)
                if daemon.poll() is not None or (hardware and hardware.fault):
                    raise RuntimeError(hardware.fault if hardware and hardware.fault else 'Controller exited')
                if now-started>2 and now-self.native_time>.6:
                    raise RuntimeError('Controller telemetry stopped')
                if self.retry_requested.is_set():
                    self.retry_requested.clear()
                    try:reference_endpoint.retry_bridge()
                    except ValueError as exc:
                        # An operator moving while a queued click arrives is a
                        # rejected request, not a reason to drop the saved hold.
                        self.report['last_retry_rejection']=str(exc)
                    else:
                        handoff_blocked=False;policy_enabled=False;bridge_prepared=True
                        bridge_started=now
                        home_settled_since=None;healthy_since=None
                        self.report['supported_retry_count']=self.report.get('supported_retry_count',0)+1
                with self.lock:native=self.native
                recovering=self.observe_native_recovery(native)
                command=self.commands.command()
                rpc(socket_path,'robot.move',dict(zip(('vx','vy','vyaw'),command)))
                pose=self.commands.pose()
                if pose!=last_pose:
                    pose_result=rpc(socket_path,'robot.pose',pose)
                    if pose_result.get('accepted') is not True:
                        raise RuntimeError('Controller rejected standing pose: '+str(pose_result))
                    last_pose=pose;last_pitch=pose['pitch'];healthy_since=None
                if any(command):self.report['nonzero_command_updates']+=1
                if now-last_health>=.5:
                    health=rpc(socket_path,'robot.health');last_health=now
                gravity=(hardware.frame['imu']['gravity'] if hardware and hardware.frame else
                         (native or {}).get('safety',{}).get('gravity',[0.,0.,-1.]))
                tilt=math.degrees(math.acos(max(-1.,min(1.,-gravity[2]))))
                policy=(native or {}).get('policy')
                hz=health.get('control_loop',{}).get('achieved_hz') or 0.
                healthy=bool(health.get('healthy')) and hz>=45
                if preparation is not None and bridge_prepared and not policy_enabled and not handoff_blocked:
                    if tilt>15:
                        reference_endpoint.reject_preparation('Measured tilt exceeded the supported preparation range')
                    elif bridge_started is not None and now-bridge_started>20 and not preparation.matches_gravity(gravity):
                        reference_endpoint.reject_preparation('Preparation pose did not match measured trunk orientation')
                if reference_endpoint and reference_endpoint.handoff_rejection is not None and not handoff_blocked:
                    result=rpc(socket_path,'robot.enable',{'on':False})
                    if result.get('accepted') is not True:raise RuntimeError('Could not return to saved stance hold')
                    handoff_blocked=True;policy_enabled=False
                    self.report['handoff_rejection']=reference_endpoint.handoff_rejection
                ready_now=healthy and policy in ('walk','stand') and not handoff_blocked and tilt<12 and not recovering
                if preparation is not None:
                    ready_now=ready_now and reference_endpoint.first_policy_target_checked
                if not policy_enabled:
                    expected=preparation.pose if bridge_prepared else reference.pose if reference else HOME
                    measured=hardware.frame['positions'] if hardware else list(HOME)
                    home_ready=healthy and ready_for_handoff(measured,expected,gravity,hz,policy,floor_recovery=floor_recovery)
                    if bridge_prepared:
                        home_ready=home_ready and reference_endpoint.bridge_complete and preparation.matches_gravity(gravity)
                    home_settled_since=(home_settled_since if home_settled_since is not None else now) if home_ready else None
                    if not handoff_blocked and home_settled_since is not None and now-home_settled_since>=.5:
                        if preparation is not None and not bridge_prepared:
                            reference_endpoint.prepare_bridge();bridge_prepared=True;home_settled_since=None;bridge_started=now
                            self.report['preparation_started_s']=now-started
                        else:
                            if reference_endpoint:reference_endpoint.begin_handoff()
                            emit(self.trace,'policy_enable_requested',gravity=gravity,measured=measured,expected=expected)
                            enabled=rpc(socket_path,'robot.enable',{'on':True})
                            if enabled.get('accepted') is not True:raise RuntimeError(str(enabled))
                            self.report['enable']=enabled;policy_enabled=True
                            self.report['handoff_after_measured_settle_s']=now-started
                            healthy_since=None
                    # Not meeting a readiness gate is not a motor fault. Keep
                    # holding the saved pose; a timeout must not drop a standing
                    # robot. Sensor, electrical, fall and operator-stop guards
                    # remain active independently.
                healthy_since=(healthy_since or now) if ready_now else None
                self.commands.set_ready_for_pose(last_pose,bool(healthy_since and now-healthy_since>=3))
                if now-started>5 and not healthy:
                    unhealthy_since=unhealthy_since or now
                    if now-unhealthy_since>2:raise RuntimeError('Control loop is not healthy enough for balance')
                else:unhealthy_since=None
                phase='walking' if any(command) else ('balancing' if policy in ('walk','stand') else 'homing')
                if bridge_prepared and not policy_enabled and not handoff_blocked:phase='preparing_pose'
                if self.commands.unlocked:message='方向控制已启用；松键停步，继续平衡'
                elif last_pitch or pose['z']:message='站姿对照测试中，请继续扶稳；俯仰和高度均归零并站稳后才能启用行走'
                elif self.commands.ready:message='零速平衡运行中；逐渐减轻扶持，双手在旁保护。站稳后确认启用方向'
                else:message='正在调整姿态并运行平衡；请继续扶稳'
                if not policy_enabled:
                    tracking=max(abs(math.degrees(a-b)) for a,b in zip(measured,expected))
                    message=(f'保持保存站姿，等待接管：倾斜 {tilt:.1f}°（需≤12°），'
                             f'关节跟随误差 {tracking:.1f}°（需≤6°）；未达标只保持，不因等待超时卸力')
                    if floor_recovery:
                        message='地面起身准备中：等待原生初始化与控制循环就绪；模型将读取实际地面姿态'
                if handoff_blocked:
                    rejected=reference_endpoint.handoff_rejection
                    message=(f'策略接管突跳已拦截（ID {rejected["joint_id"]}，{rejected["error_degrees"]:.1f}°）；'
                             '正在保持保存站姿，行走未启用。可返回站姿微调')
                    if preparation is not None:
                        message=(f'接管未通过（ID {rejected["joint_id"]}，{rejected["error_degrees"]:.1f}°）；'
                                 '正在平滑返回保存站姿，保持加力，行走未启用')
                        if rejected.get('reason'):
                            message+='；'+rejected['reason']
                elif bridge_prepared and not policy_enabled:
                    actual_pitch,actual_roll=gravity_angles_for_preparation(gravity)
                    message=(f'扶稳，正在准备行走接管姿态：俯仰 {actual_pitch:.1f}°，目标 {preparation.pitch:.1f}°；'
                             '实测关节与姿态达标后才接管，当前保持加力')
                if recovering:
                    message='控制器正在处理倾倒／恢复姿态；前后移动已暂停'
                elif self.commands.recovery_confirmation_required:
                    message=('姿态已恢复，请重新确认独立站稳后再前后移动' if self.commands.ready else
                             '等待恢复稳定平衡；前后移动保持暂停')
                values={'phase':phase,'message':message,'policy':policy,'control_hz':hz,'healthy':healthy,
                        'diagnostic_log':str(self.trace.path) if self.trace else None,
                        'diagnostic_log_error':self.trace.error if self.trace else 'Recorder unavailable',
                        'diagnostic_dropped_events':self.trace.dropped if self.trace else 0,
                        'supported_direct_handoff':direct_trial,
                        'floor_recovery':floor_recovery,
                        'native_recovery_active':recovering,
                        'native_fallen':bool((native or {}).get('safety',{}).get('fallen')),
                        'native_limp':bool((native or {}).get('safety',{}).get('limp')),
                        'handoff_target_diagnostic':reference_endpoint.handoff_diagnostic if reference_endpoint else None,
                        'can_retry_handoff':bool(preparation and handoff_blocked and healthy and policy=='held' and reference_endpoint.can_retry),
                        'tilt_deg':tilt,'elapsed_s':now-started,
                        'pitch_deg':math.degrees(math.atan2(gravity[0],math.hypot(gravity[1],gravity[2]))),
                        'roll_deg':math.degrees(math.atan2(-gravity[1],-gravity[2])),
                        'stand_pitch_deg':math.degrees(last_pitch),
                        'stand_height_mm':pose['z']*1000.,
                        'volts_min':hardware.minimum_volts if hardware else health.get('battery',{}).get('volts'),
                        'temperature_max':max(hardware.slow_frame['temps_c']) if hardware else health.get('motors',{}).get('max_c')}
                log_expected=preparation.pose if bridge_prepared else reference.pose if reference else HOME
                log_actual=hardware.frame['positions'] if hardware else list(HOME)
                values['handoff_gates']={'policy_enable_requested':policy_enabled,'blocked':handoff_blocked,
                    'floor_recovery':floor_recovery,
                    'tilt_deg':tilt,'tilt_limit_deg':None if floor_recovery else 12.,'tilt_passes':floor_recovery or tilt<=12.,
                    'tracking_max_deg':max(abs(math.degrees(a-b)) for a,b in zip(log_actual,log_expected)),
                    'tracking_limit_deg':None if floor_recovery else 6.,'healthy':healthy,'native_policy':policy}
                self.update(**values)
                emit(self.trace,'control_state',**values,command=command,
                     torque_readback=hardware.last_torque_readback if hardware else {},
                     hardware_errors=hardware.last_hardware_errors if hardware else {})
                if now-last_sample>=.1:
                    sample={**values,'command':command,
                                         'joint_degrees':{str(sid):math.degrees(angle) for sid,angle in zip(IDS,hardware.frame['positions'])} if hardware else {},
                                         'gravity':list(gravity),
                                         'joint_currents_ma':dict(zip(map(str,IDS),hardware.frame['currents_ma'])) if hardware else {},
                                         'requested_joint_degrees':dict(zip(map(str,IDS),map(math.degrees,hardware.last_requested_targets))) if hardware and hardware.last_requested_targets else {},
                                         'commanded_joint_degrees':dict(zip(map(str,IDS),map(math.degrees,hardware.last_commanded_targets))) if hardware and hardware.last_commanded_targets else {},
                                         'max_current_ma':max(hardware.frame['currents_ma']) if hardware else 0,
                                         'read_retries':bus.read_retries if bus else 0}
                    self.samples.append(sample)
                    self.update(diagnostics=sample)
                    last_sample=now
                if args.test_seconds and now-started>=args.test_seconds:
                    self.commands.receive({'action':'off'})
                time.sleep(.05)
            reason=self.commands.abort_reason()
            if reason!='Operator requested torque-off':raise RuntimeError(reason)
            self.report['success']=True
        except BaseException as exc:
            self.report['error']=str(exc) or type(exc).__name__
            emit(self.trace,'session_exception',reason=self.report['error'],exception_type=type(exc).__name__)
        finally:
            emit(self.trace,'session_cleanup_started',reason=self.report.get('error') or self.commands.abort_reason(),
                 hardware_failure=hardware.failure_cause if hardware else None)
            self.finished.set();self.commands.ready=False
            self.update(phase='stopping',message='正在卸力并恢复配置；请托住机身')
            if daemon and daemon.poll() is None:
                try:rpc(socket_path,'robot.move',{'vx':0,'vy':0,'vyaw':0});rpc(socket_path,'robot.relax')
                except Exception:pass
                daemon.terminate()
                try:daemon.wait(timeout=5)
                except subprocess.TimeoutExpired:daemon.kill();daemon.wait()
                emit(self.trace,'controller_exited',pid=daemon.pid,returncode=daemon.returncode,
                     terminating_signal=-daemon.returncode if daemon.returncode<0 else None,requested_by_session=True)
            if endpoint:endpoint.close()
            elif hardware:hardware.stop()
            if hardware:
                self.report['motor_frame_failure']=hardware.first_motor_frame_failure
                self.report['imu_frame_failure']=getattr(hardware,'first_imu_frame_failure',None)
                self.report['handoff_target_diagnostic']=reference_endpoint.handoff_diagnostic if reference_endpoint else None
                try:
                    self.report['hardware_failure_cause']=hardware.failure_cause
                    self.report['torque_after']=hardware.torque()
                    self.report['homing_after']={sid:hardware.read_reg(sid,20,4,True) for sid in IDS}
                    self.report['cleanup']=hardware.cleanup
                    self.report['register_write_packets']=bus.write_packets
                    self.report['read_retries']=bus.read_retries
                    self.report['read_retries_by_operation']=bus.read_retries_by_operation
                    self.report['target_frames']=hardware.target_frames
                    self.report['sensor_frames']=hardware.reads
                    self.report['nominal_limit_clamps']=hardware.clamps
                    self.report['prepare_pose_change_deg']=hardware.prepare_pose_change_deg
                    self.report['last_sensor_frame']=hardware.frame
                    self.report['last_requested_targets']=hardware.last_requested_targets
                    self.report['last_commanded_targets']=hardware.last_commanded_targets
                    if any(self.report['torque_after'].values()) or (hardware.cleanup and not hardware.cleanup['restored']):
                        self.report['success']=False
                        self.report['cleanup_error']='Torque-off or restoration not confirmed'
                        self.report.setdefault('error',self.report['cleanup_error'])
                except Exception as exc:
                    self.report['success']=False;self.report['cleanup_error']=str(exc)
                    self.report.setdefault('error',str(exc))
            elif args.mode=='fake':self.report['torque_after']={str(sid):0 for sid in IDS}
            if bus:bus.close()
            if log:log.close()
            self.report['independent_standing_confirmed_by_operator']=args.mode=='live' and self.commands.unlocked
            self.report['samples']=list(self.samples)
            self.report['elapsed_s']=time.monotonic()-started if started else 0
            self.report['finished_unix']=time.time()
            (args.output_dir/'result.json').write_text(json.dumps(self.report,indent=2)+'\n')
            durable_result=ROOT/'diagnostics'/args.output_dir.name/'result.json'
            with durable_result.open('w') as stream:
                json.dump(self.report,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
            emit(self.trace,'session_finished',report=self.report)
            if self.trace:self.trace.close()
            confirmed=len(self.report.get('torque_after',{}))==15 and not any(self.report['torque_after'].values())
            unchanged=hardware is not None and bus.write_packets==0
            restored=confirmed and (hardware is None or unchanged or bool(hardware.cleanup and hardware.cleanup.get('restored')))
            message=('已卸力，配置未改动' if confirmed and unchanged else '已卸力，配置已恢复' if restored else
                     '已卸力；配置恢复尚未确认' if confirmed else '停止后状态未确认，请继续托稳')
            if self.report.get('error'):message+='；'+self.report['error']
            self.update(phase='complete' if self.report['success'] else 'error',message=message,running=False,report=self.report)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('live','fake'),required=True)
    parser.add_argument('--supported',action='store_true')
    parser.add_argument('--floor-recovery',action='store_true',help='Explicitly authorized ground start; keep native v8 get-up and all physical joint/electrical guards')
    parser.add_argument('--certificate',type=Path)
    parser.add_argument('--reference',type=Path)
    handoff=parser.add_mutually_exclusive_group()
    handoff.add_argument('--handoff-pose',type=Path,help='Opt-in supported preparation pose; not automatically enabled')
    handoff.add_argument('--supported-direct-handoff',action='store_true',help='Supported trial: record target gaps without the extra twelve-degree refusal; hardware guards remain active')
    parser.add_argument('--step-speed',type=float,choices=(.05,.1,.15,.2,.3),default=.05)
    parser.add_argument('--token-file',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--port',type=int,default=18769)
    parser.add_argument('--test-seconds',type=float,help='Only for fake verification')
    args=parser.parse_args()
    if args.mode=='live' and (not args.supported or not args.certificate or args.test_seconds):
        parser.error('Live requires physical support and IMU evidence; no timed torque-off')
    if args.handoff_pose and args.mode!='live':
        parser.error('Preparation pose requires measured physical feedback; use the offline physics rehearsal for simulation')
    token=args.token_file.read_text().strip()
    if len(token)<32:parser.error('Invalid control token')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    session=Session(args)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_POST(self):
            if self.headers.get('X-Control-Token')!=token:self.send_error(403);return
            size=int(self.headers.get('Content-Length','0'))
            if not 0<=size<=1000:self.send_error(413);return
            try:
                value=json.loads(self.rfile.read(size) or b'{}')
                if not isinstance(value,dict):raise ValueError('Command must be an object')
                if self.path=='/command':
                    if value.get('action') not in ('heartbeat','drive') or any(value.get(k,0) for k in ('vx','vy','vyaw')):
                        emit(session.trace,'operator_command',command={k:v for k,v in value.items() if k in ('action','supported','vx','vy','vyaw','seconds','degrees','millimeters','sequence')})
                    if value.get('action')=='retry_handoff':session.request_retry(value.get('supported'))
                    else:session.commands.receive(value)
                elif self.path=='/close':
                    if session.state['running']:raise ValueError('Session still running')
                    session.close_requested.set()
                elif self.path!='/status':raise ValueError('Unknown endpoint')
                response=session.public();status=200
            except (ValueError,TypeError) as exc:response={'error':str(exc)};status=400
            body=json.dumps(response,ensure_ascii=False).encode()
            self.send_response(status);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    def process_signal(signum, _frame):
        emit(session.trace,'process_signal',signum=signum,signal_name=signal.Signals(signum).name)
        session.commands.receive({'action':'off'})
    signal.signal(signal.SIGTERM,process_signal)
    signal.signal(signal.SIGINT,process_signal)
    session.run()
    session.close_requested.wait(60)
    server.shutdown()


if __name__=='__main__':main()
