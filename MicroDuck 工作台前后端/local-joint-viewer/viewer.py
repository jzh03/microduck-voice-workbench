#!/usr/bin/env python3
"""MicroDuck comparison console: measured pose, draggable targets, bounded control."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

import numpy as np
import trimesh
import viser

from kinematics import CadRig, JOINTS, MODEL_DIR, ROOT, color_for
from console_relay import ControlRelay
from console_ui import JointConsole

OUT=ROOT/'outputs/live-joint-viewer-20261017'
SOFTWARE_REFERENCE=ROOT/'local-joint-viewer/software-zero-reference.json'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True)
    parser.add_argument('--port',type=int,default=8766)
    parser.add_argument('--usb-serial',help='USB ACM console, e.g. /dev/cu.usbmodem1101')
    parser.add_argument('--soft-support',action='store_true',help='Open whole-body support controls without arming')
    parser.add_argument('--reference',action='store_true',help='Start the target model at model zero')
    parser.add_argument('--stop-file',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    profile_sha=hashlib.sha256(SOFTWARE_REFERENCE.read_bytes()).hexdigest()
    OUT.mkdir(parents=True,exist_ok=True)
    rig=CadRig();server=viser.ViserServer(host='127.0.0.1',port=args.port,label='MicroDuck · 关节对照台')
    server.gui.configure_theme(control_layout='collapsible',control_width='large',dark_mode=False,
        show_logo=False,show_share_button=False,brand_color=(13,139,135))
    server.scene.set_up_direction('+z');server.scene.world_axes.visible=False
    server.scene.add_grid('/ground',width=.6,height=.6,cell_size=.025,section_size=.1,
        cell_color=(207,219,222),section_color=(167,188,191),plane_color=(243,247,246),
        plane_opacity=.9,fade_distance=1,shadow_opacity=.14,position=(0,0,.001))
    server.gui.add_markdown('## MicroDuck · 关节对照台\n白色实物姿态 · 青色目标姿态',order=0)
    status=server.gui.add_markdown('正在连接实物…',order=1)
    selected=server.gui.add_dropdown('当前关节',[f'{sid} · {v[1]}' for sid,v in JOINTS.items()],
        initial_value='14 · 右踝',order=2)
    activity=server.gui.add_markdown('点击模型选关节，拖动圆环调整。',order=3)
    with server.gui.add_folder('视角与显示',order=20,expand_by_default=False):
        camera_view=server.gui.add_dropdown('观察方向',['斜前方','正面','右侧面'],initial_value='斜前方')
        camera_reset=server.gui.add_button('恢复默认视角')
        all_labels=server.gui.add_checkbox('显示全部关节 ID',False)
        xray=server.gui.add_checkbox('透视外壳',False)
        server.gui.add_markdown('空白处拖动旋转视角，滚轮缩放。鸭子可平躺，模型保持直立便于观察关节关系。嘴部连杆显示已简化。')
    numbers={}
    with server.gui.add_folder('全部实测角度',order=21,expand_by_default=False):
        for sid,(_,label) in JOINTS.items():numbers[sid]=server.gui.add_number(f'{sid} · {label} / °',0.,disabled=True)
        raw_gui=server.gui.add_number('当前关节原始编码',0,disabled=True,step=1)
    server.gui.add_markdown('软件零位采用已保存的参考姿态。页面操作不修改零位文件。',order=22)

    def set_camera(client):
        client.camera.up_direction=(0.,0.,1.)
        positions={'斜前方':(.34,-.42,.26),'正面':(.52,0.,.143),'右侧面':(.025,-.52,.143)}
        client.camera.position=positions[camera_view.value];client.camera.look_at=(.025,0.,.143)
        client.camera.fov=math.radians(50);client.camera.near=.01;client.camera.far=10.
    server.on_client_connect(set_camera)
    def restore_camera(_):
        for client in server.get_clients().values():set_camera(client)
    camera_reset.on_click(restore_camera);camera_view.on_update(restore_camera)

    paths,frames,markers,labels,axes={},{},{},{},{}
    shells=[];click_meshes=[];mesh_cache={}
    server.scene.add_frame('/duck',show_axes=False)
    for link in rig.links:
        path=(paths[link.parent] if link.parent else '/duck')+'/'+link.name;paths[link.name]=path
        frames[link.name]=server.scene.add_frame(path,show_axes=False,position=tuple(link.position),wxyz=tuple(link.quaternion))
        for i,geom in enumerate(link.geometries):
            if geom.mesh not in mesh_cache:
                mesh_cache[geom.mesh]=trimesh.load_mesh(MODEL_DIR/'assets'/rig.mesh_files[geom.mesh],process=False)
            mesh=mesh_cache[geom.mesh]
            handle=server.scene.add_mesh_simple(path+f'/mesh-{i}',vertices=np.asarray(mesh.vertices,dtype=np.float32),
                faces=np.asarray(mesh.faces,dtype=np.uint32),color=color_for(geom.mesh),position=tuple(geom.position),
                wxyz=tuple(geom.quaternion),material='standard',flat_shading=False,side='double')
            if geom.mesh in ('top_head_shell','left_shell','right_shell','upper_leg_left','upper_leg_right'):shells.append(handle)
            sid=geom.motor_id or link.joint_id
            if sid is not None:click_meshes.append((handle,sid))
        if link.joint_id is not None:
            sid=link.joint_id
            markers[sid]=server.scene.add_icosphere(path+'/joint-marker',radius=.0025,color=(20,157,151),subdivisions=1,cast_shadow=False)
            labels[sid]=server.scene.add_label(path+'/joint-label',f'{sid} · {JOINTS[sid][1]}',position=(.002,.002,.009),font_screen_scale=.8,visible=sid==14)
            axes[sid]=server.scene.add_frame(path+'/joint-axis',axes_length=.018,axes_radius=.0005,origin_radius=.001,visible=sid==14)
            click_meshes.append((markers[sid],sid))
    relay=ControlRelay(args.host,usb_serial=args.usb_serial)
    console=JointConsole(server,rig,relay,selected,mesh_cache,profile_sha)
    if args.soft_support:
        console.mode.value='柔和支撑摆姿态'
    for handle,sid in click_meshes:console.attach_mesh(handle,sid)
    if args.reference:console.initialized=True
    stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    actual={sid:0. for sid in JOINTS};last_frame=None;last_actual=actual.copy()
    last_save=0;last_gui=0;last_xray=None
    try:
        while not stop.is_set() and not (args.stop_file and args.stop_file.exists()):
            now=time.monotonic();frame,received,hz,message=relay.snapshot()
            mapped={int(k):v for k,v in (frame or {}).get('joint_angles_rad',{}).items()}
            ready=bool(frame and now-received<.5 and frame.get('profile_sha256')==profile_sha and
                       set(mapped)==set(JOINTS) and frame.get('metadata_age_s',1000)<1.5)
            if ready:actual=mapped
            snapshot=console.tick(actual,frame,received,ready);sid=snapshot['selected_id']
            whole_body=snapshot['mode']=='柔和支撑摆姿态'
            with server.atomic():
                for joint,link in rig.joint_links.items():
                    frames[link.name].wxyz=rig.local_quat(link,actual)
                    labels[joint].visible=all_labels.value or (not whole_body and joint==sid)
                    axes[joint].visible=not whole_body and joint==sid
                    markers[joint].color=(243,160,36) if not whole_body and joint==sid else (135,160,164)
                if xray.value!=last_xray:
                    for mesh in shells:mesh.opacity=.18 if xray.value else 1.
                    last_xray=xray.value
            if now-last_gui>.2:
                torque=(frame or {}).get('torque',{})
                torque_known=len(torque)==15 and ready
                active_id=snapshot['active_id']
                if not ready:
                    status.content='**实物读数未就绪 · 模型预览可用**\n\n'+message
                else:
                    torque_text='14 个关节柔和支撑中' if snapshot.get('soft_active') else f'ID {active_id} 力矩开启' if active_id else ('全部力矩关闭' if torque_known and not any(torque.values()) else '力矩状态需确认')
                    transport='USB 数据线' if args.usb_serial else 'Wi-Fi'
                    status.content=f'**实物在线 15 / 15** · {transport} · {hz:.0f} Hz · {torque_text}'
                    if active_id:status.content+='\n\n当前关节正在带电控制，请勿手动掰动。'
                changes=sorted(JOINTS,key=lambda i:abs(actual[i]-last_actual[i]),reverse=True)
                activity.content=(f'**正在变化**：{changes[0]} · {JOINTS[changes[0]][1]}' if abs(actual[changes[0]]-last_actual[changes[0]])>math.radians(.2) else '点击模型选关节 · 圆环 / W S 调整目标')
                activity.visible=not whole_body;raw_gui.visible=not whole_body
                for joint,number in numbers.items():number.value=round(math.degrees(actual[joint]),2)
                raw=(frame or {}).get('positions',{}).get(str(sid))
                if raw is not None:raw_gui.value=raw
                last_actual=actual.copy();last_gui=now
            if now-last_save>.5:
                state={'unix_time':time.time(),'source':'software-zero-live' if ready else 'frozen',
                    'transport':'USB 数据线' if args.usb_serial else 'Wi-Fi',
                    'view':camera_view.value,'online_ids':list(JOINTS) if ready else [],'hz':hz,
                    'frame_age_s':now-received if received else None,'selected_id':sid,
                    'raw_positions':(frame or {}).get('positions',{}),
                    'logical_degrees':{i:math.degrees(q) for i,q in actual.items()} if ready else {},
                    'displayed_joint_degrees':{i:math.degrees(q) for i,q in actual.items()},
                    'software_zero_data_ready':ready,'software_reference_sha256':profile_sha,
                    'powered_motion_verified_ids':[14],'console':snapshot,
                    'torque':(frame or {}).get('torque',{}),
                    'health':(frame or {}).get('health',{}),
                    'registers_written':snapshot['register_write_count']>0,
                    'motion_commanded':bool(active_id or snapshot.get('soft_active'))}
                temp=OUT/'state.tmp';temp.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n');temp.replace(OUT/'state.json');last_save=now
            time.sleep(.035)
    finally:
        console.stop();relay.stop();server.stop()


if __name__=='__main__':main()
