"""CAD joint hierarchy and encoder math shared by the live viewer and checks."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = ROOT / 'official-microduck-rl/src/mjlab_microduck/robot/microduck'
MODEL_XML = MODEL_DIR / 'robot_walk.xml'

JOINTS = {
    10: ('right_hip_yaw', '右髋偏航'), 11: ('right_hip_roll', '右髋横滚'),
    12: ('right_hip_pitch', '右髋俯仰'), 13: ('right_knee', '右膝'),
    14: ('right_ankle', '右踝'), 20: ('left_hip_yaw', '左髋偏航'),
    21: ('left_hip_roll', '左髋横滚'), 22: ('left_hip_pitch', '左髋俯仰'),
    23: ('left_knee', '左膝'), 24: ('left_ankle', '左踝'),
    30: ('neck_pitch', '颈俯仰'), 31: ('head_pitch', '头俯仰'),
    32: ('head_yaw', '头偏航'), 33: ('head_roll', '头横滚'), 34: ('mouth', '嘴部'),
}
BY_NAME = {v[0]: k for k,v in JOINTS.items()}
HOME = {10: 0, 11: .0873, 12: .4579, 13: .0049, 14: -.4530,
        20: 0, 21: -.0873, 22: -.4579, 23: -.0049, 24: .4530,
        30: .3491, 31: .3491, 32: 0, 33: 0, 34: 0}
PART_IDS = {'xl330':20, 'xl330_2':10, 'xl330_3':21, 'xl330_4':23,
            'xl330_5':22, 'xl330_6':24, 'xl330_7':30, 'xl330_8':31,
            'xl330_9':32, 'xl330_10':34, 'xl330_11':33, 'xl330_12':11,
            'xl330_13':13, 'xl330_14':12, 'xl330_15':14}


def vector(text, default='0 0 0'):
    return np.array([float(v) for v in (text or default).split()], dtype=float)


def quat_mul(a, b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    return np.array([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw])


def quat_matrix(q):
    w,x,y,z = np.array(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def axis_quat(axis, angle):
    axis = np.array(axis)/np.linalg.norm(axis)
    return np.concatenate(([math.cos(angle/2)], axis*math.sin(angle/2)))


def transform(position, quaternion):
    out = np.eye(4)
    out[:3,:3] = quat_matrix(quaternion)
    out[:3,3] = position
    return out


class EncoderTracker:
    """Track real relative rotation, including firmware jumps by one encoder turn."""
    def __init__(self):
        self.raw = {}
        self.unwrapped = {}
        self.reference = {}

    def update(self, values):
        for key, value in values.items():
            sid = int(key)
            if sid not in JOINTS:
                continue
            value = int(value)
            if sid not in self.raw:
                self.unwrapped[sid] = float(value)
                self.reference[sid] = float(value)
            else:
                delta = (value-self.raw[sid]+2048) % 4096-2048
                self.unwrapped[sid] += delta
            self.raw[sid] = value
        return self.relative()

    def relative(self):
        return {sid: (value-self.reference[sid])*math.tau/4096
                for sid,value in self.unwrapped.items()}

    def recenter(self):
        self.reference = self.unwrapped.copy()


@dataclass
class Geometry:
    name: str
    mesh: str
    position: np.ndarray
    quaternion: np.ndarray
    motor_id: int | None = None


@dataclass
class Link:
    name: str
    parent: str | None
    position: np.ndarray
    quaternion: np.ndarray
    joint_id: int | None = None
    axis: np.ndarray = field(default_factory=lambda: np.array([0.,0.,1.]))
    geometries: list[Geometry] = field(default_factory=list)


class CadRig:
    def __init__(self):
        root = ET.parse(MODEL_XML, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))).getroot()
        self.links = []
        self.joint_links = {}
        self.mesh_files = {m.get('name') or Path(m.get('file')).stem: m.get('file')
                           for m in root.findall('asset/mesh')}

        def walk(body, parent):
            joint = body.find('joint')
            sid = BY_NAME.get(joint.get('name')) if joint is not None else None
            link = Link(body.get('name'), parent, vector(body.get('pos')),
                        vector(body.get('quat'), '1 0 0 0'), sid,
                        vector(joint.get('axis'), '0 0 1') if joint is not None else np.array([0.,0.,1.]))
            self.links.append(link)
            if sid is not None:
                self.joint_links[sid] = link
            part = ''
            for elem in body:
                if elem.tag is ET.Comment and (elem.text or '').strip().startswith('Part '):
                    part = elem.text.strip()[5:]
                elif elem.tag == 'geom' and elem.get('class') == 'visual':
                    link.geometries.append(Geometry(part, elem.get('mesh'), vector(elem.get('pos')),
                                                   vector(elem.get('quat'), '1 0 0 0'), PART_IDS.get(part)))
                elif elem.tag == 'body':
                    walk(elem, link.name)

        walk(root.find('worldbody/body'), None)
        # The RL mesh export fixes the bill to the head. Add only its visual hinge,
        # using motor 34's CAD shaft line; its closed-loop linkage is simplified.
        head = next(link for link in self.links if link.name == 'jaw_soft')
        motor = next(g for g in head.geometries if g.motor_id == 34)
        rot = quat_matrix(motor.quaternion)
        pivot = motor.position + rot @ np.array([-.0145,0.,0.])
        mouth = Link('mouth_visual_hinge', head.name, pivot, np.array([1.,0.,0.,0.]),
                     34, rot @ np.array([-1.,0.,0.]))
        for geom in list(head.geometries):
            if geom.mesh in ('jaw','jaw_soft'):
                head.geometries.remove(geom)
                geom.position = geom.position-pivot
                mouth.geometries.append(geom)
        self.links.append(mouth)
        self.joint_links[34] = mouth
        assert set(self.joint_links) == set(JOINTS)

    def local_quat(self, link, angles):
        if link.joint_id is None:
            return link.quaternion
        return quat_mul(link.quaternion, axis_quat(link.axis, angles.get(link.joint_id, 0.)))

    def world_transforms(self, angles):
        world = {}
        for link in self.links:
            local = transform(link.position, self.local_quat(link, angles))
            world[link.name] = (world[link.parent] @ local) if link.parent else local
        return world


def color_for(mesh):
    if mesh == 'xl330' or mesh in ('np_f970','lens','m12_lens_holder','speaker'):
        return (38,43,49)
    if 'bearing__' in mesh:
        return (159,167,170)
    if mesh.startswith('sole') or mesh in ('jaw_soft','soft_mouth_top'):
        return (249,196,57)
    if mesh.startswith(('foot','ankle','bottom_head')) or mesh == 'jaw':
        return (237,116,43)
    if mesh in ('hip_l','leg','bearing_roll','neck_pitch','neck','face_part'):
        return (129,144,152)
    if 'pcb' in mesh:
        return (44,114,87)
    return (226,230,225)
