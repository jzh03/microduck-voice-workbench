"""Bind a recipient's calibrated position and IMU mount to the packaged runtime."""
import argparse
import hashlib
import ipaddress
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / 'walking-runtime'
VIEWER = ROOT / 'local-joint-viewer'
BOARD_FILES = '''
run_teleop.py run_manual_stance.py hardware_bridge.py run_bridge.py
policy_coordinates.py stance_kinematics.py stance_reference.py stance_state.py
teleop_state.py handoff_pose.py flight_recorder.py control_lease.py
recover_motor_state.py runtime-contract.json bridge-robotd.toml ground-robotd.toml
'''.split()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('host', type=ipaddress.IPv4Address)
    parser.add_argument('--profile', type=Path, required=True, help='This robot’s calibrated software-zero-reference.json')
    parser.add_argument('--imu-mount', type=float, nargs=4, required=True, metavar=('W', 'X', 'Y', 'Z'))
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in args.imu_mount) or abs(math.hypot(*args.imu_mount) - 1.) > .001:
        parser.error('IMU installation quaternion must have unit length')
    contract_path = RUNTIME / 'runtime-contract.json'
    contract = json.loads(contract_path.read_bytes())
    profile = args.profile.read_bytes()
    import sys
    sys.path.insert(0, str(VIEWER))
    from joint_calibration import JointCalibration
    calibration = JointCalibration(json.loads(profile))
    if set(calibration.joints) != set(contract['joint_ids']):
        parser.error('Calibration must cover the 15 expected servo IDs')
    contract['robot_imu_mount_wxyz'] = args.imu_mount
    save(contract_path, contract)
    profile_sha = sha(profile)
    (VIEWER/'software-zero-reference.json').write_bytes(profile)
    limits = json.loads((VIEWER/'runtime-joint-limits.template.json').read_bytes())
    limits['software_reference_sha256'] = profile_sha
    save(VIEWER/'runtime-joint-limits.json', limits)
    ref = {'kind': 'model_home_reference_v1', 'profile_sha256': profile_sha,
           'contract_sha256': sha(contract_path.read_bytes()), 'joint_radians': contract['home_rad'],
           'automatic_start': False}
    save(RUNTIME/'standing-reference.json', ref)
    save(RUNTIME/'bridge-deployment.json', {'profile_sha256': profile_sha,
         'sha256': {name: sha((RUNTIME/name).read_bytes()) for name in BOARD_FILES}})
    save(RUNTIME/'workbench-config.json', {'host': str(args.host),
         'certificate': 'outputs/imu-direction/imu-direction-verified.json',
         'reference': 'walking-runtime/standing-reference.json',
         'manual_reference': 'walking-runtime/manual-standing-reference.json',
         'deployment_report': 'walking-runtime/bridge-deployment.json'})
    print('配置已生成。安装板端文件后，在自己的设备上核对 IMU 方向。')


if __name__ == '__main__':
    main()
