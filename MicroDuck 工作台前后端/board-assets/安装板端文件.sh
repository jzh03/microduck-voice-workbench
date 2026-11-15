#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
base=/root/microduck-imu-update
if [ "$(uname -m)" != aarch64 ] || [ "$(id -u)" != 0 ]; then
    echo '请在已配置的 ARM64 Linux 主板上以 root 执行。' >&2
    exit 1
fi
if [ -e "$base" ] || [ -L "$base" ] || [ -e /opt/robot/daemon/current ] || [ -L /opt/robot/daemon/current ] || [ -e /opt/robot/policies/current ] || [ -L /opt/robot/policies/current ]; then
    echo '检测到现有部署，请先对照现有版本。此脚本用于首次安装。' >&2
    exit 1
fi
command -v fuser >/dev/null
sha256sum -c board-assets/SHA256SUMS
if systemctl is-active --quiet robotd.service || fuser /dev/ttyS2 >/dev/null 2>&1; then
    echo '请先停止控制和串口采集，确认舵机卸力后再安装。' >&2
    exit 1
fi
python3 - <<'PY'
import hashlib
import json
from pathlib import Path

runtime = Path('walking-runtime')
contract = json.loads((runtime/'runtime-contract.json').read_bytes())
if not contract.get('robot_imu_mount_wxyz'):
    raise SystemExit('请先运行配置实机.py')
manifest = json.loads((runtime/'bridge-deployment.json').read_bytes())
def digest(file):
    return hashlib.sha256(file.read_bytes()).hexdigest()
for name, expected in manifest['sha256'].items():
    if digest(runtime/name) != expected:
        raise SystemExit('文件哈希不符 ' + name)
for file, expected in (
    ('local-joint-viewer/software-zero-reference.json', manifest['profile_sha256']),
    ('board-assets/robotd', contract['native_home_override']['binary_sha256']),
    ('board-assets/policies-v8/velstand.onnx', contract['policy_sha256']),
):
    if digest(Path(file)) != expected:
        raise SystemExit('文件哈希不符 ' + file)
PY
stamp="$(date +%Y%m%d-%H%M%S)-$$"
native="/opt/robot/daemon/releases/workbench-$stamp"
policy="/opt/robot/policies/releases/workbench-$stamp"
stage="$base.install-$stamp"
mkdir -p "$native/bin" "$policy" "$stage/walking-runtime" "$stage/tools" /opt/robot/onnxruntime
install -m 755 board-assets/robotd "$native/bin/robotd"
install -m 644 board-assets/policies-v8/*.onnx "$policy/"
tar -xzf board-assets/onnxruntime-linux-aarch64-1.28.0.tgz -C /opt/robot/onnxruntime
ort=/opt/robot/onnxruntime/onnxruntime-linux-aarch64-1.28.0/lib
LD_LIBRARY_PATH="$ort" ldd "$native/bin/robotd" > "$stage/robotd-libraries.txt"
if grep -q 'not found' "$stage/robotd-libraries.txt"; then
    cat "$stage/robotd-libraries.txt" >&2
    exit 1
fi
install -m 644 walking-runtime/bridge-deployment.json "$stage/walking-runtime/"
python3 - "$stage/walking-runtime" <<'PY'
import json
from pathlib import Path
import shutil
import sys
runtime = Path('walking-runtime')
manifest = json.loads((runtime/'bridge-deployment.json').read_bytes())
for name in manifest['sha256']:
    shutil.copy2(runtime/name, Path(sys.argv[1])/name)
PY
for item in console_worker.py joint_calibration.py joint_limits.py support_power.py soft_follow.py software-zero-reference.json runtime-joint-limits.json; do
    install -m 644 "local-joint-viewer/$item" "$stage/"
done
install -m 644 mumu-duck/imu-to-ttl/tools/dxl.py "$stage/tools/dxl.py"
systemctl mask robotd.service
mv "$stage" "$base"
ln -s "$native" "/opt/robot/daemon/current.new-$stamp"
mv -Tf "/opt/robot/daemon/current.new-$stamp" /opt/robot/daemon/current
ln -s "$policy" "/opt/robot/policies/current.new-$stamp"
mv -Tf "/opt/robot/policies/current.new-$stamp" /opt/robot/policies/current
echo '板端文件已安装。继续核对关节状态和 IMU 方向。'
