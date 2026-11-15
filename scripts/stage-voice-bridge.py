"""Prepare a bridge overlay from the board's reviewed runtime and leave it untouched."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


HERE = Path(__file__).resolve().parent
BASELINE = '722869546f243390bfc6595550e98b1a789275db27ee94742c38b76454a6754d'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime', type=Path)
    parser.add_argument('candidate', type=Path)
    args = parser.parse_args()
    runtime = args.runtime.resolve(strict=True)
    if digest(runtime / 'run_teleop.py') != BASELINE:
        raise SystemExit('The deployed control bridge differs from the reviewed baseline. Rebase the patch first.')
    manifest = json.loads((runtime / 'bridge-deployment.json').read_text(encoding='utf-8'))
    if manifest['sha256'].get('run_teleop.py') != BASELINE:
        raise SystemExit('The control bridge differs from its deployment manifest.')
    for name, expected in manifest['sha256'].items():
        if Path(name).name != name or digest(runtime / name) != expected:
            raise SystemExit(f'Deployment differs: {name}')
    args.candidate.mkdir(mode=0o700)
    for name in manifest['sha256']:
        shutil.copy2(runtime / name, args.candidate / name)
    shutil.copy2(HERE.parent / 'src' / 'voice_control.py', args.candidate)
    subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1', '-i', str(HERE / 'run_teleop-voice.patch')],
                   cwd=args.candidate, check=True)
    for name in ('run_teleop.py', 'voice_control.py'):
        manifest['sha256'][name] = digest(args.candidate / name)
    (args.candidate / 'bridge-deployment.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
