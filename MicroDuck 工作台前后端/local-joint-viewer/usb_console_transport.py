#!/usr/bin/env python3
"""Authenticated USB ACM transport for the existing JSON console worker.

It sends no arm command. The remote worker owns the motor UART and releases on
controller-heartbeat expiry. A separate 3-second link lease also exits an idle
worker after cable loss, so a subsequent connection can recover the login shell.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import select
import signal
import sys
import termios
import time
import tty
import uuid

ROOT = Path(__file__).resolve().parent.parent
CREDENTIALS = ROOT/'outputs/radxa-flash-20261017/access/board-credentials.txt'


class UsbConsole:
    def __init__(self, port):
        if not re.fullmatch(r'/dev/cu\.usbmodem[A-Za-z0-9._-]+', port):
            raise ValueError('Expected a USB ACM device')
        directory = ROOT/'outputs/live-joint-viewer-20261017'
        directory.mkdir(parents=True, exist_ok=True)
        self.lock = (directory/'usb-console.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.original = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        fields = dict(line.split(':', 1) for line in CREDENTIALS.read_text().splitlines() if ':' in line)
        self.password = {k.strip().lower(): v.strip() for k, v in fields.items()}['root password']

    def close(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
        finally:
            os.close(self.fd)
            self.lock.close()

    def send(self, data):
        data = memoryview(data.encode() if isinstance(data, str) else data)
        deadline = time.monotonic() + 2
        while data:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError('USB write timed out')
            count = os.write(self.fd, data)
            if count <= 0:
                raise OSError('USB disconnected')
            data = data[count:]

    def receive(self):
        data = os.read(self.fd, 65536)
        if not data:
            raise OSError('USB disconnected')
        return data

    def until(self, pattern, timeout):
        text = ''
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], .2)[0]:
                text += self.receive().decode(errors='replace')
                text = text[-131072:]
                clean = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text)
                if re.search(pattern, clean):
                    return clean
        raise TimeoutError('USB prompt timed out: ' + text.replace(self.password, '[redacted]')[-500:])

    def login(self):
        self.send('\r')
        prompt = self.until(r'(?im)(login:\s*$|password:\s*$|root@mumu-duck[^\n]*#\s*$)', 15)
        if re.search(r'(?im)login:\s*$', prompt):
            self.send('root\r')
            prompt = self.until(r'(?im)password:\s*$', 10)
        if re.search(r'(?im)password:\s*$', prompt):
            self.send(self.password + '\r')
            prompt = self.until(r'(?im)(root@mumu-duck[^\n]*#\s*$|login incorrect|login:\s*$)', 15)
        if not re.search(r'(?m)root@mumu-duck[^\n]*#\s*$', prompt):
            raise RuntimeError('USB login failed')


def stream(console, stop, stdin_fd=0, emit=None):
    if emit is None:
        def emit(frame):
            print(json.dumps(frame), flush=True)
    marker = '__MICRODUCK_USB_EXIT_' + uuid.uuid4().hex
    command = ("stty -echo -icanon min 1 time 0; "
               "cd /root/microduck-imu-update && python3 -u console_worker.py --transport-lease 3; "
               "stty icanon echo; printf '\\n" + marker + "\\n'\r")
    console.send(command)
    incoming = b''
    commands = b''
    generation = None
    last_frame = time.monotonic()
    last_ping = 0
    exited = False
    try:
        while not stop():
            now = time.monotonic()
            if now-last_frame > 5:
                raise TimeoutError('USB frame stream stalled')
            if generation and now-last_ping > .4:
                console.send(json.dumps({'action': 'link_ping', 'generation': generation})+'\n')
                last_ping = now
            ready, _, _ = select.select([console.fd, stdin_fd], [], [], .05)
            if console.fd in ready:
                incoming += console.receive()
                while b'\n' in incoming:
                    line, incoming = incoming.split(b'\n', 1)
                    if line.strip() == marker.encode():
                        exited = True
                        raise RuntimeError('USB worker exited')
                    try:
                        frame = json.loads(line)
                    except (ValueError, UnicodeDecodeError):
                        continue
                    if isinstance(frame, dict) and frame.get('type') == 'frame':
                        generation = frame['control']['generation']
                        last_frame = time.monotonic()
                        frame['transport'] = 'USB 数据线'
                        emit(frame)
                if len(incoming) > 131072:
                    raise RuntimeError('Oversized USB frame')
            if stdin_fd in ready:
                data = os.read(stdin_fd, 8192)
                if not data:
                    break
                commands += data
                while b'\n' in commands:
                    line, commands = commands.split(b'\n', 1)
                    if len(line) > 8192:
                        raise ValueError('Oversized command')
                    request = json.loads(line)
                    if not isinstance(request, dict):
                        raise ValueError('Expected a command object')
                    console.send(json.dumps(request)+'\n')
                if len(commands) > 8192:
                    raise ValueError('Oversized command')
    finally:
        if not exited:
            try:
                # Ctrl-C is handled by the worker's SIGINT cleanup, not a hard kill.
                console.send('{"action":"off"}\n\x03')
                console.until(re.escape(marker)+r'\r?\n', 10)
            except (OSError, TimeoutError):
                # Remote heartbeat and link lease remain responsible after cable loss.
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port')
    args = parser.parse_args()
    stopping = [False]
    def halt(*_):
        stopping[0] = True
    signal.signal(signal.SIGTERM, halt)
    signal.signal(signal.SIGINT, halt)
    console = UsbConsole(args.port)
    try:
        console.login()
        stream(console, lambda: stopping[0])
    finally:
        console.close()


if __name__ == '__main__':
    main()
