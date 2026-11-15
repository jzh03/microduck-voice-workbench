"""Local motion requests for the existing run_teleop control loop."""

import json
import os
from pathlib import Path
import secrets
import socket
import stat
import struct
import time


PRESETS = {
    'forward': (0.05, 0.0, 0.0),
    'left': (0.0, 0.0, 0.20),
    'right': (0.0, 0.0, -0.20),
}
DURATION = 0.4
COOLDOWN = 3.0
CONFIRM_WINDOW = 10.0
LEASE = 0.35
ZERO = (0., 0., 0.)


class VoiceControl:
    def __init__(self, commands, emit, *, clock=time.monotonic, directions=PRESETS):
        self.commands = commands
        self.clock = clock
        self.emit = emit
        self.directions = directions
        self.listener = self.peer = self.pending = self.active = None
        self.lock_fd = None
        self.path = None
        self.last_seen = self.last_action = float('-inf')
        self.last_request = 0

    def open(self, filename):
        if not filename:
            return
        import fcntl

        path = Path(filename)
        path.parent.mkdir(mode=0o700, exist_ok=True)
        directory = path.parent.lstat()
        if (not stat.S_ISDIR(directory.st_mode) or directory.st_uid != 0 or
                directory.st_mode & 0o077 or os.geteuid() != 0):
            raise ValueError('Voice socket requires a private root directory')
        self.lock_fd = os.open(str(path) + '.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            stale = path.lstat()
        except FileNotFoundError:
            pass
        else:
            if not stat.S_ISSOCK(stale.st_mode) or stale.st_uid != 0:
                raise ValueError('Refusing to replace the voice socket path')
            path.unlink()
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET | socket.SOCK_NONBLOCK)
        self.listener.bind(str(path))
        self.path = path
        path.chmod(0o600)
        self.listener.listen(1)

    def close(self):
        with self.commands.lock:
            self._disconnect()
            if self.listener is not None:
                self.listener.close()
                self.listener = None
            if self.path is not None:
                try:
                    self.path.unlink(missing_ok=True)
                except OSError as exc:
                    self.emit('voice_socket_cleanup_failed', reason=str(exc))
                self.path = None
            if self.lock_fd is not None:
                os.close(self.lock_fd)
                self.lock_fd = None

    def _disconnect(self):
        if self.peer is not None:
            self.peer.close()
            self.peer = None
        self.pending = self.active = None
        self.last_seen = float('-inf')

    def _send(self, **message):
        if self.peer is not None:
            try:
                self.peer.send(json.dumps(message, separators=(',', ':')).encode())
            except OSError:
                self._disconnect()

    def _cancel_pending(self, reason):
        if self.pending is not None:
            request, *_ = self.pending
            self.pending = None
            self._send(event='result', request=request, status=reason)

    def operator(self, value):
        """Accept existing operator commands and arbitrate them with voice motion."""
        action = value.get('action')
        with self.commands.lock:
            # Background neutral drive frames must not cancel a voice action.
            neutral = (action == 'drive' and
                       all(type(value.get(key)) in (float, int) and value[key] == 0
                           for key in ('vx', 'vy', 'vyaw')))
            self.commands.receive(value)
            if action in ('step', 'off', 'confirm_balance', 'stand_pitch', 'stand_height') or (
                    action == 'drive' and not neutral):
                self._cancel_pending('manual_takeover')
                if self.active is not None:
                    request, *_ = self.active
                    self.active = None
                    self._send(event='result', request=request, status='manual_takeover')

    def _ready(self, healthy):
        return (healthy and self.commands.ready and self.commands.unlocked and
                not self.commands.stopped and not self.commands.recovering and
                not self.commands.recovery_confirmation_required and self.commands.command() == ZERO)

    def _handle(self, message, healthy):
        if not isinstance(message, dict):
            raise ValueError('Expected a voice message object')
        action = message.get('action')
        now = self.clock()
        if action == 'heartbeat':
            if now - self.last_seen >= LEASE:
                self._disconnect_motion()
            self.last_seen = now
            return
        request = message.get('request')
        if type(request) is not int or not 0 < request < 2**63:
            raise ValueError('Expected a voice request number')
        if action == 'cancel':
            if self.pending is not None and self.pending[0] == request:
                self._cancel_pending('cancelled')
        elif action == 'request':
            if request <= self.last_request:
                return
            self.last_request = request
            direction = message.get('direction')
            if (not isinstance(direction, str) or direction not in self.directions or self.pending or
                    self.active or now - self.last_action < COOLDOWN or now - self.last_seen >= LEASE or
                    not self._ready(healthy)):
                self._log_rejection(request, healthy, now)
                self._send(event='result', request=request, status='rejected')
                return
            token = secrets.token_hex(16)
            self.pending = request, direction, token, now + CONFIRM_WINDOW
            self._send(event='confirm', request=request, direction=direction, token=token)
        elif action == 'confirm':
            if self.pending is None or self.pending[0] != request:
                return
            _, direction, token, deadline = self.pending
            self.pending = None
            if (now >= deadline or now - self.last_seen >= LEASE or token != message.get('token') or
                    now - self.last_action < COOLDOWN or not self._ready(healthy)):
                self._send(event='result', request=request, status='rejected')
                return
            self.last_action = now
            self.active = request, direction, now + DURATION
            self.emit('voice_motion_accepted', direction=direction, request=request)
        else:
            raise ValueError('Unknown voice action')

    def _log_rejection(self, request, healthy, now):
        self.emit('voice_motion_rejected', request=request, healthy=healthy,
                  ready=self.commands.ready, unlocked=self.commands.unlocked,
                  lease_current=now - self.last_seen < LEASE, manual_command=self.commands.command())

    def _disconnect_motion(self):
        self._cancel_pending('connection_lost')
        if self.active is not None:
            request, *_ = self.active
            self.active = None
            self._send(event='result', request=request, status='connection_lost')
        self.last_seen = float('-inf')

    def _poll(self, healthy):
        if self.peer is None and self.listener is not None:
            try:
                peer, _ = self.listener.accept()
            except BlockingIOError:
                return
            _, uid, _ = struct.unpack('3i', peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            if uid != 0:
                peer.close()
                return
            peer.setblocking(False)
            self.peer = peer
            self.last_request = 0
        for _ in range(12):
            if self.peer is None:
                break
            try:
                data = self.peer.recv(513)
                if not data or len(data) > 512:
                    self._disconnect()
                    break
                self._handle(json.loads(data), healthy)
            except BlockingIOError:
                break
            except (OSError, ValueError, TypeError):
                self._disconnect()
                break

    def send_command(self, rpc, healthy):
        """The session's single writer of robot.move."""
        with self.commands.lock:
            self._poll(healthy)
            now = self.clock()
            if now - self.last_seen >= LEASE:
                self._disconnect_motion()
            if self.pending is not None and (now >= self.pending[3] or not self._ready(healthy)):
                self._cancel_pending('expired_or_unavailable')
            completion = None
            command = self.commands.command()
            if self.active is not None:
                request, direction, deadline = self.active
                if not self._ready(healthy):
                    self.active = None
                    completion = request, 'control_unavailable'
                elif now >= deadline:
                    self.active = None
                    completion = request, 'completed'
                else:
                    command = PRESETS[direction]
        try:
            ack = rpc('robot.move', dict(zip(('vx', 'vy', 'vyaw'), command)))
            if ack.get('accepted') is not True:
                raise RuntimeError('Controller rejected velocity command')
        except Exception:
            with self.commands.lock:
                if completion is not None or self.active is not None:
                    request, *_ = completion or self.active
                    self.active = None
                    self._send(event='result', request=request, status='unknown')
            raise
        if completion is not None:
            with self.commands.lock:
                request, status = completion
                self._send(event='result', request=request, status=status)
            self.emit('voice_motion_finished', request=request, status=status)
        return command
