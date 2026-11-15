"""Token-bound input lease for a USB-supervised physical trial."""
import json
import os
import select
import threading
import time


class ControlLease:
    def __init__(self, token, *, timeout=.5, clock=time.monotonic):
        if not isinstance(token, str) or len(token) < 16:
            raise ValueError('Control token must have at least 16 characters')
        self.token, self.timeout, self.clock = token, timeout, clock
        self.started = clock()
        self.last = None
        self.reason = None

    def receive(self, command):
        if not isinstance(command, dict) or command.get('token') != self.token:
            return False
        if command.get('action') == 'heartbeat':
            self.last = self.clock()
            return True
        if command.get('action') == 'off':
            self.reason = 'Operator requested stop'
            return True
        return False

    def abort_reason(self):
        if self.reason:
            return self.reason
        if self.clock() - (self.last if self.last is not None else self.started) > self.timeout:
            return 'USB operator heartbeat expired'
        return None


def watch_stdin(lease, finished, on_stop, *, fd=0):
    """Read receipt time independently of motor sampling and configuration."""
    def run():
        pending = b''
        while not finished.is_set():
            if select.select([fd], [], [], .02)[0]:
                data = os.read(fd, 4096)
                if not data:
                    lease.reason = 'USB operator input closed'
                pending += data
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    try:
                        lease.receive(json.loads(line))
                    except (ValueError, UnicodeDecodeError):
                        lease.reason = 'Invalid USB operator command'
                if len(pending) > 4096:
                    lease.reason = 'Oversized USB operator command'
            if lease.abort_reason() and not finished.is_set():
                on_stop()
                return
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread
