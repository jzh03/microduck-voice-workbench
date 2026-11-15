"""Bounded, asynchronous diagnostics. This module has no device/control API."""
import json
import os
from pathlib import Path
import queue
import threading
import time


class FlightRecorder:
    def __init__(self, path, *, capacity=4096):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.stream=os.fdopen(os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_APPEND,0o600),'w',buffering=1)
        self.queue=queue.Queue(maxsize=capacity)
        self.dropped=0;self.error=None;self.closed=False
        self.thread=threading.Thread(target=self._write,daemon=True);self.thread.start()

    def record(self,event,**data):
        if self.closed or self.error:return False
        item={'event':event,'unix_time':time.time(),'monotonic_ns':time.monotonic_ns(),**data}
        try:self.queue.put_nowait(item);return True
        except queue.Full:self.dropped+=1;return False

    def _write(self):
        last_sync=time.monotonic()
        try:
            while True:
                try:item=self.queue.get(timeout=.5)
                except queue.Empty:
                    self.stream.flush();os.fsync(self.stream.fileno());last_sync=time.monotonic();continue
                if item is None:break
                self.stream.write(json.dumps(item,separators=(',',':'),ensure_ascii=False,allow_nan=False)+'\n')
                if time.monotonic()-last_sync>=.5 or item['event'] in ('hardware_fault','torque_off_intent','session_exception','session_finished','process_signal','controller_exited'):
                    self.stream.flush();os.fsync(self.stream.fileno());last_sync=time.monotonic()
            self.stream.write(json.dumps({'event':'recorder_closed','unix_time':time.time(),'dropped':self.dropped})+'\n')
            self.stream.flush();os.fsync(self.stream.fileno())
        except Exception as exc:self.error=str(exc)
        finally:self.stream.close()

    def close(self):
        if self.closed:return
        self.closed=True
        try:self.queue.put(None,timeout=.5)
        except queue.Full:self.error=self.error or 'Recorder queue did not drain';return
        self.thread.join(timeout=2)


def emit(recorder,event,**data):
    if recorder is not None:
        try:recorder.record(event,**data)
        except Exception:pass
