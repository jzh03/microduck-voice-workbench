"""Bound the live tail while retaining losslessly compressed sensor archives."""
import gzip
import hashlib
from pathlib import Path
import queue
import threading
import time
import uuid


def archive(path):
    path = Path(path)
    target = path.with_suffix('.gz')
    temporary = target.with_suffix('.gz.part')
    source_hash = hashlib.sha256()
    with path.open('rb') as source, gzip.open(temporary, 'wb', compresslevel=1) as dest:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            source_hash.update(chunk)
            dest.write(chunk)
    restored_hash = hashlib.sha256()
    with gzip.open(temporary, 'rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            restored_hash.update(chunk)
    if source_hash.digest() != restored_hash.digest():
        raise RuntimeError('Sensor log archive failed its integrity check')
    temporary.replace(target)
    target.with_suffix('.gz.sha256').write_text(source_hash.hexdigest() + '\n')
    path.unlink()
    return target


class FrameLog:
    def __init__(self, path, limit=64 * 1024 * 1024):
        self.path = Path(path)
        self.limit = limit
        self.stream = self.path.open('a', buffering=1, encoding='utf-8')
        self.size = self.path.stat().st_size
        self.pending = queue.Queue()
        self.error = None
        threading.Thread(target=self.compress, daemon=True).start()
        for old in self.path.parent.glob('encoder-frames-*.pending.jsonl'):
            self.pending.put(old)

    def compress(self):
        while True:
            path = self.pending.get()
            try:
                archive(path)
            except Exception as exc:
                # Keep the complete raw file if compression or validation fails.
                self.error = str(exc)
            finally:
                self.pending.task_done()

    def write(self, text):
        if self.size >= self.limit:
            self.stream.close()
            name = f'encoder-frames-{time.time_ns()}-{uuid.uuid4().hex[:8]}.pending.jsonl'
            old = self.path.with_name(name)
            self.path.replace(old)
            self.stream = self.path.open('a', buffering=1, encoding='utf-8')
            self.size = 0
            # Compression never blocks the control relay or sensor receipt.
            self.pending.put(old)
        self.stream.write(text)
        self.size += len(text.encode('utf-8'))

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.stream.close()
