"""Exercise Chinese terminal confirmation on Linux with run_teleop --mode fake."""

import asyncio
import json
import os
from pathlib import Path
import pty
import secrets
import socket
import sys
import tempfile
import time
from urllib.request import Request, urlopen

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosedError


async def verify(bridge, client, directory):
    token = secrets.token_hex(24)
    token_file = directory / 'control-token'
    token_file.write_text(token, encoding='ascii')
    voice_socket = directory / 'v.sock'
    output = directory / directory.name
    trace = bridge.parent / 'diagnostics' / output.name / 'flight.jsonl'
    with socket.socket() as port_socket:
        port_socket.bind(('127.0.0.1', 0))
        port = port_socket.getsockname()[1]

    def post(path, **payload):
        request = Request(f'http://127.0.0.1:{port}/{path}',
                          data=json.dumps(payload).encode(), headers={'X-Control-Token': token})
        with urlopen(request, timeout=0.5) as result:
            return json.load(result)

    with (directory / 'teleop.log').open('wb') as logs:
        teleop = await asyncio.create_subprocess_exec(
            sys.executable, '-B', str(bridge), '--mode', 'fake', '--voice-socket', str(voice_socket),
            '--token-file', str(token_file), '--output-dir', str(output), '--port', str(port),
            '--test-seconds', '25', stdout=logs, stderr=logs,
        )
    child = None
    master = slave = None
    try:
        deadline = time.monotonic() + 4
        while not voice_socket.exists():
            assert teleop.returncode is None and time.monotonic() < deadline, 'Control socket missing'
            await asyncio.sleep(.05)
        deadline = time.monotonic() + 12
        while True:
            state = await asyncio.to_thread(post, 'command', action='heartbeat')
            if state.get('can_confirm_balance'):
                break
            assert state.get('running') and time.monotonic() < deadline, state
            await asyncio.sleep(.1)
        robot_socket = output / 'robotd.sock'
        children = Path(f'/proc/{teleop.pid}/task/{teleop.pid}/children').read_text().split()
        assert len(children) == 1
        robot = Path('/proc') / children[0]
        argv = (robot / 'cmdline').read_bytes().split(b'\0')
        assert b'--fake' in argv and os.fsencode(robot_socket) in argv, argv
        for fd in (robot / 'fd').iterdir():
            try:
                target = os.readlink(fd)
            except FileNotFoundError:
                continue
            assert not target.startswith('/dev/ttyS'), target
        await asyncio.to_thread(post, 'command', action='confirm_balance')
        # Leaving the supported pose resets the bridge's three-second readiness window.
        await asyncio.sleep(.25)
        deadline = time.monotonic() + 6
        while not (await asyncio.to_thread(post, 'status'))['can_confirm_balance']:
            assert time.monotonic() < deadline, 'Control did not settle after balance confirmation'
            await asyncio.sleep(.1)

        connection = asyncio.Future()
        events = asyncio.Queue()
        receiving_audio = asyncio.Event()
        closing = False

        async def handle(ws):
            connection.set_result(ws)
            try:
                async for data in ws:
                    event = json.loads(data)
                    if event['type'] == 'input_audio_buffer.append':
                        receiving_audio.set()
                    else:
                        await events.put(event)
            except ConnectionClosedError:
                if not closing:
                    raise

        async with serve(handle, '127.0.0.1', 0) as server:
            ws_port = server.sockets[0].getsockname()[1]
            master, slave = pty.openpty()
            child = await asyncio.create_subprocess_exec(
                str(client), f'ws://127.0.0.1:{ws_port}/', str(voice_socket), stdin=slave,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
            )
            ws = await asyncio.wait_for(connection, 4)

            async def send(kind, **fields):
                await ws.send(json.dumps(dict(type=kind, **fields)))

            async def receive(kind):
                event = await events.get()
                assert event['type'] == kind, event
                return event

            async def line(prefix):
                value = (await child.stdout.readline()).decode().strip()
                assert value.startswith(prefix), (prefix, value)

            async def proposal(turn):
                await send('input_audio_buffer.speech_started', item_id=f'item-{turn}')
                await send('input_audio_buffer.speech_stopped', item_id=f'item-{turn}')
                response = f'response-{turn}'
                await send('response.created', response={'id': response})
                await send('response.function_call_arguments.done', response_id=response,
                           call_id=f'call-{turn}', name='move_short', arguments='{"direction":"left"}')
                await send('response.done', response={'id': response, 'status': 'completed'})
                await line('待确认 左转')

            await send('session.created')
            settings = (await receive('session.update'))['session']
            await send('session.updated', session=settings)
            await line('Listening (1)')
            os.write(master, '确认\n'.encode())
            await receiving_audio.wait()
            await proposal(1)
            os.write(master, '确认 1\n'.encode())
            await asyncio.sleep(.3)
            assert 'voice_motion_accepted' not in trace.read_text(encoding='utf-8')
            os.write(master, '确认\n'.encode())
            await line('动作结果 completed')
            result = (await receive('conversation.item.create'))['item']
            assert result['call_id'] == 'call-1' and json.loads(result['output'])['status'] == 'completed'
            await receive('response.create')
            await asyncio.sleep(3.1)
            await proposal(2)
            closing = True
            await ws.close()
            _, errors = await asyncio.wait_for(child.communicate(), 4)
            assert child.returncode == 1 and b'WebSocket was closed' in errors, errors
        await asyncio.sleep(.5)
        await asyncio.to_thread(post, 'command', action='off')
        while (await asyncio.to_thread(post, 'status'))['running']:
            await asyncio.sleep(.05)
        await asyncio.to_thread(post, 'close')
        await asyncio.wait_for(teleop.wait(), 4)
        records = [json.loads(line) for line in trace.read_text(encoding='utf-8').splitlines()]
        names = [entry.get('event') for entry in records]
        assert names.count('voice_motion_accepted') == names.count('voice_motion_finished') == 1
        commands = [entry['command'] for entry in records if entry.get('event') == 'control_state']
        nonzero = [index for index, command in enumerate(commands) if any(command)]
        assert nonzero and all(commands[index] == [0., 0., .20] for index in nonzero), nonzero
        assert commands[max(nonzero) + 1] == [0., 0., 0.]
        print(f'PASS: robotd --fake, terminal confirmation, {len(nonzero)} motion updates, zero ack, '
              'and disconnected proposal revoked')
    finally:
        if child is not None and child.returncode is None:
            child.kill()
            await child.wait()
        for fd in (master, slave):
            if fd is not None:
                os.close(fd)
        if teleop.returncode is None:
            try:
                status = await asyncio.to_thread(post, 'status')
                if status['running']:
                    await asyncio.to_thread(post, 'command', action='off')
                    deadline = time.monotonic() + 6
                    while (await asyncio.to_thread(post, 'status'))['running'] and time.monotonic() < deadline:
                        await asyncio.sleep(.05)
                await asyncio.to_thread(post, 'close')
                await asyncio.wait_for(teleop.wait(), 4)
            except (OSError, TimeoutError):
                if teleop.returncode is None:
                    teleop.terminate()
                    print(f'Control process still exiting, PID {teleop.pid}', file=sys.stderr)


if __name__ == '__main__':
    stage = Path(sys.argv[3]).resolve()
    work = Path(tempfile.mkdtemp(prefix='trial-', dir=stage))
    print(f'Validation directory {work}', flush=True)
    asyncio.run(asyncio.wait_for(verify(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(), work), 32))
