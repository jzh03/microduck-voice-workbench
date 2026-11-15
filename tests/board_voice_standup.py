"""Validate delayed voice actions with the hardware-free board bridge."""

import asyncio
import json
from pathlib import Path
import sys
import tempfile

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed


async def verify(client, bridge, root, *, walk=False):
    with tempfile.TemporaryDirectory(prefix='standup-test-', dir='/run') as directory:
        sock = Path(directory) / 'voice.sock'
        # Exercise a read-only preflight longer than the heartbeat lease. The
        # fixture intercepts the controller launch and never opens a device.
        fixture = """
import runpy, sys, time
module = runpy.run_path(sys.argv.pop(1))
control = module['StandupControl']
def preflight(self):
    self.log('fixture_preflight', request=self.active[0])
    time.sleep(.7)
def run_trial(self, cancelled):
    self.log('simulated_floor_recovery')
    return 'standup_simulated'
control.preflight = preflight
control.run_trial = run_trial
module['main']()
"""
        runner = [str(bridge), '--simulation', '--walk'] if walk else ['-c', fixture, str(bridge)]
        controller = await asyncio.create_subprocess_exec(
            sys.executable, '-B', *runner, '--runtime', str(root),
            '--certificate', str(root), '--reference', str(root), '--socket', str(sock),
            stdout=asyncio.subprocess.PIPE,
        )
        child = None
        records = []
        try:
            while not sock.exists():
                await asyncio.sleep(.02)
            connection, events = asyncio.Future(), asyncio.Queue()

            async def handle(ws):
                connection.set_result(ws)
                try:
                    async for raw in ws:
                        event = json.loads(raw)
                        if event['type'] != 'input_audio_buffer.append':
                            await events.put(event)
                except ConnectionClosed:
                    pass

            async with serve(handle, '127.0.0.1', 0) as server:
                port = server.sockets[0].getsockname()[1]
                child = await asyncio.create_subprocess_exec(str(client), f'ws://127.0.0.1:{port}/',
                    str(sock), 'standup-walk' if walk else 'standup',
                    stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE)
                ws = await connection

                async def send(kind, **fields):
                    await ws.send(json.dumps(dict(type=kind, **fields)))

                async def result(call, status):
                    event = await events.get()
                    assert event['type'] == 'conversation.item.create', event
                    assert event['item']['call_id'] == call, event
                    assert json.loads(event['item']['output'])['status'] == status, event

                async def ask(n, tools):
                    await send('input_audio_buffer.speech_started', item_id=f'user-{n}')
                    await send('input_audio_buffer.speech_stopped', item_id=f'user-{n}')
                    await send('response.created', response={'id': n})
                    for name, args, call in tools:
                        await send('response.function_call_arguments.done', response_id=n, name=name,
                                   arguments=json.dumps(args), call_id=call)
                    await send('response.done', response={'id': n, 'status': 'completed'})

                await send('session.created')
                settings = (await events.get())['session']
                names = ['stand_up', 'step_forward'] if walk else ['stand_up']
                assert [tool['name'] for tool in settings['tools']] == names, settings
                await send('session.updated', session=settings)
                assert b'Listening' in await child.stdout.readline()
                await asyncio.sleep(.2)
                clock = asyncio.get_running_loop().time
                received = clock()
                await ask('first', [('move_short', {'direction':'forward'}, 'walk'),
                                    ('stand_up', {'speed':1}, 'invalid'), ('stand_up', {}, 'okay')])
                await result('walk', 'invalid_tool_or_arguments')
                await result('invalid', 'invalid_tool_or_arguments')
                await result('okay', 'standup_simulated')
                assert clock() - received >= 1.
                assert (await events.get())['type'] == 'response.create'
                assert 'standup_simulated' in (await child.stdout.readline()).decode()
                await send('input_audio_buffer.speech_started', item_id='speaker-echo')
                await send('input_audio_buffer.speech_stopped', item_id='speaker-echo')
                await send('error', error={'param':'call_id', 'message':'Unknown function call id: okay'})
                assert b'Listening (2)' in await child.stdout.readline()
                await asyncio.sleep(3.1)
                if walk:
                    received = clock()
                    await ask('second', [('step_forward', {}, 'forward')])
                    await result('forward', 'forward_step_simulated')
                    assert clock() - received >= 1.
                    assert (await events.get())['type'] == 'response.create'
                else:
                    await ask('second', [('stand_up', {}, 'aborted')])
                    while True:
                        line = await controller.stdout.readline()
                        assert line, 'Board bridge exited before the preflight record'
                        if not line.startswith(b'{'):
                            continue
                        record = json.loads(line)
                        records.append(record)
                        if record['event'] == 'fixture_preflight' and record.get('request') == 2:
                            break
                await ws.close()
                await child.wait()
        finally:
            if child and child.returncode is None:
                child.kill()
                await child.wait()
            controller.terminate()
            data, _ = await controller.communicate()
        records.extend(json.loads(line) for line in data.splitlines() if line.startswith(b'{'))
        assert [x['event'] for x in records].count('simulated_floor_recovery') == 1, records
        if walk:
            assert [x['event'] for x in records].count('simulated_forward_step') == 1, records
            print('PASS: standup and forward each waited one second using no terminal input')
        else:
            print('PASS: stand_up allowlist, delayed start, slow preflight, disconnect during preflight')


if __name__ == '__main__':
    inputs = [Path(x).resolve() for x in sys.argv[1:]]
    asyncio.run(asyncio.wait_for(verify(*inputs), 18))
    asyncio.run(asyncio.wait_for(verify(*inputs, walk=True), 18))
