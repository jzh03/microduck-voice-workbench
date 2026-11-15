"""Exercise the action-tool contract without any robot connection."""

import asyncio
import json
from pathlib import Path
import sys

from websockets.asyncio.server import serve


async def verify(client):
    connected = asyncio.Future()

    async def connect(ws):
        connected.set_result(ws)
        await ws.wait_closed()

    async with serve(connect, '127.0.0.1', 0) as server:
        port = server.sockets[0].getsockname()[1]
        process = await asyncio.create_subprocess_exec(
            str(client), f'ws://127.0.0.1:{port}/', 'tools',
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
        )
        try:
            ws = await asyncio.wait_for(connected, 4)

            async def send(event_type, **fields):
                await ws.send(json.dumps({'type': event_type, **fields}))

            async def receive(event_type):
                while True:
                    event = json.loads(await asyncio.wait_for(ws.recv(), 4))
                    if event['type'] == event_type:
                        return event
                    assert event['type'] == 'input_audio_buffer.append', event

            async def result(call, status):
                item = (await receive('conversation.item.create'))['item']
                assert item['type'] == 'function_call_output' and item['call_id'] == call, item
                assert json.loads(item['output']) == {'status': status}, item

            async def tool(response, call, arguments, name='move_short'):
                await send('response.function_call_arguments.done', response_id=response,
                           call_id=call, arguments=arguments, name=name)

            await send('session.created')
            settings = (await receive('session.update'))['session']
            assert [tool['name'] for tool in settings['tools']] == ['move_short'], settings
            await send('session.updated', session=settings)
            await send('input_audio_buffer.speech_started', item_id='user-1')
            await send('input_audio_buffer.speech_stopped', item_id='user-1')
            await send('response.created', response={'id': 'first'})
            await tool('stale', 'late-1', '{"direction":"left"}')
            await tool('first', 'query', '{}', 'get_robot_status')
            await tool('first', 'invalid', '{"direction":"forward","seconds":2}')
            await tool('first', 'ok', '{"direction":"left"}')
            await tool('first', 'ok', '{"direction":"right"}')
            await tool('first', 'extra', '{"direction":"right"}')
            await send('response.done', response={'id': 'first', 'status': 'completed'})
            await result('query', 'invalid_tool_or_arguments')
            await result('invalid', 'invalid_tool_or_arguments')
            await result('extra', 'request_not_allowed_in_this_response')
            await result('ok', 'completed')
            await receive('response.create')
            await send('response.created', response={'id': 'follow-up'})
            await tool('follow-up', 'follow-up-call', '{"direction":"forward"}')
            await send('response.done', response={'id': 'follow-up', 'status': 'completed'})
            await result('follow-up-call', 'request_not_allowed_in_this_response')
            await send('input_audio_buffer.speech_started', item_id='user-2')
            await send('input_audio_buffer.speech_stopped', item_id='user-2')
            await send('response.created', response={'id': 'aborted'})
            await tool('aborted', 'aborted-call', '{"direction":"right"}')
            await send('input_audio_buffer.speech_started', item_id='user-3')
            await receive('response.cancel')
            await send('response.done', response={'id': 'aborted', 'status': 'completed'})
            await ws.close()
            output, errors = await asyncio.wait_for(process.communicate(), 4)
            requests = [line for line in output.splitlines() if line.startswith(b'Requested ')]
            assert requests == [b'Requested left'], (output, errors)
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()


if __name__ == '__main__':
    asyncio.run(verify(Path(sys.argv[1]).resolve()))
    print('PASS: allowlist, parameter checks, deduplication, stale calls and tool continuation')
