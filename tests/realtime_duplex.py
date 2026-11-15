"""Mock WebSocket tests for playback interruption on a continuously open mic."""

import asyncio
import base64
import json
import struct
import sys
from pathlib import Path

from websockets.asyncio.server import serve


async def verify(client):
    connection = asyncio.Future()

    async def handle(ws):
        connection.set_result(ws)
        await ws.wait_closed()

    async with serve(handle, "127.0.0.1", 0) as server:
        port = server.sockets[0].getsockname()[1]
        process = await asyncio.create_subprocess_exec(
            str(client), f"ws://127.0.0.1:{port}/",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
        )
        try:
            ws = await asyncio.wait_for(connection, 3)

            async def send(event_type, **fields):
                await ws.send(json.dumps({"type": event_type, **fields}))

            async def receive(event_type):
                async def next_event():
                    while True:
                        event = json.loads(await ws.recv())
                        if event["type"] == event_type:
                            return event
                        assert event["type"] == "input_audio_buffer.append", event
                return await asyncio.wait_for(next_event(), 3)

            async def line(expected):
                received = await asyncio.wait_for(process.stdout.readline(), 3)
                assert received.decode().strip() == expected, (received, expected)

            async def audio(response_id, sample):
                delta = base64.b64encode(struct.pack("<h", sample)).decode()
                await send("response.audio.delta", response_id=response_id, delta=delta)

            await send("session.created")
            update = await receive("session.update")
            assert update["session"]["turn_detection"]["type"] == "server_vad"
            assert update["session"]["turn_detection"]["threshold"] == 0.50
            await send("session.updated", session=update["session"])
            await line("Listening (1)")

            await send("input_audio_buffer.speech_started", item_id="first")
            await send("input_audio_buffer.speech_stopped", item_id="first")
            await send("response.created", response={"id": "old"})
            await audio("old", 101)
            await line("Queued 101")
            await receive("input_audio_buffer.append")

            await send("input_audio_buffer.speech_started", item_id="second")
            await line("Dropped playback")
            await line("Interrupted playback")
            cancel = await receive("response.cancel")
            await send("input_audio_buffer.speech_started", item_id="second")
            await send("response.done", response={"id": "old", "status": "completed"})
            await send("error", error={"event_id": cancel["event_id"],
                                       "message": "No active response to cancel"})
            await audio("old", 999)
            await receive("input_audio_buffer.append")
            await send("input_audio_buffer.speech_stopped", item_id="second")
            await send("response.created", response={"id": "draining"})
            await audio("old", 999)
            await audio("draining", 202)
            await send("response.done", response={"id": "draining", "status": "completed"})
            await line("Queued 202")
            await receive("input_audio_buffer.append")

            await send("input_audio_buffer.speech_started", item_id="third")
            await line("Dropped playback")
            await line("Interrupted playback")
            # A second cancel is unexpected, including when reading subsequent input.
            await receive("input_audio_buffer.append")
            await send("input_audio_buffer.committed", item_id="third")
            await send("response.created", response={"id": "old"})
            await send("response.created", response={"id": "new"})
            await audio("draining", 999)
            await audio("new", 303)
            await line("Queued 303")
            await ws.close()
            stdout, stderr = await asyncio.wait_for(process.communicate(), 3)
            assert process.returncode == 1 and not stdout, (stdout, stderr)
            assert b"WebSocket was closed" in stderr, stderr
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()
    print("PASS: generation and drain interruption, cancel race, late audio and disconnect")


if __name__ == "__main__":
    asyncio.run(verify(Path(sys.argv[1]).resolve()))
